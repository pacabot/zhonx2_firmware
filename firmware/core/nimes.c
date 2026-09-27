#include "nimes.h"
#include <limits.h>
#include <string.h>
#define STATES (NM_CELLS * 4)
int nm_neighbour(int cell, unsigned direction)
{
    if (cell < 0 || cell >= NM_CELLS || direction > 3) return -1;
    int x = cell % NM_SIDE, y = cell / NM_SIDE;
    switch (direction) {
    case NM_NORTH: return y + 1 < NM_SIDE ? cell + NM_SIDE : -1;
    case NM_EAST: return x + 1 < NM_SIDE ? cell + 1 : -1;
    case NM_SOUTH: return y ? cell - NM_SIDE : -1;
    default: return x ? cell - 1 : -1;
    }
}
unsigned nm_size(const nm_map_t *m) {return m->side;}
nm_pose_t nm_origin(const nm_map_t *m) {return (nm_pose_t){m->start_x,m->start_y,m->start_heading};}
int nm_next(const nm_map_t *m,int c,unsigned d)
{
    if(c<0 || c>=NM_CELLS || c%NM_SIDE>=m->side || c/NM_SIDE>=m->side)return -1;
    int n=nm_neighbour(c,d);
    return n>=0 && n%NM_SIDE<m->side && n/NM_SIDE<m->side?n:-1;
}
void nm_init_size(nm_map_t *map,unsigned side,int automatic)
{
    memset(map,0,sizeof *map);
    map->side=(side==6 || side==9 || side==16)?side:9;
    map->axes=automatic?0:3;
    map->start_x=map->start_y=automatic?(map->side-1)/2:0;
    if(!automatic)for(int c=0;c<NM_CELLS;++c)for(unsigned d=0;d<4;++d)
        if(nm_next(map,c,d)<0) {map->cell[c].known|=1u<<d;map->cell[c].walls|=1u<<d;}
}
void nm_init(nm_map_t *map) {nm_init_size(map,16,0);}
int nm_edge(nm_map_t *map, int c, unsigned d, int wall)
{
    if(c<0 || c>=NM_CELLS || d>3 || c%NM_SIDE>=map->side || c/NM_SIDE>=map->side)return -1;
    int n = nm_next(map,c,d);
    unsigned bit = 1u << d, opposite = 1u << ((d + 2) % 4);
    if (n < 0 && !wall) return -1;
    if ((map->cell[c].known & bit) && !!(map->cell[c].walls & bit) != !!wall) return -1;
    if (n >= 0 && (map->cell[n].known & opposite) && !!(map->cell[n].walls & opposite) != !!wall) return -1;
    map->cell[c].known |= bit;
    if (wall) map->cell[c].walls |= bit;
    if (n >= 0) {
        map->cell[n].known |= opposite;
        if (wall) map->cell[n].walls |= opposite;
    }
    return 0;
}
int nm_observe(nm_map_t *map, nm_pose_t p, uint8_t walls)
{
    if (p.x >= map->side || p.y >= map->side || p.heading > 3) return -1;
    nm_map_t copy = *map; /* No partial observation when a sensor contradicts a known edge. */
    unsigned dirs[3] = {p.heading, (p.heading + 3u) % 4u, (p.heading + 1u) % 4u};
    int c = p.y * NM_SIDE + p.x;
    for (unsigned i = 0; i < 3; ++i)
        if (nm_edge(&copy, c, dirs[i], walls & (1u << i))) return -1;
    copy.cell[c].visited = 1;
    *map = copy;
    return 0;
}
static int open_edge(const nm_map_t *map, int c, unsigned d)
{
    unsigned b = 1u << d;
    return (map->cell[c].known & b) && !(map->cell[c].walls & b);
}
int nm_goal(const nm_map_t *map, uint8_t targets[NM_CELLS])
{
    int rooms = 0;
    memset(targets, 0, NM_CELLS);
    for (int y = 0; y < map->side - 1; ++y) for (int x = 0; x < map->side - 1; ++x) {
        int c = y * NM_SIDE + x;
        if(map->side==16 && (map->axes!=3 || x!=7 || y!=7))continue;
        if (open_edge(map,c,NM_EAST) && open_edge(map,c,NM_NORTH) &&
            open_edge(map,c+1,NM_NORTH) && open_edge(map,c+NM_SIDE,NM_EAST)) {
            ++rooms;
            targets[c] = targets[c+1] = targets[c+NM_SIDE] = targets[c+NM_SIDE+1] = 1;
        }
    }
    return rooms;
}
/* Foreground-only shared workspace: bounded heap, no O(states squared) scan,
 * no large planner arrays on the interrupt/foreground stack. */
static uint32_t distance_to[STATES];
static int16_t predecessor[STATES],heap[STATES],heap_at[STATES];
static unsigned heap_count;
static int before(int a,int b)
{return distance_to[a]<distance_to[b] || (distance_to[a]==distance_to[b] && a<b);}
static void heap_swap(unsigned a,unsigned b)
{
    int16_t t=heap[a];heap[a]=heap[b];heap[b]=t;
    heap_at[heap[a]]=(int16_t)a;heap_at[heap[b]]=(int16_t)b;
}
static void heap_up(unsigned at)
{
    while(at && before(heap[at],heap[(at-1)/2])) {heap_swap(at,(at-1)/2);at=(at-1)/2;}
}
int nm_route(const nm_map_t *map,nm_pose_t from,const uint8_t targets[NM_CELLS],
             int unknown,nm_route_t *out)
{
    out->length=0;out->cost=UINT32_MAX;
    if(from.x>=map->side || from.y>=map->side || from.heading>3)return -1;
    for(int i=0;i<STATES;++i) {distance_to[i]=UINT32_MAX;predecessor[i]=-1;heap_at[i]=-1;}
    int start=(from.y*NM_SIDE+from.x)*4+from.heading,end=-1;
    distance_to[start]=0;heap[0]=(int16_t)start;heap_at[start]=0;heap_count=1;
    while(heap_count) {
        int s=heap[0];heap_at[s]=-2;
        if(--heap_count) {
            heap[0]=heap[heap_count];heap_at[heap[0]]=0;unsigned at=0;
            for(;;) {
                unsigned child=at*2+1;if(child>=heap_count)break;
                if(child+1<heap_count && before(heap[child+1],heap[child]))++child;
                if(!before(heap[child],heap[at]))break;
                heap_swap(at,child);at=child;
            }
        }
        if(targets[s/4]) {end=s;break;}
        for(unsigned d=0;d<4;++d) {
            int c=s/4,n=nm_next(map,c,d);
            if(n<0 || (map->cell[c].walls&(1u<<d)) || (!unknown && !open_edge(map,c,d)))continue;
            unsigned turn=(d+4u-s%4)%4u;if(turn==3)turn=1;
            uint32_t cost=distance_to[s]+1024u+turn;int t=n*4+d;
            if(cost<distance_to[t]) {
                distance_to[t]=cost;predecessor[t]=(int16_t)s;
                if(heap_at[t]==-1) {heap_at[t]=(int16_t)heap_count;heap[heap_count++]=(int16_t)t;}
                if(heap_at[t]>=0)heap_up((unsigned)heap_at[t]);
            }
        }
    }
    if(end<0)return -1;
    out->cost=distance_to[end];
    for(int s=end;s!=start;s=predecessor[s]) {
        if(s<0 || out->length>=NM_CELLS)return -1;
        out->direction[out->length++]=s%4;
    }
    for(unsigned i=0;i<out->length/2;++i) {
        uint8_t t=out->direction[i];out->direction[i]=out->direction[out->length-1-i];out->direction[out->length-1-i]=t;
    }
    return 0;
}
int nm_frontier(const nm_map_t *map, nm_pose_t from, nm_route_t *route)
{
    uint8_t targets[NM_CELLS];
    for(int c=0;c<NM_CELLS;++c) targets[c]=c%NM_SIDE<map->side && c/NM_SIDE<map->side && (!map->cell[c].visited || map->cell[c].known!=15);
    return nm_route(map, from, targets, 0, route);
}
int nm_refine(const nm_map_t *map, nm_pose_t from, nm_pose_t start,
              const uint8_t goals[NM_CELLS], nm_route_t *route)
{
    nm_route_t known, optimistic;
    if(map->axes!=3) {
        /* In a disconnected practice maze the absolute border may remain
         * ambiguous forever. A completely surveyed reachable component still
         * proves its shortest route without inventing an origin corner. */
        if(!nm_frontier(map,from,route))return 0;
        return nm_route(map,start,goals,0,&known)?-1:1;
    }
    if (nm_route(map, start, goals, 1, &optimistic)) return -1;
    if (!nm_route(map,start,goals,0,&known) && known.cost == optimistic.cost) return 1;
    int c = start.y * NM_SIDE + start.x;
    for (unsigned i = 0; i < optimistic.length; ++i) {
        unsigned d = optimistic.direction[i];
        if (!open_edge(map,c,d)) {
            uint8_t target[NM_CELLS] = {0}; target[c] = 1;
            return nm_route(map,from,target,0,route);
        }
        c = nm_next(map,c,d);
    }
    return -1;
}
int nm_valid(const nm_map_t *map)
{
    if((map->side!=6 && map->side!=9 && map->side!=16) || map->axes>3 ||
       map->start_x>=map->side || map->start_y>=map->side || map->start_heading>3)return 0;
    for (int c = 0; c < NM_CELLS; ++c) for (unsigned d = 0; d < 4; ++d) {
        if(c%NM_SIDE>=map->side || c/NM_SIDE>=map->side)continue;
        const nm_cell_t *p = &map->cell[c];
        if (p->known > 15 || (p->walls & ~p->known) || p->visited > 1) return 0;
        unsigned b = 1u << d, ob = 1u << ((d+2)%4);
        int n = nm_next(map,c,d);
        if(n<0) {if((map->axes&(d%2?1:2)) && !(p->known&p->walls&b))return 0;if((p->known&b) && !(p->walls&b))return 0;}
        else if (!!(p->known & b) != !!(map->cell[n].known & ob) ||
                 !!(p->walls & b) != !!(map->cell[n].walls & ob)) return 0;
    }
    return 1;
}

/* A boundary wall is not distinguishable from an internal wall in one scan.
 * Keep relative coordinates until explored extents and the known board size
 * constrain a unique rectangle. Never guess a start corner from a dead-end. */
static int shift(nm_map_t *m,nm_pose_t *pose,int dx,int dy)
{
    nm_map_t copy=*m;
    if((int)m->start_x+dx<0 || (int)m->start_x+dx>=m->side ||
       (int)m->start_y+dy<0 || (int)m->start_y+dy>=m->side)return -1;
    memset(copy.cell,0,sizeof copy.cell);
    for(int y=0;y<m->side;++y)for(int x=0;x<m->side;++x) {
        nm_cell_t c=m->cell[y*NM_SIDE+x];int xx=x+dx,yy=y+dy;
        if(xx<0 || yy<0 || xx>=m->side || yy>=m->side) {
            if(c.visited || (c.known&~c.walls))return -1;
        } else copy.cell[yy*NM_SIDE+xx]=c;
    }
    copy.start_x+=dx;copy.start_y+=dy;
    /* Restore reciprocal observations in newly available border cells. */
    for(int c=0;c<NM_CELLS;++c)for(unsigned d=0;d<4;++d) {
        int n=nm_next(&copy,c,d);unsigned b=1u<<d,ob=1u<<((d+2)%4);
        if(n>=0 && (copy.cell[c].known&b)) {
            copy.cell[n].known|=ob;
            if(copy.cell[c].walls&b)copy.cell[n].walls|=ob;
            else copy.cell[n].walls&=~ob;
        }
    }
    pose->x+=dx;pose->y+=dy;*m=copy;return 0;
}
static int anchor(nm_map_t *m,nm_pose_t *pose)
{
    int minx=m->start_x,maxx=minx,miny=m->start_y,maxy=miny;
    for(int c=0;c<NM_CELLS;++c)if(m->cell[c].visited || (m->cell[c].known&~m->cell[c].walls)) {
        int x=c%NM_SIDE,y=c/NM_SIDE;
        if(x<minx)minx=x;
        if(x>maxx)maxx=x;
        if(y<miny)miny=y;
        if(y>maxy)maxy=y;
    }
    int count=0,firstx=0,firsty=0,samex=1,samey=1;
    for(int x=maxx-m->side+1;x<=minx;++x)for(int y=maxy-m->side+1;y<=miny;++y) {
        if((m->axes&1) && x)continue;
        if((m->axes&2) && y)continue;
        if(m->start_x!=x && m->start_x!=x+m->side-1 &&
           m->start_y!=y && m->start_y!=y+m->side-1)continue;
        if(!count) {firstx=x;firsty=y;}
        else {samex&=x==firstx;samey&=y==firsty;}
        ++count;
    }
    if(!count)return -1;
    int dx=samex?-firstx:0,dy=samey?-firsty:0;
    if((dx || dy) && shift(m,pose,dx,dy))return -1;
    if(samex)m->axes|=1;
    if(samey)m->axes|=2;
    for(int c=0;c<NM_CELLS;++c)if(c%NM_SIDE<m->side && c/NM_SIDE<m->side)
        for(unsigned d=0;d<4;++d)if((m->axes&(d%2?1:2)) && nm_next(m,c,d)<0)
            if(nm_edge(m,c,d,1))return -1;
    return 0;
}
int nm_observe_auto(nm_map_t *m,nm_pose_t *pose,uint8_t walls)
{
    nm_map_t copy=*m;nm_pose_t p=*pose;
    unsigned dirs[3]={p.heading,(p.heading+3)%4,(p.heading+1)%4};
    for(unsigned i=0;i<3;++i)if(!(walls&(1u<<i)) && nm_next(&copy,p.y*NM_SIDE+p.x,dirs[i])<0) {
        unsigned d=dirs[i];if(copy.axes&(d%2?1:2))return -1;
        int dx=d==1?-1:d==3?1:0,dy=d==0?-1:d==2?1:0;
        if(shift(&copy,&p,dx,dy))return -1;
    }
    if(nm_observe(&copy,p,walls) || anchor(&copy,&p))return -1;
    *m=copy;*pose=p;return 0;
}
