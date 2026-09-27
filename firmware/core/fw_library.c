#include "fw_library.h"
#include <string.h>
nm_pose_t fw_maze_origin(unsigned corner,unsigned heading)
{ return (nm_pose_t){corner==1 || corner==2?8:0,corner>=2?8:0,(uint8_t)heading}; }
int fw_maze_certify(const nm_map_t *m,unsigned corner,unsigned heading,nm_route_t *route)
{
    uint8_t goals[NM_CELLS]; nm_route_t optimistic={0};
    if(corner>3 || heading>3 || !nm_valid(m) || nm_goal(m,goals)!=1) return 0;
    nm_pose_t start=nm_origin(m);
    memset(route,0,sizeof *route);
    if(nm_route(m,start,goals,0,route) || !route->length)return 0;
    if(m->axes!=3)return nm_frontier(m,start,&optimistic)<0;
    return !nm_route(m,start,goals,1,&optimistic) && route->cost==optimistic.cost;
}
int fw_library_valid(const fw_library_t *l)
{
    if(l->count>FW_MAZE_SLOTS) return 0;
    for(unsigned i=0;i<l->count;++i) {
        const fw_saved_maze_t *m=&l->item[i]; nm_route_t r;
        if(!m->id || m->search_ms>(m->map.side==16?600000u:NM_SEARCH_MS) || !fw_maze_certify(&m->map,m->corner,m->heading,&r) ||
            r.length!=m->route.length || r.cost!=m->route.cost ||
            memcmp(r.direction,m->route.direction,r.length)) return 0;
        for(unsigned j=0;j<i;++j) if(l->item[j].id==m->id) return 0;
    }
    return 1;
}
int fw_library_put(fw_library_t *l,const nm_map_t *map,unsigned corner,unsigned heading,uint32_t ms)
{
    nm_route_t route;
    if(!fw_maze_certify(map,corner,heading,&route) || ms>(map->side==16?600000u:NM_SEARCH_MS)) return -1;
    unsigned at=0;
    for(;at<l->count;++at) if(l->item[at].corner==corner && l->item[at].heading==heading &&
        !memcmp(&l->item[at].map,map,sizeof *map)) break;
    if(at==FW_MAZE_SLOTS) return -1;
    if(at==l->count) {
        ++l->count; if(!++l->next_id) ++l->next_id;
        l->item[at].id=l->next_id;
    }
    else if(at+1<l->count) {
        fw_saved_maze_t recent=l->item[at];
        memmove(&l->item[at],&l->item[at+1],(l->count-at-1)*sizeof recent);
        at=l->count-1;l->item[at]=recent;
    }
    l->item[at].map=*map; l->item[at].route=route;
    l->item[at].corner=corner; l->item[at].heading=heading; l->item[at].search_ms=ms;
    return (int)at;
}
int fw_library_remove(fw_library_t *l,unsigned index)
{
    if(index>=l->count) return -1;
    memmove(&l->item[index],&l->item[index+1],(l->count-index-1)*sizeof l->item[0]);
    memset(&l->item[--l->count],0,sizeof l->item[0]); return 0;
}

static void name_letter(char out[6],unsigned *length,char letter)
{
    if(*length>=5 || strchr(out,letter))return;
    out[(*length)++]=letter;out[*length]=0;
}
void fw_maze_name(const fw_saved_maze_t *m,char out[6])
{
    unsigned length=0,at=0,heading=m->map.start_heading;
    int cell=m->map.start_y*NM_SIDE+m->map.start_x;out[0]=0;
    while(at<m->route.length && length<5) {
        unsigned exits=0;
        for(unsigned d=0;d<4;++d)if((m->map.cell[cell].known&(1u<<d)) && !(m->map.cell[cell].walls&(1u<<d)))++exits;
        if(exits>=3)name_letter(out,&length,'X');
        unsigned d=m->route.direction[at],turn=(d+4-heading)%4;
        char letter=turn==2?'O':turn?'N':((d+4-m->map.start_heading)%2?'H':'I');
        /* Two immediately consecutive corners: S bend or U bend. */
        unsigned consume=1;
        if((turn==1 || turn==3) && at+1<m->route.length) {
            unsigned next=(m->route.direction[at+1]+4-d)%4;
            if(next==1 || next==3) {letter=next==turn?'O':'Z';consume=2;}
        }
        name_letter(out,&length,letter);
        while(consume-- && at<m->route.length) {
            heading=m->route.direction[at++];int next=nm_next(&m->map,cell,heading);
            if(next<0)return;
            cell=next;
        }
    }
    if(!length)strcpy(out,"ZHONX");
}
