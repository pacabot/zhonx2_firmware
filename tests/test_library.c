#include "fw_library.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void edge(nm_map_t *m,int c,unsigned d)
{
    int next=nm_neighbour(c,d);assert(next>=0);
    m->cell[c].walls &= ~(1u<<d);m->cell[next].walls &= ~(1u<<((d+2)%4));
}
int main(void)
{
    fw_library_t library={0};nm_map_t m;nm_init_size(&m,9,0);nm_route_t r;
    assert(!fw_maze_certify(&m,0,0,&r) && fw_library_put(&library,&m,0,0,1000)<0);
    for(unsigned c=0;c<NM_CELLS;++c)m.cell[c]=(nm_cell_t){15,15,0,0};
    edge(&m,0,0);edge(&m,16,0);edge(&m,32,1);edge(&m,33,1);
    edge(&m,34,1);edge(&m,34,0);edge(&m,35,0);edge(&m,50,1);
    assert(fw_maze_certify(&m,0,0,&r) && r.length==4);
    for(unsigned i=0;i<8;++i) {
        for(unsigned b=0;b<4;++b)m.cell[50+b].visited=(i>>b)&1;
        assert(fw_library_put(&library,&m,0,0,1000)==(int)i);
    }
    assert(fw_library_valid(&library) && library.count==8);
    assert(fw_library_put(&library,&m,0,0,1000)==7 && library.count==8);
    fw_saved_maze_t repeated=library.item[2];
    assert(fw_library_put(&library,&repeated.map,repeated.corner,repeated.heading,2000)==7);
    assert(library.item[7].id==repeated.id && library.item[7].search_ms==2000 && library.count==8);
    fw_library_t before=library;m.cell[53].visited=1;
    assert(fw_library_put(&library,&m,0,0,1000)<0 && !memcmp(&before,&library,sizeof library));
    assert(!fw_library_remove(&library,3) && library.count==7 && fw_library_valid(&library));
    assert(library.item[3].id==before.item[4].id);
    library.item[0].route.direction[0]=2;assert(!fw_library_valid(&library));
    fw_saved_maze_t shape={0};nm_init_size(&shape.map,9,0);shape.map.start_x=shape.map.start_y=3;
    for(unsigned c=0;c<NM_CELLS;++c)shape.map.cell[c]=(nm_cell_t){15,15,0,0};
    char name[6];
    shape.route=(nm_route_t){.direction={0,0,1,1},.length=4};fw_maze_name(&shape,name);assert(!strcmp(name,"INH"));
    shape.route=(nm_route_t){.direction={0,1,0,0},.length=4};fw_maze_name(&shape,name);assert(!strcmp(name,"IZ"));
    shape.route=(nm_route_t){.direction={0,1,2,2},.length=4};fw_maze_name(&shape,name);assert(!strcmp(name,"IO"));
    shape.route=(nm_route_t){.direction={0,2},.length=2};fw_maze_name(&shape,name);assert(!strcmp(name,"IO"));
    shape.map.cell[3*NM_SIDE+3]=(nm_cell_t){15,0,0,0};
    shape.route=(nm_route_t){.direction={0,0,1,1,2,2,3,3,0,0},.length=10};
    fw_maze_name(&shape,name);assert(!strcmp(name,"XINH"));
    puts("library: certified routes only, multiple maps, dedup, capacity, deletion, corrupt path rejection");
}
