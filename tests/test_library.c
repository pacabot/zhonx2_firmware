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
    fw_library_t library={0};nm_map_t m;nm_init(&m);nm_route_t r;
    assert(!fw_maze_certify(&m,0,0,&r) && fw_library_put(&library,&m,0,0,1000)<0);
    for(unsigned c=0;c<NM_CELLS;++c)m.cell[c]=(nm_cell_t){15,15,0,0};
    edge(&m,0,0);edge(&m,9,0);edge(&m,18,1);edge(&m,19,1);
    edge(&m,20,1);edge(&m,20,0);edge(&m,21,0);edge(&m,29,1);
    assert(fw_maze_certify(&m,0,0,&r) && r.length==4);
    for(unsigned i=0;i<8;++i) {
        for(unsigned b=0;b<4;++b)m.cell[50+b].visited=(i>>b)&1;
        assert(fw_library_put(&library,&m,0,0,1000)==(int)i);
    }
    assert(fw_library_valid(&library) && library.count==8);
    assert(fw_library_put(&library,&m,0,0,1000)==7 && library.count==8);
    fw_library_t before=library;m.cell[53].visited=1;
    assert(fw_library_put(&library,&m,0,0,1000)<0 && !memcmp(&before,&library,sizeof library));
    assert(!fw_library_remove(&library,3) && library.count==7 && fw_library_valid(&library));
    assert(library.item[3].id==before.item[4].id);
    library.item[0].route.direction[0]=2;assert(!fw_library_valid(&library));
    puts("library: certified routes only, multiple maps, dedup, capacity, deletion, corrupt path rejection");
}
