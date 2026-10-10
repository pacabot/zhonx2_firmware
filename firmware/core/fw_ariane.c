#include "fw_ariane.h"
void fw_ariane_frame(const nm_map_t *m,int x,int y,unsigned heading,int8_t *ox,int8_t *oy,uint8_t *oh)
{
    /* Counterclockwise quarter turns bring the start heading to NORTH. */
    int dx=x-m->start_x,dy=y-m->start_y;unsigned r=m->start_heading&3;
    for(unsigned i=0;i<r;++i) {int t=dx;dx=-dy;dy=t;}
    *ox=(int8_t)dx;*oy=(int8_t)-dy;*oh=(uint8_t)((heading+4-r)&3);
}
uint8_t fw_ariane_walls(const nm_map_t *m,unsigned heading,uint8_t seen)
{
    unsigned h=heading&3,r=m->start_heading&3;
    unsigned walls=((seen&1)?1u<<h:0)|((seen&2)?1u<<((h+3)&3):0)|((seen&4)?1u<<((h+1)&3):0);
    return (uint8_t)(((walls|walls<<4)>>r)&15);
}
