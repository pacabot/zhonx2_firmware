/* ZHONX II traces in the ZHONX III frame: origin at the start cell, NORTH =
 * heading at the start, x east, y south, wall bits N1 E2 S4 W8. */
#include "fw_ariane.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static const int dx[]={0,1,0,-1},dy[]={1,0,-1,0};   /* nimes.c: north = +y */
int main(void)
{
    nm_map_t m;
    for(unsigned corner=0;corner<4;++corner)for(unsigned start=0;start<4;++start) {
        memset(&m,0,sizeof m);m.side=9;
        m.start_x=(uint8_t)(corner==1 || corner==2?8:0);m.start_y=(uint8_t)(corner>=2?8:0);m.start_heading=(uint8_t)start;
        int8_t x,y;uint8_t h;
        fw_ariane_frame(&m,m.start_x,m.start_y,start,&x,&y,&h);
        assert(x==0 && y==0 && h==0);
        /* One cell ahead of the start: ZHONX III's MC_NORTH, y - 1. */
        fw_ariane_frame(&m,m.start_x+dx[start],m.start_y+dy[start],start,&x,&y,&h);
        assert(x==0 && y==-1 && h==0);
        /* One cell to the right of the start: east. */
        unsigned right=(start+1)&3;
        fw_ariane_frame(&m,m.start_x+dx[right],m.start_y+dy[right],right,&x,&y,&h);
        assert(x==1 && y==0 && h==1);
        /* Behind and to the left. */
        fw_ariane_frame(&m,m.start_x-dx[start]-dx[right],m.start_y-dy[start]-dy[right],(start+2)&3,&x,&y,&h);
        assert(x==-1 && y==1 && h==2);
        /* Walls seen facing the start heading: front N, left W, right E. */
        assert(fw_ariane_walls(&m,start,1)==1 && fw_ariane_walls(&m,start,2)==8 && fw_ariane_walls(&m,start,4)==2);
        /* After a right turn: front E, left N, right S. */
        assert(fw_ariane_walls(&m,right,1|2|4)==(2|1|4));
        assert(fw_ariane_walls(&m,start,0)==0);
    }
    puts("ariane: ZHONX III frame from 4 corners x 4 start headings, cells and seen walls");
}
