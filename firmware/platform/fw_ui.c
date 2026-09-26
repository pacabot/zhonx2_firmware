#include "fw_ui.h"
#include "oled/ssd1306.h"
#include <stdio.h>
/* 54x54 map below a 9-pixel title. Status occupies the right half. */
static void wall(unsigned x, unsigned y, unsigned direction, int known, int present)
{
    unsigned x1=x,y1=y;
    if (direction==NM_NORTH) x1+=6;
    if (direction==NM_EAST) { x+=6; x1=x; y1+=6; }
    if (direction==NM_SOUTH) { y+=6; y1=y; x1+=6; }
    if (direction==NM_WEST) y1+=6;
    if (present) ssd1306DrawLine(x,y,x1,y1);
    else if (!known) ssd1306DrawDashedLine(x,y,x1,y1);
}
void fw_ui_maze(const nm_map_t *m,nm_pose_t p,const char *status,
                unsigned speed,uint32_t ms,uint8_t sensors)
{
    char text[24]; uint8_t goals[NM_CELLS]; nm_goal(m,goals);
    ssd1306ClearScreen();
    ssd1306DrawString(0,0,status,&Font_5x8);
    for (unsigned y=0;y<NM_SIDE;++y) for (unsigned x=0;x<NM_SIDE;++x) {
        const nm_cell_t *c=&m->cell[y*NM_SIDE+x];
        unsigned px=x*6,py=9+(8-y)*6;
        for (unsigned d=0;d<4;++d) {
            if ((d==NM_EAST && x!=8) || (d==NM_SOUTH && y!=0)) continue;
            wall(px,py,d,c->known & 1u<<d,c->walls & 1u<<d);
        }
        if (c->visited) ssd1306DrawPixel(px+3,py+3);
        if (goals[y*NM_SIDE+x]) ssd1306DrawRect(px+2,py+2,3,3);
    }
    if (p.x<NM_SIDE && p.y<NM_SIDE && p.heading<4) {
        int x=p.x*6+3,y=9+(8-p.y)*6+3;
        const int dx[]={0,2,0,-2},dy[]={-2,0,2,0};
        ssd1306FillRect(x-1,y-1,3,3);
        ssd1306DrawLine(x,y,x+dx[p.heading],y+dy[p.heading]);
    }
    snprintf(text,sizeof text,"X%u Y%u %c",p.x,p.y,"NESW"[p.heading%4]);
    ssd1306DrawString(60,11,text,&Font_3x6);
    snprintf(text,sizeof text,"%u MM/S",speed); ssd1306DrawString(60,20,text,&Font_3x6);
    snprintf(text,sizeof text,"%lu:%02lu",(unsigned long)(ms/60000),(unsigned long)(ms/1000%60));
    ssd1306DrawString(60,29,text,&Font_3x6);
    snprintf(text,sizeof text,"WALL %c%c%c",(sensors&0x20)?'-':'L',(sensors&0x08)?'-':'F',(sensors&0x01)?'-':'R');
    ssd1306DrawString(60,38,text,&Font_3x6);
    snprintf(text,sizeof text,"IR %02X",sensors); ssd1306DrawString(60,47,text,&Font_3x6);
    ssd1306DrawString(60,56,"BACK:STOP",&Font_3x6);
    ssd1306Refresh();
}
