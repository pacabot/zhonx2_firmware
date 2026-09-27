#include "fw_ui.h"
#include "fw_display.h"
#include "fw_text.h"
#include <string.h>
#include "oled/ssd1306.h"
#include <stdio.h>
/* Full 54x54 map; readable status occupies the right half. */
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
    /* Keep the full map; readable status at right, diagnostics alternate.
     * Start instructions and stop reasons wrap instead of shrinking. */
    if(!strcmp(status,"EXPLORATION"))status="Explore";
    if(!strcmp(status,"HAND IN FRONT OF F10"))status="Hand on F10";
    if(!strcmp(status,"REMOVE YOUR HAND"))status="Remove hand";
    char row[9];
    const char *rest=fw_text_line(status,row,8);fw_display_text(60,0,row);
    rest=fw_text_line(rest,row,8);fw_display_text(60,13,row);
    if(*rest) {fw_text_line(rest,row,8);fw_display_text(60,26,row);}
    else {
        if((ms/2000)%2)snprintf(text,sizeof text,"%u mm/s",speed);
        else snprintf(text,sizeof text,"X%u Y%u %c",p.x,p.y,"NESW"[p.heading%4]);
        fw_display_text(60,26,text);
    }
    snprintf(text,sizeof text,"%lu:%02lu",(unsigned long)(ms/60000),(unsigned long)(ms/1000%60));
    fw_display_text(60,39,text);
    snprintf(text,sizeof text,"IR %c%c%c",(sensors&0x20)?'-':'L',(sensors&0x08)?'-':'F',(sensors&0x01)?'-':'R');
    fw_display_text(60,52,text);ssd1306Refresh();
}
