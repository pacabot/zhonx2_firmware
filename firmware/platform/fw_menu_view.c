#include "fw_menu.h"
#include "oled/ssd1306.h"
#include <stdio.h>
/* Double-height 7x8 font: 7x16 glyphs, no hidden clipping or bitmap assets. */
static void large(unsigned x,unsigned y,const char *s)
{
    for(;*s && x+7<=128 && y+16<=64;++s,x+=8) {
        unsigned c=(unsigned char)*s;
        if(c<32 || c>127) continue;
        for(unsigned col=0;col<7;++col) {
            unsigned bits=Font_7x8.au8FontTable[(c-32)*7+col];
            for(unsigned row=0;row<8;++row) if(bits&(1u<<row)) ssd1306FillRect(x+col,y+2*row,1,2);
        }
    }
}
static void small(unsigned x,unsigned y,const char *s) { ssd1306DrawString(x,y,s,&Font_5x8); }
static void icon(unsigned type)
{
    unsigned x=4,y=21;
    if(type==FW_ICON_MAZE) {
        ssd1306DrawRect(x,y,26,26); ssd1306DrawLine(x+8,y,x+8,y+17);
        ssd1306DrawLine(x+8,y+9,x+20,y+9); ssd1306DrawLine(x+17,y+9,x+17,y+26);
        ssd1306FillRect(x+3,y+19,3,3);
    } else if(type==FW_ICON_RUN) {
        ssd1306DrawLine(x,y+2,x+24,y+13); ssd1306DrawLine(x+24,y+13,x,y+25);
        ssd1306DrawLine(x,y+2,x,y+25);
    } else if(type==FW_ICON_CALIBRATE) {
        ssd1306DrawCircle(x+13,y+13,11); ssd1306DrawCircle(x+13,y+13,5);
        ssd1306DrawLine(x,y+13,x+26,y+13);ssd1306DrawLine(x+13,y,x+13,y+26);
    } else if(type==FW_ICON_SETTINGS) {
        for(unsigned i=0;i<3;++i) {
            ssd1306DrawLine(x+3+i*9,y,x+3+i*9,y+26);
            ssd1306FillRect(x+i*9,y+4+i*5,7,5);
        }
    } else if(type==FW_ICON_UPDATE) {
        ssd1306DrawRect(x,y+17,26,9);ssd1306DrawLine(x+13,y,x+13,y+20);
        ssd1306DrawLine(x+6,y+11,x+13,y+18);ssd1306DrawLine(x+13,y+18,x+20,y+11);
    } else {
        ssd1306DrawRect(x+3,y,20,26);
        for(unsigned i=0;i<3;++i) ssd1306DrawLine(x+7,y+6+i*6,x+20,y+6+i*6);
    }
}
void fw_ui_card(const char *title,const char *first,const char *second,unsigned type,unsigned index,unsigned count)
{
    char page[12];ssd1306ClearScreen();small(0,0,title);
    snprintf(page,sizeof page,"%u/%u",index+1,count);small(98,0,page);
    icon(type);large(37,16,first);large(37,34,second);
    small(0,55,"UP/DN  OK  LEFT:BACK");ssd1306Refresh();
}
void fw_ui_library(const fw_saved_maze_t *m,unsigned index,unsigned count,int blink)
{
    char s[24];ssd1306ClearScreen();snprintf(s,sizeof s,"MAZES %u/%u",index+1,count);small(0,0,s);
    for(unsigned c=0;c<NM_CELLS;++c) {
        unsigned x=(c%9)*6,y=9+(8-c/9)*6;
        for(unsigned d=0;d<4;++d) {
            unsigned x0=x,y0=y,x1=x,y1=y;
            if(d==0) x1+=6;
            if(d==1) {x0+=6;x1+=6;y1+=6;}
            if(d==2) {y0+=6;y1+=6;x1+=6;}
            if(d==3) y1+=6;
            if(m->map.cell[c].walls&(1u<<d)) ssd1306DrawLine(x0,y0,x1,y1);
            else if(!(m->map.cell[c].known&(1u<<d))) ssd1306DrawDashedLine(x0,y0,x1,y1);
        }
    }
    nm_pose_t start=fw_maze_origin(m->corner,m->heading); int c=start.y*9+start.x;
    ssd1306FillRect(start.x*6+1,9+(8-start.y)*6+1,3,3);
    if(blink) for(unsigned i=0;i<m->route.length;++i) {
        int next=nm_neighbour(c,m->route.direction[i]);if(next<0) break;
        unsigned x=c%9*6+3,y=9+(8-c/9)*6+3,xx=next%9*6+3,yy=9+(8-next/9)*6+3;
        ssd1306DrawLine(x,y,xx,yy);ssd1306DrawLine(x+1,y,xx+1,yy);c=next;
    }
    large(58,13,"MAZE");snprintf(s,sizeof s,"%lu",(unsigned long)m->id);large(58,31,s);
    ssd1306DrawString(58,49,"UP/DN   OK:LOAD",&Font_3x6);
    ssd1306DrawString(58,57,"LEFT:BACK",&Font_3x6);ssd1306Refresh();
}
void fw_ui_setting(unsigned index,int value)
{
    static const char *const names[]={"Axle to nose","Robot width","Cell interior","Cell pitch","Post center",
        "Explore speed","Fast run speed","Start corner","Start heading"};
    char s[24];ssd1306ClearScreen();small(0,0,names[index]);
    ssd1306DrawRect(5,14,40,39);ssd1306DrawRect(17,25,16,16);
    ssd1306FillRect(15,31,2,6);ssd1306FillRect(33,31,2,6);
    if(index==0) {ssd1306DrawLine(25,33,25,25);ssd1306DrawLine(22,33,28,33);}
    else if(index==1) {ssd1306DrawLine(15,45,35,45);ssd1306DrawLine(15,42,15,48);ssd1306DrawLine(35,42,35,48);}
    else if(index==4) {ssd1306FillRect(3,49,5,5);ssd1306DrawLine(12,16,12,51);ssd1306DrawLine(9,16,15,16);ssd1306DrawLine(9,51,15,51);}
    else if(index<4) {ssd1306DrawLine(index==2?7:5,21,index==2?43:45,21);ssd1306DrawLine(7,18,7,24);ssd1306DrawLine(43,18,43,24);}
    else if(index<7) {ssd1306DrawLine(25,40,25,18);ssd1306DrawLine(20,23,25,18);ssd1306DrawLine(25,18,30,23);}
    else if(index==7) {static const unsigned xx[]={6,39,39,6},yy[]={47,47,15,15};ssd1306FillRect(xx[value],yy[value],5,5);}
    else {static const int dx[]={0,10,0,-10},dy[]={-10,0,10,0};ssd1306DrawLine(25,33,25+dx[value],33+dy[value]);}
    if(index<2) snprintf(s,sizeof s,"%d.%d",value/10,value%10);
    else if(index==7) snprintf(s,sizeof s,"%s",(const char*[]){"SW","SE","NE","NW"}[value]);
    else if(index==8) snprintf(s,sizeof s,"%s",(const char*[]){"NORTH","EAST","SOUTH","WEST"}[value]);
    else snprintf(s,sizeof s,"%d",value);
    large(59,20,s);small(59,40,index<5?"MM":index<7?"MM/S":"START");
    small(0,55,"UP/DN OK LEFT:CANCEL");ssd1306Refresh();
}
