#include "fw_menu.h"
#include "oled/ssd1306.h"
#include <stdio.h>
#include <string.h>
#include "fw_battery.h"
void fw_ui_header(const char *title)
{
    ssd1306ClearRect(0,0,128,10);
    char clipped[25];snprintf(clipped,sizeof clipped,"%.24s",title);
    ssd1306DrawString(1,0,clipped,strlen(title)>15?&Font_3x6:&Font_5x8);
    ssd1306DrawLine(0,10,127,10);
}
static void hint(unsigned x,unsigned width,const char *text)
{
    ssd1306ClearRect(x,54,width,10);
    const FONT_DEF *font=strlen(text)*6+6>width?&Font_3x6:&Font_5x8;
    unsigned text_width=strlen(text)*(font->u8Width+1u);
    ssd1306DrawString(x+(width>text_width?(width-text_width)/2:0),55,text,font);
    for(unsigned y=54;y<64;++y)for(unsigned xx=x;xx<x+width;++xx)ssd1306InvertPixel(xx,y);
}
void fw_ui_hint(const char *text) {hint(0,123,text);}
static void scroll(unsigned index,unsigned count)
{
    if(count<2)return;
    unsigned height=34/count;if(height<5)height=5;
    if(index>=count)index=count-1;
    unsigned y=16+index*(34-height)/(count-1);
    ssd1306DrawLine(126,16,126,49);ssd1306FillRect(125,y,3,height);
}
void fw_ui_battery(void)
{
    fw_battery_poll();fw_battery_status_t b=fw_battery_status();
    char value[8];
    ssd1306ClearRect(104,0,24,10);
    if(b.sample_valid && b.soc_valid)snprintf(value,sizeof value,"%u%%",b.percent);
    else snprintf(value,sizeof value,"--%%");
    ssd1306DrawString(128-strlen(value)*6,1,value,&Font_5x8);
}
void fw_ui_menu_refresh(void) {fw_ui_battery();ssd1306Refresh();}
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
static void small(unsigned x,unsigned y,const char *s) {
    if(!y) {char title[20];snprintf(title,sizeof title,"%.19s",s);ssd1306DrawString(x,y,title,&Font_5x8);}
    else ssd1306DrawString(x,y,s,&Font_5x8);
}
static void icon(unsigned type)
{
    unsigned x=2,y=21;
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
    } else if(type==FW_ICON_WALL) {
        ssd1306FillRect(x,y,27,2);ssd1306FillRect(x,y,2,27);ssd1306FillRect(x+25,y,2,27);
        ssd1306DrawRect(x+8,y+15,11,10);ssd1306DrawLine(x+13,y+14,x+13,y+5);
        ssd1306DrawLine(x+9,y+9,x+13,y+5);ssd1306DrawLine(x+13,y+5,x+17,y+9);
    } else if(type==FW_ICON_TURN) {
        ssd1306DrawCircle(x+13,y+13,11);ssd1306DrawRect(x+9,y+9,8,8);
        ssd1306ClearRect(x+16,y,11,13);
        ssd1306DrawLine(x+13,y+2,x+23,y+7);ssd1306DrawLine(x+23,y+7,x+22,y);
        ssd1306DrawLine(x+23,y+7,x+16,y+8);
    } else if(type==FW_ICON_EDGE_LEFT || type==FW_ICON_EDGE_RIGHT) {
        unsigned wall=type==FW_ICON_EDGE_LEFT?x:x+24;
        ssd1306FillRect(x,y,27,2);ssd1306FillRect(wall,y,3,14);
        ssd1306FillRect(wall-1,y+12,5,5);ssd1306DrawDashedLine(wall+1,y+19,wall+1,y+27);
        ssd1306DrawRect(x+9,y+10,9,10);ssd1306DrawLine(x+13,y+21,x+13,y+27);
        ssd1306DrawLine(x+10,y+24,x+13,y+27);ssd1306DrawLine(x+13,y+27,x+16,y+24);
    } else if(type==FW_ICON_TEST) {
        ssd1306DrawRect(x+3,y+3,20,20);
        for(unsigned i=0;i<4;++i) {ssd1306DrawLine(x,y+5+i*5,x+3,y+5+i*5);ssd1306DrawLine(x+23,y+5+i*5,x+26,y+5+i*5);}
        ssd1306DrawLine(x+8,y+13,x+12,y+17);ssd1306DrawLine(x+12,y+17,x+19,y+9);
    } else {
        ssd1306DrawRect(x+3,y,20,26);
        for(unsigned i=0;i<3;++i) ssd1306DrawLine(x+7,y+6+i*6,x+20,y+6+i*6);
    }
}
void fw_ui_card(const char *title,const char *first,const char *second,unsigned type,unsigned index,unsigned count)
{
    (void)title; /* Category names are navigation state, not screen content. */
    ssd1306ClearScreen();
    icon(type);scroll(index,count);
    int instruction=!strncmp(second,"OK:",3) || !strncmp(second,"LEFT:",5);
    if(instruction) {large(33,24,first);fw_ui_hint(second);}
    else {large(33,*second?15:25,first);if(*second)large(33,36,second);}
    fw_ui_menu_refresh();
}
void fw_ui_library(const fw_saved_maze_t *m,unsigned index,unsigned count,int blink)
{
    char s[24];ssd1306ClearScreen();scroll(index,count);
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
    hint(58,65,"OK: LOAD");fw_ui_menu_refresh();
}
void fw_ui_setting(unsigned index,int value)
{
    static const char *const names[]={"Axle to nose","Robot width","Cell interior","Cell pitch","Post center",
        "Explore speed","Fast run speed","Start corner","Start heading"};
    char s[24];ssd1306ClearScreen();fw_ui_header(names[index]);
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
    fw_ui_hint("UP/DN: EDIT  OK: SAVE");fw_ui_menu_refresh();
}
