#include "fw_menu.h"
#include "oled/ssd1306.h"
#include <stdio.h>
#include <string.h>
#include "fw_battery.h"
#include "fw_display.h"
#define large fw_display_large
void fw_ui_header(const char *title)
{
    char clipped[12];snprintf(clipped,sizeof clipped,"%.11s",title);
    ssd1306ClearRect(0,0,128,15);
    fw_display_text(0,0,clipped);
    ssd1306DrawLine(0,14,127,14);
}
static void hint(unsigned x,unsigned width,const char *text)
{
    char clipped[17];unsigned columns=width/8;
    snprintf(clipped,sizeof clipped,"%.*s",(int)columns,text);
    ssd1306ClearRect(x,52,width,12);
    fw_display_text(x+(width-strlen(clipped)*8)/2,52,clipped);
    for(unsigned y=52;y<64;++y)for(unsigned xx=x;xx<x+width;++xx)ssd1306InvertPixel(xx,y);
}
void fw_ui_hint(const char *text) {hint(0,123,text);}
#define scroll fw_display_scroll
void fw_ui_battery(void)
{
    fw_battery_poll();fw_battery_status_t b=fw_battery_status();
    char value[8];ssd1306ClearRect(96,0,32,13);
    if(b.sample_valid && b.soc_valid)snprintf(value,sizeof value,"%u%%%s",b.percent,b.lower_bound && b.percent<100?"+":"");
    else snprintf(value,sizeof value,"--%%");
    fw_display_text(128-strlen(value)*8,0,value);
}
void fw_ui_menu_refresh(void) {fw_ui_battery();ssd1306Refresh();}
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
    else {large(33,*second?16:25,first);if(*second)large(33,36,second);}
    fw_ui_menu_refresh();
}
void fw_ui_library(const fw_saved_maze_t *m,unsigned index,unsigned count,int blink)
{
    char s[24];ssd1306ClearScreen();scroll(index,count);
    unsigned side=nm_size(&m->map), scale=54/side;
    for(unsigned yy=0;yy<side;++yy)for(unsigned xx=0;xx<side;++xx) {
        unsigned c=yy*NM_SIDE+xx,x=xx*scale,y=9+(side-1-yy)*scale;
        for(unsigned d=0;d<4;++d) {
            unsigned x0=x,y0=y,x1=x,y1=y;
            if(d==0) x1+=scale;
            if(d==1) {x0+=scale;x1+=scale;y1+=scale;}
            if(d==2) {y0+=scale;y1+=scale;x1+=scale;}
            if(d==3) y1+=scale;
            if(m->map.cell[c].walls&(1u<<d)) ssd1306DrawLine(x0,y0,x1,y1);
            else if(!(m->map.cell[c].known&(1u<<d))) ssd1306DrawDashedLine(x0,y0,x1,y1);
        }
    }
    nm_pose_t start=nm_origin(&m->map); int c=start.y*NM_SIDE+start.x;
    ssd1306FillRect(start.x*scale+1,9+(side-1-start.y)*scale+1,scale-1,scale-1);
    if(blink) for(unsigned i=0;i<m->route.length;++i) {
        int next=nm_next(&m->map,c,m->route.direction[i]);if(next<0) break;
        unsigned x=c%NM_SIDE*scale+scale/2,y=9+(side-1-c/NM_SIDE)*scale+scale/2,xx=next%NM_SIDE*scale+scale/2,yy=9+(side-1-next/NM_SIDE)*scale+scale/2;
        ssd1306DrawLine(x,y,xx,yy);ssd1306DrawLine(x+1,y,xx+1,yy);c=next;
    }
    large(58,13,"MAZE");snprintf(s,sizeof s,"%lu",(unsigned long)m->id);large(58,31,s);
    hint(58,65,"LOAD");fw_ui_menu_refresh();
}
void fw_ui_setting(unsigned index,int value)
{
    static const char *const names[]={"Axle-nose","Width","Cell clear","Cell pitch","Post center",
        "Explore","Fast run","Maze size"};
    char s[24];ssd1306ClearScreen();fw_ui_header(names[index]);
    ssd1306DrawRect(5,14,40,39);ssd1306DrawRect(17,25,16,16);
    ssd1306FillRect(15,31,2,6);ssd1306FillRect(33,31,2,6);
    if(index==0) {ssd1306DrawLine(25,33,25,25);ssd1306DrawLine(22,33,28,33);}
    else if(index==1) {ssd1306DrawLine(15,45,35,45);ssd1306DrawLine(15,42,15,48);ssd1306DrawLine(35,42,35,48);}
    else if(index==4) {ssd1306FillRect(3,49,5,5);ssd1306DrawLine(12,16,12,51);ssd1306DrawLine(9,16,15,16);ssd1306DrawLine(9,51,15,51);}
    else if(index<4) {ssd1306DrawLine(index==2?7:5,21,index==2?43:45,21);ssd1306DrawLine(7,18,7,24);ssd1306DrawLine(43,18,43,24);}
    else if(index<7) {ssd1306DrawLine(25,40,25,18);ssd1306DrawLine(20,23,25,18);ssd1306DrawLine(25,18,30,23);}
    else {ssd1306ClearRect(4,14,43,39);
        for(unsigned j=0;j<=3;++j) {ssd1306DrawLine(5+j*12,16,5+j*12,52);ssd1306DrawLine(5,16+j*12,41,16+j*12);}}
    if(index<2) snprintf(s,sizeof s,"%d.%d",value/10,value%10);
    else if(index==7) snprintf(s,sizeof s,"%dx%d",value,value);
    else snprintf(s,sizeof s,"%d",value);
    large(index==7?49:59,20,s);fw_display_text(59,38,index<5?"MM":index<7?"MM/S":"CELLS");
    fw_ui_hint("UP/DN  OK:SAVE");fw_ui_menu_refresh();
}
