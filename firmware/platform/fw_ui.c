#include "fw_ui.h"
#include "fw_display.h"
#include "fw_text.h"
#include "oled/ssd1306.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* All map coordinates remain signed until clipped to the viewport. */
static int zoom=16,pan_x,pan_y,progress;
static const nm_route_t *view_route;
static int route_visible;
void fw_ui_map_route(const nm_route_t *route,int visible) {view_route=route;route_visible=visible;}
static const int dx[]={0,1,0,-1},dy[]={1,0,-1,0};
void fw_ui_map_view(unsigned scale,int x,int y) {zoom=(int)scale;pan_x=x;pan_y=y;}
void fw_ui_map_progress(int permille) {progress=permille;}
static void pixel(int x,int y) {if(x>=4 && x<124 && y>=4 && y<60)ssd1306DrawPixel(x,y);}
static void line(int x,int y,int xx,int yy,int dashed)
{
    int ax=abs(xx-x),sx=x<xx?1:-1,ay=-abs(yy-y),sy=y<yy?1:-1,err=ax+ay,n=0;
    for(;;) {if(!dashed || (n++%4)<2)pixel(x,y);if(x==xx && y==yy)break;
        int e=2*err;if(e>=ay){err+=ay;x+=sx;}if(e<=ax){err+=ax;y+=sy;}}
}
static void rotate(int *x,int *y,unsigned heading)
{int a=*x,b=*y;switch(heading%4){case 1:*x=-b;*y=a;break;case 2:*x=-a;*y=-b;break;case 3:*x=b;*y=-a;break;default:break;}}
static void band(int x,int y,unsigned w,unsigned h,int active)
{
    if(active) {ssd1306FillRect(x,y,w,h);return;}
    /* Inactive telemeters: a one-pixel dotted centreline. */
    if(w>=h)for(unsigned i=0;i<w;i+=3)ssd1306DrawPixel(x+i,y+h/2);
    else for(unsigned i=0;i<h;i+=3)ssd1306DrawPixel(x+w/2,y+i);
}
void fw_ui_maze(const nm_map_t *m,nm_pose_t p,const char *status,unsigned speed,uint32_t ms,uint8_t sensors)
{
    (void)speed;(void)ms;
    int scale=zoom?zoom:52/(int)nm_size(m);if(scale<3)scale=3;
    int rx=(int)p.x*1000+dx[p.heading%4]*progress,ry=(int)p.y*1000+dy[p.heading%4]*progress;
    int cx=zoom?rx:((int)nm_size(m)-1)*500,cy=zoom?ry:((int)nm_size(m)-1)*500;
    rotate(&cx,&cy,m->start_heading);cx+=pan_x*1000;cy+=pan_y*1000;
    ssd1306ClearScreen();
    uint8_t goals[NM_CELLS];nm_goal(m,goals);
    for(unsigned y=0;y<nm_size(m);++y)for(unsigned x=0;x<nm_size(m);++x) {
        unsigned c=y*NM_SIDE+x;const nm_cell_t *cell=&m->cell[c];
        int xx=x*1000,yy=y*1000;rotate(&xx,&yy,m->start_heading);
        int px=64+(xx-cx)*scale/1000,py=32-(yy-cy)*scale/1000;
        if(px+scale<4 || px-scale>=124 || py+scale<4 || py-scale>=60)continue;
        for(unsigned d=0;d<4;++d) {
            unsigned facing=(d+4-m->start_heading)%4;
            int a=px-scale/2,b=py-scale/2,aa=a,bb=b;
            if(facing==0)aa+=scale;
            if(facing==1){a+=scale;aa=a;bb+=scale;}
            if(facing==2){b+=scale;bb=b;aa+=scale;}
            if(facing==3)bb+=scale;
            if(cell->walls&(1u<<d))line(a,b,aa,bb,0);
            else if(!(cell->known&(1u<<d)))line(a,b,aa,bb,1);
        }
        if(cell->visited)pixel(px,py);
        if(goals[c]) {line(px-1,py-1,px+1,py+1,0);line(px-1,py+1,px+1,py-1,0);}
    }
    if(view_route && route_visible) {
        nm_pose_t start=nm_origin(m);int c=start.y*NM_SIDE+start.x;
        for(unsigned i=0;i<view_route->length;++i) {
            int n=nm_next(m,c,view_route->direction[i]);if(n<0)break;
            int x0=c%NM_SIDE*1000,y0=c/NM_SIDE*1000,x1=n%NM_SIDE*1000,y1=n/NM_SIDE*1000;
            rotate(&x0,&y0,m->start_heading);rotate(&x1,&y1,m->start_heading);
            x0=64+(x0-cx)*scale/1000;y0=32-(y0-cy)*scale/1000;
            x1=64+(x1-cx)*scale/1000;y1=32-(y1-cy)*scale/1000;
            line(x0,y0,x1,y1,0);line(x0+1,y0,x1+1,y1,0);c=n;
        }
    }
    rotate(&rx,&ry,m->start_heading);
    int px=64+(rx-cx)*scale/1000,py=32-(ry-cy)*scale/1000;
    unsigned h=(p.heading+4-m->start_heading)%4;
    for(int y=-2;y<=2;++y)for(int x=-2;x<=2;++x)pixel(px+x,py+y);
    line(px,py,px+dx[h]*6,py-dy[h]*6,0);
    /* Active-low six binary telemeters: long = 10 cm, short = 5 cm. */
    band(14,0,40,3,!(sensors&8));band(86,0,16,3,!(sensors&4));
    band(0,7,3,24,!(sensors&32));band(0,43,3,12,!(sensors&16));
    band(125,7,3,24,!(sensors&1));band(125,43,3,12,!(sensors&2));
    if(!strcmp(status,"HAND IN FRONT OF F10") || !strcmp(status,"REMOVE YOUR HAND")) {
        ssd1306ClearRect(5,24,118,15);
        fw_display_text(8,25,!strcmp(status,"REMOVE YOUR HAND")?"Remove hand":"Hand on F10");
    }
    ssd1306Refresh();
}
void fw_ui_result(const char *reason,uint32_t search,uint32_t run,unsigned page)
{
    ssd1306ClearScreen();char s[24],row[17];
    if(page==1) {
        fw_display_text(0,0,!strncmp(reason,"RUN ",4) && strlen(reason)==5?reason:"TIMES");
        snprintf(s,sizeof s,"Explore %lu:%02lu",(unsigned long)(search/60000),(unsigned long)(search/1000%60));fw_display_text(0,15,s);
        if(!run && !strncmp(reason,"RUN ",4) && strlen(reason)==5)snprintf(s,sizeof s,"Not run");
        else snprintf(s,sizeof s,"Run %lu.%03lu s",(unsigned long)(run/1000),(unsigned long)(run%1000));
        fw_display_text(0,30,s);
    } else {
        const char *rest=reason;fw_display_text(0,0,"RESULT");
        rest=fw_text_line(rest,row,16);fw_display_text(0,15,row);
        fw_text_line(rest,row,16);fw_display_text(0,30,row);
    }
    fw_display_text(0,49,"Hold Zoom: exit");ssd1306Refresh();
}
