#include "fw_menu.h"
#include "fw_battery.h"
#include "oled/ssd1306.h"
#include <math.h>

/* Square, rotationally symmetric letterforms. N is the same Z turned 90 degrees;
 * H becomes I at a quarter turn. No bitmaps, perspective squash or random motion. */
typedef struct { float x0,y0,x1,y1; } segment_t;
static const segment_t z[]={{-9,-9,9,-9},{9,-9,-9,9},{-9,9,9,9}};
static const segment_t h[]={{-9,-9,-9,9},{9,-9,9,9},{-9,0,9,0}};
static const segment_t o[]={{-5,-9,5,-9},{5,-9,9,-5},{9,-5,9,5},{9,5,5,9},
                           {5,9,-5,9},{-5,9,-9,5},{-9,5,-9,-5},{-9,-5,-5,-9}};
static const segment_t x[]={{-9,-9,9,9},{9,-9,-9,9}};
static const segment_t numeral[]={{0,-7,0,7}};
typedef struct {
    const segment_t *strokes;
    unsigned count;
    float home_x,home_y,away_x,away_y,quarter_turn,base_turn;
} letter_t;
static const letter_t letters[]={
    {z,3,15,24,14,15, 1,0},
    {h,3,39,24,38,48,-1,0},
    {o,8,63,24,64,15, 2,0},
    {z,3,87,24,89,47,-1,1},
    {x,2,111,24,113,15,1,0},
    {numeral,1,58,49,14,49,1,0},
    {numeral,1,68,49,114,49,-1,0}
};
/* Quintic interpolation: zero speed AND acceleration at each end. */
static float ease(unsigned time,unsigned start,unsigned duration)
{
    if(time<=start)return 0;
    if(time>=start+duration)return 1;
    float u=(float)(time-start)/duration;
    return u*u*u*(10+u*(-15+6*u));
}
static void rotation(float quarter,float *c,float *s)
{
    int exact=(int)quarter;
    if(quarter==(float)exact) {
        static const float cosine[]={1,0,-1,0},sine[]={0,1,0,-1};
        unsigned index=(unsigned)(exact+8)%4;
        *c=cosine[index];*s=sine[index];
    } else {
        float radians=quarter*1.57079632679f;
        *c=cosf(radians);*s=sinf(radians);
    }
}
/* Constant-width vector stroke. Evaluate pixel centres, keeping subpixel
 * positions until rasterisation to avoid premature endpoint rounding. */
static void stroke(float ax,float ay,float bx,float by)
{
    int left=(int)floorf(fminf(ax,bx)-1),right=(int)ceilf(fmaxf(ax,bx)+1);
    int top=(int)floorf(fminf(ay,by)-1),bottom=(int)ceilf(fmaxf(ay,by)+1);
    if(left<0)left=0;
    if(right>127)right=127;
    if(top<0)top=0;
    if(bottom>63)bottom=63;
    float dx=bx-ax,dy=by-ay,inverse=1/(dx*dx+dy*dy);
    for(int py=top;py<=bottom;++py)for(int px=left;px<=right;++px) {
        float vx=px+0.5f-ax,vy=py+0.5f-ay;
        float t=(vx*dx+vy*dy)*inverse;
        if(t<0)t=0;
        if(t>1)t=1;
        float ex=vx-t*dx,ey=vy-t*dy;
        if(ex*ex+ey*ey<=1.05f*1.05f)ssd1306DrawPixel(px,py);
    }
}
void fw_ui_idle(unsigned elapsed_ms)
{
    unsigned time=elapsed_ms%10000;
    ssd1306ClearScreen();
    for(unsigned i=0;i<sizeof letters/sizeof *letters;++i) {
        const letter_t *letter=&letters[i];
        float away,turn;
        if(time<5600) {
            away=1-ease(time,400+i*140,1800);
            turn=letter->base_turn+away*letter->quarter_turn;
        } else {
            away=ease(time,5600+(6-i)*140,1800);
            turn=letter->base_turn-away*letter->quarter_turn;
        }
        float cx=letter->home_x+away*(letter->away_x-letter->home_x);
        float cy=letter->home_y+away*(letter->away_y-letter->home_y);
        float cosine,sine;rotation(turn,&cosine,&sine);
        for(unsigned j=0;j<letter->count;++j) {
            const segment_t *p=&letter->strokes[j];
            stroke(cx+cosine*p->x0-sine*p->y0,cy+sine*p->x0+cosine*p->y0,
                   cx+cosine*p->x1-sine*p->y1,cy+sine*p->x1+cosine*p->y1);
        }
    }
    /* The final scattered poses differ by half turns but have identical shapes,
     * giving a seamless loop. Keep acquisition active without a menu overlay. */
    fw_battery_poll();ssd1306Refresh();
}
