#include "fw_menu.h"
#include "fw_battery.h"
#include "oled/ssd1306.h"
#include <math.h>
#include <stdint.h>

/* Square, rotationally symmetric letterforms. N is the same Z turned 90 degrees;
 * H becomes I at a quarter turn. Curated choreography, constant-width vectors. */
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
    float home_x,home_y,base_turn;
} letter_t;
static const letter_t letters[]={
    {z,3,15,24,0}, {h,3,39,24,0}, {o,8,63,24,0},
    {z,3,87,24,1}, {x,2,111,24,0},
    {numeral,1,58,49,0}, {numeral,1,68,49,0}
};
/* Every scattered layout reserves a distinct slot for each character. */
static const uint8_t layouts[][7][2]={
    {{14,15},{38,48},{64,15},{89,47},{113,15},{14,49},{114,49}},
    {{14,48},{39,15},{64,48},{89,15},{113,48},{14,15},{114,15}},
    {{39,48},{14,15},{64,15},{113,15},{89,48},{14,49},{114,49}},
    {{39,15},{14,48},{64,15},{113,48},{89,15},{14,15},{114,15}},
    {{14,14},{39,23},{64,33},{89,43},{113,49},{14,49},{114,14}},
    {{15,24},{39,24},{63,24},{87,24},{111,24},{14,49},{114,49}}
};
typedef struct {
    uint8_t layout,rank[7];
    int8_t turns[7],curve_x,curve_y;
} choreography_t;
static const choreography_t collection[]={
    /* Original weave; fan; curved orbit; crossing lanes. */
    {0,{0,1,2,3,4,5,6},{ 1,-1, 2,-1, 1, 1,-1},0,0},
    {1,{4,2,0,1,3,5,6},{-1, 1,-2, 1,-1,-1, 1},0,0},
    {2,{1,3,0,4,2,6,5},{ 2, 1,-1,-2, 1, 1,-1},2,4},
    {3,{0,2,4,3,1,5,6},{-1,-1, 2, 1, 1,-1, 1},0,4},
    /* Centre-out ripple; diagonal; typographic quarter turns; curved fan. */
    {0,{4,2,0,1,3,6,5},{-2, 1, 1,-1, 2,-1, 1},0,-4},
    {4,{0,1,2,3,4,6,5},{ 1, 2,-1,-2, 1, 1,-1},0,0},
    {5,{0,1,2,3,4,5,6},{ 1,-1, 1,-1, 1, 1,-1},0,0},
    {1,{0,2,4,3,1,6,5},{ 2,-1,-2, 1,-2,-1, 1},2,-4}
};
enum { COLLECTION_SIZE=sizeof collection/sizeof *collection, CYCLE_MS=10000 };
static uint32_t random_state=0x6d2b79f5u;
static uint8_t bag[COLLECTION_SIZE],next=COLLECTION_SIZE,choice=COLLECTION_SIZE;
static unsigned cycle,mirror,reverse,duration,stagger;
static int started;
static uint32_t random_word(void)
{
    random_state^=random_state<<13;
    random_state^=random_state>>17;
    random_state^=random_state<<5;
    return random_state;
}
static void choose_choreography(void)
{
    if(next==COLLECTION_SIZE) {
        for(unsigned i=0;i<COLLECTION_SIZE;++i)bag[i]=(uint8_t)i;
        for(unsigned i=COLLECTION_SIZE-1;i;--i) {
            unsigned j=random_word()%(i+1);
            uint8_t swap=bag[i];bag[i]=bag[j];bag[j]=swap;
        }
        /* Also avoid a repeat across two shuffled collections. */
        if(bag[0]==choice) {uint8_t swap=bag[0];bag[0]=bag[1];bag[1]=swap;}
        next=0;
    }
    choice=bag[next++];
    uint32_t variation=random_word();
    mirror=variation&1;reverse=(variation>>1)&1;
    duration=1800+((variation>>2)%4)*100;
    stagger=100+((variation>>4)%3)*20;
}
void fw_ui_idle_start(unsigned seed)
{
    /* Menu entry time supplies variation without RTC, flash writes or blocking
     * entropy acquisition. Keep the shuffled bag across successive wake-ups. */
    random_state^=seed+0x9e3779b9u;
    if(!random_state)random_state=0x6d2b79f5u;
    choose_choreography();cycle=0;started=1;
}
#ifdef FW_IDLE_TEST
unsigned fw_idle_test_choice(void) {return choice;}
#endif
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
    if(!started)fw_ui_idle_start(elapsed_ms);
    unsigned requested_cycle=elapsed_ms/CYCLE_MS,time=elapsed_ms%CYCLE_MS;
    if(requested_cycle!=cycle) {
        choose_choreography();cycle=requested_cycle;
    }
    const choreography_t *motion=&collection[choice];
    ssd1306ClearScreen();
    for(unsigned i=0;i<sizeof letters/sizeof *letters;++i) {
        const letter_t *letter=&letters[i];
        unsigned rank=mirror?6-motion->rank[i]:motion->rank[i];
        int forming=time>=4700;
        float away=forming?1-ease(time,4700+(6-rank)*stagger,duration):
                           ease(time,800+rank*stagger,duration);
        float sign=(forming?-1.0f:1.0f)*(reverse?-1.0f:1.0f);
        float turn=letter->base_turn+sign*away*motion->turns[i];
        /* Reflect both the slots and their assignment: reflecting just X would
         * send all five letters through the same point in the middle. */
        unsigned slot=mirror?(i<5?4-i:11-i):i;
        float target_x=layouts[motion->layout][slot][0];
        float target_y=layouts[motion->layout][slot][1];
        if(mirror)target_x=128-target_x;
        float bend=4*away*(1-away)*(i%2?-1:1)*(forming?-1:1);
        float cx=letter->home_x+away*(target_x-letter->home_x)+bend*motion->curve_x;
        float cy=letter->home_y+away*(target_y-letter->home_y)+bend*motion->curve_y;
        float cosine,sine;rotation(turn,&cosine,&sine);
        for(unsigned j=0;j<letter->count;++j) {
            const segment_t *p=&letter->strokes[j];
            stroke(cx+cosine*p->x0-sine*p->y0,cy+sine*p->x0+cosine*p->y0,
                   cx+cosine*p->x1-sine*p->y1,cy+sine*p->x1+cosine*p->y1);
        }
    }
    /* Each cycle starts and ends at the same motionless wordmark. At the
     * scattered hold, opposite turns differ by 180 degrees: identical glyphs.
     * Keep acquisition active without a menu overlay. */
    fw_battery_poll();ssd1306Refresh();
}
