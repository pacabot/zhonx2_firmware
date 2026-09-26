#include "fw_calibration.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef struct {
    int32_t x,y;
    unsigned heading,moves,turns,frames;
    uint8_t bits;
    int missing,stuck,cancel_at,glitch;
} robot_t;
static const fw_cal_geometry_t geometry={40000,90000,167000,179000};
static uint32_t wall_distance(const robot_t *r,unsigned direction)
{
    if (direction==0) return (uint32_t)r->y;
    if (direction==1) return 167000u-(uint32_t)r->x;
    if (direction==3) return (uint32_t)r->x;
    return 1000000;
}
static uint8_t sensors(robot_t *r)
{
    const unsigned bit[]={4,8,16,32,2,1};
    const unsigned offset[]={0,0,3,3,1,1};
    const uint32_t on[]={76000,130000,79000,132000,89000,131000};
    const uint32_t off[]={80000,136000,79000,132000,89000,131000};
    for(unsigned i=0;i<6;++i) {
        uint32_t distance=wall_distance(r,(r->heading+offset[i])%4);
        if(distance<=on[i]) r->bits &= ~bit[i];
        if(distance>off[i]) r->bits |= bit[i];
    }
    if(r->missing) r->bits |= 32;
    if(r->stuck==1) r->bits &= ~8u;
    if(r->stuck==2) r->bits |= 16u;
    uint8_t raw=r->bits;
    if(r->glitch && ++r->frames%53==0) raw ^= 4;
    return raw;
}
static int read_robot(void *context,uint8_t *bits)
{
    robot_t *r=context; unsigned stable=0; uint8_t previous=0;
    for (unsigned i=0;i<100;++i) {
        *bits=sensors(r); stable=*bits==previous?stable+1:1; previous=*bits;
        if (stable==5) return 0;
    }
    return -1;
}
static int move_robot(void *context,int32_t um,unsigned speed,fw_cal_observer sample,void *data)
{
    robot_t *r=context;
    assert(speed==10 || speed==20);
    assert(um>=-180000 && um<=180000 && um);
    if((int)++r->moves==r->cancel_at) return -1;
    int sign=um<0?-1:1, distance=abs(um), moved=0;
    while(moved<distance) {
        int step=distance-moved<100?distance-moved:100;
        moved+=step;
        if(r->heading==0) r->y-=sign*step;
        if(r->heading==1) r->x+=sign*step;
        if(r->heading==2) r->y+=sign*step;
        if(r->heading==3) r->x-=sign*step;
        /* Contact consumes motor pulses without changing physical position. */
        if(r->heading==0 && r->y<40000) r->y=40000;
        if(r->heading==1 && r->x>127000) r->x=127000;
        if(r->heading==3 && r->x<40000) r->x=40000;
        uint8_t bits=sensors(r);
        if(sample) sample(data,sign*moved,bits);
    }
    for(unsigned i=0;i<5;++i) if(sample) sample(data,um,sensors(r));
    return 0;
}
static int turn_robot(void *context,int degrees)
{
    robot_t *r=context;
    assert(degrees==90 || degrees==-90);
    /* Rectangle swept radius sqrt(40^2+45^2)=60.208 mm, plus margin. */
    assert(r->x>=63000 && r->x<=104000 && r->y>=63000);
    ++r->turns; r->heading=(r->heading+4+degrees/90)%4;
    (void)sensors(r); return 0;
}
static int run(robot_t *r,fw_cal_data_t *result)
{
    const fw_cal_io_t io={r,move_robot,turn_robot,read_robot,0};
    return fw_cal_run(&io,&geometry,result);
}
static robot_t initial(void) { return (robot_t){.x=83500,.y=83500,.bits=0x3f}; }
int main(void)
{
    robot_t r=initial(); fw_cal_data_t d={0};
    assert(!run(&r,&d)); assert(fw_cal_valid(&d));
    assert(d.front[0].on_um==76000 && d.front[0].off_um==80100);
    assert(d.front[1].on_um==130000 && d.front[1].off_um==136100);
    assert(d.side[0].near_um==78500 && d.side[0].far_um==79500);
    assert(d.side[1].near_um==88500 && d.side[1].far_um==89500);
    assert(!d.front[0].spread_um && !d.side[0].spread_um);
    assert(r.x==83500 && r.y==83500 && r.heading==0 && r.turns>10);
    printf("calibration: %u bounded moves, %u turns, front hysteresis and both 1 mm side brackets; centred finish\n",r.moves,r.turns);
    fw_cal_data_t previous=d;
    r=initial(); r.x=75000; r.y=120000; assert(!run(&r,&d));
    assert(r.x==83500 && r.y==83500 && r.heading==0);
    r=initial(); r.glitch=1; assert(!run(&r,&d));
    assert(abs((int)d.front[0].on_um-76000)<1000);
    r=initial(); r.missing=1;
    assert(run(&r,&d)==FW_CAL_WALLS && !r.moves);
    d=previous; r=initial(); r.stuck=1;
    assert(run(&r,&d)==FW_CAL_RANGE && !memcmp(&d,&previous,sizeof d));
    r=initial(); r.stuck=2;
    assert(run(&r,&d)==FW_CAL_RANGE && !memcmp(&d,&previous,sizeof d));
    r=initial(); r.cancel_at=8;
    assert(run(&r,&d)==FW_CAL_MOTION && !memcmp(&d,&previous,sizeof d));
    fw_cal_geometry_t bad=geometry; bad.nose_um=0;
    assert(!fw_cal_geometry_valid(&bad)); bad=geometry; bad.width_um=140000;
    assert(!fw_cal_geometry_valid(&bad));
    puts("calibration: noise rejected, missing wall, missing transitions, cancellation and rotation clearance");
    return 0;
}
