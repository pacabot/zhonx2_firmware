#include "fw_cal_follow.h"
#include "wall_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static const fw_cal_geometry_t geometry={47000,94000,167000,179000};
typedef struct {
    fw_cal_data_t walls;
    double lateral,heading,mismatch,seconds,worst;  /* lateral from the robot's left wall, um; heading mrad */
    double bend;   /* corridor sag over the pass, mm; + bends left going out */
    double jitter; /* threshold noise of each reading, +/- um */
    uint32_t noise;
    unsigned passes,turns,fail_after,open_side,front,latency;
    int32_t bias,estimate;int left,right,blind,alternate;uint32_t seed;
} robot_t;
static uint8_t sense(robot_t *r)
{
    const fw_cal_side_t *l=&r->walls.side[0],*q=&r->walls.side[1];
    double noise[2];
    for(unsigned i=0;i<2;++i) {
        r->noise=r->noise*1103515245u+12345u;
        noise[i]=r->jitter*(((r->noise>>8)&0xffff)/32768.0-1.0);
    }
    if(r->lateral+noise[0]<=l->near_um)r->left=1;else if(r->lateral+noise[0]>=l->far_um)r->left=0;
    double right=167000-r->lateral+noise[1];
    if(right<=q->near_um)r->right=1;else if(right>=q->far_um)r->right=0;
    uint8_t s=0x3f&~0x21;
    if(r->open_side==1)s|=0x20;
    if(r->open_side==2)s|=0x01;
    if(r->front)s&=~0x0c;
    if(r->left && !r->blind && !(s&0x20))s&=~0x10;
    if(r->right && !r->blind && !(s&0x01))s&=~0x02;
    return s;
}
static int read_robot(void *context,uint8_t mask,uint8_t *raw)
{
    (void)mask;robot_t *r=context;*raw=sense(r);return 0;
}
/* fw_motion straight at the run acceleration law; the controller restarts
 * at every move but keeps its learned bias, as start_ramp() does. */
static int pass_robot(void *context,unsigned cells,unsigned speed,fw_follow_observer observer,void *data)
{
    robot_t *r=context;
    if(r->fail_after && r->passes>=r->fail_after)return -1;
    ++r->passes;
    /* Slip that changes between passes: no common curvature. */
    double mismatch=r->alternate?((r->passes&1)?1.5:-0.5):r->mismatch;
    double odo=0;
    wall_control_t c;wall_control_reset(&c);c.yaw_bias_mrad_m=r->bias;c.lag_scans=(uint8_t)r->latency;
    double length=(cells-1)*geometry.pitch_um,accel=600+2.0*speed,v=40,s=0,remainder=0;
    uint8_t delayed[3];for(unsigned i=0;i<3;++i)delayed[i]=sense(r);
    for(unsigned tick=0;s<length;++tick) {
        double target=fmin(speed,sqrt(1600+2*accel*(length-s)/1000));
        v=fmin(target,v+accel*.01);double forward=v*10;
        uint8_t sensed=delayed[tick%r->latency];delayed[tick%r->latency]=sense(r);
        int yaw=(int)remainder;remainder-=yaw;
        double change=2.0*wall_control_position(&c,sensed,&r->walls,(int32_t)forward,yaw)*forward/83500;
        remainder+=change;odo+=change;r->heading+=change+mismatch*0.01*forward/83.5;
        /* A corridor bending left turns the robot right relative to it on the
         * way out, left on the way back (lateral is mirrored by the U-turn). */
        r->heading+=(r->passes&1?1:-1)*8*r->bend/(length*length*1e-6)*forward;
        r->lateral+=forward*r->heading/1000;s+=forward;r->seconds+=.01;
        /* Sensors read latency scans before this scan's move. */
        fw_follow_sample_t x={(int32_t)s,(int32_t)lround(odo*1000),(uint32_t)((r->latency+1)*forward),sensed};
        observer(data,&x);
        /* No wall contact; the faulty robots below only check rejection. */
        assert(r->alternate || r->blind || (r->lateral>47000 && r->lateral<120000));
        if(fabs(r->lateral-83500)>r->worst)r->worst=fabs(r->lateral-83500);
    }
    r->bias=c.yaw_bias_mrad_m;r->estimate=c.heading_mrad;
    return 0;
}
static int turn_robot(void *context,unsigned speed)
{
    robot_t *r=context;++r->turns;assert(speed==220 || speed==600);r->seconds+=.75;
    /* fw_motion_turn() removes the estimated heading of the last straight;
     * the relative heading survives a 180-degree pivot, the lateral mirrors.
     * Pivots leave up to 0.5 degree. */
    r->seed=r->seed*1103515245u+12345u;
    r->lateral=167000-r->lateral;r->heading+=-r->estimate+9*(((r->seed>>8)&0xffff)/32768.0-1.0);
    return 0;
}
static void status(void *context,const char *text,int32_t um) {(void)context;(void)text;(void)um;}
static int calibrate(robot_t *r,unsigned cells,fw_follow_data_t *out)
{
    fw_follow_io_t io={{r,0,0,read_robot,status},pass_robot,turn_robot};
    return fw_follow_run(&io,&r->walls,cells,out);
}
static robot_t robot(double mismatch,uint32_t seed)
{
    robot_t r;memset(&r,0,sizeof r);
    /* Thresholds of the 2026-09-27 wall calibration. */
    r.walls=(fw_cal_data_t){.valid=1,.repetitions=3,.geometry=geometry,
        .front={{91848,92415,408},{131991,132353,192}},.side={{84500,85500,0},{81500,83500,1000}}};
    r.lateral=85500;r.mismatch=mismatch;r.seed=seed;r.latency=3;
    return r;
}
int main(void)
{
    fw_follow_data_t d,before;
    for(int mismatch=-10;mismatch<=10;mismatch+=5)for(uint32_t seed=1;seed<=3;++seed) {
        robot_t r=robot(mismatch*0.1,seed);
        if(seed==3)r.latency=2; /* Raw 5 cm readings beside a stable wall. */
        assert(!calibrate(&r,5,&d) && fw_follow_valid(&d));
        double truth=mismatch*0.1*10000/83.5;
        printf("wheels %+.1f%% seed %u: bias %ld mrad/m (true %.0f, sd %lu, %lu passes, %lu crossings), %.0f s, worst %.1f mm from centre\n",
            mismatch*0.1,seed,(long)d.yaw_bias_mrad_m,truth,(unsigned long)d.spread_mrad_m,(unsigned long)d.passes,
            (unsigned long)d.crossings,r.seconds,r.worst/1000);
        assert(fabs(d.yaw_bias_mrad_m-truth)<=3 && d.spread_mrad_m<=30);
        assert(r.passes==FW_FOLLOW_PASSES && r.turns==FW_FOLLOW_PASSES && r.seconds<70);
        assert(fabs(r.lateral-167000+85500)<20000 || fabs(r.lateral-85500)<20000);
        assert(d.cells==5 && d.passes>=FW_FOLLOW_MIN_FITTED && !memcmp(&d.geometry,&geometry,sizeof geometry));
    }
    /* A corridor that is not straight: sag of 3 mm over the 716 mm pass, both
     * ways. Its curvature changes sign on the way back and is fitted apart. */
    for(int bend=-3;bend<=3;bend+=6)for(int mismatch=-5;mismatch<=5;mismatch+=5)for(uint32_t seed=1;seed<=2;++seed) {
        robot_t r=robot(mismatch*0.1,seed);r.bend=bend;
        assert(!calibrate(&r,5,&d) && fw_follow_valid(&d));
        const fw_follow_diag_t *g=fw_follow_last();
        double truth=mismatch*0.1*10000/83.5;
        assert(!g->result && g->yaw_bias_mrad_m==d.yaw_bias_mrad_m && g->out+g->back==d.passes);
        assert(fabs(d.yaw_bias_mrad_m-truth)<=3 && d.spread_mrad_m<=30 && abs(g->bend_um-bend*1000)<=300);
    }
    printf("bent corridor: 3 mm sag either way, wheels -0.5..+0.5 %%: bias within 3 mrad/m, sag within 0.3 mm\n");
    /* Noisy thresholds: passes with few crossings fit a loose curvature. The
     * unweighted spread read 62-96 mrad/m here (UNSTABLE) with a right bias. */
    for(int mismatch=0;mismatch<=10;mismatch+=5)for(uint32_t seed=1;seed<=4;++seed) {
        robot_t r=robot(mismatch*0.1,seed);r.jitter=1500;r.noise=seed*7919;
        assert(!calibrate(&r,5,&d) && fw_follow_valid(&d));
        assert(fabs(d.yaw_bias_mrad_m-mismatch*0.1*10000/83.5)<=5 && d.spread_mrad_m<=15);
    }
    printf("noisy thresholds: +/-1.5 mm per reading, wheels 0..+1 %%: bias within 5 mrad/m, weighted spread <= 15\n");
    before=d;
    /* Fixture and motion errors save nothing. */
    robot_t r=robot(0,1);r.open_side=1;assert(calibrate(&r,5,&d)==FW_CAL_WALLS && !r.passes);
    r=robot(0,1);r.open_side=2;assert(calibrate(&r,5,&d)==FW_CAL_WALLS && !r.passes);
    r=robot(0,1);r.front=1;assert(calibrate(&r,5,&d)==FW_CAL_WALLS && !r.passes);
    r=robot(0,1);r.fail_after=5;assert(calibrate(&r,5,&d)==FW_CAL_MOTION && fw_follow_last()->result==FW_CAL_MOTION);
    /* Slip that changes between passes. */
    r=robot(0,1);r.alternate=1;{int e=calibrate(&r,5,&d);assert(e==FW_CAL_UNSTABLE || e==FW_CAL_RANGE);}
    r=robot(0,1);r.blind=1;assert(calibrate(&r,5,&d)==FW_CAL_RANGE && r.passes==FW_FOLLOW_PASSES);
    assert(calibrate(&r,3,&d)==FW_CAL_GEOMETRY && calibrate(&r,17,&d)==FW_CAL_GEOMETRY);
    r.walls.valid=0;assert(calibrate(&r,5,&d)==FW_CAL_GEOMETRY);
    assert(!memcmp(&before,&d,sizeof d));
    /* Validity bounds. */
    fw_follow_data_t g=before;assert(fw_follow_valid(&g));
    g.yaw_bias_mrad_m=FW_FOLLOW_BIAS_LIMIT+1;assert(!fw_follow_valid(&g));g=before;
    g.spread_mrad_m=FW_FOLLOW_SPREAD_LIMIT+1;assert(!fw_follow_valid(&g));g=before;
    g.passes=FW_FOLLOW_MIN_FITTED-1;g.crossings=4*g.passes;assert(!fw_follow_valid(&g));g=before;
    g.crossings=g.passes*4-1;assert(!fw_follow_valid(&g));g=before;
    g.cells=3;assert(!fw_follow_valid(&g));g=before;
    g.geometry.inner_um=100000;assert(!fw_follow_valid(&g));
    assert(fw_follow_pass_speed(0)==220 && fw_follow_pass_speed(7)==220 && fw_follow_pass_speed(8)==600);
    puts("follow calibration: wheel bias within 3 mrad/m for -1..+1 % wheels, sensor lag 2-3 scans, 16 passes, no wall contact");
    return 0;
}
