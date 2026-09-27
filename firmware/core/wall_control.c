#include "wall_control.h"
#include <string.h>
void wall_control_reset(wall_control_t *c) { memset(c,0,sizeof *c); }
static int update(wall_control_t *c,int error)
{
    c->filtered += (error - c->filtered) / 4;
    int desired = (80 * c->filtered + 25 * (c->filtered - c->previous)) / 1024;
    c->previous = c->filtered;
    if (desired > 120) desired = 120;
    if (desired < -120) desired = -120;
    int delta = desired - c->output;
    if (delta > 8) delta = 8;
    if (delta < -8) delta = -8;
    c->output += delta;
    return c->output;
}

int wall_control_step(wall_control_t *c,uint8_t s)
{
    int left=!(s&0x20),right=!(s&0x01),near_left=!(s&0x10),near_right=!(s&0x02);
    int error=0;
    if(near_left!=near_right) error=near_left?1024:-1024;
    else if(left!=right) error=left?-256:256;
    if(!left && !right) error=0;
    return update(c,error);
}
int wall_control_calibrated(wall_control_t *c,uint8_t s,const fw_cal_data_t *d)
{
    if(!d || !d->valid) return wall_control_step(c,s);
    int32_t low=0,high=(int32_t)d->geometry.inner_um,centre=high/2;
    if(!(s&0x20)) {
        if(!(s&0x10)) high=(int32_t)d->side[0].far_um;
        else low=(int32_t)d->side[0].near_um;
    }
    if(!(s&0x01)) {
        if(!(s&0x02)) {
            int32_t bound=(int32_t)d->geometry.inner_um-(int32_t)d->side[1].far_um;
            if(bound>low) low=bound;
        } else {
            int32_t bound=(int32_t)d->geometry.inner_um-(int32_t)d->side[1].near_um;
            if(bound<high) high=bound;
        }
    }
    /* Each binary observation bounds the possible axle position. Do not steer
     * when that interval contains the centre, or when readings contradict. */
    int error=0;
    if(low<=high) {
        if(high<centre) error=(centre-high)*1024/4000;
        else if(low>centre) error=(centre-low)*1024/4000;
    }
    if(error>1024) error=1024;
    if(error<-1024) error=-1024;
    return update(c,error);
}

static int32_t limit(int32_t x,int32_t bound) {return x>bound?bound:x<-bound?-bound:x;}
void wall_control_heading_reference(wall_control_t *c,int32_t heading)
{
    /* Two independent post observations provide a bounded yaw reference. */
    c->heading_mrad += limit(heading-c->heading_mrad,40)/2;
}
int wall_control_position(wall_control_t *c,uint8_t s,const fw_cal_data_t *d,
                          int32_t forward,int32_t yaw)
{
    if(!d || !d->valid)return wall_control_step(c,s);
    int32_t centre=(int32_t)d->geometry.inner_um/2;
    int first=!c->initialized;
    if(first) {c->lateral_um=centre;c->initialized=1;c->sensors=s;}
    c->heading_mrad=limit(c->heading_mrad+yaw,200);
    c->lateral_um+=forward*c->heading_mrad/1000;
    int32_t lo=(int32_t)d->geometry.width_um/2,hi=(int32_t)d->geometry.inner_um-lo;
    int walls=0;
    if(!(s&0x20)) {
        walls=1;
        if(!(s&0x10)) {if(hi>(int32_t)d->side[0].far_um)hi=(int32_t)d->side[0].far_um;}
        else if(lo<(int32_t)d->side[0].near_um)lo=(int32_t)d->side[0].near_um;
    }
    if(!(s&0x01)) {
        walls=1;
        if(!(s&0x02)) {
            int32_t bound=(int32_t)d->geometry.inner_um-(int32_t)d->side[1].far_um;
            if(lo<bound)lo=bound;
        } else {
            int32_t bound=(int32_t)d->geometry.inner_um-(int32_t)d->side[1].near_um;
            if(hi>bound)hi=bound;
        }
    }
    if(walls && lo<=hi) {
        int32_t residual=c->lateral_um<lo?lo-c->lateral_um:c->lateral_um>hi?hi-c->lateral_um:0;
        residual=limit(residual,3000);
        c->lateral_um+=residual;
        /* A fixed 20 mm observer horizon keeps drift correction effective in
         * long corridors. Fractional mrad are retained, rather than rounded away. */
        if(forward>0) {
            c->innovation_q8+=limit(residual*256/80,6*256);
            int32_t delta=c->innovation_q8/256;
            c->heading_mrad=limit(c->heading_mrad+delta,200);c->innovation_q8-=delta*256;
        }
    }
    int reference=-1;int32_t nearest=INT32_MAX;
    for(unsigned side=0;side<2;++side)if(!(s&(side?0x01:0x20))) {
        int32_t threshold=(int32_t)(d->side[side].near_um+d->side[side].far_um)/2;
        int32_t error=threshold-centre;if(error<0)error=-error;
        if(error<nearest) {nearest=error;reference=(int)side;}
    }
    /* A constant binary state cannot distinguish the centre from a large drift
     * within the same half corridor. Follow the nearest measured switching band
     * with a small bounded heading, not an unobservable zero-error interval.
     * Prefer thresholds within 5 mm of centre; others only constrain the pose. */
    int32_t desired_heading=walls?limit((centre-c->lateral_um)*6/1000,60):0;
    if(reference>=0 && nearest<=5000) {
        unsigned bit=reference?0x02:0x10;
        int near=!(s&bit);
        if(!first && ((s^c->sensors)&bit)) {
            int32_t observed=near?(int32_t)d->side[reference].near_um:(int32_t)d->side[reference].far_um;
            if(reference)observed=(int32_t)d->geometry.inner_um-observed;
            c->lateral_um+=limit(observed-c->lateral_um,3000);
        }
        desired_heading=(near?20:-20)*(reference?-1:1);
    }
    c->sensors=s;
    int desired=limit((desired_heading-c->heading_mrad)*2,120);
    c->output+=limit(desired-c->output,8);
    return c->output;
}
