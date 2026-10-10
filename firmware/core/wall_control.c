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
static int32_t post_reference(const wall_control_t *c,int32_t heading,int32_t estimate)
{
    return c->exit_um?limit(heading-estimate,60)*3/4:limit(heading-estimate,40)/2;
}
void wall_control_heading_reference(wall_control_t *c,int32_t heading)
{
    /* Two independent post observations provide a bounded yaw reference. */
    c->heading_mrad+=post_reference(c,heading,c->heading_mrad);
    c->wheel_heading_mrad+=post_reference(c,heading,c->wheel_heading_mrad);
}
void wall_control_exit(wall_control_t *c,const fw_cal_data_t *d,uint8_t sensors,int32_t heading,uint32_t distance)
{
    c->initialized=1;c->sensors=sensors;c->lateral_um=(int32_t)d->geometry.inner_um/2;
    c->heading_mrad=limit(heading,200);c->exit_um=distance;
    c->wheel_heading_mrad=c->heading_mrad;c->unanchored=1;c->unanchored_um=0;
    /* Correct the residual turn angle immediately, before the next IR scan. */
    c->output=limit(-2*c->heading_mrad,120);
}
int wall_control_position(wall_control_t *c,uint8_t s,const fw_cal_data_t *d,
                          int32_t forward,int32_t yaw)
{
    if(!d || !d->valid)return wall_control_step(c,s);
    int32_t centre=(int32_t)d->geometry.inner_um/2;
    int first=!c->initialized;
    if(first) {
        c->lateral_um=centre;c->initialized=1;c->sensors=s;
        c->wheel_heading_mrad=c->heading_mrad;c->unanchored=1;c->unanchored_um=0;
    }
    c->bias_fraction+=forward*c->yaw_bias_mrad_m;
    int32_t bias=c->bias_fraction/1000000;c->bias_fraction-=bias*1000000;
    c->heading_mrad=limit(c->heading_mrad+yaw+bias,200);
    c->wheel_heading_mrad=limit(c->wheel_heading_mrad+yaw+bias,200);
    if(c->unanchored && forward>0 && c->unanchored_um<5000000)c->unanchored_um+=(uint32_t)forward;
    if(c->reference_side) {
        c->reference_distance+=forward;c->reference_yaw+=yaw;
        if(c->reference_distance>5000000)c->reference_side=0;
    }
    if(c->optical_side) {
        c->optical_distance+=forward;c->optical_yaw+=yaw+bias;
        c->optical_integral+=(int64_t)forward*c->optical_yaw;
        if(c->optical_distance>5000000)c->optical_side=0;
    }
    c->lateral_um+=forward*c->heading_mrad/1000;
    int32_t lo=(int32_t)d->geometry.width_um/2,hi=(int32_t)d->geometry.inner_um-lo;
    /* Project delayed binary bounds to the current axle position. Filtering
     * uses two or three scans; treating the old threshold as current destabilizes
     * the observer when a scan covers several millimetres. */
    int32_t lag=limit(forward*(c->lag_scans?c->lag_scans:3)*c->heading_mrad/1000,5000);
    int walls=0;
    if(!(s&0x20)) {
        walls=1;
        if(!(s&0x10)) {if(hi>(int32_t)d->side[0].far_um+lag)hi=(int32_t)d->side[0].far_um+lag;}
        else if(lo<(int32_t)d->side[0].near_um+lag)lo=(int32_t)d->side[0].near_um+lag;
    }
    if(!(s&0x01)) {
        walls=1;
        if(!(s&0x02)) {
            int32_t bound=(int32_t)d->geometry.inner_um-(int32_t)d->side[1].far_um+lag;
            if(lo<bound)lo=bound;
        } else {
            int32_t bound=(int32_t)d->geometry.inner_um-(int32_t)d->side[1].near_um+lag;
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
        unsigned wallbit=reference?0x01:0x20;
        if(c->reference_side!=(unsigned)reference+1 || ((s^c->sensors)&wallbit))c->reference_side=0;
        if(c->optical_side!=(unsigned)reference+1 || ((s^c->sensors)&wallbit))c->optical_side=0;
        if(!first && !((s^c->sensors)&wallbit) && ((s^c->sensors)&bit)) {
            /* Same-direction crossings of the same optical threshold form a
             * full tracking cycle. Mean physical yaw change approaches zero;
             * persistent wheel-step yaw then reveals wheel/ground mismatch.
             * Do not learn on door edges, sensor changes or a half cycle. */
            if(near) {
                if(c->reference_side && c->reference_distance>=50000) {
                    int32_t measured=limit((int32_t)(-(int64_t)c->reference_yaw*1000000/c->reference_distance),300);
                    c->yaw_bias_mrad_m+=limit(measured-c->yaw_bias_mrad_m,160)/4;
                }
                c->reference_side=(uint8_t)(reference+1);
                c->reference_distance=c->reference_yaw=0;
            }
            int32_t observed=near?(int32_t)d->side[reference].near_um:(int32_t)d->side[reference].far_um;
            if(reference)observed=(int32_t)d->geometry.inner_um-observed;
            observed+=lag;
            /* Two optical positions plus the intervening wheel yaw determine
             * heading independently of the assumed starting lateral position.
             * An estimated anchor (below) is only trusted over 100 mm. */
            if(c->unanchored && c->bias_calibrated) {
                /* First crossing since the reset (start, pivot, curve exit):
                 * the offset was unknown, not the heading, which the pivot or
                 * the curve had just corrected. Staying on one side was an
                 * offset that the innovations took for a heading error; keep
                 * the heading the wheels turned, or the robot crosses the
                 * corridor with it (sim: 30 mm start, 22-28 -> 9-12 mm).
                 * Only a calibrated bias makes the wheel heading usable; the
                 * part a bias error could explain stays with the estimate. */
                int32_t gap=c->wheel_heading_mrad-c->heading_mrad;
                int32_t doubt=(int32_t)((int64_t)c->unanchored_um*WALL_BIAS_DOUBT_MRAD_M/1000000);
                gap=gap>doubt?gap-doubt:gap< -doubt?gap+doubt:0;
                c->heading_mrad=limit(c->heading_mrad+gap,200);
            } else if(c->optical_side && c->optical_distance>=(c->anchor_estimated?100000:15000)) {
                int32_t heading=(int32_t)(((int64_t)(observed-c->optical_position)*1000-
                    c->optical_integral)/c->optical_distance)+c->optical_yaw;
                c->heading_mrad+=limit(heading-c->heading_mrad,c->anchor_estimated?30:60);
                c->heading_mrad=limit(c->heading_mrad,200);
            }
            c->optical_side=(uint8_t)(reference+1);c->optical_position=observed;
            c->optical_distance=c->optical_yaw=0;c->optical_integral=0;c->anchor_estimated=0;
            c->lateral_um+=limit(observed-c->lateral_um,3000);c->persist_um=0;c->unanchored=0;
        } else if(!c->optical_side && !((s^c->sensors)&wallbit)) {
            /* After a reset, a curve exit, a door edge or a reference change,
             * anchor the next crossing on the bounded estimate: its first
             * crossing then corrects the heading instead of waiting for a
             * second one. The one-sided evidence restarts with it. */
            c->optical_side=(uint8_t)(reference+1);c->optical_position=c->lateral_um;
            c->optical_distance=c->optical_yaw=0;c->optical_integral=0;c->anchor_estimated=1;
            c->persist_um=0;
        }
        /* A constant reading on either side of the line raises the return
         * heading from 20 to 30 mrad: binary sensors cannot tell an offset
         * from a heading error, staying on one side reveals one of them.
         * Door edges, reference changes and stops reset the evidence. */
        if(first || forward<=0 || ((s^c->sensors)&wallbit))c->persist_um=0;
        else if(c->persist_um<1000000)c->persist_um+=(uint32_t)forward;
        uint32_t band=d->side[reference].far_um-d->side[reference].near_um+d->side[reference].spread_um;
        int extra=(int)(c->persist_um*10/(100000+30*band));if(extra>10)extra=10;
        desired_heading=(near?20+extra:-20-extra)*(reference?-1:1);
    }
    else {c->reference_side=c->optical_side=0;c->persist_um=0;}
    c->sensors=s;
    /* Heading comes from the step counts of this scan: with stepper wheels a
     * constant gain is a constant spatial response (track/4 = 21 mm). The
     * correction slew is per millimetre above 220 mm/s; a per-scan bound gave
     * a 7x slower loop at 1500 mm/s, which kept 13-25 mm oscillations alive. */
    int desired=limit((desired_heading-c->heading_mrad)*2,120);
    int32_t slew=c->exit_um?16:8;
    if(forward>2200)slew=slew*forward/2200;
    c->output+=limit(desired-c->output,slew);
    if(forward>0)c->exit_um=(uint32_t)forward>=c->exit_um?0:c->exit_um-(uint32_t)forward;
    return c->output;
}
