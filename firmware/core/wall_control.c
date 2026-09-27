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
    return wall_control_position_timed(c,s,d,forward,yaw,10);
}
unsigned wall_control_speed_limit(const wall_control_t *c)
{
    return c->recovery_side?80:0;
}
int wall_control_position_timed(wall_control_t *c,uint8_t s,const fw_cal_data_t *d,
                                int32_t forward,int32_t yaw,uint32_t elapsed)
{
    if(!d || !d->valid)return wall_control_step(c,s);
    int32_t centre=(int32_t)d->geometry.inner_um/2;
    int first=!c->initialized;
    if(first) {c->lateral_um=centre;c->initialized=1;c->sensors=s;}
    c->bias_fraction+=forward*c->yaw_bias_mrad_m;
    int32_t bias=c->bias_fraction/1000000;c->bias_fraction-=bias*1000000;
    c->heading_mrad=limit(c->heading_mrad+yaw+bias,200);
    if(c->reference_side) {
        c->reference_distance+=forward;c->reference_yaw+=yaw;
        if(c->reference_distance>5000000)c->reference_side=0;
    }
    for(unsigned side=0;side<2;++side)for(unsigned edge=0;edge<2;++edge) {
        wall_edge_t *e=&c->edge[side][edge];unsigned wallbit=side?0x01:0x20;
        if((s&wallbit) || ((s^c->sensors)&wallbit))e->valid=0;
        if(e->valid) {
            e->distance+=forward;e->yaw+=yaw+bias;e->integral+=(int64_t)forward*e->yaw;
            if(e->distance>5000000)e->valid=0;
        }
    }
    c->lateral_um+=forward*c->heading_mrad/1000;
    int32_t lo=(int32_t)d->geometry.width_um/2,hi=(int32_t)d->geometry.inner_um-lo;
    /* Project delayed binary bounds to the current axle position. Filtering
     * needs three scans; treating the old threshold as current destabilizes
     * the observer when a scan covers several millimetres. */
    int32_t lag=limit(forward*3*c->heading_mrad/1000,5000);
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
        if(forward>0 && (c->edge[0][0].valid || c->edge[0][1].valid || c->edge[1][0].valid || c->edge[1][1].valid) && !c->recovery_side) {
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
        }
        desired_heading=(near?20:-20)*(reference?-1:1);
    }
    else c->reference_side=0;
    /* Keep both sides and both edge directions independently. A release on
     * the recovery wall remains observable even when the other sensor is
     * closer to the nominal centre. Same-direction crossings cancel the
     * unknown hysteresis; the first half cycle uses a lower-confidence bracket. */
    for(unsigned side=0;side<2;++side) {
        unsigned bit=side?0x02:0x10,wallbit=side?0x01:0x20;
        if(first || (s&wallbit) || ((s^c->sensors)&wallbit) || !((s^c->sensors)&bit))continue;
        unsigned near=!(s&bit);wall_edge_t *e=&c->edge[side][near];
        int same=e->valid;
        if(!same)e=&c->edge[side][!near];
        int32_t observed=(int32_t)(near?d->side[side].near_um:d->side[side].far_um);
        if(side)observed=(int32_t)d->geometry.inner_um-observed;
        observed+=lag;
        if(e->valid && e->distance>=15000) {
            int32_t heading=(int32_t)(((int64_t)(observed-e->position)*1000-e->integral)/e->distance)+e->yaw;
            int32_t band=same?0:(int32_t)(d->side[side].far_um-d->side[side].near_um+d->side[side].spread_um);
            int32_t innovation=(int32_t)((int64_t)(heading-c->heading_mrad)*e->distance/(e->distance+band*10));
            c->heading_mrad=limit(c->heading_mrad+limit(innovation,60),200);
        }
        c->edge[side][near]=(wall_edge_t){.position=observed,.valid=1};
        c->lateral_um+=limit(observed-c->lateral_um,3000);
    }
    /* The saved side calibration is a static switching bracket, NOT a pair
     * of measured hysteresis edges. Keep that uncertainty in the dwell budget.
     * Time alone does not measure yaw: also require sustained forward travel
     * beside the same 10 cm wall, and discard evidence at every doorway. */
    if(elapsed>50)elapsed=50;
    for(unsigned side=0;side<2;++side) {
        unsigned wallbit=side?0x01:0x20,bit=side?0x02:0x10;
        if(first || (s&wallbit) || ((s^c->sensors)&wallbit) || (s&bit) || forward<=0) {
            c->near_ms[side]=c->near_um[side]=0;
        } else {
            if(c->near_ms[side]<10000)c->near_ms[side]+=elapsed;
            if(c->near_um[side]<2000000)c->near_um[side]+=(uint32_t)forward;
        }
        int32_t middle=(int32_t)(d->side[side].near_um+d->side[side].far_um)/2;
        int32_t offset=middle-centre;
        uint32_t band=d->side[side].far_um-d->side[side].near_um+d->side[side].spread_um;
        uint32_t window=60000+band*30;
        int exclusive=!!(s&(side?0x10:0x02));
        if(!c->recovery_side && exclusive && offset>=-5000 && offset<=5000 &&
           c->near_ms[side]>=120 && c->near_um[side]>window) {
            c->recovery_side=(uint8_t)(side+1);c->recovery_released=0;
            c->recovery_ms=c->recovery_um=c->settle_um=0;
        }
        /* Escalate the shallow tracking angle before needing recovery, but
         * never mistake simultaneous near readings for a known lateral side. */
        if(!c->recovery_side && exclusive && reference==(int)side && nearest<=5000 && c->near_ms[side]>60) {
            int32_t extra=(int32_t)(c->near_um[side]*20/window);
            if(extra>20)extra=20;
            desired_heading=(20+extra)*(side?-1:1);
        }
    }
    if(c->recovery_side) {
        unsigned side=c->recovery_side-1,bit=side?0x02:0x10,wallbit=side?0x01:0x20;
        if((s&wallbit) || ((s^c->sensors)&wallbit)) {
            /* Door edge: not a lateral reference. Resume normal observation. */
            c->recovery_side=0;
        } else {
            c->recovery_ms+=elapsed;
            if(forward>0)c->recovery_um+=(uint32_t)forward;
            if((s&bit) && !c->recovery_released) {c->recovery_released=1;c->recovery_um=0;}
            else if(!(s&bit) && c->recovery_released) {c->recovery_released=0;c->settle_um=0;}
            if(!c->recovery_released)desired_heading=40*(side?-1:1);
            else {
                /* On the known wall's release edge, straighten gradually.
                 * Do not drive to an invented exact centre inside the bracket. */
                desired_heading=0;
                if(c->heading_mrad>=-12 && c->heading_mrad<=12 && forward>0)c->settle_um+=(uint32_t)forward;
                else c->settle_um=0;
                if(c->settle_um>=15000)c->recovery_side=0;
            }
            if(c->recovery_side && (c->recovery_um>(c->recovery_released?150000u:300000u) || c->recovery_ms>5000))c->unsafe=1;
        }
    }
    c->sensors=s;
    /* Keep angular loop gain compatible with the three-scan sensor latency
     * when each scan covers a larger distance. */
    int gain=forward>4000?1:2;
    int desired=limit((desired_heading-c->heading_mrad)*gain,120);
    /* An unchanging bit cannot validate the estimated yaw. During the bounded
     * search retain a small turn away, even if dead reckoning claims parallel. */
    if(c->recovery_side && !c->recovery_released) {
        int sign=c->recovery_side==1?1:-1;
        if(desired*sign<20)desired=sign*20;
    }
    c->output+=limit(desired-c->output,8);
    return c->output;
}
