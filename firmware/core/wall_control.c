#include "wall_control.h"
void wall_control_reset(wall_control_t *c) { c->filtered = c->previous = c->output = 0; }
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
