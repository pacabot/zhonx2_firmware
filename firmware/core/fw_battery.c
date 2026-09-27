#include "fw_battery.h"
#include <stdlib.h>
static fw_battery_reference_t reference;
static fw_battery_status_t state;
static uint32_t last,quiet;
static int initialized;
static unsigned supply_mv=3300;
void fw_battery_supply(unsigned mv) {supply_mv=mv>=2700 && mv<=3600?mv:0;}
static int32_t filtered_q8;
int fw_battery_reference_valid(const fw_battery_reference_t *r)
{
    return r && ((!r->raw && !r->pack_mv) ||
        (r->raw>=256 && r->raw<=4080 && r->pack_mv>=6000 && r->pack_mv<=8500));
}
void fw_battery_set_reference(fw_battery_reference_t r)
{
    reference=fw_battery_reference_valid(&r)?r:(fw_battery_reference_t){0};
    state=(fw_battery_status_t){0};initialized=0;
}
fw_battery_reference_t fw_battery_reference(void) {return reference;}
fw_battery_status_t fw_battery_status(void) {return state;}
unsigned fw_battery_percent(unsigned pack_mv)
{
    /* Indicative resting-voltage curve for conventional 4.20 V/cell LiPo.
     * Not cell characterization, coulomb counting or a protection threshold. */
    const unsigned mv[]={6600,7000,7200,7400,7500,7600,7700,7800,8000,8200,8400};
    const unsigned pc[]={0,5,10,15,25,40,55,70,80,90,100};
    if(pack_mv<=mv[0])return 0;
    for(unsigned i=1;i<sizeof mv/sizeof *mv;++i)
        if(pack_mv<=mv[i])return pc[i-1]+(pack_mv-mv[i-1])*(pc[i]-pc[i-1])/(mv[i]-mv[i-1]);
    return 100;
}
void fw_battery_sample(unsigned raw,uint32_t now,int moving)
{
    uint32_t dt=now-last;
    if(initialized && dt<100)return;
    int fresh_start=!initialized || dt>1500;
    last=now;state.raw=raw;
    state.calibrated=reference.raw!=0;
    state.lower_bound=raw>=4090;
    if(raw<16 || raw>4095 || (!reference.raw && !supply_mv)) {
        state.sample_valid=state.soc_valid=state.rested=0;state.pack_mv=0;initialized=0;return;
    }
    if(fresh_start) {filtered_q8=(int32_t)raw*256;quiet=now;}
    else filtered_q8+=(int32_t)(((int64_t)((int32_t)raw*256-filtered_q8)*dt)/4000);
    initialized=1;state.sample_valid=1;
    if(moving)quiet=now;
    state.rested=!moving && (uint32_t)(now-quiet)>=5000;
    unsigned filtered=(unsigned)(filtered_q8+128)/256;state.raw=filtered;
    /* TEST_BAT: 10 kOhm above PA4, 6.8 kOhm to ground. VREFINT
     * compensates actual VDDA; an optional meter reference corrects divider gain.
     * At ADC saturation this conversion is only a lower bound. */
    state.pack_mv=state.calibrated?
        (unsigned)(((uint64_t)filtered*reference.pack_mv+reference.raw/2)/reference.raw):
        (unsigned)(((uint64_t)filtered*supply_mv*168+4095*34)/(4095*68));
    if(state.pack_mv<5000 || state.pack_mv>8800) {state.sample_valid=state.soc_valid=0;initialized=0;return;}
    if(!moving && (uint32_t)(now-quiet)>=5000) {
        unsigned percent=fw_battery_percent(state.pack_mv);
        if(!state.soc_valid || abs((int)percent-(int)state.percent)>=2)state.percent=percent;
        state.soc_valid=1;
    }
}
