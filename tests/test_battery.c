#include "fw_battery.h"
#include <assert.h>
#include <stdio.h>
static uint32_t now;
static void sample(unsigned raw,unsigned seconds,int moving)
{for(unsigned i=0;i<seconds*10;++i){now+=100;fw_battery_sample(raw,now,moving);}}
int main(void)
{
    assert(!fw_battery_status().soc_valid);
    sample(3000,6,0);assert(fw_battery_status().sample_valid && !fw_battery_status().soc_valid);
    assert(!fw_battery_reference_valid(&(fw_battery_reference_t){0,8400}));
    assert(!fw_battery_reference_valid(&(fw_battery_reference_t){4095,8400}));
    assert(!fw_battery_reference_valid(&(fw_battery_reference_t){3000,9000}));
    unsigned last=0;
    for(unsigned mv=5000;mv<9000;++mv){unsigned pc=fw_battery_percent(mv);assert(pc>=last && pc<=100);last=pc;}
    assert(fw_battery_percent(8400)==100 && fw_battery_percent(6600)==0);
    assert(fw_battery_percent(7400)==15 && fw_battery_percent(7800)==70);
    fw_battery_set_reference((fw_battery_reference_t){3000,8400});
    sample(3000,4,0);assert(!fw_battery_status().soc_valid);
    sample(3000,2,0);assert(fw_battery_status().percent==100 && fw_battery_status().pack_mv==8400);
    sample(2600,10,1);assert(fw_battery_status().percent==100 && !fw_battery_status().rested);
    sample(2700,4,0);assert(fw_battery_status().percent==100);
    sample(2700,30,0);assert(fw_battery_status().percent>=30 && fw_battery_status().percent<=36);
    unsigned pc=fw_battery_status().percent;
    sample(2699,10,0);assert(fw_battery_status().percent==pc); /* Reject one-code flicker. */
    sample(0,1,0);assert(!fw_battery_status().soc_valid && !fw_battery_status().sample_valid);
    sample(4095,1,0);assert(!fw_battery_status().sample_valid);
    sample(3000,6,0);assert(fw_battery_status().percent==100);
    fw_battery_set_reference((fw_battery_reference_t){3000,8400});
    now=UINT32_MAX-3000;sample(3000,6,0);assert(fw_battery_status().soc_valid); /* Tick wrap. */
    fw_battery_set_reference((fw_battery_reference_t){0});
    sample(3000,6,0);assert(!fw_battery_status().soc_valid && !fw_battery_status().calibrated);
    puts("battery: unknown/invalid input, nonlinear 2S estimate, filtering, resting delay, load hold and tick wrap");
}
