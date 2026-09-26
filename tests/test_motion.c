#include "fw_motion.h"
#include "config/config.h"
#include "hal/hal_sensor.h"
#include "hal/hal_step_motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
static uint32_t now;
static hal_sensor_snapshot scan;
static unsigned long remain[2], rates[2], total[2];
static long commanded[2];
static double fraction[2];
static int disabled;
unsigned long hal_os_get_systicks(void) { return now; }
int hal_sensor_snapshot_read(hal_sensor_snapshot *s) { *s=scan; return scan.sequence>=3; }
void hal_step_motor_pair_release(void) { remain[0]=remain[1]=rates[0]=rates[1]=0; }
int hal_step_motor_disable(void) { disabled=1; return 0; }
int hal_step_motor_enable(void) { disabled=0; return 0; }
void hal_step_motor_wakeup(void) {}
void hal_step_motor_pair_start(long right,long left)
{
    commanded[0]=right; commanded[1]=left;
    for(int i=0;i<2;++i) {
        remain[i]=(labs(commanded[i])+1u)&~1u;
        total[i]=0; fraction[i]=0;
    }
}
unsigned long hal_step_motor_pair_remaining(unsigned int i) { return remain[i]; }
void hal_step_motor_pair_rate(unsigned long right,unsigned long left)
{
    rates[0]=remain[0]?right:0; rates[1]=remain[1]?left:0;
    assert(right<=MAX_SPEED && left<=MAX_SPEED);
}
static void setup(void)
{
    now=100; scan=(hal_sensor_snapshot){100,10,0x3f,0x3f};
    fw_motion_init(); assert(disabled);
}
static void tick(int fresh)
{
    for(unsigned i=0;i<2;++i) {
        fraction[i]+=(double)rates[i]/1000.;
        unsigned long pulses=(unsigned long)fraction[i]; fraction[i]-=pulses;
        if(pulses>remain[i]) pulses=remain[i];
        remain[i]-=pulses; total[i]+=pulses;
    }
    ++now;
    if(fresh && now%10==0) { scan.timestamp=now; ++scan.sequence; }
    fw_motion_tick(now);
}
int main(void)
{
    for(unsigned speed=20;speed<=300;speed+=20) {
        setup(); assert(!fw_motion_straight(1,speed));
        assert(commanded[0]==lroundf(180.f*2.f*(float)STEPS_PER_MM));
        assert(commanded[0]==commanded[1]);
        unsigned elapsed=0;
        while(fw_motion_busy() && elapsed++<20000) tick(1);
        assert(!fw_motion_busy() && !fw_motion_fault() && elapsed<20000);
        assert(total[0]==total[1] && total[0]==((unsigned long)commanded[0]+1u)/2*2);
    }
    setup(); assert(!fw_motion_turn(90)); assert(commanded[0]<0 && commanded[1]>0);
    assert(labs(commanded[0])==lroundf((float)M_PI/4.f*(float)WHEELS_DISTANCE*2.f*(float)STEPS_PER_MM));
    while(fw_motion_busy() && now<10000) tick(1);
    assert(!fw_motion_busy() && !fw_motion_fault());
    setup(); assert(!fw_motion_straight(9,300));
    for(int i=0;i<100;++i) tick(1);
    scan.raw &= ~SENSOR_F5_POS;
    tick(1); assert(fw_motion_fault()==2 && !fw_motion_busy() && disabled);
    assert(fw_motion_straight(1,100));
    setup(); assert(!fw_motion_straight(1,100));
    for(int i=0;i<60;++i) tick(0);
    assert(fw_motion_fault()==1 && !fw_motion_busy() && disabled);
    setup(); assert(fw_motion_straight(0,100)); assert(fw_motion_straight(1,301));
    assert(fw_motion_turn(30));
    now=UINT32_MAX-30; scan.timestamp=now;
    assert(!fw_motion_straight(1,100));
    for(int i=0;i<100;++i) tick(1);
    assert(!fw_motion_fault()); fw_motion_stop();
    puts("motion: exact pulse budgets at 20..300 mm/s, turn, collision/stale stop, timer wrap");
    return 0;
}
