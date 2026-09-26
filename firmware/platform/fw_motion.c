#include "fw_motion.h"
#include "wall_control.h"
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "config/config.h"
#include "hal/hal_os.h"
#include "hal/hal_sensor.h"
#include "hal/hal_step_motor.h"
#include <math.h>
#define TICKS_PER_MM (2.0f * (float)STEPS_PER_MM)
static volatile int active, fault;
static int straight;
static float velocity, acceleration, cruise;
static wall_control_t wall;
static uint32_t last_scan, started;
static const float max_accel = 500.0f; /* mm/s^2: requires bench calibration */
static const float max_jerk = 4000.0f; /* mm/s^3 */
void fw_motion_init(void)
{
    fw_motion_stop(); fault=0; velocity=acceleration=0; last_scan=0;
    wall_control_reset(&wall);
}
int fw_motion_busy(void) { return active; }
int fw_motion_fault(void) { return fault; }
uint32_t fw_motion_remaining(void)
{
    unsigned long a=hal_step_motor_pair_remaining(0), b=hal_step_motor_pair_remaining(1);
    return a>b ? a : b;
}
void fw_motion_stop(void)
{
    active=0;
    hal_step_motor_pair_release();
    hal_step_motor_disable();
    velocity=acceleration=0;
    TIM_Cmd(TIM5,ENABLE);
}
static int start(long right, long left, unsigned speed, int is_straight)
{
    hal_sensor_snapshot scan;
    if (active || fault || !hal_sensor_snapshot_read(&scan) ||
        (uint32_t)(hal_os_get_systicks()-scan.timestamp)>50) return -1;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    TIM_Cmd(TIM5,DISABLE); /* New controller exclusively owns speed corrections. */
    straight=is_straight; cruise=(float)speed; velocity=5.0f; acceleration=0;
    started=hal_os_get_systicks();
    wall_control_reset(&wall); last_scan=scan.sequence;
    hal_step_motor_pair_start(right,left);
    hal_step_motor_wakeup(); hal_step_motor_enable();
    active=1;
    __set_PRIMASK(mask);
    return 0;
}
int fw_motion_straight(unsigned cells, unsigned speed)
{
    if (!cells || cells>9 || speed<20 || speed>300) return -1;
    long pulses=lroundf((float)cells*180.0f*TICKS_PER_MM);
    return start(pulses,pulses,speed,1);
}
int fw_motion_turn(int degrees)
{
    if (degrees!=90 && degrees!=-90 && degrees!=180 && degrees!=-180) return -1;
    long ticks=lroundf(fabsf((float)degrees)*((float)M_PI/180.0f)*
                      ((float)WHEELS_DISTANCE/2.0f)*TICKS_PER_MM);
    /* Motor zero is the right wheel. Clockwise: right backwards, left forwards. */
    return start(degrees>0 ? -ticks : ticks, degrees>0 ? ticks : -ticks,80,0);
}
void fw_motion_tick(uint32_t now)
{
    if (!active) return;
    hal_sensor_snapshot scan;
    if (!hal_sensor_snapshot_read(&scan) || now-scan.timestamp>50 || now-started>60000) {
        fault=1; fw_motion_stop(); return;
    }
    if (straight && !(scan.raw & SENSOR_F5_POS)) {
        fault=2; fw_motion_stop(); return;
    }
    uint32_t remaining=fw_motion_remaining();
    if (!remaining) {
        active=0; velocity=acceleration=0; hal_step_motor_pair_rate(0,0); return;
    }
    if (scan.sequence!=last_scan) {
        last_scan=scan.sequence;
        if (straight) wall_control_step(&wall,scan.filtered);
    }
    float distance=(float)remaining/TICKS_PER_MM;
    /* Reserve distance for the jerk-limited acceleration reversal. */
    float reserve=velocity*max_accel/max_jerk;
    float target=sqrtf(2.0f*max_accel*fmaxf(0,distance-reserve));
    if (target>cruise) target=cruise;
    float desired=(target-velocity)*10.0f;
    if (desired>max_accel) desired=max_accel;
    if (desired<-max_accel) desired=-max_accel;
    float delta=desired-acceleration;
    float jerk_step=max_jerk*0.001f;
    if (delta>jerk_step) delta=jerk_step;
    if (delta<-jerk_step) delta=-jerk_step;
    acceleration+=delta;
    velocity+=acceleration*0.001f;
    if (velocity<5.0f) velocity=5.0f; /* Exact pulse budget provides the final stop. */
    if (velocity>cruise) velocity=cruise;
    unsigned long base=(unsigned long)(velocity*TICKS_PER_MM);
    int correction=straight ? wall.output : 0;
    /* Reduce accumulated wheel skew without unbounded integral action. */
    long r=(long)hal_step_motor_pair_remaining(0), l=(long)hal_step_motor_pair_remaining(1);
    long skew=(r-l)*2;
    if (skew>40) skew=40;
    if (skew<-40) skew=-40;
    correction-=(int)skew;
    hal_step_motor_pair_rate(base*(1000-correction)/1000,base*(1000+correction)/1000);
}
