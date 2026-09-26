#include "fw_motion.h"
#include "wall_control.h"
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "config/config.h"
#include "hal/hal_os.h"
#include "hal/hal_sensor.h"
#include "hal/hal_step_motor.h"
#include <math.h>
#include <string.h>
#define TICKS_PER_MM (2.0f * (float)STEPS_PER_MM)
static volatile int active, fault;
static int straight, allow_wall, wall_arrival;
static int calibration, travel_sign;
static uint32_t travel_ticks;
static uint32_t cell_pitch_um=CELL_LENGTH*1000u, front_allowance_um;
static int front_calibrated;
static fw_rotation_data_t rotation_profile;
static float velocity, cruise;
static wall_control_t wall;
static fw_cal_data_t wall_profile;
void fw_motion_wall_profile(const fw_cal_data_t *d)
{
    if(active) return;
    if(fw_cal_valid(d)) wall_profile=*d;
    else memset(&wall_profile,0,sizeof wall_profile);
    wall_control_reset(&wall);
}
static uint32_t last_scan, started;
static const float max_accel = 800.0f; /* mm/s^2 */
static const float end_speed = 40.0f; /* Avoid the former 5 mm/s crawl at every cell. */
void fw_motion_rotation_profile(const fw_rotation_data_t *data)
{
    if (active) return;
    if (fw_rotation_valid(data)) rotation_profile=*data;
    else memset(&rotation_profile,0,sizeof rotation_profile);
}
void fw_motion_geometry(uint32_t pitch,uint32_t front,uint32_t inner)
{
    if (active) return;
    cell_pitch_um=pitch?pitch:CELL_LENGTH*1000u;
    front_calibrated=front && inner;
    front_allowance_um=front>inner/2?front-inner/2:0;
}
int32_t fw_motion_travelled_um(void)
{
    uint32_t remaining=fw_motion_remaining();
    return travel_sign*(int32_t)lroundf((float)(travel_ticks-remaining)*1000.0f/TICKS_PER_MM);
}
void fw_motion_init(void)
{
    fw_motion_stop(); fault=0; wall_arrival=0; velocity=0; last_scan=0;
    wall_control_reset(&wall);
}
int fw_motion_busy(void) { return active; }
int fw_motion_fault(void) { return fault; }
int fw_motion_wall_arrival(void) { return wall_arrival; }
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
    velocity=0;
    TIM_Cmd(TIM5,ENABLE);
}
static int start(long right, long left, unsigned speed, int is_straight, int accept_wall, int calibrating)
{
    hal_sensor_snapshot scan;
    if (active || fault || !hal_sensor_snapshot_read(&scan) ||
        (uint32_t)(hal_os_get_systicks()-scan.timestamp)>50) return -1;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    TIM_Cmd(TIM5,DISABLE); /* New controller exclusively owns speed corrections. */
    straight=is_straight; allow_wall=accept_wall; wall_arrival=0;
    calibration=calibrating;
    cruise=(float)speed; velocity=fminf(calibration==1?5.0f:end_speed,cruise);
    started=hal_os_get_systicks();
    wall_control_reset(&wall); last_scan=scan.sequence;
    hal_step_motor_pair_start(right,left);
    travel_ticks=fw_motion_remaining(); travel_sign=right<0?-1:1;
    hal_step_motor_wakeup(); hal_step_motor_enable();
    active=1;
    __set_PRIMASK(mask);
    return 0;
}
int fw_motion_straight(unsigned cells, unsigned speed)
{
    return fw_motion_straight_to(cells,speed,0);
}
int fw_motion_straight_to(unsigned cells, unsigned speed, int accept_wall)
{
    if (!cells || cells>9 || speed<20 || speed>300) return -1;
    long pulses=lroundf((float)cells*(float)cell_pitch_um*0.001f*TICKS_PER_MM);
    return start(pulses,pulses,speed,1,accept_wall,0);
}
static int turn(int degrees,int calibrating)
{
    if (degrees!=90 && degrees!=-90 && degrees!=180 && degrees!=-180) return -1;
    unsigned speed=calibrating?40:120;
    uint32_t quarter=fw_rotation_quarter(&rotation_profile,speed,degrees<0);
    float distance=quarter?(float)quarter*0.001f*fabsf((float)degrees)/90.0f:
        fabsf((float)degrees)*((float)M_PI/180.0f)*((float)WHEELS_DISTANCE/2.0f);
    long ticks=lroundf(distance*TICKS_PER_MM);
    /* Motor zero is the right wheel. Clockwise: right backwards, left forwards. */
    return start(degrees>0 ? -ticks : ticks, degrees>0 ? ticks : -ticks,speed,0,0,0);
}
int fw_motion_turn(int degrees) { return turn(degrees,0); }
int fw_motion_calibration_turn(int degrees) { return turn(degrees,1); }
int fw_motion_calibration_move(int32_t um,unsigned speed)
{
    if (!um || um<-180000 || um>180000 || speed<5 || speed>30) return -1;
    long pulses=lroundf((float)um*0.001f*TICKS_PER_MM);
    return start(pulses,pulses,speed,0,0,1);
}
int fw_motion_calibration_traverse(int32_t um,unsigned speed)
{
    if (!um || um<-300000 || um>300000 || speed<40 || speed>220) return -1;
    long pulses=lroundf((float)um*0.001f*TICKS_PER_MM);
    return start(pulses,pulses,speed,0,0,2);
}
int fw_motion_calibration_spin(int32_t um,unsigned speed)
{
    if (!um || um<-1600000 || um>1600000 || speed<40 || speed>120) return -1;
    long pulses=lroundf((float)um*0.001f*TICKS_PER_MM);
    int r=start(-pulses,pulses,speed,0,0,2);
    if (!r) travel_sign=um<0?-1:1;
    return r;
}
void fw_motion_tick(uint32_t now)
{
    if (!active) return;
    hal_sensor_snapshot scan;
    if (!hal_sensor_snapshot_read(&scan) || now-scan.timestamp>50 || now-started>60000) {
        fault=1; fw_motion_stop(); return;
    }
    uint32_t remaining=fw_motion_remaining();
    if (!remaining) {
        active=0; velocity=0; hal_step_motor_pair_rate(0,0); return;
    }
    if (straight && !(scan.raw & SENSOR_F5_POS)) {
        /* A front wall at the final cell centre is an arrival, not a failed maze.
         * Never accept an obstacle in the middle of a corridor or before a known opening. */
        if (front_calibrated && allow_wall &&
            (float)remaining/TICKS_PER_MM<=(float)front_allowance_um*0.001f+5.0f) {
            /* Measured trigger can occur before the centre: finish the pulse
             * budget instead of labelling that early position as a cell centre. */
            wall_arrival=1;
        } else if (!front_calibrated && allow_wall && (float)remaining/TICKS_PER_MM<=30.0f) {
            hal_step_motor_pair_rate(0,0); active=0; velocity=0; wall_arrival=1;
        } else { fault=2; fw_motion_stop(); }
        if (!active) return;
    }
    if (scan.sequence!=last_scan) {
        last_scan=scan.sequence;
        if (straight) wall_control_calibrated(&wall,scan.filtered,&wall_profile);
    }
    float distance=(float)remaining/TICKS_PER_MM;
    /* Braking-distance envelope with bounded acceleration; no proportional
     * asymptotic tail and no speed-dependent discontinuity near the endpoint. */
    float accel=calibration==1?100.0f:max_accel;
    float floor=fminf(calibration==1?5.0f:end_speed,cruise);
    float target=fminf(cruise,sqrtf(floor*floor+2.0f*accel*distance));
    float change=target-velocity, step=accel*0.001f;
    if (change>step) change=step;
    if (change<-step) change=-step;
    velocity+=change;
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
