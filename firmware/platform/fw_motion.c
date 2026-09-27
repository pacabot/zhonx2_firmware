#include "fw_motion.h"
#include "fw_path.h"
#include "fw_sound.h"
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
static fw_path_t path;
static unsigned path_active,path_index;
static uint32_t path_count[2];
static float path_heading,path_trim;
static void path_tick(const hal_sensor_snapshot *scan);
static void front_tick(const hal_sensor_snapshot *scan);

static int straight, allow_wall, wall_arrival;
static int32_t obstacle_travel;
static int calibration, travel_sign;
static unsigned centre_phase,centre_count;
static int centre_direction,centre_ok;
static uint32_t centre_seen;
static int32_t centre_edge,centre_limit;
static void centre_tick(const hal_sensor_snapshot *scan);
static unsigned curve_phase,curve_speed;
static int curve_direction,curve_accept;
static float curve_ratio[2];
static uint32_t curve_budget[2];
static int32_t curve_completed_um;
static uint32_t travel_ticks,previous_count[2];
static volatile uint32_t goal_um;
static volatile int32_t longitudinal_um;
static float yaw_fraction;
static fw_corner_data_t corner_profile[2];
static uint8_t previous_sensors,previous_raw;
static int last_post[2][2];
static int32_t post_measurement[2][2];
static uint8_t post_raw_previous,post_open[2][2],side_raw_previous,side_stable[2];
static struct {int32_t position;unsigned speed;uint8_t valid,opening;} post_pending[2][2];
static int32_t raw_distance_um(void)
{
    return (int32_t)lroundf((hal_step_motor_pair_count(0)+hal_step_motor_pair_count(1))*500.0f/TICKS_PER_MM);
}
unsigned fw_motion_speed(void);

static uint32_t cell_pitch_um=CELL_LENGTH*1000u, front_allowance_um;
static int front_calibrated;
/* Two raw optical edges can validate an axle offset beyond a single sensor's
 * 15 mm window. Keep their raw travel independent of intermediate corrections. */
static int front_reference_valid;
static int32_t front_reference_raw,front_reference_goal,front_pair_error;
static unsigned front_reference_speed;
static fw_cal_data_t wall_profile;
static int32_t front_pair_window(void)
{
    int32_t body=(int32_t)wall_profile.geometry.width_um/2;
    if(body<(int32_t)wall_profile.geometry.nose_um)body=(int32_t)wall_profile.geometry.nose_um;
    int32_t room=(int32_t)wall_profile.geometry.inner_um/2-body-5000;
    return room<0?0:room<30000?room:30000;
}
static fw_rotation_data_t rotation_profile;
static float velocity, cruise;
static wall_control_t wall;
unsigned fw_motion_speed(void) {return active?(unsigned)velocity:0;}
int32_t fw_motion_lateral_um(void) {return wall.lateral_um;}
int32_t fw_motion_heading_mrad(void) {return wall.heading_mrad;}
void fw_motion_corner_profiles(const fw_corner_data_t *left,const fw_corner_data_t *right)
{
    if(active)return;
    const fw_corner_data_t *d[2]={left,right};
    for(unsigned i=0;i<2;++i) {
        if(fw_corner_valid(d[i]))corner_profile[i]=*d[i];
        else memset(&corner_profile[i],0,sizeof corner_profile[i]);
    }
}
static uint8_t control_sensors(const hal_sensor_snapshot *scan)
{
    uint8_t sensors=scan->filtered;wall.lag_scans=3;
    for(unsigned side=0;side<2;++side) {
        unsigned bit=side?SENSOR_R5_POS:SENSOR_L5_POS,wallbit=side?SENSOR_R10_POS:SENSOR_L10_POS;
        if((scan->raw^side_raw_previous)&(bit|wallbit))side_stable[side]=1;
        else if(side_stable[side]<3)++side_stable[side];
        /* Only accelerate 5 cm feedback beside an established wall. A doorway
         * or a one-scan spike must not be turned into a lateral correction. */
        if(side_stable[side]>=2 && !((scan->raw|scan->filtered)&wallbit)) {
            sensors=(uint8_t)((sensors&~bit)|(scan->raw&bit));wall.lag_scans=2;
        }
    }
    side_raw_previous=scan->raw;return sensors;
}
static void post_observation(const hal_sensor_snapshot *scan,int32_t raw)
{
    uint8_t sensors=scan->filtered;
    unsigned speed=fw_motion_speed();
    for(unsigned side=0;side<2;++side)for(int sensor=1;sensor>=0;--sensor) {
        unsigned bit=sensor?(side?SENSOR_R10_POS:SENSOR_L10_POS):(side?SENSOR_R5_POS:SENSOR_L5_POS);
        if((scan->raw^post_raw_previous)&bit) {
            post_pending[side][sensor].position=raw;post_pending[side][sensor].speed=speed;
            post_pending[side][sensor].opening=!!(scan->raw&bit);post_pending[side][sensor].valid=1;
        } else if(post_pending[side][sensor].valid && post_pending[side][sensor].valid<3)++post_pending[side][sensor].valid;
        /* The 10 cm edge can anchor a bend after two coherent raw scans.
         * Three-scan filtering remains the map/5 cm gate. A one-scan spike
         * never changes the travel reference. */
        int early=sensor && post_pending[side][sensor].valid==2 && ((scan->raw^previous_sensors)&bit);
        if(!early && !((sensors^previous_sensors)&bit))continue;
        int opening=!!((early?scan->raw:sensors)&bit);int32_t offset;
        /* Forward opening uses the opposite fixture after its 180-degree turn.
         * Forward closing uses the same fixture, traversed towards its wall. */
        int32_t edge=raw;
        if(post_pending[side][sensor].valid && post_pending[side][sensor].opening==opening) {
            if(fw_corner_raw_offset(&corner_profile[opening?1-side:side],opening?1:0,
                post_pending[side][sensor].speed,(unsigned)sensor,opening,&offset))continue;
            edge=post_pending[side][sensor].position;
        } else if(fw_corner_offset(&corner_profile[opening?1-side:side],opening?1:0,speed,(unsigned)sensor,opening,&offset))continue;
        if(!opening)offset=-offset;
        int32_t measured=edge-offset;
        int post=(measured+longitudinal_um)/(int32_t)cell_pitch_um;
        int32_t expected=(int32_t)(post*cell_pitch_um+cell_pitch_um/2);
        int32_t residual=expected-measured-longitudinal_um;
        unsigned edge_speed=post_pending[side][sensor].valid?post_pending[side][sensor].speed:speed;
        int32_t gate=15000+(edge_speed>220?(int32_t)(edge_speed-220)*5:0);
        /* At high speed a 10 ms raw sample interval adds position uncertainty.
         * Widen association only; a single edge still corrects at most 6 mm. */
        if(residual< -gate || residual>gate || post==last_post[side][sensor])continue;
        /* A 5 cm transition may simply be wall following. Use it as a post
         * only after a matching 10 cm post and with the same open/closed state. */
        if(!sensor && (last_post[side][1]!=post ||
           !!(sensors&(side?SENSOR_R10_POS:SENSOR_L10_POS))!=opening))continue;
        last_post[side][sensor]=post;post_measurement[side][sensor]=measured;post_open[side][sensor]=(uint8_t)opening;
        int paired=last_post[0][sensor]==last_post[1][sensor] && post_open[0][sensor]==post_open[1][sensor];
        if(paired)residual=expected-(post_measurement[0][sensor]+post_measurement[1][sensor])/2-longitudinal_um;
        /* A concordant pair supplies a mean position as well as yaw. */
        int32_t bound=paired?10000:6000;
        if(residual>bound)residual=bound;
        if(residual< -bound)residual= -bound;
        longitudinal_um+=residual;
        if(paired && wall_profile.valid) {
            int32_t yaw=(post_measurement[1][sensor]-post_measurement[0][sensor])*1000/(int32_t)wall_profile.geometry.inner_um;
            if(yaw>=-100 && yaw<=100)wall_control_heading_reference(&wall,yaw);
        }
    }
    previous_sensors=sensors;post_raw_previous=scan->raw;
}
void fw_motion_wall_profile(const fw_cal_data_t *d)
{
    if(active) return;
    if(fw_cal_valid(d)) wall_profile=*d;
    else memset(&wall_profile,0,sizeof wall_profile);
    wall_control_reset(&wall);
}
static uint32_t last_scan, last_scan_time, started;
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
    if(fault==2)return obstacle_travel;
    if(calibration==6)return raw_distance_um()+longitudinal_um;
    if(calibration==4)return curve_completed_um+raw_distance_um();
    if(straight && !calibration)return raw_distance_um()+longitudinal_um;
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
    if((straight && !calibration) || calibration==6) {
        int32_t left=(int32_t)goal_um-raw_distance_um()-longitudinal_um;
        return left>0?(uint32_t)lroundf(left*TICKS_PER_MM/1000.0f):0;
    }
    unsigned long a=hal_step_motor_pair_remaining(0), b=hal_step_motor_pair_remaining(1);
    return a>b ? a : b;
}
void fw_motion_stop(void)
{
    active=0;curve_phase=0;centre_phase=0;path_active=0;
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
    front_reference_valid=0;front_pair_error=0;
    calibration=calibrating;
    cruise=(float)speed; velocity=fminf(calibration==1?5.0f:end_speed,cruise);
    started=hal_os_get_systicks();
    int32_t bias=wall.yaw_bias_mrad_m;wall_control_reset(&wall);wall.yaw_bias_mrad_m=bias;
    side_raw_previous=scan.raw;memset(side_stable,0,sizeof side_stable);last_scan=scan.sequence;last_scan_time=scan.timestamp;previous_sensors=scan.filtered;previous_raw=scan.raw;post_raw_previous=scan.raw;memset(post_pending,0,sizeof post_pending);
    previous_count[0]=previous_count[1]=0;longitudinal_um=0;yaw_fraction=0;
    for(unsigned i=0;i<2;++i)for(unsigned j=0;j<2;++j)last_post[i][j]=-100;
    travel_ticks=(uint32_t)(right<0?-right:right);travel_sign=right<0?-1:1;
    goal_um=(uint32_t)lroundf(travel_ticks*1000.0f/TICKS_PER_MM);
    long reserve=straight?lroundf((25.0f+goal_um*0.00025f)*TICKS_PER_MM):0;
    hal_step_motor_pair_start(right+reserve,left+reserve);
    hal_step_motor_wakeup(); hal_step_motor_enable();
    active=1;
    __set_PRIMASK(mask);
    return 0;
}
int fw_motion_obstacle_backoff(uint32_t um)
{
    hal_sensor_snapshot scan;
    if(active || fault!=2 || !straight || um>cell_pitch_um ||
       obstacle_travel<0 || um>(uint32_t)obstacle_travel ||
       !hal_sensor_snapshot_read(&scan) ||
       (uint32_t)(hal_os_get_systicks()-scan.timestamp)>50 ||
       (scan.raw&SENSOR_F5_POS) || (scan.filtered&SENSOR_F5_POS))return -1;
    fault=0;
    if(!um) {wall_arrival=0;return 0;}
    long pulses=lroundf(um*0.001f*TICKS_PER_MM);
    /* Independent reverse mode: paired wheels, no front arrival/side observer.
     * The normal freshness watchdog remains active throughout the retreat. */
    if(start(-pulses,-pulses,80,0,0,3)) {fault=2;return -1;}
    return 0;
}
int fw_motion_centered(void) {return centre_ok;}
int fw_motion_center_wall(void)
{
    hal_sensor_snapshot scan;
    if(active || fault || !fw_cal_valid(&wall_profile) || !hal_sensor_snapshot_read(&scan) ||
       (uint32_t)(hal_os_get_systicks()-scan.timestamp)>50)return -1;
    if((scan.raw & scan.filtered & (SENSOR_F5_POS|SENSOR_F10_POS))==(SENSOR_F5_POS|SENSOR_F10_POS))return 1;
    int32_t half=(int32_t)wall_profile.geometry.inner_um/2;
    int32_t body=(int32_t)wall_profile.geometry.width_um/2;
    if(body<(int32_t)wall_profile.geometry.nose_um)body=(int32_t)wall_profile.geometry.nose_um;
    centre_limit=half-body-5000;if(centre_limit>25000)centre_limit=25000;
    int32_t offset=(int32_t)wall_profile.front[0].off_um-half;
    if(centre_limit<5000 || offset< -centre_limit || offset>centre_limit)return -1;
    centre_direction=(scan.raw&SENSOR_F5_POS)?1:-1;
    int32_t clearance=centre_direction>0?(int32_t)wall_profile.front[0].on_um-body-5000:
        (int32_t)wall_profile.geometry.inner_um-(int32_t)wall_profile.front[0].off_um-body-5000;
    clearance-=(int32_t)wall_profile.front[0].spread_um;
    if(centre_limit>clearance)centre_limit=clearance;
    if(centre_limit<5000)return -1;
    uint32_t mask=__get_PRIMASK();__disable_irq();
    centre_ok=0;
    long pulses=lroundf(centre_direction*centre_limit*TICKS_PER_MM/1000.0f);
    int result=start(pulses,pulses,40,0,0,5);
    if(!result) {centre_phase=1;centre_count=0;centre_seen=scan.sequence;}
    __set_PRIMASK(mask);return result;
}
static void centre_tick(const hal_sensor_snapshot *scan)
{
    if(centre_phase!=1 || scan->sequence==centre_seen)return;
    centre_seen=scan->sequence;
    int reached=centre_direction>0?!(scan->raw&SENSOR_F5_POS):!!(scan->raw&SENSOR_F5_POS);
    if(!reached) {centre_count=0;return;}
    if(!centre_count)centre_edge=fw_motion_travelled_um();
    if(++centre_count<3)return;
    int32_t travel=fw_motion_travelled_um();
    int32_t threshold=(int32_t)(centre_direction>0?wall_profile.front[0].on_um:wall_profile.front[0].off_um);
    int32_t target=centre_edge+threshold-(int32_t)wall_profile.geometry.inner_um/2;
    int32_t delta=target-travel;
    if(target< -15000 || target>15000 || delta< -2*centre_limit || delta>2*centre_limit) {
        fault=4;fw_motion_stop();return;
    }
    /* Read travel before release, which clears paired pulse budgets. */
    fw_motion_stop();
    if(delta>-100 && delta<100) {centre_ok=1;return;}
    long pulses=lroundf(delta*TICKS_PER_MM/1000.0f);
    if(start(pulses,pulses,40,0,0,5)) {fault=4;return;}
    centre_phase=2;
}
int fw_motion_test_wheels(int right,int left)
{
    if (right < -1 || right > 1 || left < -1 || left > 1 || (!right && !left)) return -1;
    long pulses=lroundf(20.0f*TICKS_PER_MM);
    return start(right*pulses,left*pulses,20,0,0,1);
}
int fw_motion_straight(unsigned cells, unsigned speed)
{
    return fw_motion_straight_to(cells,speed,0);
}
int fw_motion_straight_to(unsigned cells, unsigned speed, int accept_wall)
{
    if (!cells || cells>16 || speed<20 || speed>FW_RUN_MAX_SPEED) return -1;
    long pulses=lroundf((float)cells*(float)cell_pitch_um*0.001f*TICKS_PER_MM);
    return start(pulses,pulses,speed,1,accept_wall,0);
}
int fw_motion_extend(unsigned cells,int accept_wall)
{
    uint32_t mask=__get_PRIMASK();__disable_irq();
    if(!active || !straight || calibration || fault || !cells || cells>16 || !fw_motion_remaining()) {
        __set_PRIMASK(mask);return -1;
    }
    uint32_t extra=cells*cell_pitch_um;
    if(goal_um+extra>16u*cell_pitch_um) {__set_PRIMASK(mask);return -1;}
    goal_um+=extra;travel_ticks+=(uint32_t)lroundf(extra*TICKS_PER_MM/1000.0f);
    hal_step_motor_pair_extend((unsigned long)lroundf(extra*1.25f*TICKS_PER_MM/1000.0f));
    allow_wall=accept_wall;front_reference_valid=0;started=hal_os_get_systicks();
    __set_PRIMASK(mask);return 0;
}
static void curve_segment(unsigned phase)
{
    if(phase==1)curve_completed_um=0;
    else curve_completed_um+=raw_distance_um();
    float radius=cell_pitch_um*0.0005f;
    float distance[2]={radius,radius};
    if(phase==2) {
        /* Equivalent in-place wheel speed gives the calibrated yaw scale. */
        unsigned spin=(unsigned)(curve_speed*(float)WHEELS_DISTANCE/(2.0f*radius));
        if(spin<40)spin=40;
        if(spin>120)spin=120;
        float quarter=fw_rotation_quarter(&rotation_profile,spin,curve_direction<0)*0.001f;
        float arc=radius*(float)M_PI/2.0f;
        distance[0]=arc-curve_direction*quarter;
        distance[1]=arc+curve_direction*quarter;
    }
    float mean=(distance[0]+distance[1])/2.0f;
    for(unsigned i=0;i<2;++i) {
        curve_budget[i]=((uint32_t)lroundf(distance[i]*TICKS_PER_MM)+1u)&~1u;
        curve_ratio[i]=distance[i]/mean;
    }
    hal_step_motor_pair_start(curve_budget[0],curve_budget[1]);
    curve_phase=phase;
}
int fw_motion_curve(int degrees,unsigned speed,int accept)
{
    if(active || fault || (degrees!=90 && degrees!=-90) || speed<20 || speed>FW_RUN_MAX_SPEED ||
       !fw_cal_valid(&wall_profile) || !fw_rotation_valid(&rotation_profile))return -1;
    float r=cell_pitch_um*0.0005f,half=wall_profile.geometry.width_um*0.0005f;
    float nose=wall_profile.geometry.nose_um*0.001f,inside=wall_profile.geometry.inner_um*0.0005f;
    /* Swept square/rectangle must clear the outside walls and inside post. */
    if(sqrtf((r+half)*(r+half)+nose*nose)-r>inside-3.0f ||
       sqrtf((r+nose)*(r+nose)+half*half)-r>inside-3.0f || r-half<15.0f)return -1;
    uint32_t mask=__get_PRIMASK();__disable_irq();
    curve_speed=speed>FW_CURVE_MAX_SPEED?FW_CURVE_MAX_SPEED:speed;
    curve_direction=degrees>0?1:-1;curve_accept=accept;
    long pulses=lroundf(r*TICKS_PER_MM);
    int result=start(pulses,pulses,curve_speed,0,0,4);
    if(!result)curve_segment(1);
    __set_PRIMASK(mask);return result;
}
static void curve_tick(const hal_sensor_snapshot *scan)
{
    unsigned long remaining[2]={hal_step_motor_pair_remaining(0),hal_step_motor_pair_remaining(1)};
    if(!remaining[0] && !remaining[1]) {
        if(curve_phase==3) {active=0;curve_phase=0;centre_phase=0;velocity=0;hal_step_motor_pair_rate(0,0);return;}
        curve_segment(curve_phase+1);
        remaining[0]=curve_budget[0];remaining[1]=curve_budget[1];
    }
    float left=((float)remaining[0]/curve_ratio[0]+(float)remaining[1]/curve_ratio[1])/(2.0f*TICKS_PER_MM);
    if(!(scan->raw&SENSOR_F5_POS)) {
        if(curve_phase==3 && curve_accept && left<=front_allowance_um*0.001f+5.0f)wall_arrival=1;
        else {obstacle_travel=0;fault=2;fw_motion_stop();return;}
    }
    float target=curve_phase==3?fminf(cruise,sqrtf(end_speed*end_speed+2*max_accel*left)):cruise;
    float change=target-velocity,step=max_accel*0.001f;
    velocity+=fmaxf(-step,fminf(step,change));
    /* Synchronize fractional progress, preserving the inner/outer radius ratio. */
    float progress0=(curve_budget[0]-remaining[0])/curve_ratio[0];
    float progress1=(curve_budget[1]-remaining[1])/curve_ratio[1];
    float trim=fmaxf(-0.04f,fminf(0.04f,(progress0-progress1)*0.002f));
    unsigned long base=(unsigned long)(velocity*TICKS_PER_MM);
    hal_step_motor_pair_rate((unsigned long)(base*curve_ratio[0]*(1-trim)),
                             (unsigned long)(base*curve_ratio[1]*(1+trim)));
}
static int turn(int degrees,int calibrating)
{
    if (degrees!=90 && degrees!=-90 && degrees!=180 && degrees!=-180) return -1;
    unsigned speed=calibrating?FW_CAL_TURN_SPEED:120;
    uint32_t quarter=fw_rotation_quarter(&rotation_profile,speed,degrees<0);
    float angle=(float)degrees*(float)M_PI/180.0f;
    if(!calibrating && straight && wall_profile.valid)angle-=wall.heading_mrad*0.001f;
    float distance=quarter?(float)quarter*0.001f*fabsf(angle)/((float)M_PI/2.0f):
        fabsf(angle)*((float)WHEELS_DISTANCE/2.0f);
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
int fw_motion_seat_wall(void)
{
    hal_sensor_snapshot scan;
    if(!fw_cal_valid(&wall_profile) || !hal_sensor_snapshot_read(&scan) ||
       (uint32_t)(hal_os_get_systicks()-scan.timestamp)>50 ||
       ((scan.raw|scan.filtered)&SENSOR_F10_POS))return -1;
    int32_t gap=(int32_t)wall_profile.geometry.inner_um/2-(int32_t)wall_profile.geometry.nose_um;
    /* At the known start only: 20 mm covers residual pose error and the
     * deliberate overtravel. Keep the entire contact approach at 30 mm/s. */
    return fw_motion_calibration_move(gap+20000,FW_CAL_CONTACT_SPEED);
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
static void front_tick(const hal_sensor_snapshot *scan)
{
    uint32_t remaining=fw_motion_remaining();
    if(straight && (scan->raw&SENSOR_F10_POS))front_reference_valid=0;
    if(straight && allow_wall && wall_profile.valid &&
       (previous_raw&SENSOR_F10_POS) && !(scan->raw&SENSOR_F10_POS)) {
        int32_t allowance=(int32_t)wall_profile.front[1].on_um-(int32_t)wall_profile.geometry.inner_um/2;
        int32_t raw=raw_distance_um();
        int32_t residual=(int32_t)goal_um-allowance-raw-longitudinal_um;
        int32_t window=front_pair_window();
        if((scan->raw&SENSOR_F5_POS) && residual>=-window && residual<=window) {
            front_reference_raw=raw;front_reference_goal=raw+allowance;
            front_reference_speed=fw_motion_speed();front_reference_valid=1;
        }
        if(residual>=-15000 && residual<=15000) {
            if(residual>6000)residual=6000;
            if(residual< -6000)residual=-6000;
            longitudinal_um+=residual;
        }
    }
    if (straight && !(scan->raw & SENSOR_F5_POS)) {
        int32_t residual=(int32_t)goal_um-(int32_t)front_allowance_um-raw_distance_um()-longitudinal_um;
        int paired=0;
        if(front_reference_valid && wall_profile.valid && (previous_raw&SENSOR_F5_POS) &&
           !(scan->raw&SENSOR_F10_POS) && !(scan->filtered&SENSOR_F10_POS)) {
            int32_t expected=(int32_t)wall_profile.front[1].on_um-(int32_t)wall_profile.front[0].on_um;
            front_pair_error=raw_distance_um()-front_reference_raw-expected;
            unsigned speed=fw_motion_speed();if(speed<front_reference_speed)speed=front_reference_speed;
            int32_t tolerance=2000+(int32_t)(wall_profile.front[0].spread_um+wall_profile.front[1].spread_um+speed*20);
            if(tolerance>8000)tolerance=8000;
            int32_t window=front_pair_window();
            paired=expected>0 && front_pair_error>=-tolerance && front_pair_error<=tolerance &&
                   residual>=-window && residual<=window;
        }
        /* A front wall at the final cell centre is an arrival, not a failed maze.
         * Never accept an obstacle in the middle of a corridor or before a known opening. */
        if (front_calibrated && allow_wall &&
            (paired || (float)remaining/TICKS_PER_MM<=(float)front_allowance_um*0.001f+
                (wall_profile.valid?15.0f:5.0f))) {
            /* Match the 15 mm reference window used for F10 and posts. The old
             * 5 mm gate faulted on a valid F5 edge after a bounded F10 correction.
             * Use that calibrated edge to finish at the centre without reversing. */
            wall_arrival=1;front_reference_valid=0;
            if(wall_profile.valid && (previous_raw&SENSOR_F5_POS)) {
                if(paired || (residual>=-15000 && residual<=15000)) {
                    longitudinal_um+=residual;
                }
            }
        } else if (!front_calibrated && allow_wall && (float)remaining/TICKS_PER_MM<=30.0f) {
            hal_step_motor_pair_rate(0,0); active=0; velocity=0; wall_arrival=1;
        } else { obstacle_travel=fw_motion_travelled_um(); fault=2; fw_motion_stop(); }
        if (!active) return;
    }
    previous_raw=scan->raw;
}
void fw_motion_tick(uint32_t now)
{
    fw_sound_tick(now);
    if (!active) return;
    hal_sensor_snapshot scan;
    if (!hal_sensor_snapshot_read(&scan) || now-scan.timestamp>50 || now-started>(path_active?600000u:60000u)) {
        fault=1; fw_motion_stop(); return;
    }
    if(path_active) {path_tick(&scan);return;}
    if(curve_phase) {curve_tick(&scan);return;}
    if(centre_phase==1) {centre_tick(&scan);if(!active)return;}
    uint32_t remaining=fw_motion_remaining();
    if (!remaining) {
        if(centre_phase==1) {
            /* No edge: undo exactly the measured probe rather than assuming
             * that a suspect mapped wall supplied a valid position reference. */
            int32_t travel=fw_motion_travelled_um();fw_motion_stop();
            long pulses=lroundf(-travel*TICKS_PER_MM/1000.0f);
            if(start(pulses,pulses,40,0,0,5)) {fault=4;return;}
            centre_phase=3;return;
        }
        if(centre_phase==2)centre_ok=1;
        centre_phase=0;
        active=0; velocity=0; hal_step_motor_pair_rate(0,0); return;
    }
    front_tick(&scan);if(!active)return;
    remaining=fw_motion_remaining(); /* Include this scan's frontal position reference. */
    if (scan.sequence!=last_scan) {
        uint32_t elapsed=scan.timestamp-last_scan_time;last_scan_time=scan.timestamp;
        last_scan=scan.sequence;
        if(straight) {
            uint32_t count[2]={hal_step_motor_pair_count(0),hal_step_motor_pair_count(1)};
            int32_t dr=(int32_t)(count[0]-previous_count[0]),dl=(int32_t)(count[1]-previous_count[1]);
            previous_count[0]=count[0];previous_count[1]=count[1];
            uint32_t q=rotation_profile.valid?rotation_profile.point[2].quarter_um[0]:0;
            float track=q?q*4.0f/3.14159265359f:(float)WHEELS_DISTANCE*1000.0f;
            yaw_fraction+=(dl-dr)*1000000.0f/(TICKS_PER_MM*track);
            int32_t yaw=(int32_t)lroundf(yaw_fraction);yaw_fraction-=yaw;
            int32_t forward=(int32_t)lroundf((dl+dr)*500.0f/TICKS_PER_MM);
            post_observation(&scan,raw_distance_um());
            wall_control_position_timed(&wall,control_sensors(&scan),&wall_profile,forward,yaw,elapsed);
        }
    }
    float distance=(float)remaining/TICKS_PER_MM;
    if(straight && front_reference_valid) {
        /* F10 may shorten braking distance before F5 validates the position.
         * It never authorizes extra travel or suppresses obstacle protection. */
        float optical_left=(front_reference_goal-raw_distance_um())*0.001f;
        if(optical_left<distance)distance=fmaxf(0.0f,optical_left);
    }
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
    /* Keep paired calibration/turn moves synchronized. Straight moves terminate
     * on mean axle travel: equal wheel budgets would undo steering corrections. */
    long r=(long)hal_step_motor_pair_remaining(0), l=(long)hal_step_motor_pair_remaining(1);
    long skew=straight?0:(r-l)*2;
    if (skew>40) skew=40;
    if (skew<-40) skew=-40;
    correction-=(int)skew;
    hal_step_motor_pair_rate(base*(1000-correction)/1000,base*(1000+correction)/1000);
}

static float wrap_angle(float angle)
{
    while(angle>3.14159265f)angle-=6.28318531f;
    while(angle< -3.14159265f)angle+=6.28318531f;
    return angle;
}
int fw_motion_path(const nm_map_t *map,nm_pose_t pose,const nm_route_t *route,unsigned speed)
{
    if(active || fault || !fw_cal_valid(&wall_profile) || !fw_rotation_valid(&rotation_profile) ||
       fw_path_plan(&path,map,pose,route,&wall_profile.geometry,speed))return -1;
    uint32_t mask=__get_PRIMASK();__disable_irq();
    /* Reserve wheel travel once. Neither arcs nor joins reset timers/counters. */
    long budget=lroundf(path.length*.0017f*TICKS_PER_MM);
    int result=start(budget,budget,speed,0,0,6);
    if(!result) {
        goal_um=(uint32_t)lroundf(path.length);path_index=0;path_active=1;
        path_count[0]=path_count[1]=0;path_heading=pose.heading*3.14159265f/2;path_trim=0;
        straight=1;
        int c=path.end.y*NM_SIDE+path.end.x;
        curve_accept=!!(map->cell[c].walls&(1u<<path.end.heading));
    }
    __set_PRIMASK(mask);return result;
}
nm_pose_t fw_motion_path_pose(void)
{
    if(!path.count || (!path_active && !fault))return path.end;
    fw_path_point_t point;fw_path_point(&path,path_index,fw_motion_travelled_um()-path.segment[path_index].start,&point);
    int h=(int)lroundf(point.heading/(3.14159265f/2));h=(h%4+4)%4;
    int x=(int)lroundf(point.x/cell_pitch_um),y=(int)lroundf(point.y/cell_pitch_um);
    return (nm_pose_t){x<0?0:x>=NM_SIDE?NM_SIDE-1:x,y<0?0:y>=NM_SIDE?NM_SIDE-1:y,h};
}
static void path_tick(const hal_sensor_snapshot *scan)
{
    /* Observe the outgoing straight before choosing its successor. A post
     * correction on the last scan must move the bend entry, not arrive too late. */
    fw_path_segment_t *outgoing=&path.segment[path_index];
    if(scan->sequence!=last_scan && !outgoing->turn) {
        int32_t before=wall.heading_mrad;
        post_observation(scan,raw_distance_um()-(int32_t)outgoing->start+outgoing->phase_um);
        path_heading+=(wall.heading_mrad-before)*.001f;
    }
    float distance=(float)fw_motion_travelled_um();
    while(path_index+1<path.count && distance>=path.segment[path_index].start+path.segment[path_index].length) {
        ++path_index;front_reference_valid=0;
        int32_t bias=wall.yaw_bias_mrad_m;wall_control_reset(&wall);wall.yaw_bias_mrad_m=bias;
        side_raw_previous=scan->raw;memset(side_stable,0,sizeof side_stable);
        previous_raw=scan->raw;previous_sensors=scan->filtered;post_raw_previous=scan->raw;memset(post_pending,0,sizeof post_pending);
        for(unsigned a=0;a<2;++a)for(unsigned b=0;b<2;++b)last_post[a][b]=-100;
    }
    fw_path_segment_t *s=&path.segment[path_index];
    fw_path_point_t point;fw_path_point(&path,path_index,distance-s->start,&point);
    straight=!s->turn;allow_wall=path_index+1==path.count && curve_accept;
    uint32_t count[2]={hal_step_motor_pair_count(0),hal_step_motor_pair_count(1)};
    int32_t dr=(int32_t)(count[0]-path_count[0]),dl=(int32_t)(count[1]-path_count[1]);
    path_count[0]=count[0];path_count[1]=count[1];
    unsigned spin=(unsigned)(velocity*fabsf(point.curvature)*WHEELS_DISTANCE*500);
    if(spin<40)spin=40;
    if(spin>120)spin=120;
    float quarter=(float)fw_rotation_quarter(&rotation_profile,spin,s->turn<0);
    float track=quarter*4/3.14159265f;
    path_heading+=(dl-dr)*1000.0f/(TICKS_PER_MM*track);
    if(straight && !wall.initialized) {
        wall_control_exit(&wall,&wall_profile,scan->filtered,
            (int32_t)lroundf(wrap_angle(path_heading-point.heading)*1000),path_index?cell_pitch_um/2:0);
        previous_count[0]=count[0];previous_count[1]=count[1];last_scan=scan->sequence;last_scan_time=scan->timestamp;
    }
    if(straight)front_tick(scan);
    else if(!(scan->raw&SENSOR_F5_POS)) {obstacle_travel=fw_motion_travelled_um();fault=2;fw_motion_stop();}
    if(!active)return;
    if(scan->sequence!=last_scan) {
        uint32_t elapsed=scan->timestamp-last_scan_time;last_scan_time=scan->timestamp;
        last_scan=scan->sequence;
        int32_t r=(int32_t)(count[0]-previous_count[0]),l=(int32_t)(count[1]-previous_count[1]);
        previous_count[0]=count[0];previous_count[1]=count[1];
        if(straight) {
            int32_t before=wall.heading_mrad;
            yaw_fraction+=(l-r)*1000000.0f/(TICKS_PER_MM*track);
            int32_t yaw=(int32_t)lroundf(yaw_fraction);yaw_fraction-=yaw;
            wall_control_position_timed(&wall,control_sensors(scan),&wall_profile,
                (int32_t)lroundf((r+l)*500.0f/TICKS_PER_MM),yaw,elapsed);
            /* Retain sub-mrad wheel yaw; only optical innovations adjust the
             * floating heading. Replacing it by rounded state accumulated drift. */
            path_heading+=(wall.heading_mrad-before-yaw)*.001f;
        }
    }
    distance=(float)fw_motion_travelled_um();
    if(distance>=path.length) {active=path_active=0;velocity=0;hal_step_motor_pair_rate(0,0);return;}
    float left=fmaxf(0,(s->start+s->length-distance)*.001f);
    if(straight && front_reference_valid)left=fminf(left,fmaxf(0,(front_reference_goal-raw_distance_um())*.001f));
    float target=fminf(s->limit,sqrtf(s->exit_speed*s->exit_speed+2*path.acceleration*left));
    float change=target-velocity,step=path.acceleration*.001f;
    velocity+=fmaxf(-step,fminf(step,change));
    fw_path_point_t midpoint;fw_path_point(&path,path_index,distance-s->start+velocity*.5f,&midpoint);
    /* Close the heading loop between IR scans as well. The wall output
     * already contains scan-time yaw feedback: add only the newer wheel yaw. */
    float desired=straight?wall.output*.001f+2*(wall.heading_mrad*.001f-wrap_angle(path_heading-point.heading)):
        midpoint.curvature*track*.5f+2*wrap_angle(point.heading-path_heading);
    desired=fmaxf(-.85f,fminf(.85f,desired));
    path_trim+=fmaxf(-.008f,fminf(.008f,desired-path_trim));
    unsigned long base=(unsigned long)(velocity*TICKS_PER_MM);
    hal_step_motor_pair_rate((unsigned long)(base*(1-path_trim)),(unsigned long)(base*(1+path_trim)));
}
