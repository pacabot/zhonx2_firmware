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
static int disabled,trace_path;
static double path_x,path_y,path_heading;
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
unsigned long hal_step_motor_pair_count(unsigned int i) {return total[i];}
void hal_step_motor_pair_extend(unsigned long n) {remain[0]+=n;remain[1]+=n;}
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
    fw_motion_geometry(0,0,0);
    fw_motion_rotation_profile(0);
    fw_motion_wall_profile(0);fw_motion_corner_profiles(0,0);
}
static void tick(int fresh)
{
    double wheel[2]={0,0};
    for(unsigned i=0;i<2;++i) {
        fraction[i]+=(double)rates[i]/1000.;
        unsigned long pulses=(unsigned long)fraction[i]; fraction[i]-=pulses;
        if(pulses>remain[i]) pulses=remain[i];
        remain[i]-=pulses; total[i]+=pulses;
        wheel[i]=(commanded[i]<0?-1.0:1.0)*pulses/(2.0*STEPS_PER_MM);
    }
    if(trace_path) {
        double yaw=(wheel[1]-wheel[0])/WHEELS_DISTANCE;
        path_x+=(wheel[0]+wheel[1])*0.5*sin(path_heading+yaw/2);
        path_y+=(wheel[0]+wheel[1])*0.5*cos(path_heading+yaw/2);
        path_heading+=yaw;
        /* Closed outside walls of the standard 167 mm L-shaped corridor. */
        for(int a=-1;a<=1;a+=2)for(int b=-1;b<=1;b+=2) {
            double x=path_x+a*47*cos(path_heading)+b*47*sin(path_heading);
            double y=path_y-a*47*sin(path_heading)+b*47*cos(path_heading);
            assert(fabs(x)<262.5 && y<262.5 && y> -83.5);
            if(path_heading>=0) {assert(x> -83.5);if(y<83.5)assert(x<83.5);}
            else {assert(x<83.5);if(y<83.5)assert(x> -83.5);}
        }
    }
    ++now;
    if(fresh && now%10==0) { scan.timestamp=now; ++scan.sequence; }
    fw_motion_tick(now);
}
int main(void)
{
    unsigned previous_elapsed=20000;
    for(unsigned speed=20;speed<=300;speed+=20) {
        setup(); assert(!fw_motion_straight(1,speed));
        assert(commanded[0]>=lroundf((float)CELL_LENGTH*2.f*(float)STEPS_PER_MM));
        assert(commanded[0]==commanded[1]);
        unsigned elapsed=0;
        while(fw_motion_busy() && elapsed++<20000) tick(1);
        assert(!fw_motion_busy() && !fw_motion_fault() && elapsed<20000);
        assert(elapsed<previous_elapsed); previous_elapsed=elapsed;
        if (speed==220) { assert(elapsed<1250); printf("178 mm at 220 mm/s: %u ms\n",elapsed); }
        assert(total[0]==total[1] && abs(fw_motion_travelled_um()-CELL_LENGTH*1000)<400);
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
    int32_t stopped_at=fw_motion_travelled_um();
    for(int i=0;i<30;++i)tick(1);
    assert(fw_motion_travelled_um()==stopped_at);
    assert(fw_motion_obstacle_backoff(stopped_at+1));
    assert(fw_motion_obstacle_backoff(1000)); /* Raw alone is insufficient. */
    scan.filtered&=~SENSOR_F5_POS;
    assert(!fw_motion_obstacle_backoff(stopped_at));
    assert(commanded[0]<0 && commanded[0]==commanded[1]);
    while(fw_motion_busy())tick(1);
    assert(!fw_motion_fault() && abs(fw_motion_travelled_um()+stopped_at)<50);
    assert(fw_motion_obstacle_backoff(0)); /* No arbitrary fault clearing. */
    setup(); assert(!fw_motion_straight(1,100));
    for(int i=0;i<60;++i) tick(0);
    assert(fw_motion_fault()==1 && !fw_motion_busy() && disabled);
    assert(fw_motion_obstacle_backoff(0));
    /* Retreat freshness watchdog and zero-distance acknowledgement. */
    setup();assert(!fw_motion_straight_to(1,220,0));
    while(fw_motion_travelled_um()<100000)tick(1);
    scan.raw=scan.filtered=0x3f & ~SENSOR_F5_POS;tick(1);
    assert(!fw_motion_obstacle_backoff(100000));
    for(int i=0;i<60;++i)tick(0);
    assert(fw_motion_fault()==1 && disabled && !fw_motion_busy());
    setup();assert(!fw_motion_straight_to(1,220,0));
    scan.raw=scan.filtered=0x3f & ~SENSOR_F5_POS;tick(1);
    assert(fw_motion_fault()==2 && fw_motion_travelled_um()==0);
    assert(!fw_motion_obstacle_backoff(0) && !fw_motion_busy() && !fw_motion_fault());
    /* Ordinary final-cell wall detection must allow the next turn. */
    setup(); assert(!fw_motion_straight_to(1,220,1));
    while(fw_motion_remaining()>25.f*2.f*(float)STEPS_PER_MM) tick(1);
    scan.raw &= ~SENSOR_F5_POS;
    tick(1); assert(!fw_motion_fault() && !fw_motion_busy() && fw_motion_wall_arrival());
    assert(!rates[0] && !rates[1]);
    assert(!fw_motion_turn(90));
    while(fw_motion_busy()) tick(1);
    assert(!fw_motion_fault() && !fw_motion_wall_arrival());
    /* An early obstacle, or a wall across a known passage, must still fault. */
    setup(); assert(!fw_motion_straight_to(2,220,1));
    for(int i=0;i<100;++i) tick(1);
    scan.raw &= ~SENSOR_F5_POS; tick(1); assert(fw_motion_fault()==2);
    setup(); assert(!fw_motion_straight_to(1,220,0));
    while(fw_motion_remaining()>20.f*2.f*(float)STEPS_PER_MM) tick(1);
    scan.raw &= ~SENSOR_F5_POS; tick(1); assert(fw_motion_fault()==2);
    setup(); assert(fw_motion_straight(0,100)); assert(fw_motion_straight(1,FW_RUN_MAX_SPEED+1));
    assert(fw_motion_turn(30));
    now=UINT32_MAX-30; scan.timestamp=now;
    assert(!fw_motion_straight(1,100));
    for(int i=0;i<100;++i) tick(1);
    assert(!fw_motion_fault()); fw_motion_stop();
    setup(); scan.raw &= ~SENSOR_F5_POS;
    assert(!fw_motion_calibration_move(-43500,10));
    assert(commanded[0]<0 && commanded[0]==commanded[1]);
    while(fw_motion_busy()) tick(1);
    assert(!fw_motion_fault() && abs(fw_motion_travelled_um()+43500)<30);
    assert(!fw_motion_calibration_move(5000,20));
    while(fw_motion_busy()) tick(1);
    assert(!fw_motion_fault() && abs(fw_motion_travelled_um()-5000)<30);
    assert(fw_motion_calibration_move(180001,20)); assert(fw_motion_calibration_move(5000,31));
    assert(!fw_motion_calibration_move(5000,20));
    for(int i=0;i<60;++i) tick(0);
    assert(fw_motion_fault()==1 && disabled);
    setup(); fw_motion_geometry(179000,95000,167000);
    assert(!fw_motion_straight_to(1,220,1));
    assert(commanded[0]>lroundf(179.f*2.f*(float)STEPS_PER_MM));
    while(fw_motion_remaining()>10.f*2.f*(float)STEPS_PER_MM) tick(1);
    scan.raw &= ~SENSOR_F5_POS; tick(1);
    assert(fw_motion_busy() && fw_motion_wall_arrival());
    while(fw_motion_busy()) tick(1);
    assert(!fw_motion_fault() && abs(fw_motion_travelled_um()-179000)<400);
    setup(); fw_motion_geometry(179000,95000,167000);
    assert(!fw_motion_straight_to(1,220,1));
    for(int i=0;i<100;++i) tick(1);
    scan.raw &= ~SENSOR_F5_POS; tick(1); assert(fw_motion_fault()==2);
    setup();
    fw_rotation_data_t profile={.valid=1,.geometry={47000,94000,167000,179000}};
    for(unsigned i=0;i<3;++i) profile.point[i]=(fw_rotation_point_t){
        .speed=40+40*i,.quarter_um={66000+100*i,65000+100*i}};
    fw_motion_rotation_profile(&profile);
    assert(!fw_motion_turn(90));
    assert(commanded[0]==-lroundf(66.2f*2.f*(float)STEPS_PER_MM));
    while(fw_motion_busy()) tick(1);
    assert(!fw_motion_turn(-180));
    assert(commanded[0]==lroundf(130.4f*2.f*(float)STEPS_PER_MM));
    while(fw_motion_busy()) tick(1);
    assert(!fw_motion_calibration_turn(-90));
    assert(commanded[0]==lroundf(65.1f*2.f*(float)STEPS_PER_MM));
    while(fw_motion_busy()) tick(1);
    for(int dir=-1;dir<=1;dir+=2) {
        assert(!fw_motion_calibration_spin(dir*67000,80));
        while(fw_motion_busy()) tick(1);
        assert(abs(fw_motion_travelled_um()-dir*67000)<30);
    }
    assert(!fw_motion_calibration_traverse(-200000,220));
    while(fw_motion_busy()) tick(1);
    assert(abs(fw_motion_travelled_um()+200000)<30 && !fw_motion_fault());
    assert(fw_motion_calibration_traverse(300001,220));
    assert(fw_motion_calibration_spin(1600001,120));
    for(int wheel=0;wheel<3;++wheel)for(int sign=-1;sign<=1;sign+=2) {
        setup();
        assert(!fw_motion_test_wheels(wheel==0?0:sign,wheel==1?0:sign));
        long budget=lroundf(20.f*2.f*(float)STEPS_PER_MM);
        assert(commanded[0]==(wheel==0?0:sign*budget));
        assert(commanded[1]==(wheel==1?0:sign*budget));
        while(fw_motion_busy())tick(1);
        assert(!fw_motion_fault());fw_motion_stop();assert(disabled);
    }
    setup();assert(fw_motion_test_wheels(0,0));assert(fw_motion_test_wheels(2,1));
    assert(!fw_motion_test_wheels(1,0));
    for(int i=0;i<60;++i)tick(0);
    assert(fw_motion_fault()==1 && disabled);
    /* Steering must change wheel travel, not be cancelled by equal endpoints. */
    setup();
    fw_cal_data_t walls={.valid=1,.repetitions=3,.geometry={47000,94000,167000,179000},
      .front={{92000,92500,400},{132000,132500,200}},.side={{84500,85500,0},{81500,83500,1000}}};
    fw_motion_geometry(179000,92000,167000);fw_motion_wall_profile(&walls);scan.raw=scan.filtered=0x1c; /* Left clear, right near: steer left. */
    assert(!fw_motion_straight_to(1,220,1));
    while(fw_motion_busy())tick(1);
    assert(!fw_motion_fault() && total[0]>total[1]);
    assert(abs(fw_motion_travelled_um()-179000)<400);
    setup();fw_motion_geometry(179000,92000,167000);
    assert(!fw_motion_straight_to(1,220,1));
    while(fw_motion_travelled_um()<140000)tick(1);
    unsigned before=rates[0];
    assert(!fw_motion_extend(1,1) && fw_motion_busy() && rates[0]==before);
    while(fw_motion_travelled_um()<200000) {tick(1);assert(rates[0] && rates[1]);}
    while(fw_motion_busy())tick(1);
    assert(abs(fw_motion_travelled_um()-358000)<400 && !fw_motion_fault());
    assert(fw_motion_extend(1,1));
    /* A calibrated forward post transition removes a 3 mm longitudinal drift. */
    setup();fw_motion_geometry(179000,92000,167000);
    fw_corner_data_t corners[2];
    for(unsigned side=0;side<2;++side) {
        corners[side]=(fw_corner_data_t){.valid=1,.side=side,.post_um=173000,.geometry=walls.geometry};
        for(unsigned facing=0;facing<2;++facing)for(unsigned i=0;i<3;++i)
            corners[side].point[facing][i]=(fw_corner_point_t){.speed=i==0?40:i==1?120:220,.mask=2,
                .raw_open_um={0,facing?-32000:45000},.raw_close_um={0,facing?-30000:43000},
                .open_um={0,facing?-30000:47000},.close_um={0,facing?-32000:41000}};
    }
    fw_motion_corner_profiles(&corners[0],&corners[1]);scan.raw=scan.filtered=0x1f;
    assert(!fw_motion_straight_to(1,120,1));
    while(fw_motion_travelled_um()<62500)tick(1);
    scan.raw=scan.filtered=0x3f;
    for(unsigned i=0;i<10;++i)tick(1);
    int32_t counted=(int32_t)lround((total[0]+total[1])*500.0/(2.0*STEPS_PER_MM));
    int32_t trim=fw_motion_travelled_um()-counted;
    assert(trim< -2500 && trim> -5000);
    while(fw_motion_busy())tick(1);
    assert(abs(fw_motion_travelled_um()-179000)<400);
    /* Full differential-drive integration, both arc directions and speeds. */
    for(int dir=-1;dir<=1;dir+=2)for(unsigned speed=120;speed<=1000;speed+=440) {
        setup();fw_motion_geometry(179000,92000,167000);fw_motion_wall_profile(&walls);
        for(unsigned i=0;i<3;++i)profile.point[i]=(fw_rotation_point_t){.speed=40+40*i,
            .quarter_um={65581,65581}}; /* pi * 83.5 / 4, micrometres */
        fw_motion_rotation_profile(&profile);
        path_x=path_y=path_heading=0;trace_path=1;
        assert(!fw_motion_curve(dir*90,speed,0));
        int32_t last_distance=0;
        while(fw_motion_busy() && now<10000) {
            tick(1);
            if(fw_motion_busy())assert(rates[0] || rates[1]);
            int32_t distance=fw_motion_travelled_um();
            assert(distance>=last_distance && distance<321000);last_distance=distance;
        }
        assert(abs(last_distance-319586)<100);
        trace_path=0;
        assert(!fw_motion_busy() && !fw_motion_fault());
        assert(fabs(path_x-dir*179)<1 && fabs(path_y-179)<1);
        assert(fabs(path_heading-dir*M_PI/2)<0.01);
    }
    setup();assert(fw_motion_curve(90,220,0)); /* Missing profiles: pivot fallback. */
    fw_motion_geometry(179000,92000,167000);fw_motion_wall_profile(&walls);fw_motion_rotation_profile(&profile);
    assert(fw_motion_curve(180,220,0));assert(!fw_motion_curve(90,220,0));
    for(unsigned i=0;i<100;++i)tick(1);
    scan.raw&=~SENSOR_F5_POS;tick(1);assert(fw_motion_fault()==2 && disabled);
    setup();fw_motion_geometry(179000,92000,167000);fw_motion_wall_profile(&walls);fw_motion_rotation_profile(&profile);
    assert(!fw_motion_curve(-90,220,0));for(unsigned i=0;i<60;++i)tick(0);
    assert(fw_motion_fault()==1 && disabled);
    setup();assert(!fw_motion_straight(8,FW_RUN_MAX_SPEED));
    unsigned peak=0;
    while(fw_motion_busy()) {tick(1);if(fw_motion_speed()>peak)peak=fw_motion_speed();}
    assert(peak>=990 && !fw_motion_fault());
    /* Use an independently calibrated 5 cm post only after its matching 10 cm edge. */
    setup();fw_motion_geometry(179000,92000,167000);
    for(unsigned side=0;side<2;++side)for(unsigned f=0;f<2;++f)for(unsigned i=0;i<3;++i) {
        fw_corner_point_t *p=&corners[side].point[f][i];p->mask=3;
        p->raw_open_um[0]=f?-17000:35000;p->raw_close_um[0]=f?-19000:33000;
        p->open_um[0]=f?-15000:37000;p->close_um[0]=f?-21000:31000;
    }
    assert(fw_corner_valid(&corners[0]) && fw_corner_valid(&corners[1]));
    fw_motion_corner_profiles(&corners[0],&corners[1]);scan.raw=scan.filtered=0x0f;
    assert(!fw_motion_straight_to(1,120,1));
    while(fw_motion_travelled_um()<59500)tick(1);
    scan.raw=scan.filtered=0x2f;for(unsigned i=0;i<10;++i)tick(1);
    while((total[0]+total[1])*500.0/(2.0*STEPS_PER_MM)<78000)tick(1);
    scan.raw=scan.filtered=0x3f;for(unsigned i=0;i<10;++i)tick(1);
    counted=(int32_t)lround((total[0]+total[1])*500.0/(2.0*STEPS_PER_MM));
    trim=fw_motion_travelled_um()-counted;assert(trim< -2500 && trim> -5000);
    fw_motion_stop();
    setup();fw_motion_geometry(179000,92000,167000);fw_motion_wall_profile(&walls);
    assert(!fw_motion_straight_to(1,120,1));
    while(fw_motion_travelled_um()<134000)tick(1);
    scan.raw&=~SENSOR_F10_POS;tick(1);
    counted=(int32_t)lround((total[0]+total[1])*500.0/(2.0*STEPS_PER_MM));
    trim=fw_motion_travelled_um()-counted;assert(trim< -3000 && trim> -4500);
    fw_motion_stop();
    /* Reproduce the 2026-09-27 SRAM capture: four cells, F10 trim -6 mm,
     * F5 at ~14.467 mm remaining. The old 13.348 mm gate stopped here.
     * A credible edge must finish the calibrated 8.348 mm in forward motion;
     * an earlier obstacle or a known opening must still stop immediately. */
    for(unsigned mode=0;mode<4;++mode) {
        setup();fw_cal_data_t captured=walls;
        captured.front[0].on_um=91848;captured.front[0].off_um=92415;captured.front[0].spread_um=408;
        captured.front[1].on_um=131991;captured.front[1].off_um=132353;captured.front[1].spread_um=192;
        fw_motion_geometry(179000,91848,167000);fw_motion_wall_profile(&captured);
        assert(!fw_motion_straight_to(4,mode==1?600:300,mode!=3));
        while(fw_motion_travelled_um()<673509)tick(1);
        scan.raw&=~SENSOR_F10_POS;tick(1);
        int32_t edge_remaining=mode==1?22500:mode==2?26000:14467;
        while(fw_motion_remaining()>edge_remaining*0.001*2*STEPS_PER_MM)tick(1);
        scan.raw&=~SENSOR_F5_POS;tick(1);
        if(mode>=2) {assert(fw_motion_fault()==2 && !fw_motion_busy() && disabled);continue;}
        assert(!fw_motion_fault() && fw_motion_busy() && fw_motion_wall_arrival());
        unsigned long edge_count=total[0]+total[1];
        while(fw_motion_busy()) {assert(commanded[0]>0 && commanded[1]>0);tick(1);}
        double after_edge=(total[0]+total[1]-edge_count)/(4.0*STEPS_PER_MM);
        assert(fabs(after_edge-8.348)<0.15 && !fw_motion_fault());
        assert(abs(fw_motion_travelled_um()-716000)<150);
    }
    /* Position reference from the actual hysteresis edge; arbitrary initial
     * longitudinal offsets, both approach and release directions. */
    for(int offset=-12;offset<=12;offset+=6) {
        setup();fw_motion_geometry(179000,92000,167000);fw_motion_wall_profile(&walls);
        path_x=path_y=path_heading=0;trace_path=1;
        int near=83.5-offset<=92.0;
        scan.raw=scan.filtered=0x3f & ~SENSOR_F10_POS;
        if(near)scan.raw=scan.filtered=scan.raw & ~SENSOR_F5_POS;
        assert(!fw_motion_center_wall());
        while(fw_motion_busy() && now<5000) {
            double range=83.5-offset-path_y;
            if(range<=92.0)near=1;else if(range>=92.5)near=0;
            scan.raw=scan.filtered=0x3f & ~SENSOR_F10_POS;
            if(near)scan.raw=scan.filtered=scan.raw & ~SENSOR_F5_POS;
            tick(1);
        }
        trace_path=0;
        assert(!fw_motion_busy() && !fw_motion_fault() && fw_motion_centered());
        assert(fabs(path_y+offset)<0.7); /* Sensor sampling and pulse quantization. */
    }
    setup();fw_motion_wall_profile(&walls);scan.raw=scan.filtered=0x3f & ~SENSOR_F10_POS;
    path_x=path_y=path_heading=0;trace_path=1;
    assert(!fw_motion_center_wall()); /* False F10 wall, no F5 edge: return to initial pose. */
    while(fw_motion_busy() && now<5000)tick(1);
    trace_path=0;assert(!fw_motion_fault() && !fw_motion_centered() && fabs(path_y)<0.1);
    puts("motion: calibrated centring recovers +/-12 mm using on/off thresholds; false reference probe is undone");
    puts("motion: 1000 mm/s reachable; left/right arcs reach correct centre and heading, clear walls, stop on obstacles/stale scans");
    puts("motion: post offset removes longitudinal drift using the correct fixture and forward profile");
    puts("motion: steering changes wheel travel; appended cell crosses centre without stopped pulses");
    puts("motion: measured CW/CCW budgets, spin sign, free traverse and bounds");
    puts("motion: signed calibration travel, bounded contact, stale stop, calibrated final-cell budget");
    return 0;
}
