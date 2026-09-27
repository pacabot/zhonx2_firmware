#include "fw_app.h"
#include "fw_snapshot.h"
#include "fw_battery.h"
#include "fw_motion.h"
#include "fw_flash.h"
#include "fw_ui.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "config/basetypes.h"
#include "config/config.h"
#include "app/app_def.h"
#include "hal/hal_os.h"
#include <assert.h>
#undef printf
#include <stdio.h>
#include <string.h>
#undef printf
GPIO_TypeDef test_gpioc;
USART_TypeDef test_usarts[6];
RCC_TypeDef test_rcc;
PWR_TypeDef test_pwr;
RTC_TypeDef test_rtc;
app_config app_context;
const fw_flash_t fw_stm32_flash={0};
static nm_map_t ground;
static nm_pose_t physical,destination;
static uint32_t now,deadline,move_started;
static nm_pose_t segment_start;
static unsigned moving_cells,extensions,maximum_requested_speed;
static int rotating,backing,recovered,confirm_missing;
static int32_t fault_travel;
static unsigned moves,turns,wall_arrivals,draws,saves,ack,curves;
static int busy,wall_arrival,stopped,inject_obstacle;
static char last_status[32];
static unsigned char saved_snapshot[16384];
static size_t saved_size;
static uint32_t saved_schema,applied_pitch;
static int save_failure,hidden_door_cleared,hide_after_recovery;
unsigned long hal_os_get_systicks(void) { return now; }
void NVIC_SystemReset(void) { assert(0); }
int hal_ui_display_prompt(HAL_UI_HANDLE h,const char *title,const char *str)
{ (void)h; fprintf(stderr,"Unexpected prompt: %s %s\n",title,str); assert(0); return -1; }
int fw_store_load(const fw_flash_t *f,uint32_t schema,void *data,size_t size)
{
    (void)f;
    if (schema!=saved_schema || size!=saved_size) return -1;
    memcpy(data,saved_snapshot,size); return 0;
}
int fw_store_save(const fw_flash_t *f,uint32_t schema,const void *data,size_t size)
{
    (void)f;assert(!busy);++saves;
    assert(nm_valid((const nm_map_t *)((const unsigned char *)data+sizeof(robot_settings))));
    if (save_failure) return -1;
    assert(size<=sizeof saved_snapshot); memcpy(saved_snapshot,data,size);
    saved_schema=schema; saved_size=size;
    return 0;
}
int hal_sensor_snapshot_read(hal_sensor_snapshot *s)
{
    nm_pose_t sensed=physical;
    if(stopped==2 && inject_obstacle==2) {
        sensed=segment_start;sensed.y+=2;
    }
    if(busy && !rotating && !backing) {
        sensed=segment_start;
        unsigned steps=((now-move_started)*179+39000)/179000;
        if(steps>moving_cells)steps=moving_cells;
        int c=sensed.y*NM_SIDE+sensed.x;
        while(steps--) {c=nm_next(&ground,c,sensed.heading);assert(c>=0);}
        sensed.x=c%NM_SIDE;sensed.y=c/NM_SIDE;
    }
    unsigned cell=sensed.y*NM_SIDE+sensed.x,wall=ground.cell[cell].walls;
    if((inject_obstacle==5 || inject_obstacle==6 || hide_after_recovery) && sensed.x==0 && sensed.y==2) {
        if(physical.heading==NM_EAST && (inject_obstacle==5 || hide_after_recovery))hidden_door_cleared=1;
        if(!hidden_door_cleared)wall|=1u<<NM_EAST;
    }
    uint8_t bits=0x3f;
    if(wall & 1u<<physical.heading) bits &= ~SENSOR_F10_POS;
    if(wall & 1u<<((physical.heading+3)%4)) bits &= ~SENSOR_L10_POS;
    if(wall & 1u<<((physical.heading+1)%4)) bits &= ~SENSOR_R10_POS;
    if(inject_obstacle==2 && !recovered && sensed.x==0 && sensed.y==2 && physical.heading==NM_NORTH)
        bits|=SENSOR_F10_POS; /* F10 misses the wall throughout the preview. */
    if(inject_obstacle==4 && busy && !rotating && sensed.x==0 && sensed.y==2 &&
       physical.heading==NM_NORTH && (now-move_started)*179<336000)
        bits|=SENSOR_F10_POS; /* Late trigger, 22 mm before centre: old preview was too early. */
    if(stopped==2 && inject_obstacle==2 && !confirm_missing)bits&=~SENSOR_F5_POS;
    if(now<300) bits &= ~SENSOR_F10_POS; /* The user's hand at the start. */
    *s=(hal_sensor_snapshot){now,now/10+5,bits,bits}; return 1;
}
unsigned fw_motion_speed(void) {return 120;}
int32_t fw_motion_travelled_um(void) {return stopped==2?fault_travel:rotating?0:(int32_t)(now-move_started)*179;}
int fw_motion_extend(unsigned cells,int accept) {
    assert(busy && !rotating && cells==1);++extensions;++moving_cells;
    int c=destination.y*NM_SIDE+destination.x;
    assert((inject_obstacle==2 && !recovered) || !(ground.cell[c].walls&(1u<<destination.heading)));
    c=nm_next(&ground,c,destination.heading);assert(c>=0);
    destination.x=c%NM_SIDE;destination.y=c/NM_SIDE;deadline+=1000;
    wall_arrival=accept && !!(ground.cell[c].walls&(1u<<destination.heading));return 0;
}
void fw_ui_map_progress(int p) {(void)p;}
void fw_ui_map_view(unsigned z,int x,int y) {assert(z==0 || z==16 || z==24);(void)x;(void)y;}
void fw_ui_result(const char *s,uint32_t a,uint32_t b,unsigned p) {(void)s;(void)a;(void)b;(void)p;}
int fw_motion_center_wall(void) {assert(!"Navigation must correct in motion, never probe walls");return -1;}
int fw_motion_centered(void) {return 0;}
void fw_motion_init(void) { busy=stopped=wall_arrival=backing=0; }
int fw_motion_obstacle_backoff(uint32_t um)
{
    assert(stopped==2 && inject_obstacle==2 && !recovered);
    assert(um>160000 && um<179000); /* Return from just before centre 2 to centre 1. */
    destination=segment_start;destination.y+=1;
    busy=backing=1;rotating=0;stopped=0;deadline=now+1000;recovered=1;
    return 0;
}
void fw_motion_geometry(uint32_t pitch,uint32_t front,uint32_t inner)
{ applied_pitch=pitch;(void)front;(void)inner; }
void fw_motion_corner_profiles(const fw_corner_data_t *l,const fw_corner_data_t *r) {(void)l;(void)r;}
void fw_motion_wall_profile(const fw_cal_data_t *d) { (void)d; }
void fw_motion_rotation_profile(const fw_rotation_data_t *d) { (void)d; }
void fw_motion_stop(void) { busy=0; }
int fw_motion_busy(void) { return busy; }
int fw_motion_fault(void) { return stopped; }
int fw_motion_wall_arrival(void) { return wall_arrival; }
uint32_t fw_motion_remaining(void) { return 0; }
int fw_motion_straight_to(unsigned cells,unsigned speed,int accept_wall)
{
    assert(!busy && now>=400 && speed>=20 && cells>0); ++moves;
    if(speed>maximum_requested_speed)maximum_requested_speed=speed;
    destination=physical; int c=physical.y*NM_SIDE+physical.x;
    for(unsigned i=0;i<cells;++i) {
        assert(!(ground.cell[c].walls & 1u<<physical.heading));
        c=nm_neighbour(c,physical.heading); assert(c>=0);
    }
    destination.x=c%NM_SIDE; destination.y=c/NM_SIDE;
    wall_arrival=accept_wall && !!(ground.cell[c].walls & 1u<<physical.heading);
    if(wall_arrival) ++wall_arrivals;
    segment_start=physical;move_started=now;moving_cells=cells;rotating=0;
    busy=1;deadline=now+1000*cells; return 0;
}
int fw_motion_curve(int degrees,unsigned speed,int accept)
{
    assert(!busy && (degrees==90 || degrees==-90) && speed<=FW_RUN_MAX_SPEED);
    int c=physical.y*NM_SIDE+physical.x;
    assert(!(ground.cell[c].walls&(1u<<physical.heading)));
    c=nm_next(&ground,c,physical.heading);
    unsigned heading=(physical.heading+degrees/90+4)%4;
    assert(!(ground.cell[c].walls&(1u<<heading)));
    c=nm_next(&ground,c,heading);assert(c>=0);
    destination=(nm_pose_t){c%NM_SIDE,c/NM_SIDE,heading};
    wall_arrival=accept && !!(ground.cell[c].walls&(1u<<heading));
    busy=rotating=1;deadline=now+1500;++curves;return 0;
}
int fw_motion_turn(int degrees)
{
    assert(!busy && now>=400); assert(degrees==90 || degrees==-90 || degrees==180);
    ++turns; destination=physical;rotating=1;
    destination.heading=(physical.heading+degrees/90+4)%4;
    busy=1;wall_arrival=0;deadline=now+100;return 0;
}
void fw_ui_maze(const nm_map_t *m,nm_pose_t pose,const char *status,unsigned speed,uint32_t ms,uint8_t ir)
{
    (void)pose;(void)speed;(void)ms;(void)ir;
    assert(nm_valid(m)); assert(strlen(status)<=21);++draws;
    strcpy(last_status,status);
}
void fw_test_idle(void)
{
    now+=10; assert(now<200000);
    if(busy && !rotating && !backing && (inject_obstacle==1 || inject_obstacle==2) && !recovered && moves==1 &&
       now-move_started>=(inject_obstacle==2?1950u:500u)) {
        fault_travel=(int32_t)(now-move_started)*179;busy=0;stopped=2;
    }
    if(busy && now>=deadline) {
        busy=0;
        physical=destination;backing=0;
    }
    if(!strcmp(last_status,"OPTIMAL PATH") || !strcmp(last_status,"EARLY OBSTACLE") || !strcmp(last_status,"GOAL FOUND") || !strcmp(last_status,"CHECK LIMIT")) {
        ++ack;test_gpioc.IDR|=(1u<<12)|(1u<<13);
        if(ack<100)test_gpioc.IDR&=~(1u<<12); /* Long centre must NOT exit. */
        else if(ack>=110 && ack<200)test_gpioc.IDR&=~(1u<<13);
    }
}
static void edge(int c,unsigned d)
{
    int n=nm_neighbour(c,d); assert(n>=0);
    ground.cell[c].walls &= ~(1u<<d);
    ground.cell[n].walls &= ~(1u<<((d+2)%4));
}
static void scenario(int obstacle)
{
    nm_init_size(&ground,9,0);fw_maze_size=9;
    for(unsigned c=0;c<NM_CELLS;++c) ground.cell[c]=(nm_cell_t){15,15,0,0};
    edge(0,NM_NORTH);edge(16,NM_NORTH);edge(32,NM_EAST);edge(33,NM_EAST);
    edge(34,NM_EAST);edge(34,NM_NORTH);edge(35,NM_NORTH);edge(50,NM_EAST);
    hide_after_recovery=obstacle==7;hidden_door_cleared=0;now=moves=turns=wall_arrivals=draws=saves=ack=extensions=0; saved_size=0;
    busy=stopped=backing=recovered=0;confirm_missing=obstacle==3; inject_obstacle=(obstacle==3 || obstacle==7)?2:obstacle; last_status[0]=0;
    physical=(nm_pose_t){0,0,NM_NORTH}; test_gpioc.IDR=0xffff;
    zhonxSettings=(robot_settings){.initial_speed=5000,.default_accel=4,.rotate_accel=4,
      .correction_p=1600,.correction_i=4000,.max_correction=3000,.max_speed_distance=1000,.emergency_decel=50};
    fw_app_init();fw_search_speed=obstacle==8?300:220;maximum_requested_speed=0;
    fw_cal_data_t calibration={.valid=1,.repetitions=3,.geometry={47000,94000,167000,179000},
        .front={{92000,92500,400},{132000,132500,200}},.side={{84500,85500,0},{81500,83500,1000}}};
    assert(!fw_app_calibration_commit(&calibration));
    for(unsigned side=0;side<2;++side) {
        fw_corner_data_t corner={.valid=1,.side=side,.post_um=173000,.geometry=calibration.geometry};
        for(unsigned facing=0;facing<2;++facing)for(unsigned i=0;i<3;++i)
            corner.point[facing][i]=(fw_corner_point_t){.speed=i==0?40:i==1?120:220,.mask=2,
                .raw_open_um={0,facing?-30000:45000},.raw_close_um={0,facing?-32000:42000},
                .open_um={0,facing?-26000:50000},.close_um={0,facing?-35000:40000}};
        assert(!fw_app_corner_commit(&corner));
    }
    saves=0;
    int result=fw_app_discover();
    assert(saves==1 && ack>=200);
    if(obstacle==8)assert(maximum_requested_speed==220 && extensions>0);
    if(obstacle==0 || obstacle==2 || obstacle==4 || obstacle==5 || obstacle==7 || obstacle==8) {
        if(obstacle==2 || obstacle==7)assert(recovered);
        else assert(!recovered);
        assert(!result && moves>=4 && extensions>=1 && turns>=4 && wall_arrivals>=2 && draws>=10);
        assert(physical.x==0 && physical.y==0 && physical.heading==0 && !strcmp(last_status,"OPTIMAL PATH"));
        assert(fw_app_ready() && fw_app_maze_count()==1);
        fw_saved_maze_t entry=*fw_app_maze(0);
        fw_app_init();assert(fw_app_ready() && fw_app_maze_count()==1);
        assert(!memcmp(fw_app_maze(0),&entry,sizeof entry));
        save_failure=1; assert(fw_app_maze_delete(0));save_failure=0;
        assert(fw_app_maze_count()==1);
        assert(!fw_app_maze_load(0));
        unsigned exploration_moves=moves;
        now=moves=turns=ack=extensions=0;last_status[0]=0;test_gpioc.IDR=0xffff;
        assert(!fw_app_run() && !strcmp(last_status,"GOAL FOUND"));
        assert(moves<exploration_moves && moves<=2); /* Known straights are grouped, even at 260 mm/s. */
        printf("navigation: %u moves, %u turns, %u wall arrivals; goal and return OK\n",moves,turns,wall_arrivals);
        assert(!fw_app_maze_load(0));physical=(nm_pose_t){0,0,NM_NORTH};
        now=moves=turns=ack=extensions=curves=0;last_status[0]=0;test_gpioc.IDR=0xffff;
        inject_obstacle=0;
        assert(!fw_app_run_curves() && !strcmp(last_status,"GOAL FOUND") && curves>=1);
        /* The same early obstacle is fatal in a timed run: no map rewrite/backoff. */
        assert(!fw_app_maze_load(0));physical=(nm_pose_t){0,0,NM_NORTH};
        now=moves=turns=ack=extensions=0;last_status[0]=0;test_gpioc.IDR=0xffff;
        inject_obstacle=1;recovered=0;
        assert(fw_app_run()==-1 && fw_last_stop_code==2 && !recovered);
        assert(!strcmp(last_status,"EARLY OBSTACLE"));
    } else if(obstacle==6) {
        assert(result==-1 && !strcmp(last_status,"CHECK LIMIT"));
        assert(!fw_app_ready() && !fw_app_maze_count());
    } else {
        assert(result==-1 && fw_last_stop_code==2 && moves==1);
        assert(!fw_app_ready() && !fw_app_maze_count());
        assert(!strcmp(last_status,"EARLY OBSTACLE"));
    }
}
static void calibration_snapshot(void)
{
    fw_cal_data_t d={.valid=1,.repetitions=3,.geometry={40000,90000,167000,179000},
      .front={{76000,80000,0},{130000,136000,0}},.side={{78500,79500,0},{88500,89500,0}}};
    assert(fw_cal_valid(&d));
    fw_motion_init(); assert(!fw_app_calibration_commit(&d));
    assert(saved_schema==7 && applied_pitch==179000);
    fw_app_init(); assert(!memcmp(fw_app_calibration(),&d,sizeof d));
    fw_cal_data_t changed=d; changed.front[0].on_um=75000;
    save_failure=1; assert(fw_app_calibration_commit(&changed)); save_failure=0;
    assert(!memcmp(fw_app_calibration(),&d,sizeof d));
    fw_app_init(); assert(!memcmp(fw_app_calibration(),&d,sizeof d));
    assert(!fw_app_battery_commit(3000,8400));
    fw_app_init();assert(fw_battery_reference().raw==3000 && fw_battery_reference().pack_mv==8400);
    assert(!memcmp(fw_app_calibration(),&d,sizeof d));
    save_failure=1;assert(fw_app_battery_commit(3100,8300));save_failure=0;
    assert(fw_battery_reference().raw==3000);
    /* Construct the frozen legacy ABI, rather than relabelling a current map. */
    snapshot_t latest;memcpy(&latest,saved_snapshot,sizeof latest);
    snapshot_v6_t old={0};old.base.settings=zhonxSettings;
    nm_map_t empty;nm_init_size(&empty,9,0);
    for(unsigned y=0;y<9;++y)for(unsigned x=0;x<9;++x)old.base.map.cell[y*9+x]=empty.cell[y*NM_SIDE+x];
    old.base.search_speed=120;old.base.run_speed=200;
    old.measurements=latest.measurements;old.battery=latest.battery;
    const size_t sizes[]={sizeof(snapshot_v2_t),sizeof(snapshot_v3_t),sizeof(snapshot_v4_t),SNAPSHOT_V5_SIZE,sizeof(snapshot_v6_t)};
    for(unsigned schema=2;schema<=6;++schema) {
        memcpy(saved_snapshot,&old,sizeof old);saved_schema=schema;saved_size=sizes[schema-2];
        fw_app_init();
        if(schema>=3)assert(!memcmp(fw_app_calibration(),&d,sizeof d));
        else assert(!fw_app_calibration()->valid);
        assert(fw_battery_reference().raw==(schema==6?3000u:0u));
        assert(applied_pitch==179000);
        assert(!fw_app_settings_save() && saved_schema==7);
        fw_app_init();if(schema>=3)assert(!memcmp(fw_app_calibration(),&d,sizeof d));
    }
    assert(!fw_app_calibration_commit(&d));
    fw_rotation_data_t rotation={.valid=1,.geometry=d.geometry};
    for(unsigned i=0;i<3;++i) rotation.point[i]=(fw_rotation_point_t){.speed=40+40*i,.quarter_um={66000,65000}};
    assert(!fw_app_rotation_commit(&rotation));
    fw_corner_data_t corner={.valid=1,.side=0,.post_um=173000,.geometry=d.geometry};
    for(unsigned f=0;f<2;++f) for(unsigned i=0;i<3;++i)
        corner.point[f][i]=(fw_corner_point_t){.speed=i==0?40:i==1?120:220,.mask=2,
            .raw_open_um={0,10000},.raw_close_um={0,8000},.open_um={0,12000},.close_um={0,7000}};
    assert(!fw_app_corner_commit(&corner));
    fw_app_init();
    assert(!memcmp(fw_app_rotation(),&rotation,sizeof rotation));
    assert(!memcmp(fw_app_corner(0),&corner,sizeof corner));
    save_failure=1; assert(fw_app_rotation_commit(&rotation)); save_failure=0;
    assert(!memcmp(fw_app_corner(0),&corner,sizeof corner));
    assert(!fw_app_rotation_commit(&rotation));
    fw_app_init(); assert(!fw_app_corner(0)->valid);
    assert(!fw_app_corner_commit(&corner));
    rotation.geometry.nose_um=47000; rotation.geometry.width_um=94000;
    assert(!fw_app_rotation_commit(&rotation));
    fw_app_init(); assert(!fw_app_calibration()->valid && !fw_app_corner(0)->valid);
    puts("snapshot: schema 2/3 migration, complete profiles reboot, save rollback, dependent calibration invalidation");
}
int main(void)
{
    scenario(0);scenario(1);scenario(2);scenario(3);scenario(4);scenario(5);scenario(6);scenario(7);scenario(8);
    puts("navigation: missed front wall recovered; unconfirmed/mislocated obstacles and run obstacles stop");
    calibration_snapshot();
    fw_run_speed=1000;assert(!fw_app_settings_save());
    fw_run_speed=20;fw_app_init();assert(fw_run_speed==1000);
    return 0;
}
