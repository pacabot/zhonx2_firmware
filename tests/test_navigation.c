#include "fw_app.h"
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
static uint32_t now,deadline;
static unsigned moves,turns,wall_arrivals,draws,saves,ack;
static int busy,wall_arrival,stopped,inject_obstacle;
static char last_status[32];
static unsigned char saved_snapshot[8192];
static size_t saved_size;
static uint32_t saved_schema,applied_pitch;
static int save_failure;
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
    unsigned cell=physical.y*9+physical.x,wall=ground.cell[cell].walls;
    uint8_t bits=0x3f;
    if(wall & 1u<<physical.heading) bits &= ~SENSOR_F10_POS;
    if(wall & 1u<<((physical.heading+3)%4)) bits &= ~SENSOR_L10_POS;
    if(wall & 1u<<((physical.heading+1)%4)) bits &= ~SENSOR_R10_POS;
    if(now<300) bits &= ~SENSOR_F10_POS; /* The user's hand at the start. */
    *s=(hal_sensor_snapshot){now,now/10+5,bits,bits}; return 1;
}
void fw_motion_init(void) { busy=stopped=wall_arrival=0; }
void fw_motion_geometry(uint32_t pitch,uint32_t front,uint32_t inner)
{ applied_pitch=pitch;(void)front;(void)inner; }
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
    destination=physical; int c=physical.y*9+physical.x;
    for(unsigned i=0;i<cells;++i) {
        assert(!(ground.cell[c].walls & 1u<<physical.heading));
        c=nm_neighbour(c,physical.heading); assert(c>=0);
    }
    destination.x=c%9; destination.y=c/9;
    wall_arrival=accept_wall && !!(ground.cell[c].walls & 1u<<physical.heading);
    if(wall_arrival) ++wall_arrivals;
    busy=1;deadline=now+100*cells; return 0;
}
int fw_motion_turn(int degrees)
{
    assert(!busy && now>=400); assert(degrees==90 || degrees==-90 || degrees==180);
    ++turns; destination=physical;
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
    now+=10; assert(now<20000);
    if(busy && now>=deadline) {
        busy=0;
        if(inject_obstacle && moves==1 && turns==0) stopped=2;
        else physical=destination;
    }
    if(!strcmp(last_status,"OPTIMAL PATH") || !strcmp(last_status,"EARLY OBSTACLE")) {
        if(!ack++) test_gpioc.IDR &= ~GPIO_Pin_11;
        else test_gpioc.IDR |= GPIO_Pin_11;
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
    for(unsigned c=0;c<NM_CELLS;++c) ground.cell[c]=(nm_cell_t){15,15,0,0};
    edge(0,NM_NORTH);edge(9,NM_NORTH);edge(18,NM_EAST);edge(19,NM_EAST);
    edge(20,NM_EAST);edge(20,NM_NORTH);edge(21,NM_NORTH);edge(29,NM_EAST);
    now=moves=turns=wall_arrivals=draws=saves=ack=0; saved_size=0;
    busy=stopped=0; inject_obstacle=obstacle; last_status[0]=0;
    physical=(nm_pose_t){0,0,NM_NORTH}; test_gpioc.IDR=0xffff;
    zhonxSettings=(robot_settings){.initial_speed=5000,.default_accel=4,.rotate_accel=4,
      .correction_p=1600,.correction_i=4000,.max_correction=3000,.max_speed_distance=1000,.emergency_decel=50};
    fw_app_init();
    int result=fw_app_discover();
    assert(saves==1);
    if(!obstacle) {
        assert(!result && moves>=8 && turns>=4 && wall_arrivals>=2 && draws>=10);
        assert(physical.x==0 && physical.y==0 && !strcmp(last_status,"OPTIMAL PATH"));
        assert(fw_app_ready() && fw_app_maze_count()==1);
        fw_saved_maze_t entry=*fw_app_maze(0);
        fw_app_init();assert(fw_app_ready() && fw_app_maze_count()==1);
        assert(!memcmp(fw_app_maze(0),&entry,sizeof entry));
        save_failure=1; assert(fw_app_maze_delete(0));save_failure=0;
        assert(fw_app_maze_count()==1);
        assert(!fw_app_maze_load(0));
        printf("navigation: %u moves, %u turns, %u wall arrivals; goal and return OK\n",moves,turns,wall_arrivals);
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
    assert(saved_schema==6 && applied_pitch==179000);
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
    saved_schema=5;saved_size-=sizeof(fw_battery_reference_t);
    fw_app_init();assert(!fw_battery_reference().raw);
    assert(!memcmp(fw_app_calibration(),&d,sizeof d));
    size_t old_size=saved_size-sizeof(fw_library_t)-24;
    saved_schema=4; saved_size=old_size;
    fw_app_init(); assert(!memcmp(fw_app_calibration(),&d,sizeof d));
    size_t legacy_size=old_size-sizeof d-sizeof(fw_rotation_data_t)-2*sizeof(fw_corner_data_t);
    saved_schema=3; saved_size=legacy_size+sizeof d;
    fw_app_init(); assert(!memcmp(fw_app_calibration(),&d,sizeof d));
    unsigned char legacy[2048]; memcpy(legacy,saved_snapshot,legacy_size);
    saved_schema=2; saved_size=legacy_size;
    fw_app_init(); assert(!fw_cal_valid(fw_app_calibration()) && applied_pitch==179000);
    assert(!fw_app_calibration_commit(&d));
    assert(!memcmp(saved_snapshot,legacy,legacy_size));
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
    scenario(0);scenario(1);puts("navigation: early obstacle remains an explicit stop");
    calibration_snapshot(); return 0;
}
