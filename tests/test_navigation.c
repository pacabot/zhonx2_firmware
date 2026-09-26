#include "fw_app.h"
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
unsigned long hal_os_get_systicks(void) { return now; }
void NVIC_SystemReset(void) { assert(0); }
int hal_ui_display_prompt(HAL_UI_HANDLE h,const char *title,const char *str)
{ (void)h; fprintf(stderr,"Unexpected prompt: %s %s\n",title,str); assert(0); return -1; }
int fw_store_load(const fw_flash_t *f,uint32_t schema,void *data,size_t size)
{ (void)f;(void)schema;(void)data;(void)size;return -1; }
int fw_store_save(const fw_flash_t *f,uint32_t schema,const void *data,size_t size)
{
    (void)f;(void)schema;(void)size;assert(!busy);++saves;
    assert(nm_valid((const nm_map_t *)((const unsigned char *)data+sizeof(robot_settings))));
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
    if(!strcmp(last_status,"CHEMIN OPTIMAL") || !strcmp(last_status,"OBSTACLE HORS CASE")) {
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
    now=moves=turns=wall_arrivals=draws=saves=ack=0;
    busy=stopped=0; inject_obstacle=obstacle; last_status[0]=0;
    physical=(nm_pose_t){0,0,NM_NORTH}; test_gpioc.IDR=0xffff;
    zhonxSettings=(robot_settings){.initial_speed=5000,.default_accel=4,.rotate_accel=4,
      .correction_p=1600,.correction_i=4000,.max_correction=3000,.max_speed_distance=1000,.emergency_decel=50};
    fw_app_init();
    int result=fw_app_discover();
    assert(saves==1);
    if(!obstacle) {
        assert(!result && moves>=8 && turns>=4 && wall_arrivals>=2 && draws>=10);
        assert(physical.x==0 && physical.y==0 && !strcmp(last_status,"CHEMIN OPTIMAL"));
        printf("navigation: %u moves, %u turns, %u wall arrivals; goal and return OK\n",moves,turns,wall_arrivals);
    } else {
        assert(result==-1 && fw_last_stop_code==2 && moves==1);
        assert(!strcmp(last_status,"OBSTACLE HORS CASE"));
    }
}
int main(void) { scenario(0);scenario(1);puts("navigation: early obstacle remains an explicit stop");return 0; }
