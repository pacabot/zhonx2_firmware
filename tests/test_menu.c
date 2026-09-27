#include "fw_menu.h"
#include "fw_app.h"
int fw_maze_size=9;
#include "fw_battery.h"
#include "stm32f4xx.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
GPIO_TypeDef test_gpioc;
int fw_cal_nose_tenth_mm=470,fw_cal_width_tenth_mm=940,fw_cal_inner_mm=167,fw_cal_pitch_mm=179,fw_cal_post_mm=173;
int fw_search_speed=220,fw_run_speed=600,fw_start_corner,fw_start_heading;
static unsigned now,ready,explored,slow,loaded,preview,viewed;
static unsigned idle_scenario,idle_frames,home_index;
int fw_motion_busy(void) {return idle_scenario && now<10000;}
void fw_ui_idle_start(unsigned seed) {assert(seed==now);}
void fw_ui_idle(unsigned phase) {(void)phase;assert(idle_scenario && now>=39999);++idle_frames;fw_battery_sample(3000,now,0);}
static jmp_buf finish;
static fw_saved_maze_t saved;
unsigned long hal_os_get_systicks(void) { return now; }
/* New learning -> slow run -> back -> library -> direct runs -> escape/back.
 * The last idle period checks that no selection is repeated after release. */
/* Choose Run 3 then Run 1; Run 2 setup loads directly after Run 1.
 * After Run 2, back out of the automatically loaded Run 3; inspect the map. */
static const unsigned events[]={12,12,10,10,12,9,12,9,9,12,9,12,10,12,8,8,10,12,12,10,12,8,13};
void fw_test_idle(void)
{
    ++now;
    if(idle_scenario) {
        test_gpioc.IDR=0xffff;
        if(now>=47000)longjmp(finish,1);
        if((now>=46000 && now<46100) || (now>=46500 && now<46550))test_gpioc.IDR&=~(1u<<10);
        return;
    }
    unsigned slot=now/120,phase=now%120;
    if(slot>=sizeof events/sizeof events[0]+1)longjmp(finish,1);
    test_gpioc.IDR=0xffff;
    if(slot<sizeof events/sizeof events[0] && phase>=20 && phase<65)
        test_gpioc.IDR &= ~(1u<<events[slot]);
}
void fw_ui_card(const char *title,const char *a,const char *b,unsigned icon,unsigned i,unsigned n)
{
    (void)b;(void)icon;
    if(!strcmp(title,"ZHONX II"))home_index=i;
    if(!strcmp(title,"MAZE"))assert(n==(ready?5u:2u));
    if(!strcmp(title,"RUNS"))assert(ready);
    assert(strlen(a)<=11);
}
void fw_ui_library(const fw_saved_maze_t *m,unsigned i,unsigned n,int blink)
{ (void)m;(void)i;(void)n;(void)blink;++preview; }
void fw_hardware_test(unsigned n) {(void)n;assert(0);}
void fw_hardware_motor(unsigned n) {(void)n;assert(0);}
void fw_ui_menu_refresh(void) {}
void fw_ui_hint(const char *s) {(void)s;}
void fw_battery_poll(void) {}
int fw_app_battery_commit(unsigned raw,unsigned mv) {(void)raw;(void)mv;assert(0);return -1;}
int HAL_Delay(unsigned long ms) {now+=ms;return 0;}
void fw_ui_setting(unsigned i,int v) { (void)i;(void)v;assert(0); }
int fw_app_discover(void) {assert(!ready);++explored;ready=1;return 0;}
int fw_app_resume(void) {assert(0);return -1;}
int fw_app_ready(void) {return ready;}
int fw_app_run_slow(void) {assert(0);return 0;}
int fw_app_has_map(void) {return ready;}
int fw_app_at_start(void) {return ready;}
int fw_app_show_result(void) {assert(0);return 0;}
int fw_app_show_map(void) {assert(ready);++viewed;return 0;}
unsigned fw_run_acceleration(unsigned speed) {return 600+2*speed;}
void fw_ui_run_setup(unsigned n,unsigned speed,unsigned accel,int aligned)
{assert(ready && aligned && n>=1 && n<=3 && accel==600+2*speed);}
int fw_app_trial(unsigned n,unsigned speed)
{assert((n==(const unsigned[]){3,1,2}[slow] && speed==(const unsigned[]){650,140,250}[slow]));++slow;return 0;}
int fw_app_run(void) {assert(0);return 0;}
int fw_app_run_curves(void) {assert(0);return 0;}
unsigned fw_app_maze_count(void) {return ready;}
const fw_saved_maze_t *fw_app_maze(unsigned i) {assert(i==0);return &saved;}
int fw_app_maze_load(unsigned i) {assert(!i && ready);++loaded;return 0;}
int fw_app_maze_delete(unsigned i) {(void)i;assert(0);return -1;}
int fw_app_settings_save(void) {assert(0);return -1;}
int fw_app_bootloader(void) {assert(0);return -1;}
#define UNUSED_ACTION(name) int name(void) { assert(0);return -1; }
UNUSED_ACTION(fw_calibrate_menu)
UNUSED_ACTION(fw_rotation_menu)
UNUSED_ACTION(fw_corner_left_menu)
UNUSED_ACTION(fw_corner_right_menu)
UNUSED_ACTION(fw_calibration_report)
UNUSED_ACTION(fw_rotation_report)
UNUSED_ACTION(fw_corner_report)
int main(void)
{
    test_gpioc.IDR=0xffff;
    if(!setjmp(finish))fw_menu_run();
    assert(explored==1 && slow==3 && loaded==1 && preview && viewed==1 && fw_run_speed==600);
    idle_scenario=1;now=0;test_gpioc.IDR=0xffff;
    fw_battery_set_reference((fw_battery_reference_t){3000,8400});
    if(!setjmp(finish))fw_menu_run();
    assert(fw_battery_status().soc_valid && fw_battery_status().percent==100);
    assert(idle_frames>=180 && home_index==1); /* Wake DOWN consumed; next DOWN acts. */
    assert(explored==1 && slow==3 && loaded==1);
    puts("idle: 30 seconds at rest, no sleep during motion, wake consumed, selection retained");
    puts("menu: joystick press/left/escape, gated runs, completed learning and direct library loading");
}
