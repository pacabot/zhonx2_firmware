#include "fw_menu.h"
#include "fw_app.h"
#include "stm32f4xx.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
GPIO_TypeDef test_gpioc;
int fw_cal_nose_tenth_mm=470,fw_cal_width_tenth_mm=940,fw_cal_inner_mm=167,fw_cal_pitch_mm=179,fw_cal_post_mm=173;
int fw_search_speed=220,fw_run_speed=260,fw_start_corner,fw_start_heading;
static unsigned now,ready,explored,slow,loaded,preview;
static jmp_buf finish;
static fw_saved_maze_t saved;
unsigned long hal_os_get_systicks(void) { return now; }
/* New learning -> slow run -> back -> library -> direct runs -> escape/back.
 * The last idle period checks that no selection is repeated after release. */
static const unsigned events[]={12,12,12,8,10,12,12,8,8,13};
void fw_test_idle(void)
{
    ++now;
    unsigned slot=now/120,phase=now%120;
    if(slot>=sizeof events/sizeof events[0]+1)longjmp(finish,1);
    test_gpioc.IDR=0xffff;
    if(slot<sizeof events/sizeof events[0] && phase>=20 && phase<65)
        test_gpioc.IDR &= ~(1u<<events[slot]);
}
void fw_ui_card(const char *title,const char *a,const char *b,unsigned icon,unsigned i,unsigned n)
{
    (void)b;(void)icon;(void)i;
    if(!strcmp(title,"MAZE"))assert(n==(ready?5u:4u));
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
int fw_app_run_slow(void) {assert(ready);++slow;return 0;}
int fw_app_run(void) {assert(0);return 0;}
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
    assert(explored==1 && slow==1 && loaded==1 && preview);
    puts("menu: joystick press/left/escape, gated runs, completed learning and direct library loading");
}
