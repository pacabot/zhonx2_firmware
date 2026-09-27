#include "fw_menu.h"
#include "fw_app.h"
#include "fw_hardware.h"
#include "fw_battery.h"
#include "fw_motion.h"
#include <stdio.h>
#include "fw_buttons.h"
#include "hal/hal_os.h"
#include "oled/ssd1306.h"
#include "stm32f4xx.h"
#include <stddef.h>
#include <string.h>
typedef struct { const char *a,*b; unsigned icon; } card_t;
enum { KEY_NONE, KEY_UP, KEY_DOWN, KEY_OK, KEY_BACK };
static unsigned read_key(void)
{
    if(fw_cancel_pressed()) return KEY_BACK;
    if(!(GPIOC->IDR&FW_UP_PIN)) return KEY_UP;
    if(!(GPIOC->IDR&FW_DOWN_PIN)) return KEY_DOWN;
    return fw_select_pressed()?KEY_OK:KEY_NONE;
}
static unsigned key(void)
{
    unsigned k=read_key();if(!k) return 0;
    uint32_t since=hal_os_get_systicks();
    while((uint32_t)(hal_os_get_systicks()-since)<20) { if(read_key()!=k) return 0;__WFI(); }
    while(read_key()) __WFI();
    return k;
}
/* Menu-only screensaver. The wake-up key is consumed, never an action. */
static int idle_wait(uint32_t *activity)
{
    uint32_t now=hal_os_get_systicks();
    if(read_key() || fw_motion_busy()) {*activity=now;return 0;}
    if((uint32_t)(now-*activity)<30000)return 0;
    unsigned phase=0;uint32_t drawn=now;
    fw_ui_idle(phase++);
    while(!read_key() && !fw_motion_busy()) {
        now=hal_os_get_systicks();
        if((uint32_t)(now-drawn)>=2000) {fw_ui_idle(phase++);drawn=now;}
        __WFI();
    }
    while(read_key())__WFI();
    *activity=hal_os_get_systicks();
    return 1;
}
static int choose(const char *title,const card_t *items,unsigned count)
{
    /* Consume the action that closed a child screen; one press means one Back. */
    while(read_key()) __WFI();
    static struct { const char *title; unsigned selected; } history[10];
    unsigned slot=10,i=0;
    if(count>2) for(unsigned n=0;n<10;++n) {
        if(!history[n].title || !strcmp(history[n].title,title)) {
            slot=n;history[n].title=title;i=history[n].selected%count;break;
        }
    }
    fw_ui_card(title,items[i].a,items[i].b,items[i].icon,i,count);
    uint32_t refresh=hal_os_get_systicks(),activity=refresh;
    for(;;) {
        if(idle_wait(&activity))fw_ui_card(title,items[i].a,items[i].b,items[i].icon,i,count);
        if((uint32_t)(hal_os_get_systicks()-refresh)>=1000) {fw_ui_menu_refresh();refresh=hal_os_get_systicks();}
        unsigned k=key();if(k==KEY_BACK) return -1;if(k==KEY_OK) return (int)i;
        if(k==KEY_UP || k==KEY_DOWN) {
            i=(i+count+(k==KEY_UP?-1:1))%count;
            if(slot<10) history[slot].selected=i;
            fw_ui_card(title,items[i].a,items[i].b,items[i].icon,i,count);
        }
        __WFI();
    }
}
static void notice(const char *first,const char *second)
{
    fw_ui_card("NOTICE",first,second,FW_ICON_REPORT,0,1);
    while(!key()) {fw_ui_menu_refresh();HAL_Delay(100);}
}
static int confirm(const char *title)
{
    const char *action=!strcmp(title,"DELETE THIS MAZE?")?"Delete maze":"Bootloader";
    const card_t c[]={{action,"Cancel",FW_ICON_REPORT},{action,"Confirm",FW_ICON_REPORT}};
    return choose(title,c,2)==1;
}
static void runs(void)
{
    const card_t cards[]={{"Slow run","120 mm/s",FW_ICON_RUN},{"Fast run","",FW_ICON_RUN},
        {"Curves","Unavailable",FW_ICON_RUN},{"New maze","Explore",FW_ICON_MAZE}};
    while(fw_app_ready()) {
        int n=choose("RUNS",cards,4);if(n<0) return;
        if(n==0) fw_app_run_slow();
        if(n==1) fw_app_run();
        if(n==2) notice("Not yet","implemented");
        if(n==3) { if(!fw_app_discover() && fw_app_ready()) continue;return; }
    }
}
static void library_menu(int deleting)
{
    unsigned i=0,last_blink=2;
    uint32_t activity=hal_os_get_systicks();
    if(!fw_app_maze_count()) {notice("No learned","maze saved");return;}
    while(fw_app_maze_count()) {
        if(idle_wait(&activity))last_blink=2;
        if(i>=fw_app_maze_count()) i=0;
        unsigned blink=(hal_os_get_systicks()/500)%2;
        if(blink!=last_blink) {fw_ui_library(fw_app_maze(i),i,fw_app_maze_count(),blink);last_blink=blink;}
        unsigned k=key();if(k==KEY_BACK) return;
        if(k==KEY_UP || k==KEY_DOWN) {i=(i+fw_app_maze_count()+(k==KEY_UP?-1:1))%fw_app_maze_count();last_blink=2;}
        if(k==KEY_OK) {
            if(!deleting && !fw_app_maze_load(i)) {runs();return;}
            if(deleting && confirm("DELETE THIS MAZE?")) {
                if(fw_app_maze_delete(i)) notice("Save failed","Kept maze");
            }
            last_blink=2;activity=hal_os_get_systicks();
        }
        __WFI();
    }
}
static void maze_menu(void)
{
    const card_t items[]={{"New maze","Explore",FW_ICON_MAZE},{"Load maze","Library",FW_ICON_MAZE},
        {"Resume","Learning",FW_ICON_MAZE},{"Delete","Saved maze",FW_ICON_REPORT},{"Runs","",FW_ICON_RUN}};
    for(;;) {
        int n=choose("MAZE",items,fw_app_ready()?5:4);if(n<0) return;
        if(n==0) {if(!fw_app_discover() && fw_app_ready()) runs();}
        if(n==1) library_menu(0);
        if(n==2) {if(!fw_app_resume() && fw_app_ready()) runs();}
        if(n==3) library_menu(1);
        if(n==4) runs();
    }
}
static void calibration_menu(void)
{
    const card_t items[]={{"Walls","Calibrate",FW_ICON_WALL},{"Rotation","Calibrate",FW_ICON_TURN},
        {"Left edge","Calibrate",FW_ICON_EDGE_LEFT},{"Right edge","Calibrate",FW_ICON_EDGE_RIGHT},
        {"Wall report","",FW_ICON_REPORT},{"Turn report","",FW_ICON_REPORT},{"Edge report","",FW_ICON_REPORT}};
    int (*const action[])(void)={fw_calibrate_menu,fw_rotation_menu,fw_corner_left_menu,fw_corner_right_menu,
        fw_calibration_report,fw_rotation_report,fw_corner_report};
    for(;;) {int n=choose("CALIBRATION",items,7);if(n<0)return;(void)action[n]();}
}
static void settings_menu(void)
{
    const card_t items[]={{"Axle / nose","Geometry",FW_ICON_SETTINGS},{"Robot width","Geometry",FW_ICON_SETTINGS},
        {"Cell inside","Geometry",FW_ICON_SETTINGS},{"Cell pitch","Geometry",FW_ICON_SETTINGS},
        {"Post center","Geometry",FW_ICON_SETTINGS},{"Explore","Speed",FW_ICON_SETTINGS},
        {"Fast run","Speed",FW_ICON_SETTINGS},{"Start","Corner",FW_ICON_SETTINGS},{"Start","Heading",FW_ICON_SETTINGS}};
    int *const values[]={&fw_cal_nose_tenth_mm,&fw_cal_width_tenth_mm,&fw_cal_inner_mm,&fw_cal_pitch_mm,&fw_cal_post_mm,
        &fw_search_speed,&fw_run_speed,&fw_start_corner,&fw_start_heading};
    const int minimum[]={100,400,140,141,100,20,20,0,0},maximum[]={750,1400,190,210,250,300,300,3,3};
    for(;;) {
        int n=choose("SETTINGS",items,9);if(n<0)return;
        int old=*values[n],v=old;fw_ui_setting(n,v);
        uint32_t refresh=hal_os_get_systicks(),activity=refresh;
        for(;;) {
            if(idle_wait(&activity))fw_ui_setting(n,v);
            if((uint32_t)(hal_os_get_systicks()-refresh)>=1000) {fw_ui_menu_refresh();refresh=hal_os_get_systicks();}
            unsigned k=key();if(k==KEY_BACK)break;
            if(k==KEY_OK) {
                if(n<5) {
                    fw_cal_geometry_t g={fw_cal_nose_tenth_mm*100u,fw_cal_width_tenth_mm*100u,
                        fw_cal_inner_mm*1000u,fw_cal_pitch_mm*1000u};
                    if(n==0)g.nose_um=v*100u;
                    if(n==1)g.width_um=v*100u;
                    if(n==2)g.inner_um=v*1000u;
                    if(n==3)g.pitch_um=v*1000u;
                    if(!fw_cal_geometry_valid(&g) || (n==4 && v*1000u<g.inner_um/2+50000)) {
                        notice("Invalid","geometry");fw_ui_setting(n,v);continue;
                    }
                }
                *values[n]=v;if(fw_app_settings_save()) {*values[n]=old;notice("Save failed","Try again");}
                break;
            }
            int step=n==1 || n==5 || n==6?10:1;
            if(k==KEY_UP && v<maximum[n]) {v+=step;if(v>maximum[n])v=maximum[n];}
            if(k==KEY_DOWN && v>minimum[n]) {v-=step;if(v<minimum[n])v=minimum[n];}
            if(k)fw_ui_setting(n,v);
            __WFI();
        }
    }
}
void fw_menu_battery_setup(void)
{
    fw_battery_reference_t ref=fw_battery_reference();
    unsigned mv=ref.pack_mv?ref.pack_mv:8400;
    while(read_key())__WFI();
    uint32_t activity=hal_os_get_systicks();
    for(;;) {
        (void)idle_wait(&activity);
        char value[16];snprintf(value,sizeof value,"%u.%02u V",mv/1000,(mv%1000)/10);
        fw_ui_card("METER VOLTAGE / 2S","Meter volts",value,FW_ICON_SETTINGS,0,1);
        fw_ui_hint("UP/DN: EDIT  OK: SAVE");fw_ui_menu_refresh();
        unsigned k=key();
        if(k==KEY_BACK)return;
        if(k==KEY_UP && mv<8500)mv+=10;
        if(k==KEY_DOWN && mv>6000)mv-=10;
        if(k==KEY_OK) {
            fw_battery_poll();fw_battery_status_t b=fw_battery_status();
            if(!b.sample_valid)notice("ADC error","Check input");
            else if(!b.rested)notice("Wait 5 sec","At rest");
            else if(fw_app_battery_commit(b.raw,mv))notice("Save failed","Try again");
            else {notice("Saved","Battery ref");return;}
        }
        HAL_Delay(100);
    }
}
static void hardware_menu(void)
{
    const card_t items[]={{"Motors","Lift robot",FW_ICON_TEST},{"Telemeters","Live view",FW_ICON_TEST},
        {"Beeper","",FW_ICON_TEST},{"LEDs","",FW_ICON_TEST},{"Display","",FW_ICON_TEST},
        {"Buttons","Live view",FW_ICON_TEST},{"Battery","ADC",FW_ICON_TEST}};
    const card_t motors[]={{"Left wheel","Forward",FW_ICON_TEST},{"Left wheel","Reverse",FW_ICON_TEST},
        {"Right wheel","Forward",FW_ICON_TEST},{"Right wheel","Reverse",FW_ICON_TEST},
        {"Both wheels","Forward",FW_ICON_TEST},{"Both wheels","Reverse",FW_ICON_TEST}};
    for(;;) {
        int n=choose("HARDWARE TESTS",items,7);if(n<0)return;
        if(n) fw_hardware_test((unsigned)n);
        else for(;;) {int m=choose("MOTORS / LIFT ROBOT",motors,6);if(m<0)break;fw_hardware_motor((unsigned)m);}
    }
}
void fw_menu_run(void)
{
    const card_t home[]={{"Maze","",FW_ICON_MAZE},{"Calibration","",FW_ICON_CALIBRATE},
        {"Settings","",FW_ICON_SETTINGS},{"Hardware","Tests",FW_ICON_TEST},{"Update","Firmware",FW_ICON_UPDATE}};
    for(;;) {
        int n=choose("ZHONX II",home,5);if(n<0)continue;
        if(n==0)maze_menu();
        if(n==1)calibration_menu();
        if(n==2)settings_menu();
        if(n==3)hardware_menu();
        if(n==4 && confirm("ENTER BOOTLOADER?"))fw_app_bootloader();
    }
}
