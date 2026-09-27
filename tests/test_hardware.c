#include "fw_hardware.h"
#include "fw_menu.h"
#include "fw_buttons.h"
#include "app/app_def.h"
#include "oled/ssd1306.h"
#include <assert.h>
void fw_ui_header(const char *s) {ssd1306DrawString(1,0,s,&Font_5x8);ssd1306DrawLine(0,8,127,8);}
void fw_ui_hint(const char *s) {ssd1306DrawString(1,55,s,&Font_3x6);}
#include <stdio.h>
#include <string.h>
GPIO_TypeDef test_gpioc;
app_config app_context;
volatile unsigned short convertedValues[1]={3700};
static unsigned now,started,starts,escape_at,press_until;
static int active,stuck,right,left,beeper_state,led_state;
unsigned long hal_os_get_systicks(void) {return now;}
void fw_test_idle(void)
{
    ++now;assert(now<5000);test_gpioc.IDR=0xffff;
    if(now>=20 && now<press_until)test_gpioc.IDR&=~(1u<<12);
    if(now>=escape_at)test_gpioc.IDR&=~FW_ESCAPE_PIN;
}
int HAL_Delay(unsigned long ms) {while(ms--)fw_test_idle();return 0;}
void fw_battery_poll(void) {}
void fw_menu_battery_setup(void) {}
void fw_motion_stop(void) {active=0;}
void fw_motion_init(void) {assert(!active);}
int fw_motion_test_wheels(int r,int l) {right=r;left=l;active=1;started=now;++starts;return 0;}
int fw_motion_busy(void) {return active && (stuck || now-started<100);}
int fw_motion_fault(void) {return 0;}
int hal_sensor_snapshot_read(hal_sensor_snapshot *s)
{*s=(hal_sensor_snapshot){.timestamp=now,.sequence=now,.raw=SENSOR_R5_POS|SENSOR_L10_POS,.filtered=0x3f};return 1;}
int hal_beeper_get_state(HAL_BEEPER_HANDLE h,int *s) {(void)h;*s=beeper_state;return 0;}
int hal_beeper_set_state(HAL_BEEPER_HANDLE h,int s) {(void)h;beeper_state=s;return 0;}
int hal_beeper_beep(HAL_BEEPER_HANDLE h,long freq,long ms) {(void)h;assert(freq>=800 && freq<=1800 && ms==120);return HAL_Delay(ms);}
int hal_led_set_state(HAL_LED_HANDLE h,unsigned char color,unsigned char state) {(void)h;(void)color;led_state=state;return 0;}
int hal_led_reset(HAL_LED_HANDLE h) {(void)h;led_state=0;return 0;}
void fw_ui_card(const char *t,const char *a,const char *b,unsigned icon,unsigned i,unsigned n)
{(void)t;(void)a;(void)b;(void)icon;(void)i;(void)n;}
void fw_ui_menu_refresh(void)
{
    if(now==0) {
        FILE *f=fopen("build/ui-hardware-live.pgm","wb");assert(f);fputs("P5\n128 64\n255\n",f);
        for(unsigned y=0;y<64;++y)for(unsigned x=0;x<128;++x)fputc(ssd1306GetPixel(x,y)?255:0,f);
        fclose(f);
    }
    ssd1306Refresh();
}
static void reset(void)
{now=starts=0;active=stuck=0;escape_at=300;press_until=65;test_gpioc.IDR=0xffff;}
int main(void)
{
    for(unsigned i=0;i<6;++i) {
        reset();fw_hardware_motor(i);assert(starts==1 && !active);
        int sign=i%2?-1:1;assert(right==(i<2?0:sign));assert(left==(i>=2 && i<4?0:sign));
    }
    reset();press_until=500;escape_at=700;fw_hardware_motor(0);assert(starts==1 && started>=500 && !active);
    reset();escape_at=90;stuck=1;fw_hardware_motor(2);assert(starts==1 && now==90 && !active);
    reset();escape_at=4000;stuck=1;fw_hardware_motor(2);assert(starts==1 && !active);
    for(unsigned test=6;test>0;--test) {
        reset();beeper_state=0;fw_hardware_test(test);assert(!active && !beeper_state && !led_state);
    }
    puts("hardware: six motor directions, no held-key repeat, Escape/timeout cleanup, live tests and beeper setting restored");
}
