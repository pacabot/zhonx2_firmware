#include "fw_hardware.h"
#include "fw_menu.h"
#include "fw_motion.h"
#include "fw_buttons.h"
#include "fw_battery.h"
#include "hal/hal_os.h"
#include "app/app_def.h"
#include "config/config.h"
#include "oled/ssd1306.h"
#include <stdio.h>
static void text(unsigned x,unsigned y,const char *s)
{ ssd1306DrawString(x,y,s,&Font_5x8); }
static void frame(const char *title)
{ ssd1306ClearScreen();fw_ui_header(title); }
static int press(void)
{
    if(!fw_select_pressed()) return 0;
    unsigned start=hal_os_get_systicks();
    while((unsigned)(hal_os_get_systicks()-start)<20) {
        if(fw_cancel_pressed() || !fw_select_pressed())return 0;
        __WFI();
    }
    return 1;
}
static void release(void)
{ while(fw_select_pressed() && !fw_cancel_pressed()) __WFI(); }
static void telemeters(int filtered)
{
    hal_sensor_snapshot s={0};
    int valid=hal_sensor_snapshot_read(&s) && (unsigned)(hal_os_get_systicks()-s.timestamp)<=50;
    frame(filtered?"TELEMETERS FILTERED":"TELEMETERS RAW");
    const unsigned mask[2][3]={{SENSOR_L5_POS,SENSOR_F5_POS,SENSOR_R5_POS},
                              {SENSOR_L10_POS,SENSOR_F10_POS,SENSOR_R10_POS}};
    text(27,11,"LEFT");text(58,11,"FRONT");text(94,11,"RIGHT");
    text(0,23,"5cm");text(0,36,"10cm");
    for(unsigned row=0;row<2;++row)for(unsigned col=0;col<3;++col) {
        unsigned x=35+col*32,y=22+row*13;
        ssd1306DrawRect(x,y,11,10);
        if(valid && !((filtered?s.filtered:s.raw)&mask[row][col]))ssd1306FillRect(x+2,y+2,7,6);
    }
    text(0,45,valid?"FILLED = WALL":"SENSOR DATA STALE");
    fw_ui_hint("OK: RAW / FILTERED");
    /* Compact footer keeps the six indicators readable. */
    fw_ui_menu_refresh();
}
void fw_hardware_test(unsigned test)
{
    unsigned mode=0,last=hal_os_get_systicks()-100;
    int beeper_state=HAL_BEEPER_STATE_OFF;
    fw_motion_stop();release();
    if(test==2) {hal_beeper_get_state(app_context.beeper,&beeper_state);hal_beeper_set_state(app_context.beeper,HAL_BEEPER_STATE_ON);}
    while(!fw_cancel_pressed()) {
        if(test!=5 && press()) {
            ++mode;
            if(test==6)fw_menu_battery_setup();
            if(test==2)hal_beeper_beep(app_context.beeper,(const long[]){800,1200,1800}[(mode-1)%3],120);
            release();last=hal_os_get_systicks()-100;
        }
        if((unsigned)(hal_os_get_systicks()-last)>=100) {
            last=hal_os_get_systicks();char s[24];
            if(test==1)telemeters(mode%2);
            else {
                frame(test==2?"BEEPER":test==3?"LEDS":test==4?"DISPLAY":test==5?"BUTTONS":"BATTERY ADC");
                if(test==2) {text(0,24,"800 / 1200 / 1800 Hz");fw_ui_hint("OK: PLAY NEXT TONE");}
                if(test==3) {
                    hal_led_set_state(app_context.led,HAL_LED_COLOR_RED,mode%3==1);
                    hal_led_set_state(app_context.led,HAL_LED_COLOR_ORANGE,mode%3==2);
                    text(0,20,(const char*[]){"OFF","RED","ORANGE"}[mode%3]);fw_ui_hint("OK: NEXT LED");
                }
                if(test==4) {
                    fw_ui_hint("OK: NEXT PATTERN");
                    if(mode%3==0) {ssd1306DrawRect(0,12,128,39);text(28,24,"BORDER TEST");}
                    if(mode%3==1)ssd1306FillRect(0,12,128,39);
                    if(mode%3==2)for(unsigned y=12;y<51;y+=4)for(unsigned x=0;x<128;x+=4)
                        if((x+y)%8)ssd1306FillRect(x,y,4,4);
                }
                if(test==5) {
                    snprintf(s,sizeof s,"UP:%u     DOWN:%u",!(GPIOC->IDR&FW_UP_PIN),!(GPIOC->IDR&FW_DOWN_PIN));text(0,18,s);
                    snprintf(s,sizeof s,"RIGHT:%u  PRESS:%u",!(GPIOC->IDR&(1u<<11)),!(GPIOC->IDR&(1u<<12)));text(0,32,s);
                    fw_ui_hint("LEFT / ESC: EXIT");
                }
                if(test==6) {
                    fw_battery_poll();fw_battery_status_t b=fw_battery_status();
                    snprintf(s,sizeof s,"ADC: %u / 4095",b.raw);text(0,16,s);
                    if(b.calibrated && b.sample_valid) {
                        snprintf(s,sizeof s,"PACK: %u.%02u V",b.pack_mv/1000,b.pack_mv%1000/10);text(0,28,s);
                        if(b.soc_valid)snprintf(s,sizeof s,"SOC: ~%u%%",b.percent);
                        else snprintf(s,sizeof s,"Wait at rest...");
                        text(0,40,s);
                    } else text(0,32,b.sample_valid?"Set meter voltage":"ADC INPUT INVALID");
                    fw_ui_hint("OK: SET METER VOLTS");
                }
                fw_ui_menu_refresh();
            }
        }
        __WFI();
    }
    if(test==2)hal_beeper_set_state(app_context.beeper,beeper_state);
    if(test==3)hal_led_reset(app_context.led);
    fw_motion_stop();
}
void fw_hardware_motor(unsigned selection)
{
    if(selection>=6)return;
    fw_motion_stop();release();
    fw_ui_card("MOTOR TEST / LIFT","Lift robot","OK: 20 mm",FW_ICON_TEST,selection,6);
    while(!fw_cancel_pressed()) {
        if(press()) {
            release();if(fw_cancel_pressed())break;
            int sign=selection%2?-1:1;
            fw_motion_init();
            int result=fw_motion_test_wheels(selection<2?0:sign,selection>=2 && selection<4?0:sign);
            fw_ui_card("MOTOR TEST","Running","LEFT: STOP",FW_ICON_TEST,selection,6);
            unsigned start=hal_os_get_systicks();
            while(!result && fw_motion_busy() && !fw_cancel_pressed()) {
                if((unsigned)(hal_os_get_systicks()-start)>3000) {result=-1;break;}
                __WFI();
            }
            result=result || fw_motion_fault();fw_motion_stop();
            if(fw_cancel_pressed())break;
            fw_ui_card("MOTOR TEST",result?"Stopped":"Done","OK: repeat",FW_ICON_TEST,selection,6);
        }
        __WFI();
    }
    fw_motion_stop();
}
