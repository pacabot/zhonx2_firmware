#include "fw_app.h"
#include "fw_motion.h"
#include "hal/hal_sensor.h"
#include "stm32f4xx_gpio.h"
#include "oled/ssd1306.h"
#include <assert.h>
#include <stdio.h>
GPIO_TypeDef test_gpioc;
int fw_cal_nose_tenth_mm,fw_cal_width_tenth_mm,fw_cal_inner_mm=167,fw_cal_pitch_mm=179;
static unsigned pages,toggle;
static const fw_cal_data_t data={.valid=1,.repetitions=3,.geometry={40000,90000,167000,179000},
  .front={{76000,80000,200},{130000,136000,400}},.side={{78500,79500,0},{88500,90500,1000}}};
const fw_cal_data_t *fw_app_calibration(void) { return &data; }
int fw_app_calibration_commit(const fw_cal_data_t *d) { (void)d;assert(0);return -1; }
int fw_motion_busy(void) { return 0; }
int fw_motion_fault(void) { return 0; }
void fw_motion_init(void) { assert(0); }
void fw_motion_stop(void) { assert(0); }
int fw_motion_calibration_move(int32_t um,unsigned speed) { (void)um;(void)speed;assert(0);return -1; }
int fw_motion_calibration_turn(int degrees) { (void)degrees;assert(0);return -1; }
int32_t fw_motion_travelled_um(void) { assert(0);return 0; }
int hal_sensor_snapshot_read(hal_sensor_snapshot *s) { (void)s;assert(0);return 0; }
unsigned long hal_os_get_systicks(void) { return 0; }
int HAL_Delay(unsigned long ms) { (void)ms;return 0; }
void fw_test_idle(void)
{
    if (!toggle) {
        char name[64]; snprintf(name,sizeof name,"build/ui-calibration-%u.pgm",pages++);
        FILE *f=fopen(name,"wb");assert(f);fputs("P5\n128 64\n255\n",f);
        for(unsigned y=0;y<64;++y) for(unsigned x=0;x<128;++x)
            fputc(ssd1306GetPixel(x,y)?255:0,f);
        fclose(f); test_gpioc.IDR &= ~GPIO_Pin_11;
    } else test_gpioc.IDR |= GPIO_Pin_11;
    toggle=!toggle;
    assert(pages<=5);
}
int main(void)
{
    test_gpioc.IDR=0xffff;
    assert(!fw_calibration_report() && pages==5);
    puts("OLED calibration: all five saved-report pages rendered with the real driver");
    return 0;
}
