#include "fw_app.h"
#include "fw_motion.h"
#include "hal/hal_sensor.h"
#include "stm32f4xx_gpio.h"
#include "oled/ssd1306.h"
#include <assert.h>
#include <stdio.h>
GPIO_TypeDef test_gpioc;
int fw_cal_nose_tenth_mm,fw_cal_width_tenth_mm,fw_cal_inner_mm=167,fw_cal_pitch_mm=179,fw_cal_post_mm=173;
static unsigned pages,toggle,now,mode,io_moves;
static unsigned diagram_side;
static void save_image(const char *name)
{
    FILE *f=fopen(name,"wb");assert(f);fputs("P5\n128 64\n255\n",f);
    for(unsigned y=0;y<64;++y) for(unsigned x=0;x<128;++x)
        fputc(ssd1306GetPixel(x,y)?255:0,f);
    fclose(f);
}
static fw_rotation_data_t rotation;
static fw_corner_data_t corner[2];
const fw_rotation_data_t *fw_app_rotation(void) { return &rotation; }
const fw_corner_data_t *fw_app_corner(unsigned s) { return &corner[s]; }
int fw_app_rotation_commit(const fw_rotation_data_t *d) { (void)d;assert(0);return -1; }
int fw_app_corner_commit(const fw_corner_data_t *d) { (void)d;assert(0);return -1; }
int fw_motion_calibration_spin(int32_t um,unsigned s) { (void)um;(void)s;assert(0);return -1; }
int fw_motion_calibration_traverse(int32_t um,unsigned s) { (void)um;(void)s;assert(0);return -1; }
static const fw_cal_data_t data={.valid=1,.repetitions=3,.geometry={40000,90000,167000,179000},
  .front={{76000,80000,200},{130000,136000,400}},.side={{78500,79500,0},{88500,90500,1000}}};
const fw_cal_data_t *fw_app_calibration(void) { return &data; }
int fw_app_calibration_commit(const fw_cal_data_t *d) { (void)d;assert(0);return -1; }
int fw_motion_busy(void) { return 0; }
int fw_motion_fault(void) { return 0; }
void fw_motion_init(void) { assert(mode); }
void fw_motion_stop(void) { assert(mode); }
int fw_motion_calibration_move(int32_t um,unsigned speed)
{
    assert(mode==2); ++io_moves;
    if(io_moves==1) { assert(um>0 && speed==20);return 0; }
    assert(io_moves==2 && um<0 && speed==10); return -1; /* Stop after successful seating. */
}
int fw_motion_calibration_turn(int degrees) { (void)degrees;assert(0);return -1; }
int32_t fw_motion_travelled_um(void) { assert(0);return 0; }
int hal_sensor_snapshot_read(hal_sensor_snapshot *s)
{
    assert(mode>=2);
    uint8_t raw=0x3f & ~(SENSOR_F10_POS|SENSOR_L10_POS|SENSOR_R10_POS);
    /* R5 alternates every scan: it must not prevent a front/fixture check. */
    if((now/10)%2) raw ^= SENSOR_R5_POS;
    if(io_moves) raw |= SENSOR_F5_POS|SENSOR_F10_POS; /* Both blind at contact. */
    if(mode==3 && (now/10)%2) raw ^= SENSOR_F10_POS; /* Required bit really unstable. */
    *s=(hal_sensor_snapshot){now,now/10+3,raw,raw}; return 1;
}
unsigned long hal_os_get_systicks(void) { return now; }
int HAL_Delay(unsigned long ms) { (void)ms;return 0; }
void fw_test_idle(void)
{
    if (mode==1) {
        char name[64];snprintf(name,sizeof name,"build/ui-placement-%u.pgm",diagram_side);
        save_image(name);
        assert(ssd1306GetPixel(20,12)); /* Front wall. */
        assert(!!ssd1306GetPixel(14,20)==(diagram_side!=2));
        assert(!!ssd1306GetPixel(33,20)==(diagram_side!=1));
        assert(ssd1306GetPixel(19,17) && ssd1306GetPixel(24,20)); /* Robot and heading. */
        test_gpioc.IDR &= ~(1u<<8); return;
    }
    if (mode>=2) {
        now+=10;assert(now<2000);
        if (now==10) test_gpioc.IDR &= ~GPIO_Pin_11; /* Start. */
        if (now==20) test_gpioc.IDR |= GPIO_Pin_11;
        if (now==1500) {
            save_image(mode==2?"build/ui-calibration-io-error.pgm":"build/ui-calibration-unstable.pgm");
            test_gpioc.IDR &= ~GPIO_Pin_11;
        }
        if (now==1510) test_gpioc.IDR |= GPIO_Pin_11;
        return;
    }
    if (!toggle) {
        char name[64]; snprintf(name,sizeof name,"build/ui-calibration-%u.pgm",pages++);
        save_image(name); test_gpioc.IDR &= ~GPIO_Pin_11;
    } else test_gpioc.IDR |= GPIO_Pin_11;
    toggle=!toggle;
    assert(pages<=35);
}

int main(void)
{
    test_gpioc.IDR=0xffff;
    assert(!fw_calibration_report() && pages==5);
    rotation.valid=1; rotation.geometry=data.geometry;
    for (unsigned i=0;i<3;++i) rotation.point[i]=(fw_rotation_point_t){
        .speed=40+40*i,.quarter_um={66000,65000},.spread_um={1200,800},
        .quarter_error_mdeg={1200,0},.quarter_checks={8,0}};
    assert(!fw_rotation_report() && pages==11);
    for (unsigned s=0;s<2;++s) {
        corner[s]=(fw_corner_data_t){.valid=1,.side=s,.post_um=173000,.geometry=data.geometry};
        for(unsigned f=0;f<2;++f) for(unsigned i=0;i<3;++i)
            corner[s].point[f][i]=(fw_corner_point_t){.speed=i==0?40:i==1?120:220,.mask=3,
                .raw_open_um={-12000,12500},.raw_close_um={-10000,11200},
                .open_um={-13000,13000},.close_um={-10500,10000},.spread_um={1000,2000}};
    }
    assert(!fw_corner_report() && pages==35);
    puts("OLED calibration: 35 wall, rotation and corner report pages rendered with the real driver");
    fw_cal_nose_tenth_mm=400;fw_cal_width_tenth_mm=900;
    mode=1; diagram_side=0; assert(fw_calibrate_menu()==-1);
    test_gpioc.IDR=0xffff; assert(fw_rotation_menu()==-1);
    test_gpioc.IDR=0xffff; diagram_side=1; assert(fw_corner_left_menu()==-1);
    test_gpioc.IDR=0xffff; diagram_side=2; assert(fw_corner_right_menu()==-1);
    mode=2;test_gpioc.IDR=0xffff;now=0;
    assert(fw_calibrate_menu()==-1 && io_moves==2); /* It did request reverse after contact. */
    mode=3;test_gpioc.IDR=0xffff;now=0;io_moves=0;
    assert(fw_calibrate_menu()==-1 && !io_moves); /* Missing reliable front signal still stops. */
    puts("calibration UI: placement diagrams, noisy unrelated sensor, blind contact, required-sensor instability");
    return 0;
}
