#include "fw_app.h"
#include "fw_motion.h"
#include "hal/hal_os.h"
#include "hal/hal_sensor.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "oled/ssd1306.h"
#include <stdio.h>
#include <string.h>
static int cancelled(void) { return !(GPIOC->IDR&GPIO_Pin_13); }
static int fresh(hal_sensor_snapshot *s)
{
    return hal_sensor_snapshot_read(s) && (uint32_t)(hal_os_get_systicks()-s->timestamp)<=50;
}
static void line(unsigned y,const char *text) { ssd1306DrawString(0,y,text,&Font_5x8); }
static int acknowledge(void)
{
    while (!(GPIOC->IDR&GPIO_Pin_11)) { if (cancelled()) return -1; __WFI(); }
    while (GPIOC->IDR&GPIO_Pin_11) { if (cancelled()) return -1; __WFI(); }
    while (!(GPIOC->IDR&GPIO_Pin_11)) { if (cancelled()) return -1; __WFI(); }
    return 0;
}
static void message(const char *title,const char *text)
{
    ssd1306ClearScreen(); line(0,title); line(20,text); line(54,"RIGHT:OK BACK:EXIT");
    ssd1306Refresh(); (void)acknowledge();
}
static int read_settled(void *context,uint8_t *raw)
{
    (void)context;
    uint32_t start=hal_os_get_systicks(),seen=0; unsigned count=0; uint8_t previous=0;
    while ((uint32_t)(hal_os_get_systicks()-start)<1000) {
        hal_sensor_snapshot scan;
        if (cancelled() || fw_motion_fault() || !fresh(&scan)) return -1;
        if (scan.sequence!=seen) {
            seen=scan.sequence;
            count=scan.raw==previous?count+1:1; previous=scan.raw;
            if (count>=5) { *raw=previous; return 0; }
        }
        __WFI();
    }
    return -1;
}
static int wait_move(fw_cal_observer observer,void *context)
{
    uint32_t start=hal_os_get_systicks(),seen=0;
    do {
        hal_sensor_snapshot scan;
        if (cancelled() || fw_motion_fault() || !fresh(&scan) ||
            (uint32_t)(hal_os_get_systicks()-start)>55000) { fw_motion_stop(); return -1; }
        if (observer && seen!=scan.sequence) {
            observer(context,fw_motion_travelled_um(),scan.raw,scan.filtered); seen=scan.sequence;
        }
        __WFI();
    } while (fw_motion_busy());
    /* Complete debounce even if a threshold was crossed at the last pulse. */
    start=hal_os_get_systicks();
    do {
        hal_sensor_snapshot scan;
        if (cancelled() || fw_motion_fault() || !fresh(&scan)) { fw_motion_stop(); return -1; }
        if (observer && seen!=scan.sequence) {
            observer(context,fw_motion_travelled_um(),scan.raw,scan.filtered); seen=scan.sequence;
        }
        __WFI();
    } while ((uint32_t)(hal_os_get_systicks()-start)<50);
    return 0;
}
static int move_robot(void *context,int32_t um,unsigned speed,fw_cal_observer observer,void *data)
{
    (void)context;
    if (cancelled()) return -1;
    int result=speed>30?fw_motion_calibration_traverse(um,speed):fw_motion_calibration_move(um,speed);
    if (result) return -1;
    return wait_move(observer,data);
}
static int turn_robot(void *context,int degrees)
{
    (void)context;
    if (cancelled() || fw_motion_calibration_turn(degrees)) return -1;
    return wait_move(0,0);
}
static void status(void *context,const char *text,int32_t um)
{
    char value[32];
    ssd1306ClearScreen(); line(0,context?(const char *)context:"WALL CALIBRATION"); line(16,text);
    snprintf(value,sizeof value,"AXLE-WALL %ld.%ld MM",(long)(um/1000),(long)(um%1000/100));
    if (um) line(32,value);
    line(54,"BACK: STOP"); ssd1306Refresh();
}
static void distance_line(unsigned y,const char *label,uint32_t um)
{
    char value[32];
    snprintf(value,sizeof value,"%s %lu.%lu MM",label,(unsigned long)(um/1000),(unsigned long)(um%1000/100));
    line(y,value);
}
int fw_calibration_report(void)
{
    const fw_cal_data_t *d=fw_app_calibration();
    if (!fw_cal_valid(d)) { message("WALL CALIBRATION","NO SAVED MEASUREMENT"); return -1; }
    for (unsigned page=0;page<5;++page) {
        ssd1306ClearScreen();
        if (!page) {
            line(0,"SAVED IR / AXLE-WALL");
            distance_line(12,"AXLE-NOSE",d->geometry.nose_um);
            distance_line(22,"WIDTH",d->geometry.width_um);
            distance_line(32,"CELL CLEAR",d->geometry.inner_um);
            distance_line(42,"CELL PITCH",d->geometry.pitch_um);
        } else if (page<3) {
            const fw_cal_front_t *f=&d->front[page-1];
            line(0,page==1?"FRONT 5CM / 3 RUNS":"FRONT 10CM / 3 RUNS");
            distance_line(12,"TRIGGER",f->on_um);
            distance_line(22,"RELEASE",f->off_um);
            distance_line(32,"HYSTERESIS",f->off_um-f->on_um);
            distance_line(42,"SPREAD",f->spread_um);
        } else {
            const fw_cal_side_t *s=&d->side[page-3];
            line(0,page==3?"LEFT 5CM / 3 RUNS":"RIGHT 5CM / 3 RUNS");
            distance_line(12,"DETECT <=",s->near_um);
            distance_line(22,"CLEAR >=",s->far_um);
            distance_line(32,"SPREAD",s->spread_um);
            line(42,"STATIC / 1 MM STEP");
        }
        line(54,page==4?"RIGHT:END BACK:END":"RIGHT:NEXT BACK:END");
        ssd1306Refresh(); if (acknowledge()) break;
    }
    return 0;
}
int fw_calibrate_menu(void)
{
    if (fw_motion_busy()) return -1;
    if (fw_cal_nose_tenth_mm<100 || fw_cal_nose_tenth_mm>750 ||
        fw_cal_width_tenth_mm<400 || fw_cal_width_tenth_mm>1400 ||
        fw_cal_inner_mm<140 || fw_cal_inner_mm>190 || fw_cal_pitch_mm>210 ||
        fw_cal_pitch_mm<=fw_cal_inner_mm) {
        message("INVALID GEOMETRY","SET NOSE / WIDTH"); return -1;
    }
    fw_cal_geometry_t geometry={fw_cal_nose_tenth_mm*100u,fw_cal_width_tenth_mm*100u,
                               fw_cal_inner_mm*1000u,fw_cal_pitch_mm*1000u};
    if (!fw_cal_geometry_valid(&geometry)) {
        message("INVALID GEOMETRY","SET NOSE / WIDTH"); return -1;
    }
    fw_motion_init();
    ssd1306ClearScreen(); line(0,"WALL CALIBRATION");
    line(12,"CENTER IN 3-WALL CELL"); line(22,"CLEAR 10 CM BEHIND");
    line(32,"SLOW WALL CONTACT"); line(42,"BACK: STOP"); line(54,"RIGHT: START");
    ssd1306Refresh(); if (acknowledge()) return -1;
    const fw_cal_io_t io={0,move_robot,turn_robot,read_settled,status};
    fw_cal_data_t result;
    int error=fw_cal_run(&io,&geometry,&result);
    fw_motion_stop();
    if (error) {
        const char *text=error==FW_CAL_WALLS?"CHECK FIXTURE WALLS":
            error==FW_CAL_RANGE?"NO EDGE IN RANGE":error==FW_CAL_UNSTABLE?"UNSTABLE MEASUREMENTS":"STOP OR SENSOR FAULT";
        if (!cancelled()) message("CALIBRATION STOPPED",text);
        return -1;
    }
    if (fw_app_calibration_commit(&result)) { message("WALL CALIBRATION","FLASH SAVE FAILED"); return -1; }
    return fw_calibration_report();
}

static int configured_geometry(fw_cal_geometry_t *g)
{
    if (fw_cal_nose_tenth_mm<100 || fw_cal_nose_tenth_mm>750 ||
        fw_cal_width_tenth_mm<400 || fw_cal_width_tenth_mm>1400 ||
        fw_cal_inner_mm<140 || fw_cal_inner_mm>190 || fw_cal_pitch_mm>210 ||
        fw_cal_pitch_mm<=fw_cal_inner_mm) return -1;
    *g=(fw_cal_geometry_t){fw_cal_nose_tenth_mm*100u,fw_cal_width_tenth_mm*100u,
                           fw_cal_inner_mm*1000u,fw_cal_pitch_mm*1000u};
    return fw_cal_geometry_valid(g)?0:-1;
}
static int spin_robot(void *context,int32_t um,unsigned speed,fw_cal_observer observer,void *data)
{
    (void)context;
    if (cancelled() || fw_motion_calibration_spin(um,speed)) return -1;
    return wait_move(observer,data);
}
static int calibration_error(int error)
{
    fw_motion_stop();
    if (!cancelled()) message("CALIBRATION STOPPED",error==FW_CAL_RANGE?"NO EDGE IN RANGE":
        error==FW_CAL_UNSTABLE?"UNSTABLE MEASUREMENTS":error==FW_CAL_WALLS?"CHECK FIXTURE WALLS":"STOP OR SENSOR FAULT");
    return -1;
}
int fw_rotation_report(void)
{
    const fw_rotation_data_t *d=fw_app_rotation();
    if (!fw_rotation_valid(d)) { message("ROTATION REPORT","NO SAVED MEASUREMENT"); return -1; }
    for (unsigned i=0;i<FW_CAL_SPEEDS;++i) for (unsigned dir=0;dir<2;++dir) {
        const fw_rotation_point_t *p=&d->point[i]; char title[32];
        ssd1306ClearScreen(); snprintf(title,sizeof title,"%s %lu MM/S",dir?"CCW":"CW",(unsigned long)p->speed);
        line(0,title);
        distance_line(12,"EFF.TRACK",(uint32_t)((uint64_t)p->quarter_um[dir]*4000000/3141593));
        distance_line(22,"90DEG ARC",p->quarter_um[dir]);
        distance_line(32,"360 SPREAD",p->spread_um[dir]);
        if (p->quarter_checks[dir]) {
            snprintf(title,sizeof title,"90 CHECK %lu.%lu DEG",(unsigned long)(p->quarter_error_mdeg[dir]/1000),
                     (unsigned long)(p->quarter_error_mdeg[dir]%1000/100)); line(42,title);
        } else line(42,"90: PERIOD / 4 ONLY");
        line(54,"RIGHT:NEXT BACK:END"); ssd1306Refresh(); if (acknowledge()) return 0;
    }
    return 0;
}
int fw_rotation_menu(void)
{
    fw_cal_geometry_t g;
    if (fw_motion_busy()) return -1;
    if (configured_geometry(&g)) { message("INVALID GEOMETRY","SET NOSE / WIDTH"); return -1; }
    fw_motion_init(); ssd1306ClearScreen(); line(0,"ROTATION CALIBRATION");
    line(12,"CENTER IN 3-WALL CELL"); line(22,"OPENING BEHIND ROBOT");
    line(32,"CW + CCW / 3 SPEEDS"); line(42,"BACK: STOP"); line(54,"RIGHT: START");
    ssd1306Refresh(); if (acknowledge()) return -1;
    const fw_cal_extra_io_t io={{"ROTATION CALIBRATION",move_robot,turn_robot,read_settled,status},spin_robot};
    fw_rotation_data_t result;
    int error=fw_rotation_run(&io,&g,&result); fw_motion_stop();
    if (error) return calibration_error(error);
    if (fw_app_rotation_commit(&result)) { message("ROTATION CALIBRATION","FLASH SAVE FAILED"); return -1; }
    return fw_rotation_report();
}
static void signed_distance_line(unsigned y,const char *label,int32_t um)
{
    char value[40]; uint32_t magnitude=(uint32_t)(um<0?-um:um);
    snprintf(value,sizeof value,"%s %c%lu.%lu MM",label,um<0?'-':'+',
             (unsigned long)(magnitude/1000),(unsigned long)(magnitude%1000/100)); line(y,value);
}
int fw_corner_report(void)
{
    unsigned pages=0;
    for (unsigned fixture=0;fixture<2;++fixture) {
        const fw_corner_data_t *d=fw_app_corner(fixture);
        if (!fw_corner_valid(d)) continue;
        for (unsigned facing=0;facing<2;++facing) for (unsigned v=0;v<FW_CAL_SPEEDS;++v)
            for (unsigned sensor=0;sensor<2;++sensor) {
                const fw_corner_point_t *p=&d->point[facing][v]; if (!(p->mask&(1u<<sensor))) continue;
                char title[32]; unsigned physical_side=facing?1-fixture:fixture;
                ssd1306ClearScreen(); snprintf(title,sizeof title,"%c%u OPEN %s %lu",physical_side?'R':'L',sensor?10:5,
                    facing?"FWD":"BACK",(unsigned long)p->speed); line(0,title);
                signed_distance_line(10,"RAW OPEN",p->raw_open_um[sensor]);
                signed_distance_line(18,"FILT OPEN",p->open_um[sensor]);
                signed_distance_line(26,"RAW CLOSE",p->raw_close_um[sensor]);
                signed_distance_line(34,"FILT CLOSE",p->close_um[sensor]);
                distance_line(42,"SPREAD",p->spread_um[sensor]);
                line(54,"RIGHT:NEXT BACK:END"); ssd1306Refresh(); ++pages;
                if (acknowledge()) return 0;
            }
    }
    if (!pages) { message("CORNER REPORT","NO SAVED MEASUREMENT"); return -1; }
    return 0;
}
static int corner_menu(unsigned side)
{
    fw_cal_geometry_t g; char value[32];
    if (fw_motion_busy()) return -1;
    if (configured_geometry(&g) || fw_cal_post_mm<0 || fw_cal_post_mm>250 ||
        fw_cal_post_mm*1000u<g.inner_um/2+50000) { message("INVALID GEOMETRY","CHECK POST POSITION"); return -1; }
    if (!fw_rotation_valid(fw_app_rotation()) || memcmp(&g,&fw_app_rotation()->geometry,sizeof g)) { message("CORNER CALIBRATION","RUN ROTATION FIRST"); return -1; }
    fw_motion_init(); ssd1306ClearScreen(); line(0,"CORNER CALIBRATION");
    line(12,side?"FRONT + RIGHT WALL":"FRONT + LEFT WALL");
    snprintf(value,sizeof value,"POST CENTER %d MM",fw_cal_post_mm); line(22,value);
    line(32,"CLEAR 2 CELLS BEHIND"); line(42,"BACK: STOP"); line(54,"RIGHT: START");
    ssd1306Refresh(); if (acknowledge()) return -1;
    const fw_cal_io_t io={"CORNER CALIBRATION",move_robot,turn_robot,read_settled,status};
    fw_corner_data_t result;
    int error=fw_corner_run(&io,&g,side,fw_cal_post_mm*1000u,&result); fw_motion_stop();
    if (error) return calibration_error(error);
    if (fw_app_corner_commit(&result)) { message("CORNER CALIBRATION","FLASH SAVE FAILED"); return -1; }
    return fw_corner_report();
}
int fw_corner_left_menu(void) { return corner_menu(0); }
int fw_corner_right_menu(void) { return corner_menu(1); }
