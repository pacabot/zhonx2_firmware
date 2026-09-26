#include "fw_app.h"
#include "fw_motion.h"
#include "hal/hal_os.h"
#include "hal/hal_sensor.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "oled/ssd1306.h"
#include <stdio.h>
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
    ssd1306ClearScreen(); line(0,title); line(20,text); line(54,"DROITE:OK RETOUR:FIN");
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
            (uint32_t)(hal_os_get_systicks()-start)>25000) { fw_motion_stop(); return -1; }
        if (observer && seen!=scan.sequence) {
            observer(context,fw_motion_travelled_um(),scan.raw); seen=scan.sequence;
        }
        __WFI();
    } while (fw_motion_busy());
    /* Complete debounce even if a threshold was crossed at the last pulse. */
    start=hal_os_get_systicks();
    do {
        hal_sensor_snapshot scan;
        if (cancelled() || fw_motion_fault() || !fresh(&scan)) { fw_motion_stop(); return -1; }
        if (observer && seen!=scan.sequence) {
            observer(context,fw_motion_travelled_um(),scan.raw); seen=scan.sequence;
        }
        __WFI();
    } while ((uint32_t)(hal_os_get_systicks()-start)<50);
    return 0;
}
static int move_robot(void *context,int32_t um,unsigned speed,fw_cal_observer observer,void *data)
{
    (void)context;
    if (cancelled() || fw_motion_calibration_move(um,speed)) return -1;
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
    (void)context; char value[24];
    ssd1306ClearScreen(); line(0,"CALIBRATION IR"); line(16,text);
    snprintf(value,sizeof value,"AXE-MUR %ld.%ld MM",(long)(um/1000),(long)(um%1000/100));
    line(32,value); line(54,"RETOUR: ARRET"); ssd1306Refresh();
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
    if (!fw_cal_valid(d)) { message("CALIBRATION IR","PAS DE MESURE SAUVEE"); return -1; }
    for (unsigned page=0;page<5;++page) {
        ssd1306ClearScreen();
        if (!page) {
            line(0,"IR SAUVE / AXE-MUR");
            distance_line(12,"AXE-AVANT",d->geometry.nose_um);
            distance_line(22,"LARGEUR",d->geometry.width_um);
            distance_line(32,"INTERIEUR",d->geometry.inner_um);
            distance_line(42,"PAS CASE",d->geometry.pitch_um);
        } else if (page<3) {
            const fw_cal_front_t *f=&d->front[page-1];
            line(0,page==1?"FRONT 5 CM / 3 ESSAIS":"FRONT 10CM / 3 ESSAIS");
            distance_line(12,"DECLENCHE",f->on_um);
            distance_line(22,"RELACHE",f->off_um);
            distance_line(32,"HYSTERESIS",f->off_um-f->on_um);
            distance_line(42,"DISPERSION",f->spread_um);
        } else {
            const fw_cal_side_t *s=&d->side[page-3];
            line(0,page==3?"GAUCHE 5CM / 3 ESSAIS":"DROITE 5CM / 3 ESSAIS");
            distance_line(12,"DETECTE <=",s->near_um);
            distance_line(22,"LIBRE >=",s->far_um);
            distance_line(32,"DISPERSION",s->spread_um);
            line(42,"STATIQUE, PAS 1 MM");
        }
        line(54,page==4?"DROITE:FIN RET:FIN":"DROITE:SUITE RET:FIN");
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
        message("GEOMETRIE INVALIDE","REGLER AXE / LARGEUR"); return -1;
    }
    fw_cal_geometry_t geometry={fw_cal_nose_tenth_mm*100u,fw_cal_width_tenth_mm*100u,
                               fw_cal_inner_mm*1000u,fw_cal_pitch_mm*1000u};
    if (!fw_cal_geometry_valid(&geometry)) {
        message("GEOMETRIE INVALIDE","REGLER AXE / LARGEUR"); return -1;
    }
    fw_motion_init();
    ssd1306ClearScreen(); line(0,"CALIBRATION IR");
    line(12,"AU CENTRE, 3 MURS"); line(22,"ARRIERE LIBRE 10 CM");
    line(32,"APPUI INITIAL BORNE"); line(42,"RETOUR: ARRET"); line(54,"DROITE: LANCER");
    ssd1306Refresh(); if (acknowledge()) return -1;
    const fw_cal_io_t io={0,move_robot,turn_robot,read_settled,status};
    fw_cal_data_t result;
    int error=fw_cal_run(&io,&geometry,&result);
    fw_motion_stop();
    if (error) {
        const char *text=error==FW_CAL_WALLS?"VERIFIER LES 3 MURS":
            error==FW_CAL_RANGE?"SEUIL HORS PLAGE":error==FW_CAL_UNSTABLE?"MESURES INSTABLES":"ARRET OU CAPTEURS";
        if (!cancelled()) message("CALIBRATION ARRETEE",text);
        return -1;
    }
    if (fw_app_calibration_commit(&result)) { message("CALIBRATION IR","ECHEC SAUVEGARDE"); return -1; }
    return fw_calibration_report();
}
