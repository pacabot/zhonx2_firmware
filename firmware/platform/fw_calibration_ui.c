#include "fw_app.h"
#include "fw_menu.h"
#include "fw_display.h"
#include "fw_text.h"
#include "fw_motion.h"
#include "fw_buttons.h"
#include "hal/hal_os.h"
#include "hal/hal_sensor.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "oled/ssd1306.h"
#include <stdio.h>
#include <string.h>
static const char *io_error;
static char last_stage[22];
static hal_sensor_snapshot last_scan;
static int calibration_error(int error);
static int cancelled(void) { return fw_cancel_pressed(); }
static int fresh(hal_sensor_snapshot *s)
{
    int available=hal_sensor_snapshot_read(s);
    last_scan=*s;
    return available && (uint32_t)(hal_os_get_systicks()-s->timestamp)<=50;
}
static void line(unsigned y,const char *text) {
    if(!y)fw_ui_header(text);else if(y>=54)fw_ui_hint(text);
    else fw_display_text(0,y,text);
}
static int acknowledge(void)
{
    while (fw_select_pressed()) { if (cancelled()) return -1; __WFI(); }
    while (!fw_select_pressed()) { if (cancelled()) return -1; __WFI(); }
    while (fw_select_pressed()) { if (cancelled()) return -1; __WFI(); }
    return 0;
}
static void message(const char *title,const char *text)
{
    char row[16];ssd1306ClearScreen();line(0,title);
    for(unsigned y=18;*text && y<=48;y+=15) {
        text=fw_text_line(text,row,15);fw_display_text(0,y,row);
    }
    fw_ui_menu_refresh();(void)acknowledge();
}
/* Each report page has three readable rows and a scrollbar. Up/Down revisit
 * pages; OK advances, including exiting at the end; Left/Escape exits anywhere. */
typedef struct {char title[12],row[3][16];} report_page;
static report_page report[80];
static unsigned report_count;
static report_page *page(const char *title)
{
    if(report_count>=80)return &report[79];
    report_page *p=&report[report_count++];memset(p,0,sizeof *p);
    snprintf(p->title,sizeof p->title,"%s",title);return p;
}
static void mm(report_page *p,unsigned row,const char *label,int32_t um)
{
    unsigned n=(unsigned)(um<0?-(int64_t)um:um);
    if(n>999999) {snprintf(p->row[row],16,"OUT OF RANGE");return;}
    snprintf(p->row[row],sizeof p->row[row],"%.6s %s%u.%umm",label,um<0?"-":"",n/1000,n%1000/100);
}
static int show_report(void)
{
    unsigned index=0;
    while(report_count) {
        ssd1306ClearScreen();fw_ui_header(report[index].title);
        for(unsigned i=0;i<3;++i)fw_display_text(0,16+i*16,report[index].row[i]);
        fw_display_scroll(index,report_count);fw_ui_menu_refresh();
        unsigned key=0;
        while(!key) {
            if(cancelled())return 0;
            if(fw_select_pressed())key=1;
            else if(!(GPIOC->IDR&FW_UP_PIN))key=2;
            else if(!(GPIOC->IDR&FW_DOWN_PIN))key=3;
            else __WFI();
        }
        while(fw_select_pressed() || !(GPIOC->IDR&FW_UP_PIN) || !(GPIOC->IDR&FW_DOWN_PIN)) {
            if(cancelled())return 0;
            __WFI();
        }
        if(key==2) {if(index)--index;}
        else if(index+1<report_count)++index;
        else if(key==1)return 0;
    }
    return 0;
}
/* Desired placement, viewed from above. fixture: 0=three walls,
 * 1=left corner, 2=right corner. This is a guide, not a measured robot pose. */
static int placement(const char *title,unsigned fixture)
{
    io_error=0; snprintf(last_stage,sizeof last_stage,"CHECK FIXTURE");
    memset(&last_scan,0,sizeof last_scan);
    ssd1306ClearScreen(); line(0,title);
    /* Two cells behind the reference corner: dotted edges are unrestricted.
     * The side containing the measured post MUST be open in the next cell. */
    unsigned x=14, y=16, pitch=18;
    ssd1306FillRect(x,y,21,2);
    if(fixture!=2) ssd1306FillRect(x,y,2,pitch+2);
    if(fixture!=1) ssd1306FillRect(x+19,y,2,pitch+2);
    if(fixture) {
        unsigned post=fixture==1?x:x+19;
        ssd1306FillRect(post-1,y+pitch-1,4,4);
        /* Only the far-side boundaries of the straight are irrelevant. */
        unsigned other=fixture==1?x+20:x;
        ssd1306DrawDashedLine(other,y+pitch,other,60);
        ssd1306DrawDashedLine(post,y+2*pitch,post,60);
        ssd1306DrawDashedLine(x-10,y+pitch,x-2,y+pitch);
        ssd1306DrawDashedLine(x+23,y+pitch,x+31,y+pitch);
    }
    ssd1306DrawLine(x+10,y+21,x+10,59); /* Clear travel axis. */
    ssd1306DrawLine(x+7,55,x+10,59); ssd1306DrawLine(x+10,59,x+13,55);
    ssd1306DrawRect(x+5,y+5,11,11);
    ssd1306FillRect(x+3,y+9,2,5); ssd1306FillRect(x+16,y+9,2,5);
    ssd1306DrawLine(x+10,y+14,x+10,y+7);
    ssd1306DrawLine(x+7,y+10,x+10,y+7); ssd1306DrawLine(x+10,y+7,x+13,y+10);
    fw_display_text(57,17,fixture?"2 cells":"10cm");
    fw_display_text(57,32,"clear");
    if(fixture) {ssd1306DrawDashedLine(57,47,120,47);fw_display_text(57,51,"Any wall");}
    else fw_display_text(57,48,"behind");
    fw_ui_menu_refresh();
    return acknowledge();
}
static int healthy(hal_sensor_snapshot *scan)
{
    if (cancelled()) { io_error="BACK PRESSED"; return 0; }
    if (fw_motion_fault()) { io_error="MOTION FAULT"; return 0; }
    if (!fresh(scan)) { io_error="IR DATA STALE"; return 0; }
    return 1;
}
static int read_settled(void *context,uint8_t stable_mask,uint8_t *raw)
{
    (void)context;
    uint32_t start=hal_os_get_systicks(),seen=0; unsigned count=0; uint8_t previous=0;
    while ((uint32_t)(hal_os_get_systicks()-start)<1000) {
        hal_sensor_snapshot scan;
        if (!healthy(&scan)) return -1;
        if (scan.sequence!=seen) {
            seen=scan.sequence;
            /* An unrelated sensor at its threshold must not block this read. */
            count=((scan.raw^previous)&stable_mask)?1:count+1; previous=scan.raw;
            if (count>=5) { *raw=previous; return 0; }
        }
        __WFI();
    }
    io_error="IR NOT STABLE";
    return -1;
}
static int wait_move(fw_cal_observer observer,void *context)
{
    uint32_t start=hal_os_get_systicks(),seen=0;
    do {
        hal_sensor_snapshot scan;
        if (!healthy(&scan)) { fw_motion_stop(); return -1; }
        if ((uint32_t)(hal_os_get_systicks()-start)>55000) {
            io_error="MOVE TIMEOUT"; fw_motion_stop(); return -1;
        }
        if (observer && seen!=scan.sequence) {
            observer(context,fw_motion_travelled_um(),scan.raw,scan.filtered); seen=scan.sequence;
        }
        __WFI();
    } while (fw_motion_busy());
    /* Complete debounce even if a threshold was crossed at the last pulse. */
    start=hal_os_get_systicks();
    do {
        hal_sensor_snapshot scan;
        if (!healthy(&scan)) { fw_motion_stop(); return -1; }
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
    if (result) { io_error="MOVE REJECTED"; return -1; }
    return wait_move(observer,data);
}
static int turn_robot(void *context,int degrees)
{
    (void)context;
    if (cancelled()) return -1;
    if (fw_motion_calibration_turn(degrees)) { io_error="TURN REJECTED"; return -1; }
    return wait_move(0,0);
}
static void status(void *context,const char *text,int32_t um)
{
    (void)context;char row[16],value[24];
    snprintf(last_stage,sizeof last_stage,"%s",text);
    ssd1306ClearScreen();line(0,"Calibrating");
    text=fw_text_line(text,row,15);fw_display_text(0,18,row);
    fw_text_line(text,row,15);fw_display_text(0,33,row);
    snprintf(value,sizeof value,"%ld.%ld mm",(long)(um/1000),(long)(um%1000/100));
    if(um)fw_display_text(0,49,value);
    fw_ui_menu_refresh();
}
int fw_calibration_report(void)
{
    const fw_cal_data_t *d=fw_app_calibration();report_count=0;
    if(!fw_cal_valid(d)) {message("Walls","MISSING: run wall calibration");return -1;}
    report_page *p=page("Geometry");
    mm(p,0,"Nose",d->geometry.nose_um);mm(p,1,"Width",d->geometry.width_um);mm(p,2,"Clear",d->geometry.inner_um);
    p=page("Geometry");mm(p,0,"Pitch",d->geometry.pitch_um);
    snprintf(p->row[1],16,"3 samples");snprintf(p->row[2],16,"1mm side steps");
    for(unsigned i=0;i<2;++i) {
        p=page(i?"Front 10cm":"Front 5cm");mm(p,0,"On",d->front[i].on_um);
        mm(p,1,"Off",d->front[i].off_um);mm(p,2,"Spread",d->front[i].spread_um);
        p=page(i?"Front 10cm":"Front 5cm");mm(p,0,"Hyst.",d->front[i].off_um-d->front[i].on_um);
        p=page(i?"Right 5cm":"Left 5cm");mm(p,0,"Near",d->side[i].near_um);
        mm(p,1,"Far",d->side[i].far_um);mm(p,2,"Spread",d->side[i].spread_um);
    }
    return show_report();
}
int fw_calibrate_menu(void)
{
    if (fw_motion_busy()) return -1;
    if (fw_cal_nose_tenth_mm<100 || fw_cal_nose_tenth_mm>750 ||
        fw_cal_width_tenth_mm<400 || fw_cal_width_tenth_mm>1400 ||
        fw_cal_inner_mm<140 || fw_cal_inner_mm>190 || fw_cal_pitch_mm>210 ||
        fw_cal_pitch_mm<=fw_cal_inner_mm) {
        message("Geometry","SET NOSE / WIDTH"); return -1;
    }
    fw_cal_geometry_t geometry={fw_cal_nose_tenth_mm*100u,fw_cal_width_tenth_mm*100u,
                               fw_cal_inner_mm*1000u,fw_cal_pitch_mm*1000u};
    if (!fw_cal_geometry_valid(&geometry)) {
        message("Geometry","SET NOSE / WIDTH"); return -1;
    }
    fw_motion_init();
    if (placement("Walls",0)) return -1;
    const fw_cal_io_t io={0,move_robot,turn_robot,read_settled,status};
    fw_cal_data_t result;
    int error=fw_cal_run(&io,&geometry,&result);
    fw_motion_stop();
    if (error) return calibration_error(error);
    if (fw_app_calibration_commit(&result)) { message("Walls","FLASH SAVE FAILED"); return -1; }
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
    if (cancelled()) return -1;
    if (fw_motion_calibration_spin(um,speed)) { io_error="SPIN REJECTED"; return -1; }
    return wait_move(observer,data);
}
static int calibration_error(int error)
{
    int motion_fault=fw_motion_fault();
    fw_motion_stop();
    if (!cancelled()) {
        char text[32];
        const char *reason=error==FW_CAL_RANGE?"NO EDGE IN RANGE":
            error==FW_CAL_UNSTABLE?"UNSTABLE MEASUREMENTS":
            error==FW_CAL_WALLS?"CHECK FIXTURE WALLS":io_error?io_error:"MOVE FAILED";
        report_count=0;report_page *p=page("Cal stopped");
        const char *rest=fw_text_line(reason,p->row[0],15);
        fw_text_line(rest,p->row[1],15);
        snprintf(text,sizeof text,"Error %d / M%d",-error,motion_fault);
        snprintf(p->row[2],16,"%.15s",text);
        p=page("Last stage");rest=fw_text_line(last_stage,p->row[0],15);
        fw_text_line(rest,p->row[1],15);
        snprintf(text,sizeof text,"F5:%u F10:%u",!(last_scan.raw&SENSOR_F5_POS),!(last_scan.raw&SENSOR_F10_POS));
        snprintf(p->row[2],16,"%.15s",text);show_report();
    }
    return -1;
}
int fw_rotation_report(void)
{
    const fw_rotation_data_t *d=fw_app_rotation();report_count=0;
    if(!fw_rotation_valid(d)) {message("Rotation","MISSING: run rotation calibration");return -1;}
    for(unsigned i=0;i<FW_CAL_SPEEDS;++i)for(unsigned dir=0;dir<2;++dir) {
        const fw_rotation_point_t *p=&d->point[i];char title[12];
        snprintf(title,sizeof title,"%s%lu mm/s",dir?"CCW":"CW",(unsigned long)p->speed);
        report_page *r=page(title);
        mm(r,0,"Track",(uint32_t)((uint64_t)p->quarter_um[dir]*4000000/3141593));
        mm(r,1,"90 arc",p->quarter_um[dir]);mm(r,2,"Spread",p->spread_um[dir]);
        r=page(title);snprintf(r->row[0],16,"90 deg check:");
        if(p->quarter_checks[dir])snprintf(r->row[1],16,"%lu.%lu deg",(unsigned long)(p->quarter_error_mdeg[dir]/1000),
            (unsigned long)(p->quarter_error_mdeg[dir]%1000/100));
        else {snprintf(r->row[1],16,"MISSING");snprintf(r->row[2],16,"Period / 4 only");}
    }
    return show_report();
}
int fw_rotation_menu(void)
{
    fw_cal_geometry_t g;
    if (fw_motion_busy()) return -1;
    if (configured_geometry(&g)) { message("Geometry","SET NOSE / WIDTH"); return -1; }
    fw_motion_init();
    if (placement("Rotation",0)) return -1;
    const fw_cal_extra_io_t io={{"Rotation",move_robot,turn_robot,read_settled,status},spin_robot};
    fw_rotation_data_t result;
    int error=fw_rotation_run(&io,&g,&result); fw_motion_stop();
    if (error) return calibration_error(error);
    if (fw_app_rotation_commit(&result)) { message("Rotation","FLASH SAVE FAILED"); return -1; }
    return fw_rotation_report();
}
static int corner_report(int selected)
{
    report_count=0;report_page *r=page("Edge status");
    int valid[2]={fw_corner_valid(fw_app_corner(0)),fw_corner_valid(fw_app_corner(1))};
    snprintf(r->row[0],16,"Left: %s",valid[0]?"SAVED":"MISSING");
    snprintf(r->row[1],16,"Right: %s",valid[1]?"SAVED":"MISSING");
    unsigned missing5=0;
    for(unsigned side=0;side<2;++side)if(valid[side])
        for(unsigned f=0;f<2;++f)for(unsigned v=0;v<FW_CAL_SPEEDS;++v)
            missing5+=!(fw_app_corner(side)->point[f][v].mask&1);
    snprintf(r->row[2],16,"%s",!valid[0]||!valid[1]?"INCOMPLETE":missing5?"5cm MISSING":"Both fixtures");
    for(unsigned fixture=0;fixture<2;++fixture) {
        if((selected>=0 && fixture!=(unsigned)selected) || !valid[fixture])continue;
        const fw_corner_data_t *d=fw_app_corner(fixture);
        for(unsigned facing=0;facing<2;++facing)for(unsigned v=0;v<FW_CAL_SPEEDS;++v)
            for(unsigned sensor=0;sensor<2;++sensor) {
                const fw_corner_point_t *p=&d->point[facing][v];
                for(unsigned filtered=0;filtered<2;++filtered) {
                    r=page(fixture?"Right setup":"Left setup");
                    snprintf(r->row[0],16,"%c%u %s %lu",(fixture^facing)?'R':'L',sensor?10:5,
                        facing?"FWD":"BACK",(unsigned long)p->speed);
                    if(!(p->mask&(1u<<sensor))) {snprintf(r->row[1],16,"5cm: NO WALL");snprintf(r->row[2],16,"At start pose");break;}
                    mm(r,1,filtered?"Open":"Raw O",filtered?p->open_um[sensor]:p->raw_open_um[sensor]);
                    mm(r,2,filtered?"Close":"Raw C",filtered?p->close_um[sensor]:p->raw_close_um[sensor]);
                }
                if(p->mask&(1u<<sensor)) {
                    r=page(fixture?"Right setup":"Left setup");
                    snprintf(r->row[0],16,"%c%u %s %lu",(fixture^facing)?'R':'L',sensor?10:5,
                        facing?"FWD":"BACK",(unsigned long)p->speed);
                    mm(r,1,"Spread",p->spread_um[sensor]);
                }
            }
    }
    return show_report();
}
int fw_corner_report(void) { return corner_report(-1); }
static int corner_menu(unsigned side)
{
    fw_cal_geometry_t g;
    if (fw_motion_busy()) return -1;
    if (configured_geometry(&g) || fw_cal_post_mm<0 || fw_cal_post_mm>250 ||
        fw_cal_post_mm*1000u<g.inner_um/2+50000) { message("Geometry","CHECK POST POSITION"); return -1; }
    if (!fw_rotation_valid(fw_app_rotation()) || memcmp(&g,&fw_app_rotation()->geometry,sizeof g)) { message("Edge setup","RUN ROTATION FIRST"); return -1; }
    fw_motion_init();
    if (placement(side?"Right setup":"Left setup",side+1)) return -1;
    const fw_cal_io_t io={"Edge setup",move_robot,turn_robot,read_settled,status};
    fw_corner_data_t result;
    int error=fw_corner_run(&io,&g,side,fw_cal_post_mm*1000u,&result); fw_motion_stop();
    if (error) return calibration_error(error);
    if (fw_app_corner_commit(&result)) { message("Edge setup","FLASH SAVE FAILED"); return -1; }
    return corner_report((int)side);
}
int fw_corner_left_menu(void) { return corner_menu(0); }
int fw_corner_right_menu(void) { return corner_menu(1); }
