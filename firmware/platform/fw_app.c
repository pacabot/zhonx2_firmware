#include "fw_app.h"
#include "fw_start.h"
#include "fw_ui.h"
#include "fw_layout.h"
#include "fw_store.h"
#include "fw_motion.h"
#include "nimes.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"
#include "app/app_def.h"
#include "hal/hal_os.h"
#include "hal/hal_step_motor.h"
#include <stdio.h>
#include <string.h>
#define SNAPSHOT_SCHEMA 4
/* ARM ABI-specific snapshot. Change schema for any layout or settings ABI change. */
typedef struct {
    robot_settings settings;
    nm_map_t map;
    uint32_t corner, heading, search_speed, run_speed, search_ms, has_map;
} snapshot_v2_t;
typedef struct { snapshot_v2_t base; fw_cal_data_t calibration; } snapshot_v3_t;
typedef struct {
    fw_cal_data_t wall;
    fw_rotation_data_t rotation;
    fw_corner_data_t corner[2];
} calibration_bundle_t;
typedef struct { snapshot_v2_t base; calibration_bundle_t measurements; } snapshot_t;
_Static_assert(sizeof(snapshot_t)%4==0,"snapshot alignment");
static nm_map_t maze;
static uint32_t search_ms, search_epoch, search_base;
static int search_clock_running;
static uint32_t search_used(uint32_t now)
{
    uint32_t elapsed=search_clock_running ? now-search_epoch : 0;
    uint32_t base=search_clock_running ? search_base : search_ms;
    return elapsed>=NM_SEARCH_MS-base ? NM_SEARCH_MS : base+elapsed;
}
static int has_map;
static calibration_bundle_t measurements;
int fw_cal_nose_tenth_mm=470, fw_cal_width_tenth_mm=940;
int fw_cal_inner_mm=167, fw_cal_pitch_mm=179, fw_cal_post_mm=173;
const fw_cal_data_t *fw_app_calibration(void) { return &measurements.wall; }
const fw_rotation_data_t *fw_app_rotation(void) { return &measurements.rotation; }
const fw_corner_data_t *fw_app_corner(unsigned side) { return side<2?&measurements.corner[side]:0; }
static int bundle_valid(const calibration_bundle_t *b)
{
    if ((b->wall.valid && !fw_cal_valid(&b->wall)) ||
        (b->rotation.valid && !fw_rotation_valid(&b->rotation))) return 0;
    const fw_cal_geometry_t *g=b->wall.valid?&b->wall.geometry:
        b->rotation.valid?&b->rotation.geometry:0;
    if (b->rotation.valid && g && memcmp(g,&b->rotation.geometry,sizeof *g)) return 0;
    for (unsigned i=0;i<2;++i) if (b->corner[i].valid) {
        if (!fw_corner_valid(&b->corner[i]) || b->corner[i].side!=i) return 0;
        if (g && memcmp(g,&b->corner[i].geometry,sizeof *g)) return 0;
        g=&b->corner[i].geometry;
    }
    return 1;
}
static void apply_calibration(void)
{
    const fw_cal_data_t *wall=&measurements.wall;
    const fw_cal_geometry_t *g=wall->valid?&wall->geometry:measurements.rotation.valid?
        &measurements.rotation.geometry:measurements.corner[0].valid?&measurements.corner[0].geometry:
        measurements.corner[1].valid?&measurements.corner[1].geometry:0;
    if (g) {
        fw_cal_nose_tenth_mm=(int)g->nose_um/100; fw_cal_width_tenth_mm=(int)g->width_um/100;
        fw_cal_inner_mm=(int)g->inner_um/1000; fw_cal_pitch_mm=(int)g->pitch_um/1000;
    }
    fw_motion_geometry(g?g->pitch_um:179000,wall->valid?wall->front[0].on_um:0,g?g->inner_um:167000);
    fw_motion_rotation_profile(&measurements.rotation);
    if (measurements.corner[0].valid) fw_cal_post_mm=(int)measurements.corner[0].post_um/1000;
    else if (measurements.corner[1].valid) fw_cal_post_mm=(int)measurements.corner[1].post_um/1000;
}
static nm_pose_t displayed_pose;
volatile unsigned fw_last_stop_code;
int fw_start_corner=0, fw_start_heading=NM_NORTH, fw_search_speed=220, fw_run_speed=260;
static int parameters_valid(void)
{
    return fw_start_corner>=0 && fw_start_corner<=3 && fw_start_heading>=0 && fw_start_heading<4 &&
        fw_search_speed>=20 && fw_search_speed<=300 && fw_run_speed>=20 && fw_run_speed<=300;
}
static int settings_valid(const robot_settings *s)
{
    return s->initial_speed>0 && s->initial_speed<=MAX_SPEED && s->default_accel>0 &&
        s->default_accel<=1000 && s->rotate_accel>0 && s->rotate_accel<=1000 &&
        s->correction_p>0 && s->correction_i>0 && s->max_correction<=MAX_SPEED &&
        s->max_speed_distance<=1000000 && s->emergency_decel>0 && s->emergency_decel<=10000;
}
static int load(void)
{
    snapshot_t current={0}; snapshot_v2_t s;
    int legacy=0;
    if (!fw_store_load(&fw_stm32_flash,SNAPSHOT_SCHEMA,&current,sizeof current)) {
        if (!bundle_valid(&current.measurements)) return -1;
        s=current.base;
    } else {
        snapshot_v3_t old;
        if (!fw_store_load(&fw_stm32_flash,3,&old,sizeof old)) {
            if (old.calibration.valid && !fw_cal_valid(&old.calibration)) return -1;
            s=old.base; current.measurements.wall=old.calibration;
        } else if (fw_store_load(&fw_stm32_flash,2,&s,sizeof s)) {
            legacy=1;
            if (fw_store_load(&fw_stm32_flash,1,&s,sizeof s)) return -1;
        }
    }
    if (!nm_valid(&s.map) ||
        !settings_valid(&s.settings) || s.corner>3 || s.heading>3 || s.search_speed<20 ||
        s.search_speed>300 || s.run_speed<20 || s.run_speed>300 || s.has_map>1 ||
        s.search_ms>NM_SEARCH_MS) return -1;
    zhonxSettings=s.settings; maze=s.map; has_map=s.has_map; search_ms=s.search_ms;
    search_clock_running=0;
    fw_start_corner=s.corner; fw_start_heading=s.heading;
    fw_search_speed=legacy && s.search_speed==120 ? 220 : (int)s.search_speed;
    fw_run_speed=legacy && s.run_speed==200 ? 260 : (int)s.run_speed;
    measurements=current.measurements; apply_calibration();
    return 0;
}
void fw_app_init(void) { nm_init(&maze); memset(&measurements,0,sizeof measurements); apply_calibration(); (void)load(); }
static int save(void)
{
    if (fw_motion_busy() || !parameters_valid() || !settings_valid(&zhonxSettings)) return -1;
    search_ms=search_used(hal_os_get_systicks());
    snapshot_t s; memset(&s,0,sizeof s);
    s.base.settings=zhonxSettings; s.base.map=maze; s.base.has_map=has_map; s.base.search_ms=search_ms;
    s.base.corner=fw_start_corner; s.base.heading=fw_start_heading;
    s.base.search_speed=fw_search_speed; s.base.run_speed=fw_run_speed;
    s.measurements=measurements;
    /* Single-bank flash stalls instruction fetch. Never save while motors move. */
    fw_motion_stop();
    return fw_store_save(&fw_stm32_flash,SNAPSHOT_SCHEMA,&s,sizeof s);
}
static void discard_other_geometry(const fw_cal_geometry_t *g)
{
    if (memcmp(g,&measurements.wall.geometry,sizeof *g)) memset(&measurements.wall,0,sizeof measurements.wall);
    if (memcmp(g,&measurements.rotation.geometry,sizeof *g)) memset(&measurements.rotation,0,sizeof measurements.rotation);
    for (unsigned i=0;i<2;++i) if (memcmp(g,&measurements.corner[i].geometry,sizeof *g))
        memset(&measurements.corner[i],0,sizeof measurements.corner[i]);
}
static int commit_measurements(const calibration_bundle_t *previous)
{
    int result=save();
    if (result) measurements=*previous;
    apply_calibration();
    return result;
}
int fw_app_calibration_commit(const fw_cal_data_t *data)
{
    if (!fw_cal_valid(data) || fw_motion_busy()) return -1;
    calibration_bundle_t previous=measurements;
    discard_other_geometry(&data->geometry); measurements.wall=*data;
    return commit_measurements(&previous);
}
int fw_app_rotation_commit(const fw_rotation_data_t *data)
{
    if (!fw_rotation_valid(data) || fw_motion_busy()) return -1;
    calibration_bundle_t previous=measurements;
    discard_other_geometry(&data->geometry); measurements.rotation=*data;
    /* Corner measurements depend on the rotation used to face each fixture. */
    memset(measurements.corner,0,sizeof measurements.corner);
    return commit_measurements(&previous);
}
int fw_app_corner_commit(const fw_corner_data_t *data)
{
    if (!fw_corner_valid(data) || fw_motion_busy()) return -1;
    calibration_bundle_t previous=measurements;
    discard_other_geometry(&data->geometry); measurements.corner[data->side]=*data;
    return commit_measurements(&previous);
}
int fw_app_save(void)
{
    int result=save();
    hal_ui_display_prompt(app_context.ui,"SAVE",result ? "FLASH SAVE FAILED" : "SETTINGS AND MAP SAVED");
    return result;
}
int fw_app_restore(void)
{
    if (fw_motion_busy()) return -1;
    int result=load();
    hal_ui_display_prompt(app_context.ui,"RESTORE",result ? "NO SAVED DATA" : "SETTINGS AND MAP LOADED");
    return result;
}
static nm_pose_t start_pose(void)
{
    /* 0=SW, 1=SE, 2=NE, 3=NW; heading 0=N,1=E,2=S,3=W. */
    nm_pose_t p={(fw_start_corner==1 || fw_start_corner==2)?8:0,
                 fw_start_corner>=2?8:0,(uint8_t)fw_start_heading};
    return p;
}
static int cell(nm_pose_t p) { return p.y*NM_SIDE+p.x; }
static void advance(nm_pose_t *p, unsigned count)
{
    for (unsigned i=0;i<count;++i) {
        int n=nm_neighbour(cell(*p),p->heading);
        p->x=n%NM_SIDE; p->y=n/NM_SIDE;
    }
}
static uint8_t observation(uint8_t s)
{
    return (!(s&SENSOR_F10_POS)) | (!(s&SENSOR_L10_POS)<<1) | (!(s&SENSOR_R10_POS)<<2);
}
static void radio_off(void)
{
    /* Nimes forbids external communication during autonomous trials. */
    USART1->CR1 &= ~USART_CR1_UE;
    USART2->CR1 &= ~USART_CR1_UE;
    USART3->CR1 &= ~USART_CR1_UE;
    UART4->CR1 &= ~USART_CR1_UE; UART5->CR1 &= ~USART_CR1_UE; USART6->CR1 &= ~USART_CR1_UE;
}
static uint8_t live_sensors(void)
{
    hal_sensor_snapshot scan;
    return hal_sensor_snapshot_read(&scan) ? scan.filtered : 0x3f;
}
static int wait_hand(nm_pose_t pose)
{
    fw_start_gate_t gate={0}; uint32_t draw=hal_os_get_systicks()-100;
    for (;;) {
        uint32_t now=hal_os_get_systicks(); hal_sensor_snapshot scan;
        if (!(GPIOC->IDR & GPIO_Pin_13)) return -1;
        int fresh=hal_sensor_snapshot_read(&scan) && now-scan.timestamp<=50;
        if (fw_start_gate(&gate,now,fresh && !(scan.filtered&SENSOR_F10_POS),fresh)) return 0;
        if (now-draw>=100) {
            fw_ui_maze(&maze,pose,gate.state==FW_WAIT_HAND?"HAND IN FRONT OF F10":"REMOVE YOUR HAND",
                       fw_search_speed,0,fresh?scan.filtered:0x3f);
            draw=now;
        }
        __WFI();
    }
}
static void map_message(nm_pose_t pose, const char *message)
{
    fw_ui_maze(&maze,pose,message,0,search_ms,live_sensors());
    /* Keep the map and the full stop reason visible until acknowledged. */
    while ((GPIOC->IDR & (GPIO_Pin_11|GPIO_Pin_13))==(GPIO_Pin_11|GPIO_Pin_13)) __WFI();
    while (!(GPIOC->IDR & GPIO_Pin_11)) __WFI();
}
int fw_app_show_map(void)
{
    if (!has_map) { hal_ui_display_prompt(app_context.ui,"MAP","NO MAP"); return -1; }
    map_message(displayed_pose,"SAVED MAP"); return 0;
}
/* Foreground state machine. The pulse IRQs, 200 Hz acquisition phases and
 * 1 kHz motion controller run independently while path planning/OLED run here. */
static int execute(int timed_run, int fresh)
{
    if (!parameters_valid()) {
        hal_ui_display_prompt(app_context.ui,"9x9","INVALID SETTINGS"); return -1;
    }
    if (timed_run && !has_map) {
        hal_ui_display_prompt(app_context.ui,"9x9","EXPLORE OR LOAD A MAP"); return -1;
    }
    fw_motion_init();
    if (fresh) { nm_init(&maze); search_ms=0; has_map=1; search_clock_running=0; }
    nm_pose_t origin=start_pose(), pose=origin;
    displayed_pose=pose; fw_last_stop_code=0;
    if (wait_hand(pose)) return -1;
    radio_off();
    if (timed_run) { search_ms=search_used(hal_os_get_systicks()); search_clock_running=0; }
    else if (!search_clock_running) {
        search_base=search_ms; search_epoch=hal_os_get_systicks(); search_clock_running=1;
    }
    nm_route_t route={0}, prepared={0};
    uint8_t goals[NM_CELLS], home[NM_CELLS]={0}; home[cell(origin)]=1;
    uint32_t started=hal_os_get_systicks(), stopped=started, ui_time=started, seen=0;
    unsigned moving=0, turn_heading=0, wall_reached=0;
    uint32_t mismatch_since=0; int mismatch=0;
    int turning=0, returning=0, certified=0, prepared_ok=0, state=0, result=-1, at_start=1;
    const char *message="STOPPED";
    for (;;) {
        uint32_t now=hal_os_get_systicks();
        if (!(GPIOC->IDR & GPIO_Pin_13)) { message="USER STOP"; break; }
        if (!timed_run && search_used(now)>=NM_SEARCH_MS) { message="5 MIN ELAPSED"; break; }
        if (fw_motion_fault()) {
            fw_last_stop_code=(unsigned)fw_motion_fault();
            message=fw_motion_fault()==2?"EARLY OBSTACLE":"SENSOR OR TIMEOUT"; break;
        }
        if (now-ui_time>=100) {
            fw_ui_maze(&maze,pose,returning?"RETURN":timed_run?"RUN":"EXPLORATION",
                       fw_motion_busy()?(unsigned)(timed_run?fw_run_speed:fw_search_speed):0,
                       timed_run?now-started:search_used(now),live_sensors());
            ui_time=now;
        }
        if (state==1) { /* TURN or MOVE in progress; plan from its destination now. */
            if (!prepared_ok && moving && (returning || timed_run)) {
                nm_pose_t destination=pose; advance(&destination,moving);
                prepared_ok=!nm_route(&maze,destination,returning?home:goals,0,&prepared);
            }
            if (fw_motion_busy()) { __WFI(); continue; }
            if (turning) { pose.heading=turn_heading; wall_reached=0; }
            else { advance(&pose,moving); wall_reached=fw_motion_wall_arrival(); }
            displayed_pose=pose;
            moving=0; turning=0; stopped=now; seen=0; state=0;
        }
        /* Only map at stopped cell centres, after three full stable scan periods.
         * Interior cells of grouped moves are already mapped before that move. */
        hal_sensor_snapshot scan;
        if (!hal_sensor_snapshot_read(&scan) || now-scan.timestamp>50 || now-stopped<40 || scan.sequence==seen) {
            __WFI(); continue;
        }
        seen=scan.sequence;
        if (nm_observe(&maze,pose,observation(scan.filtered) | (wall_reached?1u:0u))) {
            /* Re-read before abandoning the run for a single binary-sensor disagreement. */
            if (!mismatch) { mismatch=1; mismatch_since=now; }
            if (now-mismatch_since<150) { __WFI(); continue; }
            message="MAP CONFLICT"; fw_last_stop_code=3; break;
        }
        mismatch=0;
        if (at_start && maze.cell[cell(pose)].known==15) {
            unsigned walls=maze.cell[cell(pose)].walls, count=0;
            for (unsigned i=0;i<4;++i) count+=(walls>>i)&1u;
            if (count!=3) { message="CHECK START CELL"; break; }
            at_start=0;
        }
        int rooms=nm_goal(&maze,goals);
        if (rooms>1) { message="AMBIGUOUS GOAL"; break; }
        if (timed_run) {
            if (rooms!=1) { message="UNKNOWN GOAL"; break; }
            if (goals[cell(pose)]) { message="GOAL FOUND"; result=0; break; }
            if (prepared_ok) route=prepared;
            else if (nm_route(&maze,pose,goals,0,&route)) { message="NO PATH"; break; }
        } else if (returning) {
            if (home[cell(pose)]) { message=certified?"OPTIMAL PATH":"BACK AT START"; result=0; break; }
            if (prepared_ok) route=prepared;
            else if (nm_route(&maze,pose,home,0,&route)) { message="NO RETURN PATH"; break; }
        } else if (rooms==1) {
            int r=nm_refine(&maze,pose,origin,goals,&route);
            if (r==1) { certified=1; returning=1; continue; }
            if (r<0) { message="INCOMPLETE PATH"; break; }
        } else if (nm_frontier(&maze,pose,&route)) { message="NO FRONTIER"; break; }
        prepared_ok=0;
        if (!route.length) {
            /* An unobserved edge can only be behind the current pose. Face it. */
            unsigned d;
            for (d=0;d<4;++d) if (!(maze.cell[cell(pose)].known & (1u<<d))) break;
            if (d==4) { message="MAP ERROR"; break; }
            turn_heading=d;
        } else turn_heading=route.direction[0];
        if (turn_heading!=pose.heading) {
            int quarters=(turn_heading+4-pose.heading)%4;
            if (quarters==3) quarters=-1;
            if (fw_motion_turn(quarters*90)) { message="TURN REJECTED"; break; }
            turning=1; state=1;
        } else {
            /* Traverse mapped straight corridors continuously, including the
             * first unmapped destination, but never an unobserved edge. */
            moving=1; int c=nm_neighbour(cell(pose),pose.heading);
            while (moving<route.length && route.direction[moving]==pose.heading &&
                   maze.cell[c].visited && maze.cell[c].known==15) {
                c=nm_neighbour(c,pose.heading); ++moving;
            }
            /* An unseen or closed end wall may stop the final 30 mm. A known
             * passage ahead must never be mistaken for a normal wall arrival. */
            unsigned bit=1u<<pose.heading;
            int accept_wall=!(maze.cell[c].known & bit) || (maze.cell[c].walls & bit);
            if (fw_motion_straight_to(moving,timed_run?fw_run_speed:fw_search_speed,accept_wall)) {
                message="MOVE REJECTED"; break;
            }
            state=1;
        }

    }
    fw_motion_stop();
    if (!timed_run) {
        search_ms=search_used(hal_os_get_systicks());
    }
    /* Save even a partial map; pose is deliberately NOT restored after reset. */
    int saved=save();
    map_message(pose,message);
    if (saved) hal_ui_display_prompt(app_context.ui,"FLASH","FLASH SAVE FAILED");
    USART1->CR1 |= USART_CR1_UE; /* Local diagnostics outside the trial. */
    return result;
}
int fw_app_discover(void) { return execute(0,1); }
int fw_app_resume(void)
{
    if (!has_map) { hal_ui_display_prompt(app_context.ui,"MAP","NO MAP"); return -1; }
    return execute(0,0);
}
int fw_app_run(void) { return execute(1,0); }
int fw_app_bootloader(void)
{
    fw_motion_stop();
    RCC->APB1ENR |= RCC_APB1ENR_PWREN; PWR->CR |= PWR_CR_DBP;
    RTC->BKP0R=FW_REQUEST_MAGIC; __DSB(); NVIC_SystemReset();
    return 0;
}
