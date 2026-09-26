#include "fw_app.h"
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
#define SNAPSHOT_SCHEMA 1
/* ARM ABI-specific snapshot. Change schema for any layout or settings ABI change. */
typedef struct {
    robot_settings settings;
    nm_map_t map;
    uint32_t corner, heading, search_speed, run_speed, search_ms, has_map;
} snapshot_t;
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
int fw_start_corner=0, fw_start_heading=NM_NORTH, fw_search_speed=120, fw_run_speed=200;
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
    snapshot_t s;
    if (fw_store_load(&fw_stm32_flash,SNAPSHOT_SCHEMA,&s,sizeof s) || !nm_valid(&s.map) ||
        !settings_valid(&s.settings) || s.corner>3 || s.heading>3 || s.search_speed<20 ||
        s.search_speed>300 || s.run_speed<20 || s.run_speed>300 || s.has_map>1 ||
        s.search_ms>NM_SEARCH_MS) return -1;
    zhonxSettings=s.settings; maze=s.map; has_map=s.has_map; search_ms=s.search_ms;
    search_clock_running=0;
    fw_start_corner=s.corner; fw_start_heading=s.heading;
    fw_search_speed=s.search_speed; fw_run_speed=s.run_speed;
    return 0;
}
void fw_app_init(void) { nm_init(&maze); (void)load(); }
static int save(void)
{
    if (fw_motion_busy() || !parameters_valid() || !settings_valid(&zhonxSettings)) return -1;
    search_ms=search_used(hal_os_get_systicks());
    snapshot_t s; memset(&s,0,sizeof s);
    s.settings=zhonxSettings; s.map=maze; s.has_map=has_map; s.search_ms=search_ms;
    s.corner=fw_start_corner; s.heading=fw_start_heading;
    s.search_speed=fw_search_speed; s.run_speed=fw_run_speed;
    /* Single-bank flash stalls instruction fetch. Never save while motors move. */
    fw_motion_stop();
    return fw_store_save(&fw_stm32_flash,SNAPSHOT_SCHEMA,&s,sizeof s);
}
int fw_app_save(void)
{
    int result=save();
    hal_ui_display_prompt(app_context.ui,"SAVE",result ? "Save failed" : "Settings + maze saved");
    return result;
}
int fw_app_restore(void)
{
    if (fw_motion_busy()) return -1;
    int result=load();
    hal_ui_display_prompt(app_context.ui,"RESTORE",result ? "No valid snapshot" : "Settings + maze loaded");
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
/* Foreground state machine. The pulse IRQs, 200 Hz acquisition phases and
 * 1 kHz motion controller run independently while path planning/OLED run here. */
static int execute(int timed_run, int fresh)
{
    if (!parameters_valid()) {
        hal_ui_display_prompt(app_context.ui,"NIMES","Invalid start/speed"); return -1;
    }
    if (timed_run && !has_map) {
        hal_ui_display_prompt(app_context.ui,"NIMES","Explore or restore first"); return -1;
    }
    hal_ui_display_prompt(app_context.ui,"PLACE AT START","Corner + heading set?");
    fw_motion_init(); radio_off();
    if (fresh) { nm_init(&maze); search_ms=0; has_map=1; search_clock_running=0; }
    if (timed_run) { search_ms=search_used(hal_os_get_systicks()); search_clock_running=0; }
    else if (!search_clock_running) {
        search_base=search_ms; search_epoch=hal_os_get_systicks(); search_clock_running=1;
    }
    nm_pose_t origin=start_pose(), pose=origin;
    nm_route_t route={0}, prepared={0};
    uint8_t goals[NM_CELLS], home[NM_CELLS]={0}; home[cell(origin)]=1;
    uint32_t started=hal_os_get_systicks(), stopped=started, ui_time=started, seen=0;
    unsigned moving=0, turn_heading=0;
    int turning=0, returning=0, certified=0, prepared_ok=0, state=0, result=-1, at_start=1;
    const char *message="Stopped";
    for (;;) {
        uint32_t now=hal_os_get_systicks();
        if (!(GPIOC->IDR & GPIO_Pin_13)) { message="User stop"; break; }
        if (!timed_run && search_used(now)>=NM_SEARCH_MS) { message="5 minutes elapsed"; break; }
        if (fw_motion_fault()) { message="Motion/sensor fault"; break; }
        if (state==1) { /* TURN or MOVE in progress; plan from its destination now. */
            if (!prepared_ok && moving && (returning || timed_run)) {
                nm_pose_t destination=pose; advance(&destination,moving);
                prepared_ok=!nm_route(&maze,destination,returning?home:goals,0,&prepared);
            }
            if (fw_motion_busy()) { __WFI(); continue; }
            if (turning) pose.heading=turn_heading;
            else advance(&pose,moving);
            moving=0; turning=0; stopped=now; seen=0; state=0;
        }
        /* Only map at stopped cell centres, after three full stable scan periods.
         * Interior cells of grouped moves are already mapped before that move. */
        hal_sensor_snapshot scan;
        if (!hal_sensor_snapshot_read(&scan) || now-scan.timestamp>50 || now-stopped<40 || scan.sequence==seen) {
            __WFI(); continue;
        }
        seen=scan.sequence;
        if (nm_observe(&maze,pose,observation(scan.filtered))) { message="Map/sensor mismatch"; break; }
        if (at_start && maze.cell[cell(pose)].known==15) {
            unsigned walls=maze.cell[cell(pose)].walls, count=0;
            for (unsigned i=0;i<4;++i) count+=(walls>>i)&1u;
            if (count!=3) { message="Start must have 3 walls"; break; }
            at_start=0;
        }
        int rooms=nm_goal(&maze,goals);
        if (rooms>1) { message="Ambiguous goal rooms"; break; }
        if (timed_run) {
            if (rooms!=1) { message="Goal not established"; break; }
            if (goals[cell(pose)]) { message="Goal reached"; result=0; break; }
            if (prepared_ok) route=prepared;
            else if (nm_route(&maze,pose,goals,0,&route)) { message="No known goal path"; break; }
        } else if (returning) {
            if (home[cell(pose)]) { message=certified?"Shortest path proven":"Returned to start"; result=0; break; }
            if (prepared_ok) route=prepared;
            else if (nm_route(&maze,pose,home,0,&route)) { message="No return path"; break; }
        } else if (rooms==1) {
            int r=nm_refine(&maze,pose,origin,goals,&route);
            if (r==1) { certified=1; returning=1; continue; }
            if (r<0) { message="Goal path unresolved"; break; }
        } else if (nm_frontier(&maze,pose,&route)) { message="No unexplored cells"; break; }
        prepared_ok=0;
        if (!route.length) {
            /* An unobserved edge can only be behind the current pose. Face it. */
            unsigned d;
            for (d=0;d<4;++d) if (!(maze.cell[cell(pose)].known & (1u<<d))) break;
            if (d==4) { message="Planner inconsistent"; break; }
            turn_heading=d;
        } else turn_heading=route.direction[0];
        if (turn_heading!=pose.heading) {
            int quarters=(turn_heading+4-pose.heading)%4;
            if (quarters==3) quarters=-1;
            if (fw_motion_turn(quarters*90)) { message="Turn rejected"; break; }
            turning=1; state=1;
        } else {
            /* Traverse mapped straight corridors continuously, including the
             * first unmapped destination, but never an unobserved edge. */
            moving=1; int c=nm_neighbour(cell(pose),pose.heading);
            while (moving<route.length && route.direction[moving]==pose.heading &&
                   maze.cell[c].visited && maze.cell[c].known==15) {
                c=nm_neighbour(c,pose.heading); ++moving;
            }
            if (fw_motion_straight(moving,timed_run?fw_run_speed:fw_search_speed)) { message="Move rejected"; break; }
            state=1;
        }
        if (now-ui_time>=100) {
            char text[32]; snprintf(text,sizeof text,"%u,%u %s",pose.x,pose.y,timed_run?"RUN":"SEARCH");
            hal_ui_clear_scr(app_context.ui); hal_ui_display_txt(app_context.ui,0,16,text);
            hal_ui_refresh(app_context.ui); ui_time=now;
        }
    }
    fw_motion_stop();
    if (!timed_run) {
        search_ms=search_used(hal_os_get_systicks());
    }
    /* Save even a partial map; pose is deliberately NOT restored after reset. */
    int saved=save();
    hal_ui_display_prompt(app_context.ui,"NIMES",message);
    if (saved) hal_ui_display_prompt(app_context.ui,"FLASH","Snapshot failed");
    USART1->CR1 |= USART_CR1_UE; /* Local diagnostics outside the trial. */
    return result;
}
int fw_app_discover(void) { return execute(0,1); }
int fw_app_resume(void) { return has_map ? execute(0,0) : -1; }
int fw_app_run(void) { return execute(1,0); }
int fw_app_bootloader(void)
{
    fw_motion_stop();
    RCC->APB1ENR |= RCC_APB1ENR_PWREN; PWR->CR |= PWR_CR_DBP;
    RTC->BKP0R=FW_REQUEST_MAGIC; __DSB(); NVIC_SystemReset();
    return 0;
}
