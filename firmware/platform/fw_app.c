#include "fw_app.h"
#include "fw_battery.h"
#include <stddef.h>
#include "fw_start.h"
#include "fw_ui.h"
#include "fw_layout.h"
#include "fw_store.h"
#include "fw_motion.h"
#include "fw_sound.h"
#include "fw_buttons.h"
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
#include "fw_snapshot.h"
#define SNAPSHOT_SCHEMA FW_SNAPSHOT_SCHEMA
static fw_library_t library;
static int learned;
static snapshot_t snapshot_buffer; /* Foreground only; keep the 8 KiB stack for planning. */
_Static_assert(sizeof(snapshot_t)<FW_STORE_SIZE-32,"persistent library capacity");
_Static_assert(sizeof(snapshot_t)%4==0,"snapshot alignment");
#ifdef __arm__
_Static_assert(sizeof(snapshot_t)==12492,"snapshot dump ARM ABI");
_Static_assert(offsetof(snapshot_t,measurements)==1120,"calibration bundle offset");
_Static_assert(offsetof(snapshot_t,nose)==12460,"geometry offset");
_Static_assert(offsetof(snapshot_t,battery)==12480,"battery reference offset");
_Static_assert(sizeof(snapshot_v6_t)==4708,"legacy snapshot ARM ABI");
_Static_assert(sizeof(snapshot_v2_t)==412,"calibration dump ARM ABI");
_Static_assert(sizeof(calibration_bundle_t)==832,"calibration dump layout");
#endif
static nm_map_t maze;
static uint32_t search_ms, search_epoch, search_base, last_run_ms;
static uint32_t search_limit(void) {return nm_size(&maze)==16?600000u:NM_SEARCH_MS;}
static int search_clock_running;
static uint32_t search_used(uint32_t now)
{
    uint32_t elapsed=search_clock_running ? now-search_epoch : 0;
    uint32_t base=search_clock_running ? search_base : search_ms;
    return base>=search_limit() || elapsed>=search_limit()-base ? search_limit() : base+elapsed;
}
static int has_map,start_prepared,goal_announced;
static uint32_t trial_ms[3];
static const char *last_result="LOADED MAP";
int fw_app_has_map(void) {return has_map;}
int fw_app_at_start(void) {return start_prepared;}

volatile unsigned fw_loaded_schema;
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
    start_prepared=0;
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
    fw_motion_wall_profile(wall);
    fw_motion_corner_profiles(&measurements.corner[0],&measurements.corner[1]);
    if (measurements.corner[0].valid) fw_cal_post_mm=(int)measurements.corner[0].post_um/1000;
    else if (measurements.corner[1].valid) fw_cal_post_mm=(int)measurements.corner[1].post_um/1000;
}
static nm_pose_t displayed_pose;
volatile unsigned fw_last_stop_code;
int fw_start_corner=0, fw_start_heading=NM_NORTH, fw_search_speed=220, fw_run_speed=260;
int fw_maze_size=9;
static int parameters_valid(void)
{
    return (fw_maze_size==6 || fw_maze_size==9 || fw_maze_size==16) && fw_start_corner>=0 && fw_start_corner<=3 && fw_start_heading>=0 && fw_start_heading<4 &&
        fw_search_speed>=20 && fw_search_speed<=300 && fw_run_speed>=20 && fw_run_speed<=(int)FW_RUN_MAX_SPEED;
}
static int settings_valid(const robot_settings *s)
{
    return s->initial_speed>0 && s->initial_speed<=MAX_SPEED && s->default_accel>0 &&
        s->default_accel<=1000 && s->rotate_accel>0 && s->rotate_accel<=1000 &&
        s->correction_p>0 && s->correction_i>0 && s->max_correction<=MAX_SPEED &&
        s->max_speed_distance<=1000000 && s->emergency_decel>0 && s->emergency_decel<=10000;
}
static void import_map(nm_map_t *map,const legacy_map_t *old,unsigned corner,unsigned heading)
{
    nm_init_size(map,9,0);
    for(unsigned y=0;y<9;++y)for(unsigned x=0;x<9;++x)map->cell[y*NM_SIDE+x]=old->cell[y*9+x];
    nm_pose_t start=fw_maze_origin(corner,heading);
    map->start_x=start.x;map->start_y=start.y;map->start_heading=start.heading;
}
static int load(void)
{
    snapshot_t *current=&snapshot_buffer;memset(current,0,sizeof *current);
    snapshot_base_t s;int legacy=0,format5=0;unsigned loaded_schema=7;
    if(!fw_store_load(&fw_stm32_flash,SNAPSHOT_SCHEMA,current,sizeof *current)) {
        format5=1;s=current->base;
    } else {
        memset(current,0,sizeof *current);
        snapshot_v6_t old;memset(&old,0,sizeof old);
        if(!fw_store_load(&fw_stm32_flash,6,&old,sizeof old)) {loaded_schema=6;format5=1;}
        else if(!fw_store_load(&fw_stm32_flash,5,&old,SNAPSHOT_V5_SIZE)) {loaded_schema=5;format5=1;}
        else if(!fw_store_load(&fw_stm32_flash,4,&old,sizeof(snapshot_v4_t)))loaded_schema=4;
        else if(!fw_store_load(&fw_stm32_flash,3,&old,sizeof(snapshot_v3_t)))loaded_schema=3;
        else if(!fw_store_load(&fw_stm32_flash,2,&old,sizeof(snapshot_v2_t)))loaded_schema=2;
        else {legacy=1;loaded_schema=1;if(fw_store_load(&fw_stm32_flash,1,&old,sizeof(snapshot_v2_t)))return -1;}
        if(loaded_schema<6)memset(&old.battery,0,sizeof old.battery);
        if(loaded_schema<5) {
            memset(&old.library,0,sizeof old.library);old.learned=0;
            old.nose=old.width=old.inner=old.pitch=old.post=0;
        }
        if(loaded_schema<4) {memset(&old.measurements.rotation,0,sizeof old.measurements.rotation);
            memset(old.measurements.corner,0,sizeof old.measurements.corner);}
        if(loaded_schema<3)memset(&old.measurements.wall,0,sizeof old.measurements.wall);
        current->base.settings=old.base.settings;
        import_map(&current->base.map,&old.base.map,old.base.corner,old.base.heading);
        current->base.corner=old.base.corner;current->base.heading=old.base.heading;
        current->base.search_speed=old.base.search_speed;current->base.run_speed=old.base.run_speed;
        current->base.search_ms=old.base.search_ms;current->base.has_map=old.base.has_map;
        current->measurements=old.measurements;current->learned=old.learned;
        current->maze_size=9;
        current->nose=old.nose;current->width=old.width;current->inner=old.inner;
        current->pitch=old.pitch;current->post=old.post;current->battery=old.battery;
        if(old.library.count>FW_MAZE_SLOTS)return -1;
        current->library.count=old.library.count;current->library.next_id=old.library.next_id;
        for(unsigned i=0;i<old.library.count;++i) {
            fw_saved_maze_t *to=&current->library.item[i];const legacy_maze_t *from=&old.library.item[i];
            import_map(&to->map,&from->map,from->corner,from->heading);
            to->corner=from->corner;to->heading=from->heading;to->id=from->id;to->search_ms=from->search_ms;
            if(!fw_maze_certify(&to->map,to->corner,to->heading,&to->route))return -1;
        }
        s=current->base;
    }
    if(!fw_battery_reference_valid(&current->battery) || !bundle_valid(&current->measurements) ||
       !fw_library_valid(&current->library) || current->learned>1)return -1;
    if (!nm_valid(&s.map) ||
        !settings_valid(&s.settings) || s.corner>3 || s.heading>3 || s.search_speed<20 ||
        s.search_speed>300 || s.run_speed<20 || s.run_speed>FW_RUN_MAX_SPEED || s.has_map>1 ||
        s.search_ms>(nm_size(&s.map)==16?600000u:NM_SEARCH_MS)) return -1;
    if(current->maze_size!=6 && current->maze_size!=9 && current->maze_size!=16)return -1;
    fw_maze_size=current->maze_size;
    zhonxSettings=s.settings; maze=s.map; has_map=s.has_map; search_ms=s.search_ms;
    search_clock_running=0;
    fw_start_corner=s.corner; fw_start_heading=s.heading;
    fw_search_speed=legacy && s.search_speed==120 ? 220 : (int)s.search_speed;
    fw_run_speed=legacy && s.run_speed==200 ? 260 : (int)s.run_speed;
    measurements=current->measurements; library=current->library;
    nm_route_t route;
    learned=has_map && (!format5 || current->learned) && fw_maze_certify(&maze,fw_start_corner,fw_start_heading,&route);
    /* Import a certified pre-library maze without losing the active partial map. */
    if(learned && !library.count) (void)fw_library_put(&library,&maze,fw_start_corner,fw_start_heading,search_ms);
    apply_calibration();
    if(current->nose>=100 && current->nose<=750 && current->width>=400 && current->width<=1400 &&
        current->inner>=140 && current->inner<=190 && current->pitch>current->inner && current->pitch<=210 &&
        current->post>=100 && current->post<=250) {
        fw_cal_nose_tenth_mm=current->nose; fw_cal_width_tenth_mm=current->width;
        fw_cal_inner_mm=current->inner; fw_cal_pitch_mm=current->pitch; fw_cal_post_mm=current->post;
    }
    fw_battery_set_reference(current->battery);fw_loaded_schema=loaded_schema;
    return 0;
}
void fw_app_init(void) { memset(trial_ms,0,sizeof trial_ms);last_result="LOADED MAP";start_prepared=0; fw_loaded_schema=0; fw_battery_set_reference((fw_battery_reference_t){0}); has_map=learned=0; memset(&library,0,sizeof library); nm_init_size(&maze,fw_maze_size,0); memset(&measurements,0,sizeof measurements); apply_calibration(); (void)load(); }
static int save(void)
{
    if (fw_motion_busy() || !parameters_valid() || !settings_valid(&zhonxSettings)) return -1;
    search_ms=search_used(hal_os_get_systicks());
    snapshot_t *s=&snapshot_buffer; memset(s,0,sizeof *s);
    s->base.settings=zhonxSettings; s->base.map=maze; s->base.has_map=has_map; s->base.search_ms=search_ms;
    s->base.corner=fw_start_corner; s->base.heading=fw_start_heading;
    s->base.search_speed=fw_search_speed; s->base.run_speed=fw_run_speed;
    s->measurements=measurements; s->library=library; s->learned=learned;
    s->nose=fw_cal_nose_tenth_mm; s->width=fw_cal_width_tenth_mm;
    s->inner=fw_cal_inner_mm; s->pitch=fw_cal_pitch_mm; s->post=fw_cal_post_mm;
    s->battery=fw_battery_reference();s->maze_size=(uint32_t)fw_maze_size;
    /* Single-bank flash stalls instruction fetch. Never save while motors move. */
    fw_motion_stop();
    return fw_store_save(&fw_stm32_flash,SNAPSHOT_SCHEMA,s,sizeof *s);
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
int fw_app_battery_commit(unsigned raw,unsigned mv)
{
    fw_battery_reference_t r={raw,mv},previous=fw_battery_reference();
    if(!raw || !fw_battery_reference_valid(&r) || fw_motion_busy())return -1;
    fw_battery_set_reference(r);
    int result=save();if(result)fw_battery_set_reference(previous);
    return result;
}
int fw_app_settings_save(void) { return save(); }
unsigned fw_app_maze_count(void) { return library.count; }
const fw_saved_maze_t *fw_app_maze(unsigned index) { return index<library.count?&library.item[index]:0; }
int fw_app_ready(void)
{
    nm_route_t route;
    return learned && has_map && fw_maze_certify(&maze,fw_start_corner,fw_start_heading,&route);
}
int fw_app_maze_load(unsigned index)
{
    if(fw_motion_busy() || index>=library.count) return -1;
    const fw_saved_maze_t *m=&library.item[index];
    maze=m->map;fw_maze_size=maze.side; fw_start_corner=(int)m->corner; fw_start_heading=(int)m->heading;
    search_ms=m->search_ms; search_clock_running=0; has_map=learned=1;
    displayed_pose=nm_origin(&m->map);start_prepared=0;last_result="LOADED MAP";memset(trial_ms,0,sizeof trial_ms);return 0;
}
int fw_app_maze_delete(unsigned index)
{
    if(fw_motion_busy() || index>=library.count) return -1;
    fw_saved_maze_t previous=library.item[index]; unsigned count=library.count;
    if(fw_library_remove(&library,index)) return -1;
    if(!save()) return 0;
    memmove(&library.item[index+1],&library.item[index],(count-index-1)*sizeof previous);
    library.item[index]=previous; library.count=count; return -1;
}
static nm_pose_t start_pose(void) {return nm_origin(&maze);}
static int cell(nm_pose_t p) { return p.y*NM_SIDE+p.x; }
static void advance(nm_pose_t *p, unsigned count)
{
    for (unsigned i=0;i<count;++i) {
        int n=nm_next(&maze,cell(*p),p->heading);
        p->x=n%NM_SIDE; p->y=n/NM_SIDE;
    }
}
static uint8_t observation(uint8_t s)
{
    return ((s&(SENSOR_F10_POS|SENSOR_F5_POS))!=(SENSOR_F10_POS|SENSOR_F5_POS)) | (!(s&SENSOR_L10_POS)<<1) | (!(s&SENSOR_R10_POS)<<2);
}
static void radio_off(void)
{
    /* Autonomous trials keep external radios disabled. */
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
        if (fw_cancel_pressed()) return -1;
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
static void map_message(nm_pose_t pose, const char *message,int map_view)
{
    unsigned page=map_view?0:2,level=0;int x=0,y=0;uint32_t drawn=0;
    const unsigned scales[]={0,16,24};
    nm_route_t best;int certified=learned && fw_maze_certify(&maze,fw_start_corner,fw_start_heading,&best);
    fw_ui_map_progress(0);
    /* Escape zooms only after motion has stopped. All four joystick directions
     * pan; centre cycles result pages, a long Escape/zoom press exits. */
    int ignore_keys=(GPIOC->IDR&0x3f00)!=0x3f00;
    for(;;) {
        uint32_t now=hal_os_get_systicks();
        if(!drawn || now-drawn>=100) {
            fw_ui_map_route(certified?&best:0,(now/500)%2);
            fw_ui_map_view(scales[level],x,y);
            if(!page)fw_ui_maze(&maze,pose,message,0,search_ms,live_sensors());
            else if(page<3)fw_ui_result(message,search_ms,last_run_ms,page);
            else fw_ui_result((const char *const[]){"RUN 1","RUN 2","RUN 3"}[page-3],search_ms,trial_ms[page-3],1);
            drawn=now;
        }
        unsigned keys=(~GPIOC->IDR)&0x3f00;
        if(ignore_keys) {if(!keys)ignore_keys=0;__WFI();continue;}
        if(keys) {
            uint32_t pressed=now;
            while(((~GPIOC->IDR)&0x3f00)==keys) {
                if((keys&FW_ESCAPE_PIN) && hal_os_get_systicks()-pressed>=800) {
                    fw_ui_map_route(0,0);fw_ui_map_view(16,0,0);return;
                }
                __WFI();
            }
            if(hal_os_get_systicks()-pressed<20)continue;
            if(keys&FW_ESCAPE_PIN) {if(map_view) {level=(level+1)%3;page=0;}else page=page==1?2:1;}
            if(keys&(1u<<12))page=map_view?(page+1)%6:page==1?2:1;
            if(map_view && (keys&FW_UP_PIN)) {++y;page=0;}
            if(map_view && (keys&FW_DOWN_PIN)) {--y;page=0;}
            if(map_view && (keys&FW_BACK_PIN)) {--x;page=0;}
            if(map_view && (keys&(1u<<11))) {++x;page=0;}
            if(x>16)x=16;
            if(x< -16)x=-16;
            if(y>16)y=16;
            if(y< -16)y=-16;
            drawn=0;
        }
        __WFI();
    }
}
int fw_app_show_map(void)
{
    if (!has_map) { hal_ui_display_prompt(app_context.ui,"MAP","NO MAP"); return -1; }
    map_message(displayed_pose,last_result,1); return 0;
}
int fw_app_show_result(void) {map_message(displayed_pose,last_result,0);return 0;}
/* Foreground planning; sensors, position observer and pulse generation remain
 * interrupt-driven. Early stable observations can append the next cell in flight. */
static unsigned run_speed_override;
static int curve_run;
static uint8_t rechecked[NM_CELLS];
static unsigned confirmation_pass;

static int plan_from(nm_pose_t pose,int timed,int *returning,int *certified,nm_route_t *route,const char **message)
{
    uint8_t goals[NM_CELLS],home[NM_CELLS]={0};nm_pose_t origin=nm_origin(&maze);
    home[cell(origin)]=1;
    int rooms=nm_goal(&maze,goals);
    if(rooms>1) {*message="AMBIGUOUS GOAL";return -1;}
    if(rooms==1 && !goal_announced) {goal_announced=1;fw_sound_play(1);}
    if(timed) {
        if(rooms!=1) {*message="UNKNOWN GOAL";return -1;}
        if(goals[cell(pose)]) {*message="GOAL FOUND";return 4;}
        if(nm_route(&maze,pose,goals,0,route)) {*message="NO PATH";return -1;}
    } else if(*returning) {
        if(home[cell(pose)] && pose.heading!=origin.heading)return origin.heading;
        if(home[cell(pose)]) {*message=*certified?"OPTIMAL PATH":"BACK AT START";return 4;}
        if(nm_route(&maze,pose,home,0,route)) {*message="NO RETURN PATH";return -1;}
    } else if(rooms==1) {
        int result=nm_refine(&maze,pose,origin,goals,route);
        if(result==1) {*certified=1;*returning=1;return plan_from(pose,0,returning,certified,route,message);}
        if(result<0) {*message="INCOMPLETE PATH";return -1;}
    } else if(nm_frontier(&maze,pose,route)) {*message=maze.axes==3?"NO FRONTIER":"START AMBIGUOUS";return -1;}
    if(route->length)return route->direction[0];
    const unsigned order[]={pose.heading,(pose.heading+3)%4,(pose.heading+1)%4,(pose.heading+2)%4};
    for(unsigned i=0;i<4;++i) {
        unsigned d=order[i];int n=nm_next(&maze,cell(pose),d);
        if(n>=0 && (maze.cell[cell(pose)].known&(1u<<d)) &&
           !(maze.cell[cell(pose)].walls&(1u<<d)) && !maze.cell[n].visited)return (int)d;
    }
    for(unsigned d=0;d<4;++d)if(!(maze.cell[cell(pose)].known&(1u<<d)))return (int)d;
    *message="MAP ERROR";return -1;
}
static int32_t preview_distance(unsigned speed)
{
    if(!fw_cal_valid(&measurements.wall))return 0;
    const fw_cal_geometry_t *g=&measurements.wall.geometry;
    /* F10 must already have triggered even with filter latency and placement
     * uncertainty (20 mm, plus the measured repeatability). Three further
     * identical scans confirm an opening; F5 also vetoes a false F10 opening. */
    int32_t window=(int32_t)measurements.wall.front[1].on_um-(int32_t)g->inner_um/2-(int32_t)speed*40-20000-(int32_t)measurements.wall.front[1].spread_um;
    for(unsigned side=0;side<2;++side)for(unsigned opening=0;opening<2;++opening) {
        int32_t offset;
        if(fw_corner_offset(&measurements.corner[opening?1-side:side],opening?1:0,speed,1,opening,&offset))return 0;
        if(!opening)offset=-offset;
        int32_t side_window=(int32_t)g->pitch_um/2-offset-5000;
        if(window>side_window)window=side_window;
    }
    if(window<0)return 0;
    return window>40000?40000:window;
}
static void rebase_checks(int dx,int dy)
{
    if((dx || dy) && confirmation_pass) {
        uint8_t shifted[NM_CELLS]={0};
        for(unsigned y=0;y<nm_size(&maze);++y)for(unsigned x=0;x<nm_size(&maze);++x) {
            int xx=(int)x+dx,yy=(int)y+dy;
            if(xx>=0 && yy>=0 && xx<(int)nm_size(&maze) && yy<(int)nm_size(&maze))
                shifted[yy*NM_SIDE+xx]=rechecked[y*NM_SIDE+x];
        }
        memcpy(rechecked,shifted,sizeof rechecked);
    }
}
static int observe_pose(nm_pose_t *target,uint8_t walls,nm_pose_t *pose,nm_pose_t *segment)
{
    nm_pose_t before=*target;
    if(nm_observe_auto(&maze,target,walls))return -1;
    int dx=(int)target->x-before.x,dy=(int)target->y-before.y;
    pose->x+=dx;pose->y+=dy;segment->x+=dx;segment->y+=dy;
    rebase_checks(dx,dy);
    return 0;
}
/* Explicit exploration-only correction. Ordinary observations keep rejecting
 * contradictions. Never close an edge whose far cell has already been visited:
 * that may be an obstacle in a known passage, not a newly discovered maze wall. */
static int missed_front(nm_map_t *map,nm_pose_t p)
{
    int c=cell(p),n=nm_next(map,c,p.heading);
    unsigned bit=1u<<p.heading,opposite=1u<<((p.heading+2)%4);
    if(c<0 || c>=NM_CELLS || p.x>=map->side || p.y>=map->side ||
       (map->cell[c].walls&bit) || (n>=0 && map->cell[n].visited))return -1;
    map->cell[c].known&=~bit;
    if(n>=0)map->cell[n].known&=~opposite;
    return nm_edge(map,c,p.heading,1);
}
static int recover_front(nm_pose_t *pose,nm_pose_t segment,unsigned moving)
{
    if(!fw_cal_valid(&measurements.wall))return -1;
    int32_t pitch=(int32_t)fw_cal_pitch_mm*1000;
    int32_t travel=fw_motion_travelled_um();
    int32_t allowance=(int32_t)measurements.wall.front[0].on_um-
                      (int32_t)measurements.wall.geometry.inner_um/2;
    if(allowance<0 || travel<0 || travel>(int32_t)moving*pitch)return -1;
    unsigned wall_cell=(unsigned)(travel+allowance+pitch/2)/pitch;
    int32_t residual=travel+allowance-(int32_t)wall_cell*pitch;
    if(wall_cell>moving || residual< -15000 || residual>15000)return -1;
    nm_pose_t wall=segment,back=segment;
    advance(&wall,wall_cell);advance(&back,(unsigned)(travel/pitch));
    nm_map_t corrected=maze;
    if(missed_front(&corrected,wall))return -1;
    /* A single raw sample stops pulses, but cannot rewrite the map. */
    uint32_t since=hal_os_get_systicks(),seen=0;unsigned stable=0;
    while(stable<3) {
        hal_sensor_snapshot scan;uint32_t now=hal_os_get_systicks();
        if(fw_cancel_pressed() || now-since>200 || !hal_sensor_snapshot_read(&scan) ||
           now-scan.timestamp>50)return -1;
        if(scan.sequence!=seen) {
            seen=scan.sequence;
            stable=((scan.raw|scan.filtered)&SENSOR_F5_POS)?0:stable+1;
        }
        __WFI();
    }
    if(fw_motion_obstacle_backoff((uint32_t)(travel%pitch)))return -1;
    since=hal_os_get_systicks();uint32_t shown=since-80;
    while(fw_motion_busy()) {
        if(fw_cancel_pressed() || hal_os_get_systicks()-since>5000) {fw_motion_stop();return -1;}
        uint32_t now=hal_os_get_systicks();
        if(now-shown>=80) {
            fw_ui_map_progress(0);
            fw_ui_maze(&maze,back,"RECOVERING",fw_motion_speed(),search_used(now),live_sensors());
            shown=now;
        }
        __WFI();
    }
    if(fw_motion_fault())return -1;
    maze=corrected;*pose=back;fw_motion_init();return 0;
}
static int choose_confirmation(nm_pose_t p,nm_route_t *route,nm_pose_t *target)
{
    if(!confirmation_pass)confirmation_pass=1;
    while(confirmation_pass<=3) {
        if(!nm_recheck_route(&maze,p,rechecked,route,target))return 0;
        ++confirmation_pass;memset(rechecked,0,sizeof rechecked);
    }
    return -1;
}
static int choose_conflict(nm_pose_t p,uint8_t observed,nm_pose_t *target)
{
    if(!confirmation_pass)confirmation_pass=1;
    const unsigned dirs[]={p.heading,(p.heading+3)%4,(p.heading+1)%4};
    for(unsigned i=0;i<3;++i) {
        unsigned bit=1u<<dirs[i];
        if((nm_next(&maze,cell(p),dirs[i])>=0 || !(maze.axes&(dirs[i]%2?1:2))) && (maze.cell[cell(p)].known&bit) &&
           !(rechecked[cell(p)]&bit) && !!(maze.cell[cell(p)].walls&bit)!=!!(observed&(1u<<i))) {
            *target=p;target->heading=dirs[i];return 0;
        }
    }
    return -1;
}
static void checked_front(nm_pose_t p)
{
    int c=cell(p),n=nm_next(&maze,c,p.heading);
    rechecked[c]|=1u<<p.heading;
    if(n>=0)rechecked[n]|=1u<<((p.heading+2)%4);
}
static int execute(int timed_run,int fresh)
{
    if(!parameters_valid()) {hal_ui_display_prompt(app_context.ui,"Maze","INVALID SETTINGS");return -1;}
    if(timed_run && !fw_app_ready()) {hal_ui_display_prompt(app_context.ui,"Maze","LEARN A MAZE FIRST");return -1;}
    unsigned speed=timed_run?(run_speed_override?run_speed_override:(unsigned)fw_run_speed):(unsigned)fw_search_speed;
    /* Door offsets are measured only up to 220 mm/s. Preserve continuous
     * preview and post references instead of silently disabling both at 300. */
    if(!timed_run && speed>220 && fw_cal_valid(&measurements.wall) &&
       fw_corner_valid(&measurements.corner[0]) && fw_corner_valid(&measurements.corner[1]))speed=220;
    start_prepared=0;goal_announced=0;fw_motion_init();confirmation_pass=0;memset(rechecked,0,sizeof rechecked);
    if(fresh) {learned=0;nm_init_size(&maze,fw_maze_size,1);fw_start_corner=fw_start_heading=0;
        memset(trial_ms,0,sizeof trial_ms);search_ms=last_run_ms=0;has_map=1;search_clock_running=0;}
    fw_ui_map_view(16,0,0);fw_ui_map_progress(0);
    nm_pose_t pose=start_pose(),segment=pose;displayed_pose=pose;fw_last_stop_code=0;
    if(wait_hand(pose))return -1;
    radio_off();uint32_t started=hal_os_get_systicks();
    if(!timed_run) {search_base=search_ms;search_epoch=started;search_clock_running=1;}
    uint32_t stopped=started,ui_time=started,seen=0,mismatch_since=0;
    unsigned moving=0,turn_heading=0,stable=0;int in_window=0;uint8_t candidate=0;
    int checking=0;nm_pose_t check_target=pose;
    int state=0,turning=0,curving=0,returning=0,certified=0,result=-1,previewed=0,prepared=-1,mismatch=0;
    const char *message="STOPPED";nm_route_t route;nm_pose_t curve_end=pose;
    int32_t pitch=(int32_t)fw_cal_pitch_mm*1000,preview=curve_run?0:preview_distance(speed);
    for(;;) {
        uint32_t now=hal_os_get_systicks();
        if(fw_cancel_pressed()) {message="USER STOP";break;}
        if(!timed_run && search_used(now)>=search_limit()) {message="TIME LIMIT";break;}
        if(fw_motion_fault()) {
            if(!timed_run && state && !turning && fw_motion_fault()==2 &&
               !recover_front(&pose,segment,moving)) {
                segment=pose;displayed_pose=pose;stopped=hal_os_get_systicks();
                state=turning=previewed=returning=certified=mismatch=in_window=0;
                moving=stable=0;seen=0;prepared=-1;checking=0;continue;
            }
            fw_last_stop_code=(unsigned)fw_motion_fault();
            message=fw_cancel_pressed()?"USER STOP":fw_motion_fault()==1?"SENSOR OR TIMEOUT":fw_motion_fault()==4?"CENTER FAILED":"EARLY OBSTACLE";break;
        }
        if(now-ui_time>=80) {
            nm_pose_t visual=pose;
            if(state && !turning && !curving) {
                int32_t travel=fw_motion_travelled_um();if(travel<0)travel=0;
                unsigned passed=(unsigned)(travel/pitch);if(passed>=moving)passed=moving-1;
                visual=segment;advance(&visual,passed);
                fw_ui_map_progress((travel-(int32_t)passed*pitch)*1000/pitch);
            }
            else fw_ui_map_progress(0);
            fw_ui_maze(&maze,visual,returning?"RETURN":timed_run?"RUN":"EXPLORATION",
                       fw_motion_speed(),timed_run?now-started:search_used(now),live_sensors());ui_time=now;
        }
        hal_sensor_snapshot scan;int scan_ready=hal_sensor_snapshot_read(&scan) && now-scan.timestamp<=50 && scan.sequence!=seen;
        int window=state && !turning && !curving && preview>0 && fw_motion_travelled_um()>=(int32_t)moving*pitch-preview;
        if(window && !in_window)stable=0;
        in_window=window;
        if(scan_ready) {
            seen=scan.sequence;uint8_t value=observation(scan.filtered & (scan.raw | (uint8_t)~(SENSOR_F10_POS|SENSOR_F5_POS)));
            stable=value==candidate?stable+1:1;candidate=value;
        }
        if(state && fw_motion_busy()) {
            if(!checking && !turning && !curving && !previewed && scan_ready && stable>=3 &&
               fw_motion_travelled_um()>=(int32_t)moving*pitch-preview && preview>0) {
                nm_pose_t destination=segment;advance(&destination,moving);
                if(!observe_pose(&destination,candidate,&pose,&segment)) {
                    prepared=plan_from(destination,timed_run,&returning,&certified,&route,&message);
                    previewed=1;
                    nm_pose_t next=destination;
                    if(prepared==(int)pose.heading && !(candidate&1))advance(&next,1);
                    int accept=!(maze.cell[cell(next)].known&(1u<<pose.heading)) ||
                                (maze.cell[cell(next)].walls&(1u<<pose.heading));
                    if(prepared==(int)pose.heading && !(candidate&1) && !fw_motion_extend(1,accept)) {
                        pose=destination;displayed_pose=pose;++moving;previewed=0;stable=0;in_window=0;
                    }
                }
            }
            __WFI();continue;
        }
        if(state) {
            if(curving) {pose=curve_end;stopped=now;stable=0;curving=0;}
            else if(turning) {pose.heading=turn_heading;stopped=now;stable=0;}
            else {pose=segment;advance(&pose,moving);stopped=previewed?now-40:now;}
            displayed_pose=pose;moving=0;turning=0;state=0;
        }
        /* Position is corrected by the motion interrupt from calibrated wall
         * and post transitions. Never add stop/turn/probe cycles at cell centres. */
        if(checking) {
            if(cell(pose)==cell(check_target)) {
                if(pose.heading!=check_target.heading)prepared=check_target.heading;
                else {
                    if(!scan_ready || stable<5 || now-stopped<60) {__WFI();continue;}
                    int wall=!!(candidate&1);
                    nm_pose_t before=pose;
                    if(nm_revise_front_auto(&maze,&pose,wall)) {message="MAP CONFLICT";break;}
                    int dx=(int)pose.x-before.x,dy=(int)pose.y-before.y;
                    segment.x+=dx;segment.y+=dy;rebase_checks(dx,dy);
                    checked_front(pose);checking=0;returning=certified=mismatch=0;
                    /* Only the frontal remeasurement can revise a wall. Side
                     * readings after a bad approach must not stop recovery. */
                    prepared=plan_from(pose,0,&returning,&certified,&route,&message);previewed=1;
                }
            } else {
                uint8_t goals[NM_CELLS]={0};goals[cell(check_target)]=1;
                if(nm_route(&maze,pose,goals,0,&route)) {checking=0;prepared=-1;}
                else prepared=route.direction[0];
            }
        }
        if(!checking && !previewed) {
            if(!scan_ready || stable<3 || now-stopped<30) {__WFI();continue;}
            nm_pose_t target=pose;
            if(observe_pose(&target,candidate|(fw_motion_wall_arrival()?1:0),&pose,&segment)) {
                if(!mismatch) {mismatch=1;mismatch_since=now;}
                if(now-mismatch_since<150) {__WFI();continue;}
                nm_map_t corrected=maze;
                uint8_t walls=candidate|(fw_motion_wall_arrival()?1:0);
                if(!timed_run && (walls&1) &&
                   !missed_front(&corrected,pose) && !nm_observe(&corrected,pose,walls)) {
                    maze=corrected;returning=certified=0;
                } else if(!timed_run) {
                    if(choose_conflict(pose,walls,&check_target) && choose_confirmation(pose,&route,&check_target)) {
                        message="CHECK LIMIT";fw_last_stop_code=3;break;
                    }
                    checking=1;previewed=0;returning=certified=0;continue;
                } else {message="MAP CONFLICT";fw_last_stop_code=3;break;}
            }
            pose=target;mismatch=0;
            prepared=plan_from(pose,timed_run,&returning,&certified,&route,&message);
        }
        previewed=0;displayed_pose=pose;
        if(prepared<0) {
            if(timed_run)break;
            if(choose_confirmation(pose,&route,&check_target)) {message="CHECK LIMIT";break;}
            checking=1;returning=certified=0;stable=0;continue;
        }
        if(prepared==4) {result=0;break;}
        turn_heading=(unsigned)prepared;
        if(turn_heading!=pose.heading) {
            int quarters=(turn_heading+4-pose.heading)%4;if(quarters==3)quarters=-1;
            if(fw_motion_turn(quarters*90)) {message="TURN REJECTED";break;}
            turning=1;state=1;stable=0;
        } else {
            /* A rounded corner consumes two orthogonal route edges. Never use
             * sensor preview to choose an arc: all three cells must be known. */
            if(curve_run && timed_run && route.length>=2 && route.direction[0]==pose.heading &&
               ((route.direction[1]+4-pose.heading)%2)==1) {
                nm_pose_t corner=pose;advance(&corner,1);
                curve_end=corner;curve_end.heading=route.direction[1];advance(&curve_end,1);
                unsigned exitbit=1u<<curve_end.heading;
                if(maze.cell[cell(pose)].visited && maze.cell[cell(corner)].visited &&
                   maze.cell[cell(corner)].known==15 && maze.cell[cell(curve_end)].visited &&
                   (maze.cell[cell(curve_end)].known&exitbit)) {
                    int degrees=(curve_end.heading+4-pose.heading)%4==1?90:-90;
                    int accept=!!(maze.cell[cell(curve_end)].walls&exitbit);
                    if(!fw_motion_curve(degrees,speed,accept)) {
                        curving=state=1;stable=0;continue;
                    }
                }
            }
            unsigned cells=1;
            /* Known stretches can be submitted as one command, including runs
             * faster than the calibrated observation window. */
            if(route.length && route.direction[0]==pose.heading) {
                nm_pose_t end=pose;advance(&end,1);
                while(!checking && cells<route.length && cells<16 && route.direction[cells]==pose.heading &&
                      maze.cell[cell(end)].visited && maze.cell[cell(end)].known==15) {
                    /* Leave the last straight edge for the tangent lead of an arc. */
                    if(curve_run && cells+1<route.length && route.direction[cells+1]!=pose.heading)break;
                    ++cells;advance(&end,1);
                }
            }
            nm_pose_t end=pose;advance(&end,cells);
            int accept=!(maze.cell[cell(end)].known&(1u<<pose.heading)) ||
                        (maze.cell[cell(end)].walls&(1u<<pose.heading));
            if(fw_motion_straight_to(cells,speed,accept)) {message="MOVE REJECTED";break;}
            segment=pose;moving=cells;state=1;stable=0;
        }
    }
    fw_motion_stop();
    if(timed_run)last_run_ms=hal_os_get_systicks()-started;
    else {search_ms=search_used(hal_os_get_systicks());search_clock_running=0;}
    int archive_failed=0;
    if(!timed_run) {learned=result==0 && certified;
        if(learned)archive_failed=fw_library_put(&library,&maze,fw_start_corner,fw_start_heading,search_ms)<0;}
    int saved=save();
    displayed_pose=pose;last_result=message;
    if(!result && !timed_run && fw_app_prepare_run()) {result=-1;last_result="START ALIGN FAILED";}
    fw_ui_result(last_result,search_ms,last_run_ms,2);
    if(result && !fw_cancel_pressed())fw_sound_play(0);
    if(archive_failed)hal_ui_display_prompt(app_context.ui,"Library","ACTIVE MAZE SAVED");
    if(saved)hal_ui_display_prompt(app_context.ui,"Flash","FLASH SAVE FAILED");
    USART1->CR1|=USART_CR1_UE;return result;
}
int fw_app_discover(void) { return execute(0,1); }
int fw_app_resume(void)
{
    if (!has_map) { hal_ui_display_prompt(app_context.ui,"MAP","NO MAP"); return -1; }
    return execute(0,0);
}
int fw_app_run(void) { return execute(1,0); }
int fw_app_run_curves(void)
{
    curve_run=1;int result=execute(1,0);curve_run=0;return result;
}
int fw_app_run_slow(void)
{
    run_speed_override=120;
    int result=execute(1,0); run_speed_override=0; return result;
}
int fw_app_bootloader(void)
{
    fw_motion_stop();
    RCC->APB1ENR |= RCC_APB1ENR_PWREN; PWR->CR |= PWR_CR_DBP;
    RTC->BKP0R=FW_REQUEST_MAGIC; __DSB(); NVIC_SystemReset();
    return 0;
}

static int wait_move(nm_pose_t *pose,const char *status,int path_mode)
{
    uint32_t began=hal_os_get_systicks(),drawn=began;
    while(fw_motion_busy()) {
        uint32_t now=hal_os_get_systicks();
        if(fw_cancel_pressed() || now-began>(path_mode?600000u:6000u)) {fw_motion_stop();return -1;}
        if(path_mode)*pose=fw_motion_path_pose();
        if(now-drawn>=80) {fw_ui_maze(&maze,*pose,status,fw_motion_speed(),last_run_ms,live_sensors());drawn=now;}
        __WFI();
    }
    if(path_mode && !fw_motion_fault())*pose=fw_motion_path_pose();
    displayed_pose=*pose;return fw_motion_fault()?-1:0;
}
static int face_start(nm_pose_t *pose,unsigned heading)
{
    int quarters=(heading+4-pose->heading)%4;if(!quarters)return 0;
    if(quarters==3)quarters=-1;
    if(fw_motion_turn(quarters*90) || wait_move(pose,"START ALIGN",0))return -1;
    pose->heading=heading;return 0;
}
int fw_app_prepare_run(void)
{
    if(start_prepared)return 0;
    if(!fw_app_ready() || !fw_cal_valid(&measurements.wall) || !fw_rotation_valid(&measurements.rotation))return -1;
    nm_pose_t pose=start_pose();fw_motion_init();radio_off();
    int result=0;
    /* Only at the known start, between trials. No probes during exploration
     * or on the timed path; retain all measured calibration coefficients. */
    for(unsigned axis=0;axis<2 && !result;++axis) {
        int reference=0,centered=0;
        for(unsigned opposite=0;opposite<2;++opposite) {
            unsigned d=(maze.start_heading+(axis?0:1)+2*opposite)%4;
            if(!(maze.cell[cell(pose)].walls&(1u<<d)))continue;
            reference=1;
            if(face_start(&pose,d)) {result=-1;break;}
            int r=fw_motion_center_wall();
            if(r<0 || (!r && wait_move(&pose,"START ALIGN",0))) {result=-1;break;}
            if(!r && fw_motion_centered()) {centered=1;break;}
        }
        if(reference && !centered)result=-1;
    }
    if(!result) {
        nm_route_t route;uint8_t goals[NM_CELLS];
        if(nm_goal(&maze,goals)!=1 || nm_route(&maze,start_pose(),goals,0,&route) || !route.length)result=-1;
        else result=face_start(&pose,route.direction[0]);
    }
    fw_motion_stop();displayed_pose=pose;USART1->CR1|=USART_CR1_UE;
    start_prepared=!result;return result;
}
static int follow_route(nm_pose_t *pose,nm_route_t *route,unsigned speed,const char *status)
{
    if(!route->length)return 0;
    if(face_start(pose,route->direction[0]))return -1;
    if(fw_motion_path(&maze,*pose,route,speed))return -1;
    return wait_move(pose,status,1);
}
int fw_app_trial(unsigned number,unsigned speed)
{
    if(number<1 || number>3 || speed<20 || speed>FW_RUN_MAX_SPEED || !fw_app_ready())return -1;
    const char *error="START ALIGN FAILED";int result=-1;
    if(fw_app_prepare_run())goto done;
    nm_pose_t pose=displayed_pose;nm_route_t route;uint8_t goals[NM_CELLS];
    if(wait_hand(pose)) {error="USER STOP";goto done;}
    fw_motion_init();radio_off();start_prepared=0;
    error="NO PATH";
    if(nm_goal(&maze,goals)!=1 || nm_route(&maze,pose,goals,0,&route))goto done;
    /* Time starts after the start gate; alignment/return are not part of a run. */
    uint32_t began=hal_os_get_systicks();
    error="RUN STOPPED";
    if(follow_route(&pose,&route,speed,"RUN"))goto done;
    last_run_ms=hal_os_get_systicks()-began;trial_ms[number-1]=last_run_ms;
    fw_motion_stop();fw_sound_play(1);
    uint8_t home[NM_CELLS]={0};home[cell(start_pose())]=1;
    error="RETURN STOPPED";
    fw_motion_init();
    if(nm_route(&maze,pose,home,0,&route) || follow_route(&pose,&route,120,"RETURN"))goto done;
    if(face_start(&pose,maze.start_heading))goto done;
    displayed_pose=pose;error="START ALIGN FAILED";
    if(fw_app_prepare_run())goto done;
    result=0;error="RUN COMPLETE";
done:
    fw_last_stop_code=(unsigned)fw_motion_fault();fw_motion_stop();USART1->CR1|=USART_CR1_UE;
    if(result) {
        start_prepared=0;
        if(fw_cancel_pressed())error="USER STOP";
        else {
            if(fw_motion_fault()==1)error="SENSOR OR TIMEOUT";
            if(fw_motion_fault()==2)error="EARLY OBSTACLE";
            if(fw_motion_fault()==4)error="START ALIGN FAILED";
            fw_sound_play(0);
        }
    }
    last_result=error;fw_ui_result(last_result,search_ms,last_run_ms,2);
    return result;
}
