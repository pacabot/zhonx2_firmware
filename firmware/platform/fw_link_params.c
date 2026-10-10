/* Parameters ZHONX II exposes to Ariane (PARAM_LIST, PARAM_GET, PARAM_SET).
 * PERSISTENT ones are part of the saved settings: PARAM_SAVE writes them
 * with the maze and calibrations (fw_app_settings_save, motors stopped).
 * Ids are stable, never reuse one; names are 15 characters at most. A
 * parameter that means the same as on ZHONX III has its id and name
 * (scan_max = Explore speed, run1/run2 = RUN 1/2 speed when the command
 * says 0, maze_max_side); ids 4 and 50 exist only on ZHONX II. */
#include "fw_app.h"
#include "fw_bt.h"
#include "fw_limits.h"
#include "fw_link.h"
#include "fw_sound.h"
#include "middleware/link/link_params.h"

static float get_telemetry(void) {return (float)fw_link_telemetry_rate();}
static void set_telemetry(float v) {fw_link_set_telemetry_rate((int)v);}
static float get_blip(void) {return (float)fw_link_blip_period();}
static void set_blip(float v) {fw_link_set_blip_period((int)v);}
static float get_tx_rate(void) {return (float)fw_bt_rate();}
static void set_tx_rate(float v) {fw_bt_set_rate((unsigned)v);}
static float get_explore(void) {return (float)fw_search_speed;}
static void set_explore(float v) {fw_search_speed=(int)v;}
static float get_run1(void) {return (float)(fw_link_run1_speed?fw_link_run1_speed:fw_search_speed);}
static void set_run1(float v) {fw_link_run1_speed=(int)v;}
static float get_run2(void) {return (float)fw_run_speed;}
static void set_run2(float v) {fw_run_speed=(int)v;}
static float get_maze(void) {return (float)fw_maze_size;}
/* 6, 9 or 16 cells, as in Settings > Maze size. */
static void set_maze(float v) {fw_maze_size=v<7.5f?6:v<12.5f?9:16;}
static float get_sound(void) {return (float)fw_sound_level();}
static void set_sound(float v) {fw_sound_set_level((unsigned)v);}

#define P LINK_PARAM_FLAGS_PERSISTENT
#define I32(id,name,flags,lo,hi,get,set) {id,name,LINK_PARAM_TYPE_I32,flags,lo,hi,get,set}
const link_param_t fw_link_param_table[]={
    I32(1,"telemetry_hz",0,0.0f,100.0f,get_telemetry,set_telemetry),
    I32(2,"blip_period_ms",0,100.0f,60000.0f,get_blip,set_blip),
    I32(4,"bt_tx_bps",0,(float)FW_BT_TX_RATE_MIN,(float)FW_BT_TX_RATE_MAX,get_tx_rate,set_tx_rate),
    I32(10,"scan_max_mm_s",P,20.0f,(float)FW_RUN_MAX_SPEED,get_explore,set_explore),
    I32(13,"run1_max_mm_s",0,20.0f,(float)FW_RUN_MAX_SPEED,get_run1,set_run1),
    I32(16,"run2_max_mm_s",P,20.0f,(float)FW_RUN_MAX_SPEED,get_run2,set_run2),
    I32(20,"maze_max_side",P,6.0f,16.0f,get_maze,set_maze),
    I32(50,"sound_level",P,0.0f,3.0f,get_sound,set_sound),
};
const unsigned fw_link_param_table_count=sizeof fw_link_param_table/sizeof fw_link_param_table[0];
