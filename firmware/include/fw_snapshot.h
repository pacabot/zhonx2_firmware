#ifndef FW_SNAPSHOT_H
#define FW_SNAPSHOT_H
#include "fw_library.h"
#include "fw_cal_extra.h"
#include "fw_battery.h"
#include "config/config.h"
#include <stddef.h>
#define FW_SNAPSHOT_SCHEMA 7u
typedef struct {fw_cal_data_t wall;fw_rotation_data_t rotation;fw_corner_data_t corner[2];} calibration_bundle_t;
typedef struct {
    robot_settings settings;nm_map_t map;
    uint32_t corner,heading,search_speed,run_speed,search_ms,has_map;
} snapshot_base_t;
typedef struct {
    snapshot_base_t base;calibration_bundle_t measurements;fw_library_t library;
    uint32_t learned;int32_t nose,width,inner,pitch,post;fw_battery_reference_t battery;
    uint32_t maze_size;
} snapshot_t;
/* Frozen schema 1..6 ABI. Never substitute the variable-size current maze. */
typedef struct {nm_cell_t cell[81];} legacy_map_t;
typedef struct {uint8_t direction[81];uint16_t length;uint32_t cost;} legacy_route_t;
typedef struct {
    legacy_map_t map;legacy_route_t route;uint32_t id,corner,heading,search_ms;
} legacy_maze_t;
typedef struct {uint32_t count,next_id;legacy_maze_t item[8];} legacy_library_t;
typedef struct {
    robot_settings settings;legacy_map_t map;
    uint32_t corner,heading,search_speed,run_speed,search_ms,has_map;
} snapshot_v2_t;
typedef struct {snapshot_v2_t base;fw_cal_data_t calibration;} snapshot_v3_t;
typedef struct {snapshot_v2_t base;calibration_bundle_t measurements;} snapshot_v4_t;
typedef struct {
    snapshot_v2_t base;calibration_bundle_t measurements;legacy_library_t library;
    uint32_t learned;int32_t nose,width,inner,pitch,post;fw_battery_reference_t battery;
} snapshot_v6_t;
#define SNAPSHOT_V5_SIZE offsetof(snapshot_v6_t,battery)
#endif
