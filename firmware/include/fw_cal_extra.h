#ifndef FW_CAL_EXTRA_H
#define FW_CAL_EXTRA_H
#include "fw_calibration.h"
#define FW_CAL_SPEEDS 3
/* Positive spin wheel travel means clockwise. Distances are commanded per wheel. */
typedef struct {
    fw_cal_io_t base;
    int (*spin)(void *,int32_t wheel_um,unsigned speed,fw_cal_observer,void *);
} fw_cal_extra_io_t;
typedef struct {
    uint32_t speed, quarter_um[2], spread_um[2], quarter_error_mdeg[2], quarter_checks[2];
} fw_rotation_point_t;
typedef struct {
    uint32_t valid;
    fw_cal_geometry_t geometry;
    fw_rotation_point_t point[FW_CAL_SPEEDS];
} fw_rotation_data_t;
typedef struct {
    uint32_t speed, mask;
    int32_t raw_open_um[2], raw_close_um[2], open_um[2], close_um[2];
    uint32_t spread_um[2];
} fw_corner_point_t;
typedef struct {
    uint32_t valid, side, post_um;
    fw_cal_geometry_t geometry;
    /* facing_out=0: reference-side sensor, reverse opening / forward closing.
     * facing_out=1: opposite sensor, forward opening / reverse closing. */
    fw_corner_point_t point[2][FW_CAL_SPEEDS]; /* 5/10 cm offsets from post centre. */
} fw_corner_data_t;
int fw_rotation_valid(const fw_rotation_data_t *);
int fw_corner_valid(const fw_corner_data_t *);
int fw_rotation_run(const fw_cal_extra_io_t *,const fw_cal_geometry_t *,fw_rotation_data_t *);
int fw_corner_run(const fw_cal_io_t *,const fw_cal_geometry_t *,unsigned side,uint32_t post_um,fw_corner_data_t *);
/* Returns zero unless a valid measured profile covers the requested speed. */
uint32_t fw_rotation_quarter(const fw_rotation_data_t *,unsigned speed,unsigned direction);
int fw_corner_offset(const fw_corner_data_t *,unsigned facing_out,unsigned speed,unsigned sensor,int opening,int32_t *offset);
#endif
