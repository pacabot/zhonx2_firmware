#ifndef FW_PATH_H
#define FW_PATH_H
#include "nimes.h"
#include "fw_calibration.h"
#define FW_PATH_SEGMENTS (NM_CELLS*2)
/* Forward-only route with smooth 90 degree bends; no mid-route pivots. */
typedef struct {
    float start,length,x,y,limit,exit_speed;
    uint8_t heading;int8_t turn;uint32_t phase_um;
} fw_path_segment_t;
typedef struct {
    fw_path_segment_t segment[FW_PATH_SEGMENTS];
    unsigned count;float length,acceleration,radius;
    nm_pose_t end;
} fw_path_t;
typedef struct {float x,y,heading,curvature;} fw_path_point_t;
unsigned fw_run_acceleration(unsigned speed);
int fw_path_plan(fw_path_t *,const nm_map_t *,nm_pose_t,const nm_route_t *,
                 const fw_cal_geometry_t *,unsigned speed);
void fw_path_point(const fw_path_t *,unsigned segment,float distance,fw_path_point_t *);
#endif
