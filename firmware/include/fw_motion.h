#ifndef FW_MOTION_H
#define FW_MOTION_H
#include <stdint.h>
#include "fw_cal_extra.h"
#define FW_RUN_MAX_SPEED 1000u
#define FW_CURVE_MAX_SPEED 220u
void fw_motion_init(void);
/* Bench only: each wheel -1, 0, +1, bounded to 20 mm at 20 mm/s. */
int fw_motion_test_wheels(int right, int left);
/* Nonblocking: whole calibrated cells, speeds in mm/s, clockwise turns in degrees. */
int fw_motion_straight(unsigned cells, unsigned speed_mm_s);
/* Allow a front-wall arrival only in the last 30 mm of the final cell. */
int fw_motion_straight_to(unsigned cells, unsigned speed_mm_s, int allow_front_wall);
int fw_motion_wall_arrival(void);
/* Exploration recovery only, after a confirmed F5 obstacle. Reverse at 80 mm/s,
 * at most one cell and never farther than the interrupted forward travel.
 * Zero distance acknowledges an obstacle detected at the starting centre. */
int fw_motion_obstacle_backoff(uint32_t distance_um);
/* Append a proven open cell while moving; no pulse or controller reset. */
int fw_motion_extend(unsigned cells,int allow_front_wall);
unsigned fw_motion_speed(void);
void fw_motion_corner_profiles(const fw_corner_data_t *left,const fw_corner_data_t *right);
int32_t fw_motion_lateral_um(void);
int32_t fw_motion_heading_mrad(void);
int fw_motion_turn(int clockwise_degrees);
/* Centre -> half-cell lead -> 90-degree arc -> half-cell tail -> centre.
 * Two known orthogonal edges, both wheels forward. Caller checks the map. */
int fw_motion_curve(int clockwise_degrees,unsigned speed_mm_s,int allow_front_wall);
/* Deliberate contact/sensing moves: bounded, slow, no wall steering or F5 stop.
 * Only the calibration menu may use these; stale sensors still stop the robot. */
int fw_motion_calibration_move(int32_t distance_um, unsigned speed_mm_s);
int fw_motion_calibration_turn(int clockwise_degrees);
int fw_motion_calibration_spin(int32_t wheel_um,unsigned speed_mm_s);
int fw_motion_calibration_traverse(int32_t distance_um,unsigned speed_mm_s);
void fw_motion_wall_profile(const fw_cal_data_t *);
void fw_motion_rotation_profile(const fw_rotation_data_t *);
int32_t fw_motion_travelled_um(void);
void fw_motion_geometry(uint32_t pitch_um, uint32_t front_on_um, uint32_t inner_um);
void fw_motion_tick(uint32_t now_ms);
void fw_motion_stop(void);
int fw_motion_busy(void);
int fw_motion_fault(void);
uint32_t fw_motion_remaining(void);
#endif
