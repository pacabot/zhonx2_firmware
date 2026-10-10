#ifndef FW_MOTION_H
#define FW_MOTION_H
#include <stdint.h>
#include "fw_cal_extra.h"
#include "fw_cal_follow.h"
#include "nimes.h"
#include "fw_limits.h"
#define FW_CURVE_MAX_SPEED 220u
void fw_motion_init(void);
/* One continuous forward path, including consecutive smooth 90-degree bends. */
int fw_motion_path(const nm_map_t *,nm_pose_t,const nm_route_t *,unsigned speed);
nm_pose_t fw_motion_path_pose(void);
/* Segment of the running path: index, -1 when none. turn: 0 straight, 1 right arc, -1 left arc. */
int fw_motion_path_segment(int *turn,uint32_t *length_um);
/* Bench only: each wheel -1, 0, +1, bounded to 20 mm at 20 mm/s. */
int fw_motion_test_wheels(int right, int left);
/* Nonblocking: whole calibrated cells, speeds in mm/s, clockwise turns in degrees.
 * Straights accelerate and brake as runs: fw_run_acceleration(speed). */
int fw_motion_straight(unsigned cells, unsigned speed_mm_s);
/* Allow a front-wall arrival only in the last 30 mm of the final cell. */
int fw_motion_straight_to(unsigned cells, unsigned speed_mm_s, int allow_front_wall);
int fw_motion_wall_arrival(void);
/* Bounded non-contact F5 threshold search, then calibrated centre approach.
 * 0 started, 1 no front reference visible, -1 invalid/busy. Fault 4 if the
 * centre would exceed the correction envelope. A missing threshold cancels
 * the probe and leaves fw_motion_centered() false. */
int fw_motion_center_wall(void);
int fw_motion_centered(void);
/* Pre-run only: confirmed front wall, slow bounded seating from cell centre. */
int fw_motion_seat_wall(void);
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
/* In-place turns follow the travel speed of the move they belong to: wheel
 * speed clamped to 120 mm/s (the measured rotation profile) .. 300 mm/s (the
 * run curve ceiling), acceleration fw_run_acceleration() of that wheel speed. */
#define FW_TURN_MIN_SPEED 120u
#define FW_TURN_MAX_SPEED 300u
unsigned fw_motion_turn_speed(unsigned travel_mm_s);
/* Distance in which a straight started at cruise brakes from speed to its end speed. */
uint32_t fw_motion_brake_um(unsigned speed_mm_s,unsigned cruise_mm_s);
int fw_motion_turn(int clockwise_degrees,unsigned travel_mm_s);
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
/* Wheel yaw bias preloaded into the wall follower at every session start
 * (fw_motion_init, profile change); straights keep what it learns.
 * calibrated: measured by Wall follow, so the wheel heading may replace the
 * estimate at the first crossing after a reset (never while recording). */
void fw_motion_follow_bias(int32_t mrad_m,int calibrated);
int32_t fw_motion_yaw_bias(void); /* What the follower uses now. */
/* Wall-follow calibration: one sample per IR scan of each straight while
 * recording. Returns 1 with a sample, 0 when empty, -1 if samples were lost. */
void fw_motion_follow_record(int enable);
int fw_motion_follow_sample(fw_follow_sample_t *);
void fw_motion_rotation_profile(const fw_rotation_data_t *);
int32_t fw_motion_travelled_um(void);
void fw_motion_geometry(uint32_t pitch_um, uint32_t front_on_um, uint32_t inner_um);
void fw_motion_tick(uint32_t now_ms);
void fw_motion_stop(void);
int fw_motion_busy(void);
int fw_motion_fault(void);
/* What the controller runs now (Ariane telemetry). */
enum { FW_MOTION_IDLE, FW_MOTION_STRAIGHT, FW_MOTION_TURN, FW_MOTION_CURVE, FW_MOTION_CALIB };
int fw_motion_kind(void);
uint32_t fw_motion_remaining(void);
#endif
