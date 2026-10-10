#ifndef WALL_CONTROL_H
#define WALL_CONTROL_H
#include <stdint.h>
#include "fw_calibration.h"
typedef struct {
    int32_t filtered, previous, output;
    int32_t lateral_um, heading_mrad, innovation_q8;
    int32_t yaw_bias_mrad_m, bias_fraction, reference_distance, reference_yaw;
    int32_t optical_distance,optical_yaw,optical_position;
    int64_t optical_integral;
    uint32_t persist_um,exit_um;
    /* Heading from the reset, wheel yaw and post pairs only, without the
     * lateral innovations, and travel since the reset. With a calibrated
     * yaw bias it replaces the estimate at the first crossing after a reset. */
    int32_t wheel_heading_mrad;
    uint32_t unanchored_um;
    uint8_t initialized, sensors, reference_side,optical_side,lag_scans,anchor_estimated;
    uint8_t unanchored,bias_calibrated;
} wall_control_t;
/* Yaw bias error tolerated on a calibrated bias, per metre since the reset. */
#define WALL_BIAS_DOUBT_MRAD_M 20
void wall_control_reset(wall_control_t *c);
/* Active-low legacy sensor bits. Output is differential speed in permille.
 * Positive output means steer right (left wheel faster).
 * Called once per complete 10 ms sensor scan, never for each motor pulse. */
int wall_control_step(wall_control_t *c, uint8_t sensors);
int wall_control_calibrated(wall_control_t *,uint8_t,const fw_cal_data_t *);
/* Quantized position observer: integrate wheel travel, constrain it with measured
 * binary thresholds, then control heading as well as lateral position. */
int wall_control_position(wall_control_t *,uint8_t,const fw_cal_data_t *,
                          int32_t forward_um,int32_t yaw_delta_mrad);
void wall_control_heading_reference(wall_control_t *,int32_t heading_mrad);
void wall_control_exit(wall_control_t *,const fw_cal_data_t *,uint8_t,int32_t,uint32_t);
#endif
