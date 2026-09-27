#ifndef WALL_CONTROL_H
#define WALL_CONTROL_H
#include <stdint.h>
#include "fw_calibration.h"
typedef struct {
    int32_t filtered, previous, output;
    int32_t lateral_um, heading_mrad, innovation_q8;
    uint8_t initialized, sensors;
} wall_control_t;
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
#endif
