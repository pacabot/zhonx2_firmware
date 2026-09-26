#ifndef WALL_CONTROL_H
#define WALL_CONTROL_H
#include <stdint.h>
#include "fw_calibration.h"
typedef struct { int32_t filtered, previous, output; } wall_control_t;
void wall_control_reset(wall_control_t *c);
/* Active-low legacy sensor bits. Output is differential speed in permille.
 * Positive output means steer right (left wheel faster).
 * Called once per complete 10 ms sensor scan, never for each motor pulse. */
int wall_control_step(wall_control_t *c, uint8_t sensors);
int wall_control_calibrated(wall_control_t *,uint8_t,const fw_cal_data_t *);
#endif
