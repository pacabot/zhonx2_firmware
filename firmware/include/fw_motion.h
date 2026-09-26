#ifndef FW_MOTION_H
#define FW_MOTION_H
#include <stdint.h>
void fw_motion_init(void);
/* Nonblocking: whole calibrated cells, speeds in mm/s, clockwise turns in degrees. */
int fw_motion_straight(unsigned cells, unsigned speed_mm_s);
/* Allow a front-wall arrival only in the last 30 mm of the final cell. */
int fw_motion_straight_to(unsigned cells, unsigned speed_mm_s, int allow_front_wall);
int fw_motion_wall_arrival(void);
int fw_motion_turn(int clockwise_degrees);
void fw_motion_tick(uint32_t now_ms);
void fw_motion_stop(void);
int fw_motion_busy(void);
int fw_motion_fault(void);
uint32_t fw_motion_remaining(void);
#endif
