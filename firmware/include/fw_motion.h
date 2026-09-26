#ifndef FW_MOTION_H
#define FW_MOTION_H
#include <stdint.h>
void fw_motion_init(void);
/* Nonblocking: whole 180 mm cells, speeds in mm/s, clockwise turns in degrees. */
int fw_motion_straight(unsigned cells, unsigned speed_mm_s);
int fw_motion_turn(int clockwise_degrees);
void fw_motion_tick(uint32_t now_ms);
void fw_motion_stop(void);
int fw_motion_busy(void);
int fw_motion_fault(void);
uint32_t fw_motion_remaining(void);
#endif
