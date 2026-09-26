#ifndef FW_UI_H
#define FW_UI_H
#include "nimes.h"
void fw_ui_maze(const nm_map_t *, nm_pose_t, const char *status,
                unsigned speed, uint32_t milliseconds, uint8_t sensors);
#endif
