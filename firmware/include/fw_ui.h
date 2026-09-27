#ifndef FW_UI_H
#define FW_UI_H
#include "nimes.h"
void fw_ui_maze(const nm_map_t *, nm_pose_t, const char *status,
                unsigned speed, uint32_t milliseconds, uint8_t sensors);
void fw_ui_map_view(unsigned scale,int pan_x,int pan_y);
void fw_ui_map_progress(int permille);
void fw_ui_result(const char *reason,uint32_t search_ms,uint32_t run_ms,unsigned page);
#endif
