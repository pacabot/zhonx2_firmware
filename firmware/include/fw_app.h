#ifndef FW_APP_H
#define FW_APP_H
#include <stdint.h>
#include "fw_calibration.h"
#include "fw_cal_extra.h"
#include "fw_library.h"
void fw_app_init(void);
int fw_app_discover(void);
int fw_app_resume(void);
int fw_app_run(void);
int fw_app_run_slow(void);
int fw_app_run_curves(void);
int fw_app_ready(void);
unsigned fw_app_maze_count(void);
const fw_saved_maze_t *fw_app_maze(unsigned index);
int fw_app_maze_load(unsigned index);
int fw_app_maze_delete(unsigned index);
int fw_app_settings_save(void);
int fw_app_battery_commit(unsigned raw,unsigned mv);
int fw_app_save(void);
int fw_app_show_map(void);
extern volatile unsigned fw_last_stop_code;
int fw_app_restore(void);
int fw_app_bootloader(void);
extern int fw_maze_size;
extern int fw_start_corner, fw_start_heading, fw_search_speed, fw_run_speed;
extern int fw_cal_nose_tenth_mm, fw_cal_width_tenth_mm, fw_cal_inner_mm, fw_cal_pitch_mm;
const fw_cal_data_t *fw_app_calibration(void);
int fw_app_calibration_commit(const fw_cal_data_t *);
int fw_calibrate_menu(void);
int fw_calibration_report(void);
const fw_rotation_data_t *fw_app_rotation(void);
const fw_corner_data_t *fw_app_corner(unsigned reference_side);
int fw_app_rotation_commit(const fw_rotation_data_t *);
int fw_app_corner_commit(const fw_corner_data_t *);
int fw_rotation_menu(void);
int fw_rotation_report(void);
int fw_corner_left_menu(void);
int fw_corner_right_menu(void);
int fw_corner_report(void);
extern int fw_cal_post_mm;
#endif
