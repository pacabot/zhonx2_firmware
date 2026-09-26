#ifndef FW_APP_H
#define FW_APP_H
#include <stdint.h>
#include "fw_calibration.h"
void fw_app_init(void);
int fw_app_discover(void);
int fw_app_resume(void);
int fw_app_run(void);
int fw_app_save(void);
int fw_app_show_map(void);
extern volatile unsigned fw_last_stop_code;
int fw_app_restore(void);
int fw_app_bootloader(void);
extern int fw_start_corner, fw_start_heading, fw_search_speed, fw_run_speed;
extern int fw_cal_nose_tenth_mm, fw_cal_width_tenth_mm, fw_cal_inner_mm, fw_cal_pitch_mm;
const fw_cal_data_t *fw_app_calibration(void);
int fw_app_calibration_commit(const fw_cal_data_t *);
int fw_calibrate_menu(void);
int fw_calibration_report(void);
#endif
