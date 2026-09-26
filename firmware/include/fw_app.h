#ifndef FW_APP_H
#define FW_APP_H
#include <stdint.h>
void fw_app_init(void);
int fw_app_discover(void);
int fw_app_resume(void);
int fw_app_run(void);
int fw_app_save(void);
int fw_app_restore(void);
int fw_app_bootloader(void);
extern int fw_start_corner, fw_start_heading, fw_search_speed, fw_run_speed;
#endif
