#ifndef FW_MENU_H
#define FW_MENU_H
#include "fw_library.h"
enum { FW_ICON_MAZE, FW_ICON_CALIBRATE, FW_ICON_SETTINGS, FW_ICON_RUN, FW_ICON_REPORT, FW_ICON_UPDATE };
void fw_menu_run(void);
void fw_ui_card(const char *title,const char *first,const char *second,unsigned icon,unsigned index,unsigned count);
void fw_ui_library(const fw_saved_maze_t *,unsigned index,unsigned count,int blink);
void fw_ui_setting(unsigned index,int value);
#endif
