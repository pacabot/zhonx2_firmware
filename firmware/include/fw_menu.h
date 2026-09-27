#ifndef FW_MENU_H
#define FW_MENU_H
#include "fw_library.h"
enum { FW_ICON_MAZE, FW_ICON_CALIBRATE, FW_ICON_SETTINGS, FW_ICON_RUN, FW_ICON_REPORT, FW_ICON_UPDATE, FW_ICON_TEST,
    FW_ICON_WALL, FW_ICON_TURN, FW_ICON_EDGE_LEFT, FW_ICON_EDGE_RIGHT };
void fw_menu_run(void);
void fw_ui_menu_refresh(void);
void fw_ui_battery(void);
void fw_ui_header(const char *title);
void fw_ui_hint(const char *text);
void fw_menu_battery_setup(void);
void fw_ui_card(const char *title,const char *first,const char *second,unsigned icon,unsigned index,unsigned count);
void fw_ui_library(const fw_saved_maze_t *,unsigned index,unsigned count,int blink);
void fw_ui_setting(unsigned index,int value);
#endif
