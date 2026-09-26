#include "fw_app.h"
/*---------------------------------------------------------------------------
 *
 *      menus.c
 *
 *---------------------------------------------------------------------------*/

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx.h"

#include "config/basetypes.h"
#include "config/errors.h"
#include "config/config.h"

#include "hal/hal_os.h"
#include "hal/hal_adc.h"
#include "hal/hal_sensor.h"
#include "hal/hal_serial.h"
#include "hal/hal_beeper.h"
#include "hal/hal_led.h"
#include "hal/hal_nvm.h"
#include "hal/hal_ui.h"
#include "hal/hal_step_motor.h"

#include "oled/arrows_bmp.h"

#include "app/app_def.h"
#include "app/menus.h"

#include <stdio.h>
#include <string.h>

/* Extern functions */
extern int test_hal_adc(void);
extern int test_hal_color_sensor(void);
extern int test_hal_sensor(void);
//extern int test_hal_beeper(void);
//extern int test_hal_led(void);
//extern int test_hal_ui(void);
//extern int test_hal_nvm(void);
//extern int test_hal_step_motor(void);
extern int test_oled1(void);
extern int test_oled2(void);
extern int test_oled3(void);
extern int test_step_motor_driver(void);
//extern int test_remote_control(void);
extern int test_motor_rotate(void);
extern int distance_cal(void);
extern int mazeBertrand(int i, int str);
static int start_maze(void) { return fw_app_discover(); }

extern int sensor_calibrate(void);

/********************
 * Extern variables *
 ********************/
//extern char _ChaineCommande[(MAZE_MAX_SIZE * MAZE_MAX_SIZE) / 4];

/********************
 * Global variables *
 ********************/
maze_object *current_maze = null;

/* Container of stored mazes (in flash memory) */
__attribute__ ((section(".robot_settings")))
const maze_object stored_mazes[5] =
    {
        {
            {
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000")
            },
            {"000000000000000000000000000000"},
            (0),
            (0)
        },
        {
            {
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000")
            },
            {"000000000000000000000000000000"},
            (0),
            (0)
        },
        {
            {
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000")
            },
            {"000000000000000000000000000000"},
            (0),
            (0)
        },
        {
            {
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000")
            },
            {"000000000000000000000000000000"},
            (0),
            (0)
        },
        {
            {
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000"),
                ("0000000000")
            },
            {"000000000000000000000000000000"},
            (0),
            (0)
        }
    };

__attribute__ ((section(".robot_settings")))
const robot_settings stored_settings[5] =
    {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };


/* Main menu */
static const menu_item main_menu[] =
{
    {"ZHONX II                 V1.0",   null},
    {"Maze menu",                       display_maze_menu},
    {"Parameters",                      display_parameters},
    {"Tests menu",                      display_tests_menu},
    {"[b]Beeper Enabled?   ",           null, &zhonxSettings.beeper_enabled},
    {null,                              null}
};

/* Parameters menu */
static const menu_item params_menu[] =
{
    {"PARAMETERS",                  null},
    {"Motion settings",             display_motion_settings},
    {"PID settings",                display_PID_settings},
    {"Sensor settings",             display_sensor_settings},
    {"Save settings",               display_save_settings},
    {"Restore settings",            display_restore_settings},
    {null,                          null}
};

/* Tests menu */
static const menu_item tests_menu[] =
{
    {"TESTS MENU",                  null},
    {"Test ADC",                    test_hal_adc},
    //{"Test beeper",                 test_hal_beeper},
//    {"Test OLED",                   display_oled_menu},
//    {"Test Flash",                  test_hal_nvm},
    {"Test motor",                  display_motor_menu},
    {"Test sensor",                 test_hal_sensor},
    {"Test color sensor",           test_hal_color_sensor},
    {null,                          null}
};

/* Motor menu */
static const menu_item motor_menu[] =
{
    {"MOTOR MENU",                  null},
    {"Random move",                 null},
    {"Rotate calib.",               test_motor_rotate},
    {"Distance calib.",             distance_cal},
    {"Stepper motor move",          test_step_motor_driver},
    {"Acceleration calib.",         null},
    {null,                          null}
};

/* OLED menu */
static const menu_item oled_menu[] =
{
    {"OLED MENU",                   null},
    {"OLED Test 1",                 test_oled1},
    {"OLED Test 2",                 test_oled2},
    {"OLED Test 3",                 test_oled3},
    {null,                          null}
};

/* Maze menu */
static const menu_item maze_menu[] =
{
    {"MAZE MENU",                   null},
    {"New maze",                    start_maze},
    {"[l]x finish",                null, &zhonxSettings.x_finish_maze, 1},
    {"[l]y finish",                null, &zhonxSettings.y_finish_maze, 1},
//    {"Trajectory...",               display_trajectory_menu},
//    {"[b]Calibration  :",           null, &zhonxSettings.calibration_enabled},
//    {"Restore maze",                display_restore_maze_menu},
    {null,                          null}
};

/* Motion settings menu */
static const menu_item motion_settings_menu[] =
{
    {"MOTION SETTINGS",             null},
    {"[l]Initial speed :",          null, &zhonxSettings.initial_speed, 10},
    {"[l]Max speed dist:",          null, &zhonxSettings.max_speed_distance, 10},
    {"[l]Default accel :",          null, &zhonxSettings.default_accel, 1},
    {"[l]Rotate accel  :",          null, &zhonxSettings.rotate_accel, 1},
    {"[l]Emerg decel   :",          null, &zhonxSettings.emergency_decel, 5},
    {null,                          null}
};

/* PID settings menu */
static const menu_item pid_settings_menu[] =
{
    {"PID SETTINGS",                null},
    {"[l]Proportional  :",          null, &zhonxSettings.correction_p, 10},
    {"[l]Integral      :",          null, &zhonxSettings.correction_i, 10},
    {"[l]Max correction:",          null, &zhonxSettings.max_correction, 10},
    {null,                          null}
};


/* Color sensor settings menu */
static const menu_item sensor_settings_menu[] =
{
    {"COLOR SENSOR SETTINGS",       null},
    {"[b]Enabled?      :",          null, &zhonxSettings.color_sensor_enabled},
    {"Calibrate color",             sensor_calibrate},
    {"[l]Threshold val :",          null, &zhonxSettings.threshold_color, 500},
    {"[b]End Greater?  :",          null, &zhonxSettings.threshold_greater},
    {null,                          null}
};

/* Trajectory menu */
static const menu_item trajectory_menu[] =
{
    {"SELECT TRAJECTORY",           null},
    {"Trajectory 1",                null},
    {"Trajectory 2",                null},
    {"Trajectory 3",                null},
    {null,                          null}
};

/* Restore maze menu */
static const menu_item restore_maze_menu[] =
{
    {"SELECT MAZE",                 null},
    {"Maze 1",                      null},
    {"Maze 2",                      null},
    {"Maze 3",                      null},
    {"Maze 4",                      null},
    {"Maze 5",                      null},
    {null,                          null}
};

static const menu_item settings_menu[] =
{
    {"SELECT MEMORY BLOCK",         null},
    {"Settings 1",                  null},
    {"Settings 2",                  null},
    {"Settings 3",                  null},
    {"Settings 4",                  null},
    {"Settings 5",                  null},
    {null,                          null},
};

/* Orientation menu */
static const bmp_menu_item orientation_menu[] =
{
    {null,
     null,
     null,
     0, 0,
     0, 0,
     0,
     "Favorite direction"},

    {null,
     up_arrow,
     up_arrow_hover,
     52, 0,
     24, 24,
     MENU_DIRECTION_UP,
     "North"},

    {null,
     down_arrow,
     down_arrow_hover,
     52, 40,
     24, 24,
     MENU_DIRECTION_DOWN,
     "South"},

    {null,
     left_arrow,
     left_arrow_hover,
     0, 20,
     24, 24,
     MENU_DIRECTION_LEFT,
     "West"},

    {null,
     right_arrow,
     right_arrow_hover,
     104, 20,
     24, 24,
     MENU_DIRECTION_RIGHT,
     "East"}
};


// Displays the main menu
int display_main_menu(void)
{
    int rv;
    int selected_index = 0;

    while (true)
    {
        rv = hal_ui_display_menu(app_context.ui,
                                 (void *)main_menu, &selected_index, false);
    }
    return rv;
}


// Displays the parameters menu
int display_parameters(void)
{
    int rv;
    int selected_index = 0;

    rv = hal_ui_display_menu(app_context.ui,
                             (void *)params_menu, &selected_index, false);
    if (rv != HAL_UI_E_SUCCESS)
    {
        return rv;
    }
    return selected_index;
}


// Displays motion parameters
int display_motion_settings(void)
{
    int rv;
    int selected_index = 0;

    rv = hal_ui_display_menu(app_context.ui,
                             (void *)motion_settings_menu,
                             &selected_index, false);
    if (rv != HAL_UI_E_SUCCESS)
    {
        return rv;
    }
    return selected_index;
}


// Displays PID parameters
int display_PID_settings(void)
{
    int rv;
    int selected_index = 0;

    rv = hal_ui_display_menu(app_context.ui,
                             (void *)pid_settings_menu, &selected_index, false);
    if (rv != HAL_UI_E_SUCCESS)
    {
        return rv;
    }
    return selected_index;
}


// Displays color sensor settings
int display_sensor_settings(void)
{
    int rv;
    int selected_index = 0;

    rv = hal_ui_display_menu(app_context.ui,
                             (void *)sensor_settings_menu,
                             &selected_index, false);
    if (rv != HAL_UI_E_SUCCESS)
    {
        return rv;
    }
    return selected_index;
}


// Displays the 'Save settings' menu
int display_save_settings(void)
{
    return fw_app_save();
}


// Displays the 'Restore settings' menu
int display_restore_settings(void)
{
    return fw_app_restore();
}


// Displays tests menu
int display_tests_menu(void)
{
    int rv;
    int selected_index = 0;

    rv = hal_ui_display_menu(app_context.ui,
                             (void *)tests_menu, &selected_index, false);
    if (rv != HAL_UI_E_SUCCESS)
    {
        return rv;
    }
    return selected_index;
}


// Displays motor menu
int display_motor_menu(void)
{
    int rv;
    int selected_index = 0;

    rv = hal_ui_display_menu(app_context.ui,
                             (void *)motor_menu, &selected_index, false);
    if (rv != HAL_UI_E_SUCCESS)
    {
        return rv;
    }
    return selected_index;
}


// Displays screen menu
int display_oled_menu(void)
{
    int rv;
    int selected_index = 0;

    rv = hal_ui_display_menu(app_context.ui,
                             (void *)oled_menu, &selected_index, false);
    if (rv != HAL_UI_E_SUCCESS)
    {
        return rv;
    }
    return selected_index;
}


/* Let the user choose the preferred direction */
int display_orientation_menu(void)
{
    int rv;

    rv = hal_ui_display_bmp_menu(app_context.ui,
                                 (void *)orientation_menu, null);
    if (rv != HAL_UI_E_SUCCESS)
    {
        return rv;
    }
    return 0;
}


/* Displays maze menu */
int display_maze_menu(void)
{
    int rv;
    int selected_index = 0;

    rv = hal_ui_display_menu(app_context.ui,
                             (void *)maze_menu, &selected_index, false);
    if (rv != HAL_UI_E_SUCCESS)
    {
        return rv;
    }
    return selected_index;
}


/* Displays trajectory menu */
int display_trajectory_menu(void)
{
    int rv;
    int selected_index = 0;
    char str[30];

    rv = hal_ui_display_menu(app_context.ui,
                             (void *)trajectory_menu, &selected_index, true);
    if (rv != HAL_UI_E_SUCCESS)
    {
        return rv;
    }

    hal_ui_clear_scr(app_context.ui);
    sprintf(str, "Trajectory %i", selected_index);
    hal_ui_display_txt(app_context.ui, 4, 24, str);
    hal_ui_refresh(app_context.ui);

    //run_trajectory(selected_index);

    return selected_index;
}


/* Displays restore maze menu */
int display_restore_maze_menu(void)
{
    int     rv;
    int     selected_index = 0;
    char    str[50];

    rv = hal_ui_display_menu(app_context.ui,
                             (void *)restore_maze_menu, &selected_index, true);
    if (rv != HAL_UI_E_SUCCESS)
    {
        return rv;
    }

    if (selected_index > 0)
    {
        current_maze = (maze_object *)&stored_mazes[selected_index - 1];
        sprintf(str, "Maze %i restored!", selected_index);
        hal_ui_display_prompt(app_context.ui, "MAZE RESTORED", str);
    }

    return selected_index;
}
