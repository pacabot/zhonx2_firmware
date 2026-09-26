/* config.h */

#ifndef __CONFIG_H__
#define __CONFIG_H__

/* Define the folowing variable to activate debug mode */
//#define CONFIG_DEBUG
//#define LCD_DEBUG_MODE

/* Enable interrupts for hal_serial module */
//#define CONFIG_HAL_SERIAL_ENABLE_INTERRUPTS

/* Enable interrupts for hal_us_sensor module */
//#define CONFIG_HAL_US_SENSOR_ENABLE_INTERRUPTS

/* TIMER Channels convention */
#define TIMER_CH1   0
#define TIMER_CH2   1
#define TIMER_CH3   2
#define TIMER_CH4   3

#define ZHONX2


/***********************************************************************
 *                      Stepper motors definitions                     *
 ***********************************************************************/

/* Stepper motor precision */
#define FULL_STEP       0
#define HALF_STEP       1
#define QUARTER_STEP    2
#define EIGHTH_STEP     4
#define SIXTEENTH_STEP  8

/* Default stepper motor precision */
#define STEPPER_MOTOR_PRECISION SIXTEENTH_STEP

/* Number of full steps per revolution */
#define FULL_STEPS_PER_REVOLUTION   (200)
/* Compute number of effective steps per revolution */
#if ((STEPPER_MOTOR_PRECISION) != (FULL_STEP))
# define STEPS_PER_REVOLUTION       (2 * (STEPPER_MOTOR_PRECISION) * (FULL_STEPS_PER_REVOLUTION))
#else
# define STEPS_PER_REVOLUTION       (2 * (FULL_STEPS_PER_REVOLUTION))
#endif
/* Wheel diameter in millimeters */
#define WHEEL_DIAMETER              (24.45)//(24.45)
/* Number of millimeters per wheel revolution */
#define MM_PER_REVOLUTION           ((M_PI) * (WHEEL_DIAMETER))
/* Number of steps per millimeter */
#define STEPS_PER_MM                ((STEPS_PER_REVOLUTION) / (MM_PER_REVOLUTION))
/* Distance between the 2 wheels */
#define WHEELS_DISTANCE             (83.5)//(84.1)
#define WHEELS_DISTANCE_STEPS       ((WHEELS_DISTANCE) * (STEPS_PER_REVOLUTION))

/* Maze Properties */
#define CELL_LENGTH_STEP            (CELL_LENGTH * STEPS_PER_MM)
#define CELL_LENGTH                 (178)


/* Correction factor for in-place rotation */
#define ANGLE_CORRECTION            (1)//(1.0465)

/* Initial speed */
#define INITIAL_SPEED               (5000) //(6000)//1500
/* Absolute Maximum speed */
#define MAX_SPEED                   (100000)//(100000)
/* Minimum distance to reach the maximum speed */
#define MAX_SPEED_DISTANCE          (155 * STEPS_PER_MM) // MAX 400
/* Added frequency at each step for acceleration and deceleration */
#define DEFAULT_ACCEL               (4)//5
/* Acceleration value for angular movement */
#define ROTATE_ACCEL                (4)//5
/* Deceleration value for emergency stop */
#define EMERGENCY_DECEL             (50)
/* Correction 1/P factor */
#define CORRECTION_P                (1600)//1500       // frequence de correction en hz, plus la valeur est grande plus la correction est grande
/* Correction I factor */
#define CORRECTION_I                (4000)		//200// frequence en hz du timer d'integration
/* Max correction */
#define MAX_CORRECTION              (3000)


//pour le run :
//5500
//4
//4
//1500

/* Battery parameters */
#define BATTERY_MIN_VALUE           (2700)  // Theoretically corresponds to 6V
#define BATTERY_MAX_VALUE           (4095)  // Theoretically corresponds to 8.25V
#define BATTERY_USAGE_ZONE          (BATTERY_MAX_VALUE - BATTERY_MIN_VALUE)

/* Maze settings */
#define MAZE_MAX_SIZE               16

typedef struct
{
    unsigned long   initial_speed;
    unsigned long   max_speed_distance;
    unsigned long   default_accel;
    unsigned long   rotate_accel;
    unsigned long   emergency_decel;
    unsigned long   correction_p;
    unsigned long   correction_i;
    unsigned long   max_correction;
    unsigned char   calibration_enabled;
    unsigned char   color_sensor_enabled;
    unsigned char   beeper_enabled;
    unsigned long   threshold_color;
    unsigned char   threshold_greater;
    unsigned int	x_finish_maze;
    unsigned int	y_finish_maze;
    int wall_know_cost;
    int cell_cost;
    int start_orientation;
}robot_settings;

typedef struct
{
    char    maze[MAZE_MAX_SIZE][MAZE_MAX_SIZE];
    char    cmd_string[(MAZE_MAX_SIZE * MAZE_MAX_SIZE) / 4];
    int     cmd_size; // Size of the command string
    short   magic_word;
} maze_object;

robot_settings zhonxSettings;

#endif // __CONFIG_H__

