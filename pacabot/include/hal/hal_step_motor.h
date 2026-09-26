/*---------------------------------------------------------------------------
 *
 *      hal_step_motor.h
 *
 *---------------------------------------------------------------------------*/

#ifndef __HAL_STEP_MOTOR_H__
#define __HAL_STEP_MOTOR_H__

/* Module Identifier */
#define HAL_STEP_MOTOR_MODULE_ID    12

/* Error codes */
#define HAL_STEP_MOTOR_E_SUCCESS        0
#define HAL_STEP_MOTOR_E_ERROR          MAKE_ERROR(HAL_STEP_MOTOR_MODULE_ID, 1)
#define HAL_STEP_MOTOR_E_BAD_HANDLE     MAKE_ERROR(HAL_STEP_MOTOR_MODULE_ID, 2)
#define HAL_STEP_MOTOR_E_BAD_DIRECTION  0xFF

/* Stepper Motor definitions */
#define DIRECTION_FWD           1   // Forward direction
#define DIRECTION_BKW           0   // Backward direction

#define STATE_STOP              0   // Stop
#define STATE_MAINTAIN          1   // Maintain current velocity
#define STATE_ACCEL             2   // Accelerate
#define STATE_DECEL             3   // Decelerate
#define STATE_CURVE_ROTATE_L    4   // Curve rotation left
#define STATE_CURVE_ROTATE_R    5   // Curve rotation right



typedef void *HAL_STEP_MOTOR_HANDLE;


/**
 * @brief Initializes Stepper Motor module
 *
 * @description This function initializes all the variables and timers
 *              needed to generate Stepper Motor signals.
 *
 * @param       None
 * @retval      #HAL_STEP_MOTOR_E_SUCCESS if operation was successful
 */
int hal_step_motor_init(void);


int hal_step_motor_terminate(void);
int hal_step_motor_open(HAL_STEP_MOTOR_HANDLE *handle, void *params);
int hal_step_motor_close(HAL_STEP_MOTOR_HANDLE handle);


/**
 * @brief Enables the stepper motor output
 *
 * @param[in] handle Valid HAL_STEP_MOTOR_HANDLE
 *
 * @retval    HAL_STEP_MOTOR_E_SUCCESS if operation was successful
 */
int hal_step_motor_enable(void);


/**
 * @brief Disables the stepper motor output
 *
 * @param[in] handle Valid HAL_STEP_MOTOR_HANDLE
 *
 * @retval    HAL_STEP_MOTOR_E_SUCCESS if operation was successful
 */
int hal_step_motor_disable(void);


/**
 * @brief Wake-up the stepper motors from sleep mode
 *
 * @param   None
 * @retval  None
 */
void hal_step_motor_wakeup(void);


/**
 * @brief Puts the stepper motors in sleep mode (low consumption)
 *
 * @param   None
 * @retval  None
 */
void hal_step_motor_sleep(void);


/**
 * @brief Gets the frequency of a stepper motor
 *
 * @param[in] handle Valid HAL_STEP_MOTOR_HANDLE
 *
 * @retval    The frequency of the stepper motor
 */
long hal_step_motor_get_freq(HAL_STEP_MOTOR_HANDLE handle);


/**
 * @brief Sets the frequency of a stepper motor
 *
 * @param[in] handle Valid HAL_STEP_MOTOR_HANDLE
 * @param[in] freq   Desired frequency (in Hertz).
 *
 * @retval    #HAL_STEP_MOTOR_E_SUCCESS if operation was successful
 */
int hal_step_motor_set_freq(HAL_STEP_MOTOR_HANDLE handle, long freq);


/**
 * @brief Sets the stepper motor resolution
 *
 * @param[in] resolution the desired resolution.
 *   This parameter can be one of the following values:
 *     @arg FULL_STEP
 *     @arg HALF_STEP
 *     @arg QUARTER_STEP
 *     @arg EIGHTH_STEP
 *     @arg SIXTEENTH_STEP
 * @retval  #HAL_STEP_MOTOR_E_SUCCESS if operation was successful
 */
int hal_step_motor_set_resolution(unsigned char resolution);


/**
 * @brief Sets the direction of a stepper motor
 *
 * @param[in] handle Valid HAL_STEP_MOTOR_HANDLE
 * @param[in] direction The desired direction, can be one of the following:
 *              @arg DIRECTION_FWD
 *              @arg DIRECTION_BKW
 *
 * @retval  #HAL_STEP_MOTOR_E_SUCCESS if operation was successful
 */
int hal_step_motor_set_direction(HAL_STEP_MOTOR_HANDLE handle, unsigned char direction);


/**
 * @brief Gets the direction of a stepper motor
 *
 * @param[in] handle Valid HAL_STEP_MOTOR_HANDLE
 *
 * @retval    The direction of the stepper motor, which can be:
 *              @arg DIRECTION_FWD
 *              @arg DIRECTION_BKW
 *            #HAL_STEP_MOTOR_E_BAD_HANDLE in case of error
 */
unsigned short hal_step_motor_get_direction(HAL_STEP_MOTOR_HANDLE handle);


/**
 * @brief Retrieves the number of steps performed by a motor
 *
 * @param[in]   handle Valid HAL_STEP_MOTOR_HANDLE
 *
 * @retval      The number of steps that the motor has performed
 *              #HAL_STEP_MOTOR_E_BAD_HANDLE in case of error
 */
long hal_step_motor_get_counter(HAL_STEP_MOTOR_HANDLE handle);


/**
 * @brief Gets the acceleration value of a stepper motor
 *
 * @param[in]   handle Valid HAL_STEP_MOTOR_HANDLE
 *
 * @retval      The acceleration value of the stepper motor
 */
int hal_step_motor_get_acceleration(HAL_STEP_MOTOR_HANDLE handle);


/**
 * @brief Sets the acceleration value of a stepper motor
 *
 * @param[in]   handle Valid HAL_STEP_MOTOR_HANDLE
 * @param[in]   accel  The desired acceleration value for the motor
 *
 * @retval      #HAL_STEP_MOTOR_E_SUCCESS if operation was successful
 *              #HAL_STEP_MOTOR_E_BAD_HANDLE otherwise
 */
int hal_step_motor_set_acceleration(HAL_STEP_MOTOR_HANDLE handle, int accel);


/**
 * @brief Directs a stepper motor to accelerate
 *
 * @param[in]   handle Valid HAL_STEP_MOTOR_HANDLE
 *
 * @retval      #HAL_STEP_MOTOR_E_SUCCESS if operation was successful
 *              #HAL_STEP_MOTOR_E_BAD_HANDLE otherwise
 */
void hal_step_motor_accelerate(HAL_STEP_MOTOR_HANDLE handle);


/**
 * @brief Directs a stepper motor to decelerate
 *
 * @param[in]   handle Valid HAL_STEP_MOTOR_HANDLE
 *
 * @retval      #HAL_STEP_MOTOR_E_SUCCESS if operation was successful
 *              #HAL_STEP_MOTOR_E_BAD_HANDLE otherwise
 */
void hal_step_motor_decelerate(HAL_STEP_MOTOR_HANDLE handle);


/**
 * @brief Directs a stepper motor to maintain its current speed
 *
 * @param[in]   handle Valid HAL_STEP_MOTOR_HANDLE
 *
 * @retval      #HAL_STEP_MOTOR_E_SUCCESS if operation was successful
 *              #HAL_STEP_MOTOR_E_BAD_HANDLE otherwise
 */
void hal_step_motor_maintain(HAL_STEP_MOTOR_HANDLE handle);


/**
 * @brief Directs a stepper motor to accomplish a curve rotation
 *
 * This function directs the inner stepper motor to regulate itself from
 * the outer stepper motor in order to accomplish a curve rotation.
 *
 * @param[in]   handle Valid HAL_STEP_MOTOR_HANDLE
 *
 * @retval      #HAL_STEP_MOTOR_E_SUCCESS if operation was successful
 *              #HAL_STEP_MOTOR_E_BAD_HANDLE otherwise
 */
void hal_step_motor_curve_rotate(HAL_STEP_MOTOR_HANDLE handle,
                                 double curve_rad, unsigned char state);


/**
 * @brief Gets a stepper motor state
 *
 * This function gets the stepper motor state
 *
 * @param[in]   handle Valid HAL_STEP_MOTOR_HANDLE
 *
 * @retval      The stepper motor state if operation was successful
 *              #HAL_STEP_MOTOR_E_ERROR otherwise
 */
int hal_step_motor_get_state(HAL_STEP_MOTOR_HANDLE handle);


void hal_step_motor_pair_start(long right, long left);
void hal_step_motor_pair_stop(void);
void hal_step_motor_pair_rate(unsigned long right, unsigned long left);
unsigned long hal_step_motor_pair_remaining(unsigned int wheel);
void hal_step_motor_pair_release(void);
#endif /* __HAL_STEP_MOTOR_H__ */
