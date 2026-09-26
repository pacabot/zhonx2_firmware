/**
 * hal_pid.c
 */

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

/* Declarations for this module */
#include "hal/hal_pid.h"

/* ST Lib declarations */
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

#ifdef STM32F100RB
/* APB2 bus frequency (in Hz) */
# define APB2_FREQ 24000000
/* PWM Timer base frequency (in Hz) */
# define TIMER_FREQ 240000//36000000

#else
/* APB2 bus frequency (in Hz) */
# define APB2_FREQ 42000000
/* Timer base frequency (in KHz) */
# define TIMER_FREQ 32000
#endif // STM32F100RB

/* Toggle frequency (in Hz) */
#define PID_FREQ CORRECTION_I												// Param for Integer current
/* Timer prescaler (PSC register) */
#define TIMER_PRESCALER (((APB2_FREQ) / (TIMER_FREQ)) - 1)
/* Timer period (ARR register) */
#define TIMER_PERIOD    (((TIMER_FREQ) / (PID_FREQ)) - 1)

#define PID_TIM TIM5

#define L_CORRECTION    1
#define R_CORRECTION    2
#define NO_CORRECTION   3

#define L_WALL_CORRECTION     1
#define R_WALL_CORRECTION     2

#define MAX_CHANNELS    1

/* Macros */

/* Types definitions */
typedef struct {
    volatile long  current_left_counter;
    volatile long  current_right_counter;
    volatile long  old_left_counter;
    volatile long  old_right_counter;
    volatile int   type_correction;
    volatile int   wall_correction;
} pid_handle;

static pid_handle pid;

/* Static functions */
static void RCC_Configuration(void);
static void NVIC_Configuration(void);

int hal_pid_init(void)
{
//	int rv;
    memset(&pid, 0, sizeof(pid_handle));

    /* System Clocks Configuration */
    RCC_Configuration();

    /* Nested Vector Interrupt Controller configuration */
    NVIC_Configuration();
    hal_pid_open();

    pid.wall_correction = NO_CORRECTION;

    return HAL_PID_E_SUCCESS;
}


int hal_pid_terminate(void)
{
    memset(&pid, 0, sizeof(pid));

    return HAL_PID_E_SUCCESS;
}

int hal_pid_open()//HAL_PID_HANDLE *handle, void *params)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

//    UNUSED(params);

//    *handle = (HAL_PID_HANDLE)&pid;

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = TIMER_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = TIMER_PRESCALER;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(PID_TIM, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(PID_TIM, ENABLE);

    /* Enable interrupts on Capture/Compare */
    TIM_ITConfig(PID_TIM, TIM_IT_Update, ENABLE);

    TIM_ClearFlag(PID_TIM, TIM_FLAG_Update);

    /* Enable timer counter */
    TIM_Cmd(PID_TIM, ENABLE);

    return HAL_PID_E_SUCCESS;
}

int hal_pid_close()
{
//    UNUSED(handle);

    return HAL_PID_E_SUCCESS;
}

long hal_pid_left_get_count()
{
//    pid_handle *h;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return;
    }
#endif // CONFIG_DEBUG

//    h = (pid_handle *)handle;

	return pid.current_left_counter;
}

long hal_pid_right_get_count()
{
//    pid_handle *h;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return;
    }
#endif // CONFIG_DEBUG

//    h = (pid_handle *)handle;

	return pid.current_right_counter;
}

int hal_pid_left_set_count(long count)
{
//    pid_handle *h;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return;
    }
#endif // CONFIG_DEBUG

//    h = (pid_handle *)count;

    pid. current_left_counter = count;

    return HAL_PID_E_SUCCESS;
}

int  hal_pid_right_set_count(long count)
{
//    pid_handle *h;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return;
    }
#endif // CONFIG_DEBUG

//    h = (pid_handle *)count;

    pid. current_right_counter = count;

    return HAL_PID_E_SUCCESS;
}

long hal_pid_left_get_old_count()
{
	return pid.old_left_counter;
}

long hal_pid_right_get_old_count()
{
	return pid.old_right_counter;
}

int hal_pid_left_set_old_count(long count)
{
    pid.old_left_counter = count;

    return HAL_PID_E_SUCCESS;
}

int hal_pid_right_set_old_count(long count)
{
    pid.old_right_counter = count;

    return HAL_PID_E_SUCCESS;
}

int hal_pid_get_type_correction()
{
	return pid.type_correction;
}

int hal_pid_set_type_correction(int type)
{
    pid. type_correction = type;

    return HAL_PID_E_SUCCESS;
}

int hal_pid_get_wall_correction()
{
	return pid.wall_correction;
}

int hal_pid_set_wall_correction(int type)
{
    pid. wall_correction = type;

    return HAL_PID_E_SUCCESS;
}

/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
    /* Timer clocks enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
}

void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable TIMER 5 Interrupt handling */
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**********************
 * Interrupt handlers *
 **********************/

void TIM5_IRQHandler(void)
{
    if (TIM_GetFlagStatus(PID_TIM, TIM_FLAG_Update) == RESET)
    {
        return;
    }
    TIM_ClearFlag(PID_TIM, TIM_FLAG_Update);

//    state = ~state;
//
//    if (state > 0)
//    {
//        hal_led_set_state(app_context.led, HAL_LED_COLOR_RED, DISABLE);
//        hal_led_set_state(app_context.led, HAL_LED_COLOR_ORANGE, ENABLE);
//
//    }
//    else
//    {
//        hal_led_set_state(app_context.led, HAL_LED_COLOR_RED, ENABLE);
//        hal_led_set_state(app_context.led, HAL_LED_COLOR_ORANGE, DISABLE);
//    }

    switch (pid.type_correction)
    {
        case L_CORRECTION:
        	if (pid.current_left_counter < zhonxSettings.max_correction)
        	{
        		pid.current_left_counter++;
        	}
            break;
        case R_CORRECTION:
        	if (pid.current_right_counter < zhonxSettings.max_correction)
        	{
        		pid.current_right_counter++;
        	}
            break;
        case NO_CORRECTION:
 //       	pid.current_right_counter = 0;
 //       	pid.current_left_counter  = 0;
            break;
    }
}
