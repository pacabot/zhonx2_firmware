/*---------------------------------------------------------------------------
 *
 *      hal_beeper.c
 *
 *---------------------------------------------------------------------------*/

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

/* ST Lib declarations */
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"

#include "hal/hal_os.h"

/* Declarations for this module */
#include "hal/hal_beeper.h"

#include <string.h>

#define HAL_BEEPER_ON

#define BEEPER_TIMER    TIM4

/* APB2 bus frequency (in Hz) */
#define APB2_FREQ 42000000
/* Beeper Timer base frequency (in Hz) */
#define TIMER_FREQ 128000
/* Beeper frequency (in Hz) */
#define BEEPER_FREQ 16000
/* Timer prescaler (PSC register) */
#define TIMER_PRESCALER (((APB2_FREQ) / (TIMER_FREQ)) - 1)
/* Timer period (ARR register) */
#define TIMER_PERIOD (((TIMER_FREQ) / (BEEPER_FREQ)) - 1)

#define TIM_CH3_PIN     	GPIO_Pin_8
#define TIM_CH3_PIN_SOURCE  GPIO_PinSource8

/* Static functions */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);

/* Types definitions */
typedef struct
{
    unsigned char channel;
    long          freq;
    bool          isInitialized;
}
beeper_handle;

static beeper_handle beeper;


int hal_beeper_init(void)
{
#ifdef HAL_BEEPER_ON
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    if (beeper.isInitialized == true)
    {
        return HAL_BEEPER_E_SUCCESS;
    }
    memset(&beeper, 0, sizeof(beeper_handle));

    /* System Clocks Configuration */
    RCC_Configuration();
    /* GPIO Configuration */
    GPIO_Configuration();

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = TIMER_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = TIMER_PRESCALER;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(BEEPER_TIMER, &TIM_TimeBaseStructure);
#endif
    return HAL_BEEPER_E_SUCCESS;
}


int hal_beeper_terminate(void)
{
    return HAL_BEEPER_E_SUCCESS;
}


int hal_beeper_open(HAL_BEEPER_HANDLE *handle, void *params)
{
#ifdef HAL_BEEPER_ON
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    UNUSED(params);

    if (beeper.isInitialized == false)
    {
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = 0;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

        TIM_OC3Init(BEEPER_TIMER, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(BEEPER_TIMER, TIM_OCPreload_Enable);

        TIM_ARRPreloadConfig(BEEPER_TIMER, ENABLE);
        beeper.isInitialized = true;
        BEEPER_TIMER->CCER &= ~TIM_CCER_CC3E;
        zhonxSettings.beeper_enabled = true;
    }
    *handle = (HAL_BEEPER_HANDLE)&beeper;
#endif
    return HAL_BEEPER_E_SUCCESS;
}


int hal_beeper_close(HAL_BEEPER_HANDLE handle)
{
    return HAL_BEEPER_E_SUCCESS;
}


int hal_beeper_get_state(HAL_BEEPER_HANDLE handle, int *state)
{
    UNUSED(handle);

    if (zhonxSettings.beeper_enabled == true)
    {
        *state = HAL_BEEPER_STATE_ON;
    }
    else
    {
        *state = HAL_BEEPER_STATE_OFF;
    }

    return HAL_BEEPER_E_SUCCESS;
}


int hal_beeper_set_state(HAL_BEEPER_HANDLE handle, int state)
{
    UNUSED(handle);

    /* Ignore the state if it is not defined */
    switch (state)
    {
        case HAL_BEEPER_STATE_OFF:
            zhonxSettings.beeper_enabled = false;
            break;
        case HAL_BEEPER_STATE_ON:
            zhonxSettings.beeper_enabled = true;
            break;
    }
    return HAL_BEEPER_E_SUCCESS;
}


int hal_beeper_tone(HAL_BEEPER_HANDLE handle, long freq)
{
#ifdef HAL_BEEPER_ON
    if (handle == null) return HAL_BEEPER_E_BAD_HANDLE;
    beeper_handle *h = (beeper_handle *)handle;
    if (freq <= 0 || !zhonxSettings.beeper_enabled) {
        BEEPER_TIMER->CCER &= ~TIM_CCER_CC3E;
        TIM_Cmd(BEEPER_TIMER, DISABLE); h->freq = 0;
        return HAL_BEEPER_E_SUCCESS;
    }
    if (freq > TIMER_FREQ) freq = TIMER_FREQ;
    unsigned long period = TIMER_FREQ / freq - 1;
    if (period > 0xffff) period = 0xffff;
    TIM_SetAutoreload(BEEPER_TIMER, period);
    h->freq = freq;
    BEEPER_TIMER->CCER |= TIM_CCER_CC3E;
    TIM_Cmd(BEEPER_TIMER, ENABLE);
#else
    UNUSED(handle); UNUSED(freq);
#endif
    return HAL_BEEPER_E_SUCCESS;
}
int hal_beeper_beep(HAL_BEEPER_HANDLE handle, long freq, long duration)
{
    int result = hal_beeper_tone(handle, freq);
    if (result) return result;
    HAL_Delay(duration);
    return hal_beeper_tone(handle, 0);
}


/**
 * @brief  Configure the different system clocks.
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* GPIOB clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
}


/**
 * @brief  Configure the timer output channels.
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_PinAFConfig(GPIOB, TIM_CH3_PIN_SOURCE, GPIO_AF_TIM4);

    GPIO_InitStructure.GPIO_Pin = TIM_CH3_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
