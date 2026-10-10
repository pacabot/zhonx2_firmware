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

/* TIM4 counts at 1 MHz: APB1 timer clock (2 x PCLK1 = 84 MHz) / 84. */
#define TIMER_FREQ 1000000
/* Initial period, overwritten by every tone. */
#define TIMER_PERIOD 999

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
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    TIM_TimeBaseStructure.TIM_Period = TIMER_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(2 * clocks.PCLK1_Frequency / TIMER_FREQ - 1);
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
        /* PWM: the output is high (Q1 on, current in the buzzer) during
         * the first CCR3 counts of each period, see hal_beeper_tone(). */
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = 0;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

        TIM_OC3Init(BEEPER_TIMER, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(BEEPER_TIMER, TIM_OCPreload_Enable);

        TIM_ARRPreloadConfig(BEEPER_TIMER, ENABLE);
        beeper.isInitialized = true;
        BEEPER_TIMER->CCER &= ~TIM_CCER_CC3E;
        /* zhonxSettings.beeper_enabled is the saved volume level (fw_sound.h):
         * main() sets the default before the settings are loaded. */
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

    if (zhonxSettings.beeper_enabled != 0)
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
            /* Keep a lower volume level, unmute at full volume. */
            if (zhonxSettings.beeper_enabled == 0)
                zhonxSettings.beeper_enabled = 1;
            break;
    }
    return HAL_BEEPER_E_SUCCESS;
}


/* Duty cycle per volume code (fw_sound.h): a 50 % square wave drives the
 * magnetic buzzer hardest; 12 % and 4 % lower its fundamental by about
 * 9 dB and 18 dB. */
static unsigned duty_permille(void)
{
    switch (zhonxSettings.beeper_enabled)
    {
        case 2: return 120;
        case 3: return 40;
        default: return 500;
    }
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
    if (freq > TIMER_FREQ / 20) freq = TIMER_FREQ / 20;
    unsigned long period = TIMER_FREQ / freq;
    if (period > 0x10000) period = 0x10000;
    TIM_SetAutoreload(BEEPER_TIMER, period - 1);
    TIM_SetCompare3(BEEPER_TIMER, period * duty_permille() / 1000);
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
