/**
 * hal_step_motor.c
 */

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

/* Declarations for this module */
#include "hal/hal_step_motor.h"

/* ST Lib declarations */
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"

#include <string.h>

/* TIM2/TIM3 clock: APB1 at 42 MHz, timer multiplier x2. */
# define MOTOR_TIMER_CLOCK 84000000
/* Timer base frequency (in Hz) */
// TODO: Analyser pourquoi une fr�quence de 64000 ne fonctionne pas correctement
# define TIMER_FREQ 2000000

/* Toggle frequency (in Hz) */
#define STEP_MOTOR_FREQ 50
/* Timer prescaler (PSC register) */
#define TIMER_PRESCALER (((MOTOR_TIMER_CLOCK) / (TIMER_FREQ)) - 1)
/* Timer period (ARR register) */
#define TIMER_PERIOD    (((TIMER_FREQ) / (STEP_MOTOR_FREQ)) - 1)

#define STEP_MOTOR1_TIM TIM2
#define STEP_MOTOR2_TIM TIM3

#define MAX_CHANNELS 2

#define STEP_1_PIN			GPIO_Pin_1
#define STEP_2_PIN			GPIO_Pin_7
#define STEP_1_PIN_SOURCE	GPIO_PinSource1
#define STEP_2_PIN_SOURCE	GPIO_PinSource7

#define DIR_1_PIN			GPIO_Pin_5
#define DIR_2_PIN			GPIO_Pin_6
#define SLEEP_1_PIN         GPIO_Pin_2
#define SLEEP_2_PIN         GPIO_Pin_3
#define MS1_PIN             GPIO_Pin_5
#define MS2_PIN             GPIO_Pin_6
#define MS3_PIN             GPIO_Pin_7

#define STEP_1				GPIOA, STEP_1_PIN
#define STEP_2				GPIOA, STEP_2_PIN
#define DIR_1				GPIOA, DIR_1_PIN
#define DIR_2				GPIOA, DIR_2_PIN
#define SLEEP_1             GPIOA, SLEEP_1_PIN
#define SLEEP_2             GPIOA, SLEEP_2_PIN
#define MS1                 GPIOB, MS1_PIN
#define MS2                 GPIOB, MS2_PIN
#define MS3                 GPIOB, MS3_PIN

#define MOTOR_ENABLE_PIN	GPIO_Pin_8
#define MOTOR_ENABLE		GPIOA, MOTOR_ENABLE_PIN

/* Macros */
#define IS_VALID_DIRECTION(DIR) (((DIR) == DIRECTION_FWD) || \
                                 ((DIR) == DIRECTION_BKW))

#define UPDATE_FREQ(STEPPER_MOTOR, freq) \
        STEPPER_MOTOR.timer->ARR = ((TIMER_FREQ / freq) - 1)

/* Static functions */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void NVIC_Configuration(void);

/* Types definitions */
typedef struct
{
    TIM_TypeDef    *timer;
    unsigned char  channel;
    volatile long  steps;
    unsigned char  direction;
    long           freq;
    volatile char  current_state;
    int            acceleration;
    double         curve_rad;
}
step_motor_handle;

static step_motor_handle step_motor[MAX_CHANNELS];
static volatile unsigned long pair_limit[2];
static volatile int pair_owned;


int hal_step_motor_init(void)
{
    memset(step_motor, 0, sizeof(step_motor_handle) * MAX_CHANNELS);

    /* System Clocks Configuration */
    RCC_Configuration();
    /* GPIO Configuration */
    GPIO_Configuration();
    /* Nested Vector Interrupt Controller configuration */
    NVIC_Configuration();

    hal_step_motor_disable();

    return HAL_STEP_MOTOR_E_SUCCESS;
}


int hal_step_motor_terminate(void)
{
    memset(step_motor, 0, sizeof(step_motor));

    return HAL_STEP_MOTOR_E_SUCCESS;
}


int hal_step_motor_open(HAL_STEP_MOTOR_HANDLE *handle, void *params)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    static unsigned char    step_motor_position = 0;

    UNUSED(params);

    if (step_motor_position > (MAX_CHANNELS - 1))
    {
        return HAL_STEP_MOTOR_E_ERROR;
    }

    TIM_OCStructInit(&TIM_OCInitStructure);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = TIMER_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = TIMER_PRESCALER;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    //TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    /* Output Compare configuration */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    //TIM_OCInitStructure.TIM_Pulse = TIMER_PERIOD;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    //TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    switch (step_motor_position)
    {
        case 0:
            step_motor[step_motor_position].timer = STEP_MOTOR1_TIM;
            step_motor[step_motor_position].channel = step_motor_position;
            TIM_TimeBaseInit(step_motor[step_motor_position].timer, &TIM_TimeBaseStructure);
            TIM_OC2Init(step_motor[step_motor_position].timer, &TIM_OCInitStructure);
            break;
        case 1:
            step_motor[step_motor_position].timer = STEP_MOTOR2_TIM;
            step_motor[step_motor_position].channel = step_motor_position;
            TIM_TimeBaseInit(step_motor[step_motor_position].timer, &TIM_TimeBaseStructure);
            TIM_OC2Init(step_motor[step_motor_position].timer, &TIM_OCInitStructure);
            break;

        default:
            return HAL_STEP_MOTOR_E_BAD_HANDLE;
    }

    step_motor[step_motor_position].direction = DIRECTION_FWD;
    *handle = (HAL_STEP_MOTOR_HANDLE)&step_motor[step_motor_position];
    TIM_ARRPreloadConfig(step_motor[step_motor_position].timer, ENABLE);

    /* Set default acceleration */
    step_motor[step_motor_position].acceleration = zhonxSettings.default_accel;

    /* Enable timer counter */
    TIM_Cmd(step_motor[step_motor_position].timer, ENABLE);
    /* Enable interrupts on Capture/Compare */
    TIM_ITConfig(step_motor[step_motor_position].timer, TIM_IT_CC2, ENABLE);

    step_motor_position++;

    return HAL_STEP_MOTOR_E_SUCCESS;
}


int hal_step_motor_close(HAL_STEP_MOTOR_HANDLE handle)
{
    UNUSED(handle);

    return HAL_STEP_MOTOR_E_SUCCESS;
}


int hal_step_motor_enable(void)
{
	GPIO_WriteBit(MOTOR_ENABLE, Bit_RESET);
	hal_step_motor_set_freq((void *)&step_motor[0], 0);
	hal_step_motor_set_freq((void *)&step_motor[1], 0);
	hal_step_motor_wakeup();

    return HAL_STEP_MOTOR_E_SUCCESS;
}


int hal_step_motor_disable(void)
{
    GPIO_WriteBit(MOTOR_ENABLE, Bit_SET);
    hal_step_motor_sleep();

    return HAL_STEP_MOTOR_E_SUCCESS;
}


void hal_step_motor_wakeup(void)
{
    GPIO_SetBits(SLEEP_1);
    GPIO_SetBits(SLEEP_2);
}


void hal_step_motor_sleep(void)
{
    GPIO_ResetBits(SLEEP_1);
    GPIO_ResetBits(SLEEP_2);
}


long hal_step_motor_get_freq(HAL_STEP_MOTOR_HANDLE handle)
{
    step_motor_handle *h;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return;
    }
#endif // CONFIG_DEBUG

    h = (step_motor_handle *)handle;

    return h->freq;
}


int hal_step_motor_set_freq(HAL_STEP_MOTOR_HANDLE handle, long freq)
{
    step_motor_handle *h;
    unsigned short timer_period = 0;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return;
    }
#endif // CONFIG_DEBUG

    h = (step_motor_handle *)handle;

    if (freq <= 0)
    {
        /* Disable the TIM Counter */
        h->timer->CR1 &= ~TIM_CR1_CEN;
        h->freq = 0;
    }

    /* Frequency is greater than the maximum frequency */
    else if (freq > TIMER_FREQ)
    {
        timer_period = 0xFFFF;
        h->freq = TIMER_FREQ;
    }

    /* Frequency is in the correct range */
    else
    {
        /* If the timer is disabled, enable it */
        if (h->freq == 0)
        {
            /* Enable the TIM Counter */
            h->timer->CR1 |= TIM_CR1_CEN;
        }
        timer_period = ((TIMER_FREQ / freq) - 1);
        h->freq = freq;
    }

    if (timer_period > 0)
    {
        /* Set the Autoreload Register value */
        h->timer->ARR = timer_period;
    }

    return HAL_STEP_MOTOR_E_SUCCESS;
}


int hal_step_motor_set_resolution(unsigned char resolution)
{
    switch (resolution)
    {
        case FULL_STEP:
            GPIO_ResetBits(MS1);
            GPIO_ResetBits(MS2);
            GPIO_ResetBits(MS3);
            break;
        case HALF_STEP:
            GPIO_SetBits(MS1);
            GPIO_ResetBits(MS2);
            GPIO_ResetBits(MS3);
            break;
        case QUARTER_STEP:
            GPIO_ResetBits(MS1);
            GPIO_SetBits(MS2);
            GPIO_ResetBits(MS3);
            break;
        case EIGHTH_STEP:
            GPIO_SetBits(MS1);
            GPIO_SetBits(MS2);
            GPIO_ResetBits(MS3);
            break;
        case SIXTEENTH_STEP:
            GPIO_SetBits(MS1);
            GPIO_SetBits(MS2);
            GPIO_SetBits(MS3);
            break;
        default:
            return HAL_STEP_MOTOR_E_ERROR;
    }
    return HAL_STEP_MOTOR_E_SUCCESS;
}


int hal_step_motor_set_direction(HAL_STEP_MOTOR_HANDLE handle, unsigned char direction)
{
    step_motor_handle *h;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return HAL_STEP_MOTOR_E_ERROR;
    }

    if (false == IS_VALID_DIRECTION(direction))
    {
        return HAL_STEP_MOTOR_E_ERROR;
    }
#endif // CONFIG_DEBUG

    h = (step_motor_handle *)handle;

    switch(h->channel)
    {
        case 0:
            GPIO_WriteBit(DIR_1, direction);
            break;
        case 1:
            GPIO_WriteBit(DIR_2, direction);
            break;

        default:
            return HAL_STEP_MOTOR_E_ERROR;
    }

    h->direction = direction;

    return HAL_STEP_MOTOR_E_SUCCESS;
}


unsigned short hal_step_motor_get_direction(HAL_STEP_MOTOR_HANDLE handle)
{
    step_motor_handle *h;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return;
    }
#endif // CONFIG_DEBUG

    h = (step_motor_handle *)handle;

    return h->direction;
}


long hal_step_motor_get_counter(HAL_STEP_MOTOR_HANDLE handle)
{
    step_motor_handle *h;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return;
    }
#endif // CONFIG_DEBUG

    h = (step_motor_handle *)handle;

	return h->steps;
}


int hal_step_motor_get_acceleration(HAL_STEP_MOTOR_HANDLE handle)
{
    step_motor_handle *h;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return;
    }
#endif // CONFIG_DEBUG

    h = (step_motor_handle *)handle;

    return h->acceleration;
}


int hal_step_motor_set_acceleration(HAL_STEP_MOTOR_HANDLE handle, int accel)
{
    step_motor_handle *h;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return HAL_STEP_MOTOR_E_BAD_HANDLE;
    }
#endif // CONFIG_DEBUG

    h = (step_motor_handle *)handle;

    h->acceleration = accel;

    return HAL_STEP_MOTOR_E_SUCCESS;
}


void hal_step_motor_accelerate(HAL_STEP_MOTOR_HANDLE handle)
{
    step_motor_handle *h;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return;
    }
#endif // CONFIG_DEBUG

    h = (step_motor_handle *)handle;
    if (h->freq == 0)
    {
        hal_step_motor_set_freq(handle, 50);
    }

    h->current_state = STATE_ACCEL;
}


void hal_step_motor_decelerate(HAL_STEP_MOTOR_HANDLE handle)
{
    step_motor_handle *h;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return;
    }
#endif // CONFIG_DEBUG

    h = (step_motor_handle *)handle;

    h->current_state = STATE_DECEL;
}


void hal_step_motor_maintain(HAL_STEP_MOTOR_HANDLE handle)
{
    step_motor_handle *h;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return;
    }
#endif // CONFIG_DEBUG

    h = (step_motor_handle *)handle;

    h->current_state = STATE_MAINTAIN;
}


void hal_step_motor_curve_rotate(HAL_STEP_MOTOR_HANDLE handle,
                                 double curve_rad, unsigned char state)
{
    step_motor_handle *h;

#ifdef CONFIG_DEBUG
    /* Return error if handle is not valid */
    if (handle == null)
    {
        return;
    }
#endif // CONFIG_DEBUG

    h = (step_motor_handle *)handle;

    h->curve_rad = curve_rad;
    h->current_state = state;
}


/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
    /* GPIO clocks enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
                           RCC_AHB1Periph_GPIOB, ENABLE);

    /* Timer clocks enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 |
    					   RCC_APB1Periph_TIM3, ENABLE);
}


/**
 * @brief  Configure the GPIO pins.
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, STEP_1_PIN_SOURCE, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, STEP_2_PIN_SOURCE, GPIO_AF_TIM3);

    /* GPIOA configuration for timers outputs */
    GPIO_InitStructure.GPIO_Pin   = STEP_1_PIN |	   // Step 1
    							    STEP_2_PIN;        // Step 2
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* GPIOA configuration for output pins */
    GPIO_InitStructure.GPIO_Pin   = DIR_1_PIN  |       // Dir 1
                                    DIR_2_PIN  |       // Dir 2
                                    MOTOR_ENABLE_PIN | // Enable
                                    SLEEP_1_PIN |
                                    SLEEP_2_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* GPIOB configuration for step precision control */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;
    GPIO_InitStructure.GPIO_Pin   = MS1_PIN |
                                    MS2_PIN |
                                    MS3_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_WriteBit(DIR_1, DIRECTION_FWD);
	GPIO_WriteBit(DIR_2, DIRECTION_FWD);
	GPIO_SetBits(SLEEP_1);
	GPIO_SetBits(SLEEP_2);
}


void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable TIMER 2 Interrupt handling */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable TIMER 3 Interrupt handling */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
}


/**********************
 * Interrupt handlers *
 **********************/

void TIM2_IRQHandler(void)
{
    if ((STEP_MOTOR1_TIM->SR & TIM_FLAG_CC2) == RESET)
    {
        return;
    }
    STEP_MOTOR1_TIM->SR = ~TIM_FLAG_CC2;

    step_motor[0].steps++;
    if (pair_owned) {
        if ((unsigned long)step_motor[0].steps >= pair_limit[0]) {
            step_motor[0].timer->CR1 &= ~TIM_CR1_CEN;
            step_motor[0].freq = 0;
        }
        return;
    }

    switch(step_motor[0].current_state)
    {
        case STATE_ACCEL:
            if (step_motor[0].freq < MAX_SPEED)
            {
                step_motor[0].freq += step_motor[0].acceleration;
            }
            break;

        case STATE_DECEL:
            // Avoid setting the frequency under a minimum value
            if (step_motor[0].freq > zhonxSettings.initial_speed)
            {
                step_motor[0].freq -= step_motor[0].acceleration;
            }
            break;

        case STATE_CURVE_ROTATE_R:
            step_motor[0].freq = step_motor[1].freq * (((2 * step_motor[0].curve_rad) - WHEELS_DISTANCE) /
                                                       ((2 * step_motor[0].curve_rad) + WHEELS_DISTANCE));
            break;

        default:
            return;
    }
    UPDATE_FREQ(step_motor[0], step_motor[0].freq);
    //hal_step_motor_set_freq((HAL_STEP_MOTOR_HANDLE)&step_motor[0], step_motor[0].freq);
}


void TIM3_IRQHandler(void)
{

    if ((STEP_MOTOR2_TIM->SR & TIM_FLAG_CC2) == RESET)
    {
        return;
    }
    STEP_MOTOR2_TIM->SR = ~TIM_FLAG_CC2;

    step_motor[1].steps++;
    if (pair_owned) {
        if ((unsigned long)step_motor[1].steps >= pair_limit[1]) {
            step_motor[1].timer->CR1 &= ~TIM_CR1_CEN;
            step_motor[1].freq = 0;
        }
        return;
    }

    switch(step_motor[1].current_state)
    {
        case STATE_ACCEL:
            if (step_motor[1].freq < MAX_SPEED)
            {
                step_motor[1].freq += step_motor[1].acceleration;
            }
            break;

        case STATE_DECEL:
            // Avoid setting the frequency under a minimum value
            if (step_motor[1].freq > zhonxSettings.initial_speed)
            {
                step_motor[1].freq -= step_motor[1].acceleration;
            }
            break;

        case STATE_CURVE_ROTATE_L:
            step_motor[1].freq = step_motor[0].freq * (((2 * step_motor[1].curve_rad) - WHEELS_DISTANCE) /
                                                       ((2 * step_motor[1].curve_rad) + WHEELS_DISTANCE));
            break;

        default:
            return;
    }
    UPDATE_FREQ(step_motor[1], step_motor[1].freq);
    //hal_step_motor_set_freq((HAL_STEP_MOTOR_HANDLE)&step_motor[1], step_motor[1].freq);
}

/* Finite moves count hardware toggle events; two events form a full STEP pulse.
 * The pulse ISR stops at the target even if foreground path planning is busy. */
void hal_step_motor_pair_stop(void)
{
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    for (int i=0;i<2;++i) {
        if (step_motor[i].timer) step_motor[i].timer->CR1 &= ~TIM_CR1_CEN;
        step_motor[i].freq=0;
    }
    pair_limit[0]=pair_limit[1]=0;
    pair_owned=1;
    __set_PRIMASK(mask);
}
void hal_step_motor_pair_start(long right, long left)
{
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    long distances[2]={right,left};
    pair_owned=1;
    for (int i=0;i<2;++i) {
        step_motor_handle *h=&step_motor[i];
        h->timer->CR1 &= ~TIM_CR1_CEN;
        h->steps=0; h->freq=0; h->current_state=STATE_MAINTAIN;
        pair_limit[i]=(unsigned long)(distances[i]<0 ? -distances[i] : distances[i]);
        pair_limit[i]=(pair_limit[i]+1u)&~1u;
        hal_step_motor_set_direction(h,distances[i]<0 ? DIRECTION_BKW : DIRECTION_FWD);
        h->timer->CNT=0; h->timer->SR=0;
    }
    __set_PRIMASK(mask);
}
unsigned long hal_step_motor_pair_remaining(unsigned int wheel)
{
    if (wheel>1) return 0;
    unsigned long count=(unsigned long)step_motor[wheel].steps, limit=pair_limit[wheel];
    return count<limit ? limit-count : 0;
}
void hal_step_motor_pair_rate(unsigned long right, unsigned long left)
{
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    unsigned long rates[2]={right,left};
    for (int i=0;i<2;++i) {
        if (!hal_step_motor_pair_remaining(i)) rates[i]=0;
        if (rates[i]>MAX_SPEED) rates[i]=MAX_SPEED;
        step_motor_handle *h=&step_motor[i];
        if (!rates[i]) { h->timer->CR1 &= ~TIM_CR1_CEN; h->freq=0; continue; }
        h->timer->ARR=TIMER_FREQ/rates[i]-1;
        if (!(h->timer->CR1 & TIM_CR1_CEN)) {
            h->timer->EGR=TIM_EGR_UG;
            h->timer->CNT=0; h->timer->SR=0;
            h->timer->CR1 |= TIM_CR1_CEN;
        }
        h->freq=rates[i];
    }
    __set_PRIMASK(mask);
}
unsigned long hal_step_motor_pair_count(unsigned int wheel)
{
    return wheel<2?(unsigned long)step_motor[wheel].steps:0;
}
void hal_step_motor_pair_extend(unsigned long pulses)
{
    uint32_t mask=__get_PRIMASK();__disable_irq();
    pulses=(pulses+1u)&~1u;
    if(pair_owned) {pair_limit[0]+=pulses;pair_limit[1]+=pulses;}
    __set_PRIMASK(mask);
}
void hal_step_motor_pair_release(void)
{
    hal_step_motor_pair_stop(); pair_owned=0;
}
