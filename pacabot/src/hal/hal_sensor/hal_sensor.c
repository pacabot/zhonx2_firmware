/**
 * hal_sensor.c
 */

#include "config/config.h"
#include "config/basetypes.h"
#include "config/errors.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"

#include "hal/hal_sensor.h"
#include "hal/hal_os.h"

#include <string.h>

#define SENSORS_PORT        GPIOC
#define COLOR_SENSOR_PORT   GPIOB

/* GPIO definitions */
#define SENSOR_R10          SENSORS_PORT, SENSOR_R10_PIN   // Right 10cm IR sensor
#define SENSOR_R5           SENSORS_PORT, SENSOR_R5_PIN    // Right 5cm IR sensor
#define SENSOR_F10          SENSORS_PORT, SENSOR_F10_PIN   // Front 10cm IR sensor
#define SENSOR_F5           SENSORS_PORT, SENSOR_F5_PIN    // Front 5cm IR sensor
#define SENSOR_L10          SENSORS_PORT, SENSOR_L10_PIN   // Left 10cm IR sensor
#define SENSOR_L5           SENSORS_PORT, SENSOR_L5_PIN    // Left 5cm IR sensor

#define SENSOR_CR           GPIOB, SENSOR_CR_PIN    // Right floor sensor
#define SENSOR_CL           GPIOB, SENSOR_CL_PIN    // Left floor sensor

#define SENSOR_EN_5         SENSORS_PORT, SENSOR_EN_5_PIN  // Enable 5cm sensors
#define SENSOR_EN_10        SENSORS_PORT, SENSOR_EN_10_PIN // Enable 5cm sensors

/* Timer configuration */
#define SENSOR_TIMER    TIM6

/* TIM6 clock: APB1 at 42 MHz, timer multiplier x2. */
#define SENSOR_TIMER_CLOCK 84000000
/* Sensors Timer base frequency (in Hz) */
#define TIMER_FREQ 4000
/* Capture frequency (in Hz) */
#define CAPTURE_FREQ 200
/* Timer prescaler (PSC register) */
#define TIMER_PRESCALER (((SENSOR_TIMER_CLOCK) / (TIMER_FREQ)) - 1)
/* Timer period (ARR register) */
#define TIMER_PERIOD (((TIMER_FREQ) / (CAPTURE_FREQ)) - 1)

/* Macros */
#define ENABLE_5    (SENSORS_PORT->BSRR = SENSOR_EN_5_PIN)
#define DISABLE_5   (SENSORS_PORT->BSRR = (uint32_t)SENSOR_EN_5_PIN << 16)

#define CAPTURE_5   sensors_handle.state &= ~SENSORS_5CM; \
                    sensors_handle.state |= ((SENSORS_PORT->IDR) & SENSORS_5CM)

#define ENABLE_10   (SENSORS_PORT->BSRR = SENSOR_EN_10_PIN)
#define DISABLE_10  (SENSORS_PORT->BSRR = (uint32_t)SENSOR_EN_10_PIN << 16)
#define CAPTURE_10  sensors_handle.state &= ~SENSORS_10CM; \
                    sensors_handle.state |= ((SENSORS_PORT->IDR) & SENSORS_10CM)


/* Static functions */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void NVIC_Configuration(void);

typedef struct
{
    volatile unsigned char state;
}
sensor_handle;

sensor_handle sensors_handle;
static hal_sensor_snapshot scan;
static unsigned char confidence[6];
int hal_sensor_snapshot_read(hal_sensor_snapshot *out)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    *out=scan;
    __set_PRIMASK(mask);
    return out->sequence>=3;
}
static void publish_scan(void)
{
    scan.raw=sensors_handle.state;
    for (unsigned i=0;i<6;++i) {
        if (scan.raw & (1u<<i)) { if (confidence[i]<3) ++confidence[i]; }
        else if (confidence[i]) --confidence[i];
        if (confidence[i]==3) scan.filtered |= 1u<<i;
        else if (confidence[i]==0) scan.filtered &= ~(1u<<i);
    }
    scan.timestamp=hal_os_get_systicks();
    ++scan.sequence;
}


int hal_sensor_init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    memset(&sensors_handle, 0, sizeof(sensor_handle));
    sensors_handle.state=0x3f; scan.raw=scan.filtered=0x3f;
    scan.sequence=0; memset(confidence,3,sizeof confidence);

    /* System Clocks Configuration */
    RCC_Configuration();
    /* GPIO Configuration */
    GPIO_Configuration();
    /* Nested Vector Interrupt Controller configuration */
    NVIC_Configuration();

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = TIMER_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = TIMER_PRESCALER;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(SENSOR_TIMER, &TIM_TimeBaseStructure);

    /* Enable interrupts on Update event */
    TIM_ITConfig(SENSOR_TIMER, TIM_IT_Update, ENABLE);
    DISABLE_10; ENABLE_5;
    TIM_Cmd(SENSOR_TIMER, ENABLE);

    return HAL_SENSOR_E_SUCCESS;
}


int hal_sensor_terminate(void)
{
    memset(&sensors_handle, 0, sizeof(sensor_handle));
    sensors_handle.state=0x3f; scan.raw=scan.filtered=0x3f;
    scan.sequence=0; memset(confidence,3,sizeof confidence);

    return HAL_SENSOR_E_SUCCESS;
}


int hal_sensor_open(HAL_SENSOR_HANDLE *handle, void *params)
{
    UNUSED(params);

#ifdef CONFIG_DEBUG
    if (handle == null)
    {
        return HAL_SENSOR_E_BAD_HANDLE;
    }
#endif

    *handle = (HAL_SENSOR_HANDLE)&sensors_handle;

    return HAL_SENSOR_E_SUCCESS;
}


int hal_sensor_close(HAL_SENSOR_HANDLE handle)
{
#ifdef CONFIG_DEBUG
    if (handle == null)
    {
        return HAL_SENSOR_E_BAD_HANDLE;
    }
#endif // CONFIG_DEBUG

    return HAL_SENSOR_E_SUCCESS;
}


unsigned char hal_sensor_get_state(HAL_SENSOR_HANDLE handle)
{
    sensor_handle *h;

#ifdef CONFIG_DEBUG
    if (handle == null)
    {
        return HAL_SENSOR_E_BAD_HANDLE;
    }
#endif // CONFIG_DEBUG

    h = (sensor_handle *)handle;

    return h->state;
}


int hal_sensor_get_color(HAL_SENSOR_HANDLE handle)
{
    sensor_handle       *h;
    int                 duration = 0;
    unsigned char       state;
    GPIO_InitTypeDef    GPIO_InitStructure;

#ifdef CONFIG_DEBUG
    if (handle == null)
    {
        return HAL_SENSOR_E_BAD_HANDLE;
    }
#endif // CONFIG_DEBUG

    h = (sensor_handle *)handle;

    UNUSED(h);

    /* Charge the capacitors */
    GPIO_SetBits(SENSOR_CL);
    //GPIO_SetBits(SENSOR_CR);

    /* Wait for the capacitors to charge */
    HAL_Delay(2);

    GPIO_InitStructure.GPIO_Pin = SENSOR_CL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COLOR_SENSOR_PORT, &GPIO_InitStructure);

    /* Left floor sensor */
    do
    {
        state = GPIO_ReadInputDataBit(SENSOR_CL);
        duration++;
    }
    while (state > 0);

//    GPIO_InitStructure.GPIO_Pin = SENSOR_CR_PIN;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(COLOR_SENSOR_PORT, &GPIO_InitStructure);

//    /* Right floor sensor */
//    do
//    {
//        state = GPIO_ReadInputDataBit(SENSOR_CR);
//        duration++;
//    } while (state > 0);

    GPIO_InitStructure.GPIO_Pin   = SENSOR_CL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COLOR_SENSOR_PORT, &GPIO_InitStructure);

    return duration;
}


/**
 * @brief   Configures the Nested Vectored Interrupt Controller
 * @param   None
 * @retval  None
 */
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable SENSORS TIMER Interrupt handling */
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
    /* GPIO clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    /* Timer clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
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

    /* IR Sensors configuration */
    GPIO_InitStructure.GPIO_Pin = SENSOR_R10_PIN |
                                  SENSOR_R5_PIN  |
                                  SENSOR_F10_PIN |
                                  SENSOR_F5_PIN  |
                                  SENSOR_L10_PIN |
                                  SENSOR_L5_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SENSORS_PORT, &GPIO_InitStructure);

    /* Enable Pins configuration */
    GPIO_InitStructure.GPIO_Pin = SENSOR_EN_5_PIN |
                                  SENSOR_EN_10_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SENSORS_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SENSOR_CL_PIN |
                                  SENSOR_CR_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/**********************
 * Interrupt handlers *
 **********************/

void TIM6_DAC_IRQHandler(void)
{
    static bool capture5 = true;

    if ((SENSOR_TIMER->SR & TIM_FLAG_Update) == RESET)
    {
        return;
    }
    SENSOR_TIMER->SR = ~TIM_FLAG_Update;

    /* Get sensors values */
    if (capture5 == true)
    {
        CAPTURE_5;
        DISABLE_5;
        ENABLE_10;
    }
    else
    {
        CAPTURE_10;
        publish_scan();
        DISABLE_10;
        ENABLE_5;
    }

    capture5 = !capture5;
}
