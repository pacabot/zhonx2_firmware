/*---------------------------------------------------------------------------
 *
 *      hal_os.c
 *       
 *---------------------------------------------------------------------------*/

#include "stm32f4xx_usart.h"

/* Common declarations */
#include "config/config.h"
#include "config/basetypes.h"
#include "config/errors.h"

/* Declarations for this module */
#include "hal/hal_os.h"

#include "hal/hal_serial.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ST Lib declarations */
#include "stm32f4xx_rcc.h"

#define DEBUG_LOG_SIZE  100


#include "misc.h"
#include "fw_motion.h"
#include "fw_layout.h"
extern void SystemInit(void);    /*!< Setup the microcontroller system(CMSIS) */


typedef struct
{
    HAL_SERIAL_HANDLE serial;
    volatile unsigned long system_ticks;
} hal_os_ctxt;

static hal_os_ctxt os_context;

/**
 * @brief Allocates the permanent resources used by the HAL OS implementation.
 *
 * This function is called once when the system starts up, before
 * any other call to the HAL interfaces. It must perform all actions
 * required to prepare the OS service for use. This includes the allocation
 * and/or initialization of all structures and variables, and the loading of
 * any required drivers.
 *
 * @return #HAL_OS_SUCCESS if the operation is successful,
 *         #HAL_OS_ERROR otherwise.
 */
int hal_os_init(void)
{
    int rv;

    /* Initialize the system ticks */
    os_context.system_ticks = 0;

    /* Initialize system clocks */
    SystemInit();
    RCC_HSEConfig(RCC_HSE_ON);
    /* Wait until HSE clock is stabilized */
    while(RCC_WaitForHSEStartUp() == ERROR);

    /* Initialize System tick interrupts to 1tick/ms */
    SCB->VTOR = FW_APP_BASE;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_SetPriority(SysTick_IRQn,4);

    /* Initialize serial module */
    rv = hal_serial_init();
    if (rv != HAL_SERIAL_E_SUCCESS)
    {
        return rv;
    }

    /* Open serial module */
    rv = hal_serial_open(&os_context.serial, USART1, 115200);
    if (rv != HAL_SERIAL_E_SUCCESS)
    {
        return rv;
    }

    return HAL_OS_SUCCESS;
}

/**
 * Releases the resources permanently allocated by the HAL OS implementation.
 *
 * This function is called once when the system shuts down.
 * It must perform all operations required to cleanup after the interface use.
 * This includes the freeing of any allocated memory, the reinitialization of
 * any static memory (if needed), and the unloading of any drivers.
 *
 * @return #HAL_OS_SUCCESS if the operation is successful,
 *         #HAL_OS_ERROR otherwise.
 */
int hal_os_terminate(void)
{
    return HAL_OS_SUCCESS;
}

/*
 * This function returns the current system ticks value.
 *
 */
unsigned long hal_os_get_systicks(void)
{
    return os_context.system_ticks;
}


/*
 * This function induce a delay as specified in parameter.
 *
 * [in] delay in milliseconds.
 *
 * @return #HAL_OS_SUCCESS if the operation is successful
 *
 */
int HAL_Delay(unsigned long delay_ms)
{
    unsigned long start = os_context.system_ticks;
    unsigned long end = start + delay_ms;

    // Wait until system ticks have reached the desired value
    while(os_context.system_ticks != end);

    return HAL_OS_SUCCESS;
}

/*static int _vprintf(void (*putc)(char c, void **param),
                    void **param, const char *fmt, va_list ap);*/


// Default wrapper function used by _printf
//static void hal_os_write_char(char ch, void **param );

/**
 * @brief Prints a string on the trace output.
 *
 * The function prints a string on the trace output.
 *
 * @param   fmt         Format control.
 *
 * @param   parameters  Optional arguments.
 *
 * @return #HAL_OS_SUCCESS if the operation is successful,
 *         #HAL_OS_ERROR otherwise.
 */
int hal_os_diag(const char *fmt, ...)
{
    char    buffer[DEBUG_LOG_SIZE];
    va_list argptr;
    //char    *b = buffer;

    va_start(argptr, fmt);
    (void)vsnprintf(buffer, DEBUG_LOG_SIZE - 1, fmt, argptr);
    va_end(argptr);

    strcat(buffer,"");

    hal_serial_write(os_context.serial, buffer, strlen(buffer));

    return HAL_OS_SUCCESS;
}


/* System tick handler */
void SysTick_Handler(void)
{
    os_context.system_ticks++;
    fw_motion_tick(os_context.system_ticks);
}
