/*
 * link_port.h (ZHONX II)
 *
 * What the pure Ariane Link modules (rings, LED sequencer, parameters)
 * need from their host: a short critical section and the robot clocks.
 * Same contract as the ZHONX III file of the same name; this one uses the
 * CMSIS core of the Standard Peripheral Library build. The firmware
 * implements the clocks in fw_link.c; the host tests (-DLINK_HOST) provide
 * their own.
 */

#ifndef LINK_PORT_H_
#define LINK_PORT_H_

#include <stdint.h>

#ifdef LINK_HOST
static inline uint32_t link_lock(void) { return 0; }
static inline void link_unlock(uint32_t state) { (void)state; }
#else
#include "stm32f4xx.h"
/* Masks interrupts: the telemetry producer runs in SysTick, the link in PendSV. */
static inline uint32_t link_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}
static inline void link_unlock(uint32_t primask)
{
    if (primask == 0U)
        __enable_irq();
}
#endif

/* Robot time: 32-bit counter at 1 MHz (SysTick milliseconds and sub-millisecond
 * count), wraps every 71 minutes. */
uint32_t link_time_us(void);
/* Millisecond tick (hal_os_get_systicks on the robot). */
uint32_t link_time_ms(void);

#endif /* LINK_PORT_H_ */
