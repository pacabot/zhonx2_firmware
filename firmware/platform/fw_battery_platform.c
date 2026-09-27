#include "fw_battery.h"
#include "fw_motion.h"
#include "hal/hal_os.h"
extern volatile unsigned short convertedValues[];
void fw_battery_poll(void)
{
    /* STM32F405 DS8626 table 73: VREFINT measured at VDDA=3.3 V. */
    const unsigned factory=*(const unsigned short *)0x1fff7a2a;
    unsigned vref=convertedValues[1];
    fw_battery_supply(factory>=1000 && factory<=2000 && vref>=1000 && vref<=2500?
                      (3300u*factory+vref/2)/vref:0);
    fw_battery_sample(convertedValues[0],hal_os_get_systicks(),fw_motion_busy());
}
