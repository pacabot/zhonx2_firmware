#include "fw_battery.h"
#include "fw_motion.h"
#include "hal/hal_os.h"
extern volatile unsigned short convertedValues[];
void fw_battery_poll(void)
{fw_battery_sample(convertedValues[0],hal_os_get_systicks(),fw_motion_busy());}
