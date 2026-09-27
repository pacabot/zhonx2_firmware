#ifndef FW_BATTERY_H
#define FW_BATTERY_H
#include <stdint.h>
typedef struct { uint32_t raw, pack_mv; } fw_battery_reference_t;
typedef struct {
    unsigned raw, pack_mv, percent;
    int sample_valid, calibrated, soc_valid, rested, lower_bound;
} fw_battery_status_t;
int fw_battery_reference_valid(const fw_battery_reference_t *r);
void fw_battery_set_reference(fw_battery_reference_t r);
fw_battery_reference_t fw_battery_reference(void);
void fw_battery_supply(unsigned millivolts);
void fw_battery_sample(unsigned raw,uint32_t now,int moving);
fw_battery_status_t fw_battery_status(void);
unsigned fw_battery_percent(unsigned pack_mv);
void fw_battery_poll(void);
#endif
