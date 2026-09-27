#ifndef FW_STEP_CLOCK_H
#define FW_STEP_CLOCK_H
#include <stdint.h>
/* Fractional timer periods: preserve mean frequency despite integer ARR.
 * Division is confined to the 1 kHz command; pulse IRQ uses add/compare only. */
typedef struct {uint32_t rate,base,remainder,error;} fw_step_clock_t;
static inline void fw_step_clock_set(volatile fw_step_clock_t *c,uint32_t clock,uint32_t rate)
{
    if(c->rate==rate)return;
    c->rate=rate;c->base=rate?clock/rate:0;c->remainder=rate?clock%rate:0;
    c->error=rate?c->error%rate:0;
}
static inline uint32_t fw_step_clock_next(volatile fw_step_clock_t *c)
{
    uint32_t period=c->base,error=c->error+c->remainder;
    if(error>=c->rate) {error-=c->rate;++period;}
    c->error=error;return period;
}
#endif
