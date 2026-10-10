/*
 * link_led.h
 *
 * Light signal of Ariane Link (PROTOCOL.md section 5), pure C state machine
 * driven by link_led_poll() from the main loop:
 *  - start sequence: three pulses of 60 ms on / 140 ms off, then 500 ms on /
 *    200 ms off, OLED "entire display on" and the three expander LEDs
 *    together, indexes 0..3;
 *  - blips: 30 ms on every period during a motion, expander LEDs only,
 *    indexes 100, 101, ...
 * Each edge goes through the port: link_led_port_set() does the I2C write
 * from the main loop, then the robot time is read and EVENT LED_SYNC is
 * pushed with it (link_led_port_event()).
 */

#ifndef LINK_LED_H_
#define LINK_LED_H_

#include <stdint.h>

#define LINK_LED_SHORT_ON_MS      60U
#define LINK_LED_SHORT_OFF_MS     140U
#define LINK_LED_LONG_ON_MS       500U
#define LINK_LED_LONG_OFF_MS      200U
#define LINK_LED_SEQUENCE_PULSES  4U
#define LINK_LED_BLIP_ON_MS       30U
#define LINK_LED_BLIP_PERIOD_MS   10000U
#define LINK_LED_BLIP_FIRST_INDEX 100U

typedef struct
{
    /* start sequence */
    uint8_t sequence_active;
    uint8_t sequence_index;     /* pulse 0..3 */
    uint8_t sequence_on;
    uint32_t sequence_next_ms;  /* time of the next edge */
    uint8_t sequence_done;      /* set once a sequence completed, cleared by link_led_start_sequence */
    /* blips */
    uint8_t blips_enabled;
    uint32_t blip_period_ms;
    uint32_t blip_next_ms;
    uint8_t blip_on;
    uint32_t blip_off_ms;
    uint8_t blip_index;         /* next index minus LINK_LED_BLIP_FIRST_INDEX */
    uint8_t single_blip;        /* one blip requested (LED_SYNC pattern 1) */
    uint8_t on;                 /* any light on */
} link_led_t;

/* Port, implemented by link.c on the robot and by the host test. */
void link_led_port_set(int on, int with_oled);
void link_led_port_event(uint8_t index, uint8_t state, uint32_t t_us);

void link_led_init(link_led_t *led);
/* Starts the start sequence now; a running one restarts. */
void link_led_start_sequence(link_led_t *led, uint32_t now_ms);
/* One 30 ms blip now (unless the sequence runs: it is queued after it). */
void link_led_blip(link_led_t *led, uint32_t now_ms);
/* Periodic blips from now + period; disabling turns a running blip off. */
void link_led_set_blips(link_led_t *led, int enabled, uint32_t period_ms, uint32_t now_ms);
/* Resets the blip counter (indexes restart at 100). */
void link_led_reset_index(link_led_t *led);
/* Advances the state machine; returns 1 while something is scheduled. */
int  link_led_poll(link_led_t *led, uint32_t now_ms);
int  link_led_sequence_active(const link_led_t *led);
int  link_led_is_on(const link_led_t *led);

#endif /* LINK_LED_H_ */
