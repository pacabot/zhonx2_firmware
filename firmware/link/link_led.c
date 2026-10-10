/*
 * link_led.c
 *
 * Light signal state machine, see link_led.h.
 */

#include <string.h>

#include "middleware/link/link_led.h"
#include "middleware/link/link_port.h"

/* Drives the light and returns how long the write took, in milliseconds.
 *
 * The I2C write is not instantaneous: the OLED "entire display on" command waits for any
 * frame transfer already in flight, measured at 28 ms on the robot. Scheduling the opposite
 * edge from the time before the write then truncated the pulse (first pulse measured at
 * 32 ms instead of 60 on the bench, short enough for a 30 i/s camera to miss it). The caller
 * adds this duration so every deadline counts from the instant the light actually changed. */
static uint32_t edge(link_led_t *led, uint8_t index, int on, int with_oled)
{
    uint32_t before, t_us;

    before = link_time_us();
    link_led_port_set(on, with_oled);
    t_us = link_time_us();
    led->on = (uint8_t)(on != 0);
    link_led_port_event(index, (uint8_t)(on != 0), t_us);
    return (t_us - before) / 1000U;
}

void link_led_init(link_led_t *led)
{
    memset(led, 0, sizeof *led);
    led->blip_period_ms = LINK_LED_BLIP_PERIOD_MS;
}

void link_led_start_sequence(link_led_t *led, uint32_t now_ms)
{
    led->sequence_active = 1;
    led->sequence_done = 0;
    led->sequence_index = 0;
    led->sequence_on = 0;
    led->sequence_next_ms = now_ms;     /* first edge at the next poll */
    if (led->blip_on)
    {
        led->blip_on = 0;
        led->on = 0;
    }
}

void link_led_blip(link_led_t *led, uint32_t now_ms)
{
    (void)now_ms;
    led->single_blip = 1;
}

/* Blip indexes stay in LINK_LED_BLIP_FIRST_INDEX..255 and wrap there, so they never
 * collide with the start-sequence indexes 0..3 (uint8_t in EvLedSync). */
static uint8_t blip_edge_index(uint32_t blip)
{
    return (uint8_t)(LINK_LED_BLIP_FIRST_INDEX + blip % (256U - LINK_LED_BLIP_FIRST_INDEX));
}

void link_led_set_blips(link_led_t *led, int enabled, uint32_t period_ms, uint32_t now_ms)
{
    if (period_ms < LINK_LED_BLIP_ON_MS * 2U)
        period_ms = LINK_LED_BLIP_ON_MS * 2U;
    led->blip_period_ms = period_ms;
    led->blips_enabled = (uint8_t)(enabled != 0);
    led->blip_next_ms = now_ms + period_ms;
    if (!enabled && led->blip_on)
    {
        edge(led, blip_edge_index(led->blip_index - 1U), 0, 0);
        led->blip_on = 0;
    }
}

void link_led_reset_index(link_led_t *led)
{
    led->blip_index = 0;
}

static void sequence_poll(link_led_t *led, uint32_t now_ms)
{
    uint32_t on_ms, off_ms;

    if ((int32_t)(now_ms - led->sequence_next_ms) < 0)
        return;
    if (led->sequence_index == LINK_LED_SEQUENCE_PULSES - 1U)
    {
        on_ms = LINK_LED_LONG_ON_MS;
        off_ms = LINK_LED_LONG_OFF_MS;
    }
    else
    {
        on_ms = LINK_LED_SHORT_ON_MS;
        off_ms = LINK_LED_SHORT_OFF_MS;
    }
    if (!led->sequence_on)
    {
        /* Scheduled from the actual edge, write time included: a slow write never
         * shortens the pulse the camera has to see. */
        led->sequence_next_ms = now_ms + on_ms + edge(led, led->sequence_index, 1, 1);
        led->sequence_on = 1;
        return;
    }
    led->sequence_next_ms = now_ms + off_ms + edge(led, led->sequence_index, 0, 1);
    led->sequence_on = 0;
    led->sequence_index++;
    if (led->sequence_index >= LINK_LED_SEQUENCE_PULSES)
    {
        /* The last off time runs before the sequence counts as finished. */
        led->sequence_active = 2;
    }
}

int link_led_poll(link_led_t *led, uint32_t now_ms)
{
    if (led->sequence_active == 1)
    {
        sequence_poll(led, now_ms);
        return 1;
    }
    if (led->sequence_active == 2)
    {
        if ((int32_t)(now_ms - led->sequence_next_ms) < 0)
            return 1;
        led->sequence_active = 0;
        led->sequence_done = 1;
        led->blip_next_ms = now_ms + led->blip_period_ms;
    }
    if (led->blip_on)
    {
        if ((int32_t)(now_ms - led->blip_off_ms) >= 0)
        {
            edge(led, blip_edge_index(led->blip_index - 1U), 0, 0);
            led->blip_on = 0;
        }
        return 1;
    }
    if (led->single_blip || (led->blips_enabled && (int32_t)(now_ms - led->blip_next_ms) >= 0))
    {
        led->single_blip = 0;
        led->blip_off_ms = now_ms + LINK_LED_BLIP_ON_MS + edge(led, blip_edge_index(led->blip_index), 1, 0);
        led->blip_index++;
        led->blip_on = 1;
        led->blip_next_ms = now_ms + led->blip_period_ms;
        return 1;
    }
    return led->blips_enabled;
}

int link_led_sequence_active(const link_led_t *led)
{
    return led->sequence_active != 0;
}

int link_led_is_on(const link_led_t *led)
{
    return led->on;
}
