/*
 * Host checks of the Ariane Link core, without HAL:
 *  - telemetry ring: retention until ACK, decimation of the oldest records
 *    when full, RESEND from the ring, FLUSH, batch assembly of ten records;
 *  - events ring: never decimated, drops counted, ACK and RESEND;
 *  - reply FIFO;
 *  - LED sequence and blips on a simulated clock, with the edge order and
 *    the timestamps the application will see;
 *  - parameter clamping, rounding, read-only and unknown ids;
 *  - routing of the bytes received outside frames into the console line
 *    buffer with the legacy CR / LF / backspace semantics.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "middleware/link/ariane_link_schema.h"
#include "middleware/link/link_frame.h"
#include "middleware/link/link_led.h"
#include "middleware/link/link_line.h"
#include "middleware/link/link_params.h"
#include "middleware/link/link_port.h"
#include "middleware/link/link_ring.h"

/* ---- simulated clock ------------------------------------------------------ */
static uint32_t now_ms;
uint32_t link_time_ms(void) { return now_ms; }
uint32_t link_time_us(void) { return now_ms * 1000U + 7U; }

/* ---- telemetry ring ------------------------------------------------------- */
static link_telemetry_record_t slots[64];
static link_tring_t tring;

static link_telemetry_record_t record(uint32_t n)
{
    link_telemetry_record_t r;
    memset(&r, 0, sizeof r);
    r.t_us = n * 10000U;
    r.enc_left_um = (int32_t)n;
    return r;
}

static void tring_retention_and_ack(void)
{
    link_telemetry_record_t out[LINK_BATCH_RECORDS];
    uint32_t first;

    link_tring_init(&tring, slots, 64);
    assert(link_tring_pending(&tring) == 0 && link_tring_last_seq(&tring) == 0);
    for (uint32_t i = 0; i < 25; i++)
        link_tring_push(&tring, &(link_telemetry_record_t){ .t_us = i });
    assert(link_tring_pending(&tring) == 25 && link_tring_last_seq(&tring) == 25);

    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 10 && first == 1);
    assert(out[0].t_us == 0 && out[9].t_us == 9);
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 10 && first == 11);
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 5 && first == 21);
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 0);
    /* Sent but not acknowledged: still retained. */
    assert(tring.count == 25 && link_tring_pending(&tring) == 0);

    link_tring_ack(&tring, 0);              /* acknowledges nothing */
    assert(tring.count == 25);
    link_tring_ack(&tring, 10);
    assert(tring.count == 15 && tring.base_seq == 11);
    link_tring_ack(&tring, 5);              /* already forgotten */
    assert(tring.count == 15 && tring.base_seq == 11);
    link_tring_push(&tring, &(link_telemetry_record_t){ .t_us = 25 });
    link_tring_ack(&tring, 1000);           /* stale or absurd: only what was sent goes */
    assert(tring.count == 1 && tring.base_seq == 26 && link_tring_pending(&tring) == 1);
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 1 && first == 26 && out[0].t_us == 25);
    puts("tring: batches of 10 in sequence, records retained until ACK, absurd ACK clamped");
}

static void tring_decimation(void)
{
    link_telemetry_record_t out[LINK_BATCH_RECORDS];
    uint32_t first;

    link_tring_init(&tring, slots, 16);
    for (uint32_t i = 1; i <= 20; i++)
        link_tring_push(&tring, &(link_telemetry_record_t){ .t_us = i });
    assert(tring.count == 16 && tring.base_seq == 5);
    assert(tring.dropped_total == 4 && tring.dropped_unreported == 4);
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 10 && first == 5 && out[0].t_us == 5);
    /* Sent records are decimated too when nothing is acknowledged. */
    for (uint32_t i = 21; i <= 24; i++)
        link_tring_push(&tring, &(link_telemetry_record_t){ .t_us = i });
    assert(tring.base_seq == 9 && tring.sent == 6 && tring.dropped_total == 8);
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 10 && first == 15);
    /* The newest record is never lost by decimation. */
    link_tring_push(&tring, &(link_telemetry_record_t){ .t_us = 25 });
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 1 && out[0].t_us == 25);
    puts("tring: full ring decimates the oldest records, drops counted, newest kept");
}

static void tring_resend_and_flush(void)
{
    link_telemetry_record_t out[LINK_BATCH_RECORDS];
    uint32_t first;

    link_tring_init(&tring, slots, 64);
    for (uint32_t i = 1; i <= 30; i++)
        link_tring_push(&tring, &(link_telemetry_record_t){ .t_us = i });
    for (int k = 0; k < 3; k++)
        assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 10);
    link_tring_ack(&tring, 12);
    /* Range partly forgotten, partly retained. */
    assert(link_tring_resend(&tring, 10, 15) == 3);
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 3 && first == 13 && out[2].t_us == 15);
    /* Fully forgotten range: nothing. */
    assert(link_tring_resend(&tring, 1, 12) == 0);
    /* Range beyond what was sent: nothing to repeat, those go out in order. */
    for (uint32_t i = 31; i <= 35; i++)
        link_tring_push(&tring, &(link_telemetry_record_t){ .t_us = i });
    assert(link_tring_resend(&tring, 33, 34) == 0);
    /* Resent range first, then the pending records. */
    assert(link_tring_resend(&tring, 20, 40) == 11);
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 10 && first == 20);
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 1 && first == 30);
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 5 && first == 31 && out[4].t_us == 35);
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 0);
    /* FLUSH: everything retained goes out again. */
    link_tring_flush(&tring);
    assert(link_tring_pending(&tring) == 23);
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 10 && first == 13);
    /* A resend range shrinks when the ring decimates through it. */
    link_tring_init(&tring, slots, 16);
    for (uint32_t i = 1; i <= 16; i++)
        link_tring_push(&tring, &(link_telemetry_record_t){ .t_us = i });
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 10);
    assert(link_tring_resend(&tring, 1, 4) == 4);
    link_tring_push(&tring, &(link_telemetry_record_t){ .t_us = 17 });
    link_tring_push(&tring, &(link_telemetry_record_t){ .t_us = 18 });
    assert(tring.resend_from == 3 && tring.resend_left == 2);
    assert(link_tring_take(&tring, out, LINK_BATCH_RECORDS, &first) == 2 && first == 3);
    puts("tring: RESEND serves what is still in the ring, FLUSH repeats everything unacknowledged");
}

/* ---- events ring ---------------------------------------------------------- */
static link_event_t eslots[8];
static link_ering_t ering;

static void ering_checks(void)
{
    link_event_t e;
    uint8_t args[20];

    for (unsigned i = 0; i < sizeof args; i++)
        args[i] = (uint8_t)i;
    link_ering_init(&ering, eslots, 8);
    for (uint32_t i = 1; i <= 8; i++)
        assert(link_ering_push(&ering, LINK_EVENT_CELL, i * 100, args, 4) == i);
    /* Full: never decimated, the new event is dropped and counted. */
    assert(link_ering_push(&ering, LINK_EVENT_BATTERY, 900, args, 2) == 0);
    assert(ering.dropped_total == 1 && ering.dropped_unreported == 1 && ering.count == 8);
    assert(link_ering_take(&ering, &e) && e.seq == 1 && e.t_us == 100 && e.len == 4 && e.args[3] == 3);
    for (int k = 0; k < 7; k++)
        assert(link_ering_take(&ering, &e));
    assert(!link_ering_take(&ering, &e) && ering.count == 8);
    link_ering_ack(&ering, 6);
    assert(ering.count == 2 && ering.base_seq == 7);
    /* Long arguments are truncated to the frame limit. */
    assert(link_ering_push(&ering, 0x7F, 0, args, 20) == 9);
    assert(link_ering_take(&ering, &e) && e.len == LINK_EVENT_ARGS_MAX && e.args[15] == 15);
    assert(link_ering_resend(&ering, 8, 9) == 2);
    assert(link_ering_take(&ering, &e) && e.seq == 8);
    assert(link_ering_take(&ering, &e) && e.seq == 9);
    assert(link_ering_resend(&ering, 20, 30) == 0);
    link_ering_flush(&ering);
    assert(link_ering_pending(&ering) == 3);
    puts("ering: 64-style ring never decimated, drops counted, ACK, RESEND, FLUSH");
}

/* ---- fifo ----------------------------------------------------------------- */
static void fifo_checks(void)
{
    uint8_t data[8], out[8];
    link_fifo_t f;

    link_fifo_init(&f, data, sizeof data);
    assert(link_fifo_put(&f, (const uint8_t *)"abcde", 5) && link_fifo_free(&f) == 3);
    assert(!link_fifo_put(&f, (const uint8_t *)"wxyz", 4));
    assert(link_fifo_peek(&f, out, 2) == 2 && out[0] == 'a');
    link_fifo_drop(&f, 2);
    assert(link_fifo_put(&f, (const uint8_t *)"fghij", 5) && f.count == 8);
    assert(link_fifo_peek(&f, out, 8) == 8 && memcmp(out, "cdefghij", 8) == 0);
    link_fifo_drop(&f, 100);
    assert(f.count == 0);
    puts("fifo: wrap-around, refusal when full");
}

/* ---- frame builders ------------------------------------------------------- */
static void builders(void)
{
    uint8_t frame[LINK_FRAME_OVERHEAD + LINK_FRAME_MAX_PAYLOAD];
    link_parser_t parser;
    link_frame_t parsed;
    link_telemetry_batch_t header;
    link_event_header_t eh;
    size_t n;
    int got = 0;

    link_tring_init(&tring, slots, 64);
    for (uint32_t i = 1; i <= 12; i++)
    {
        link_telemetry_record_t r = record(i);
        link_tring_push(&tring, &r);
    }
    n = link_build_telemetry(&tring, frame, sizeof frame, 42);
    assert(n == LINK_FRAME_OVERHEAD + LINK_TELEMETRY_BATCH_SIZE + 10 * LINK_TELEMETRY_RECORD_SIZE);
    link_parser_init(&parser);
    for (size_t i = 0; i < n; i++)
        if (link_parser_feed(&parser, frame[i], &parsed) == LINK_FEED_FRAME)
            got++;
    assert(got == 1 && parsed.type == LINK_FRAME_TYPE_TELEMETRY && parsed.seq == 42);
    memcpy(&header, parsed.payload, LINK_TELEMETRY_BATCH_SIZE);
    assert(header.first_seq == 1 && header.count == 10 && header.record_size == LINK_TELEMETRY_RECORD_SIZE);
    for (uint32_t i = 0; i < 10; i++)
    {
        link_telemetry_record_t r = record(i + 1), got_r;
        memcpy(&got_r, parsed.payload + LINK_TELEMETRY_BATCH_SIZE + i * LINK_TELEMETRY_RECORD_SIZE, sizeof got_r);
        assert(memcmp(&r, &got_r, sizeof r) == 0);
    }
    n = link_build_telemetry(&tring, frame, sizeof frame, 43);
    assert(n == LINK_FRAME_OVERHEAD + LINK_TELEMETRY_BATCH_SIZE + 2 * LINK_TELEMETRY_RECORD_SIZE);
    assert(link_build_telemetry(&tring, frame, sizeof frame, 44) == 0);
    assert(link_build_telemetry(&tring, frame, 16, 44) == 0);

    link_ering_init(&ering, eslots, 8);
    link_ering_push(&ering, LINK_EVENT_LED_SYNC, 123456, &(link_ev_led_sync_t){ 3, 1 }, LINK_EV_LED_SYNC_SIZE);
    n = link_build_event(&ering, frame, sizeof frame, 7);
    assert(n == LINK_FRAME_OVERHEAD + LINK_EVENT_HEADER_SIZE + LINK_EV_LED_SYNC_SIZE);
    got = 0;
    for (size_t i = 0; i < n; i++)
        if (link_parser_feed(&parser, frame[i], &parsed) == LINK_FEED_FRAME)
            got++;
    assert(got == 1 && parsed.type == LINK_FRAME_TYPE_EVENT);
    memcpy(&eh, parsed.payload, LINK_EVENT_HEADER_SIZE);
    assert(eh.seq == 1 && eh.t_us == 123456 && eh.event == LINK_EVENT_LED_SYNC && eh.len == 2);
    assert(parsed.payload[LINK_EVENT_HEADER_SIZE] == 3 && parsed.payload[LINK_EVENT_HEADER_SIZE + 1] == 1);
    assert(link_build_event(&ering, frame, sizeof frame, 8) == 0);
    puts("builders: TELEMETRY batch of 10 then 2, EVENT frame, both parsed back identically");
}

/* ---- LED sequencer -------------------------------------------------------- */
typedef struct { int on, with_oled; uint8_t index, state; uint32_t t_us; } led_edge_t;
static led_edge_t edges[64];
static int edge_count;
static int hw_on;

/* Écriture lente simulée : elle consomme du temps réel, donc les deux horloges avancent.
 * Sur le robot seule la première écriture attend la fin du transfert de trame OLED en cours
 * (28 ms mesurés) ; les suivantes sont immédiates. C'est cette asymétrie qui rognait la
 * première impulsion. */
static uint32_t write_cost_ms;
static int write_slow_writes;

void link_led_port_set(int on, int with_oled)
{
    hw_on = on;
    edges[edge_count].on = on;
    edges[edge_count].with_oled = with_oled;
    if (write_slow_writes > 0)
    {
        write_slow_writes--;
        now_ms += write_cost_ms;
    }
}

void link_led_port_event(uint8_t index, uint8_t state, uint32_t t_us)
{
    assert(edge_count < 64);
    edges[edge_count].index = index;
    edges[edge_count].state = state;
    edges[edge_count].t_us = t_us;
    edge_count++;
}

static void run_ms(link_led_t *led, uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
    {
        now_ms++;
        link_led_poll(led, now_ms);
    }
}

static void led_sequence(void)
{
    static const uint32_t on_ms[4] = { 60, 60, 60, 500 };
    link_led_t led;

    link_led_init(&led);
    now_ms = 1000;
    edge_count = 0;
    assert(link_led_poll(&led, now_ms) == 0 && !link_led_is_on(&led));
    link_led_start_sequence(&led, now_ms);
    assert(link_led_sequence_active(&led));
    run_ms(&led, 1);
    assert(edge_count == 1 && edges[0].state == 1 && edges[0].index == 0 && edges[0].with_oled == 1);
    assert(link_led_is_on(&led) && hw_on);
    run_ms(&led, 2000);
    assert(edge_count == 8);
    for (int i = 0; i < 4; i++)
    {
        assert(edges[2 * i].index == i && edges[2 * i].state == 1 && edges[2 * i].with_oled);
        assert(edges[2 * i + 1].index == i && edges[2 * i + 1].state == 0 && edges[2 * i + 1].with_oled);
        assert(edges[2 * i + 1].t_us - edges[2 * i].t_us == on_ms[i] * 1000U);
        if (i < 3)
            assert(edges[2 * i + 2].t_us - edges[2 * i + 1].t_us == LINK_LED_SHORT_OFF_MS * 1000U);
    }
    /* The timestamp is the one read right after the hardware write. */
    assert(edges[0].t_us == 1001U * 1000U + 7U);
    assert(!link_led_sequence_active(&led) && led.sequence_done && !link_led_is_on(&led));
    /* Sequence complete 200 ms after the last off edge, not before. */
    link_led_init(&led);
    edge_count = 0;
    link_led_start_sequence(&led, now_ms);
    run_ms(&led, 1 + 3 * 200 + 500);
    assert(edge_count == 8 && link_led_sequence_active(&led));
    run_ms(&led, LINK_LED_LONG_OFF_MS - 1);
    assert(link_led_sequence_active(&led));
    run_ms(&led, 1);
    assert(!link_led_sequence_active(&led));
    /* A slow first write (the OLED waits for a frame transfer in flight, 28 ms on the robot)
     * must not eat into the pulse: the bench measured 32 ms of light instead of 60, short
     * enough for a 30 frame/s camera to miss the first pulse entirely. */
    link_led_init(&led);
    edge_count = 0;
    write_cost_ms = 28U;
    write_slow_writes = 1;          /* only the first write waits for the OLED frame */
    link_led_start_sequence(&led, now_ms);
    run_ms(&led, 2000);
    write_cost_ms = 0;
    write_slow_writes = 0;
    assert(edge_count == 8);
    for (int i = 0; i < 4; i++)
    {
        /* every pulse lasts its full duration, the slow one included */
        assert(edges[2 * i + 1].t_us - edges[2 * i].t_us == on_ms[i] * 1000U);
        if (i < 3)
            assert(edges[2 * i + 2].t_us - edges[2 * i + 1].t_us == LINK_LED_SHORT_OFF_MS * 1000U);
    }
    puts("led: start sequence 3 x (60 on / 140 off) + 500 on / 200 off, OLED and LEDs, indexes 0..3");
    puts("led: a slow I2C write delays the edge without shortening the pulse");
}

static void led_blips(void)
{
    link_led_t led;

    link_led_init(&led);
    now_ms = 5000;
    edge_count = 0;
    link_led_reset_index(&led);
    link_led_set_blips(&led, 1, LINK_LED_BLIP_PERIOD_MS, now_ms);
    run_ms(&led, LINK_LED_BLIP_PERIOD_MS - 1);
    assert(edge_count == 0);
    run_ms(&led, 1);
    assert(edge_count == 1 && edges[0].index == 100 && edges[0].state == 1 && edges[0].with_oled == 0);
    run_ms(&led, LINK_LED_BLIP_ON_MS);
    assert(edge_count == 2 && edges[1].index == 100 && edges[1].state == 0);
    assert(edges[1].t_us - edges[0].t_us == LINK_LED_BLIP_ON_MS * 1000U);
    run_ms(&led, LINK_LED_BLIP_PERIOD_MS);
    assert(edge_count == 4 && edges[2].index == 101 && edges[3].index == 101);
    assert(edges[2].t_us - edges[0].t_us == LINK_LED_BLIP_PERIOD_MS * 1000U);
    /* Disabling during a blip turns it off at once. */
    run_ms(&led, LINK_LED_BLIP_PERIOD_MS - 20);
    assert(edge_count == 5 && link_led_is_on(&led));
    link_led_set_blips(&led, 0, LINK_LED_BLIP_PERIOD_MS, now_ms);
    assert(edge_count == 6 && edges[5].index == 102 && edges[5].state == 0 && !link_led_is_on(&led));
    run_ms(&led, 3 * LINK_LED_BLIP_PERIOD_MS);
    assert(edge_count == 6);
    /* Single blip (LED_SYNC pattern 1) and a blip queued behind the sequence. */
    link_led_blip(&led, now_ms);
    run_ms(&led, 1);
    assert(edge_count == 7 && edges[6].index == 103 && edges[6].state == 1);
    run_ms(&led, LINK_LED_BLIP_ON_MS);
    assert(edge_count == 8 && edges[7].state == 0);
    link_led_start_sequence(&led, now_ms);
    link_led_blip(&led, now_ms);
    run_ms(&led, 1 + 3 * 200 + 500 + 199);
    assert(edge_count == 16 && edges[15].index == 3 && link_led_sequence_active(&led));
    run_ms(&led, 1);
    assert(edge_count == 17 && edges[16].index == 104 && edges[16].state == 1);
    puts("led: blips 30 ms every 10 s, LEDs only, indexes from 100, single blip, queued behind a sequence");
}

/* ---- parameters ----------------------------------------------------------- */
static float p_speed = 500.0f, p_gain = 1.0f;
static int p_flag = 1;
static float get_speed(void) { return p_speed; }
static void set_speed(float v) { p_speed = v; }
static float get_gain(void) { return p_gain; }
static void set_gain(float v) { p_gain = v; }
static float get_flag(void) { return (float)p_flag; }
static void set_flag(float v) { p_flag = v != 0.0f; }
static float get_ro(void) { return 42.0f; }

static const link_param_t test_table[] = {
    { 1, "speed_mm_s", LINK_PARAM_TYPE_I32, 0, 0.0f, 4500.0f, get_speed, set_speed },
    { 2, "gyro_gain", LINK_PARAM_TYPE_F32, 0, 0.9f, 1.1f, get_gain, set_gain },
    { 3, "return_home", LINK_PARAM_TYPE_BOOL, 0, 0.0f, 1.0f, get_flag, set_flag },
    { 9, "a_very_long_parameter_name", LINK_PARAM_TYPE_I32, LINK_PARAM_FLAGS_READ_ONLY, 0.0f, 100.0f, get_ro, NULL },
};

static void params(void)
{
    link_params_t p;
    link_param_info_t info;
    link_param_value_t value;

    link_params_init(&p, test_table, 4);
    assert(link_params_count(&p) == 4);
    assert(link_params_info(&p, 3, &info) && info.count == 4 && info.index == 3 && info.id == 9);
    assert(strlen(info.name) == 15 && memcmp(info.name, "a_very_long_par", 15) == 0);
    assert(info.value == 42.0f && info.flags == LINK_PARAM_FLAGS_READ_ONLY);
    assert(!link_params_info(&p, 4, &info));

    link_params_get(&p, 2, &value);
    assert(value.status == LINK_CMD_STATUS_DONE && value.type == LINK_PARAM_TYPE_F32 && value.value == 1.0f);
    link_params_get(&p, 5, &value);
    assert(value.status == LINK_CMD_STATUS_INVALID);

    link_params_set(&p, 1, 9999.0f, &value);
    assert(value.status == LINK_CMD_STATUS_DONE && value.value == 4500.0f && p_speed == 4500.0f);
    link_params_set(&p, 1, -3.0f, &value);
    assert(value.value == 0.0f);
    link_params_set(&p, 1, 123.6f, &value);
    assert(value.value == 124.0f && p_speed == 124.0f);
    link_params_set(&p, 2, 1.05f, &value);
    assert(value.status == LINK_CMD_STATUS_DONE && fabsf(p_gain - 1.05f) < 1e-6f);
    link_params_set(&p, 2, 5.0f, &value);
    assert(fabsf(p_gain - 1.1f) < 1e-6f);
    link_params_set(&p, 2, NAN, &value);
    assert(value.status == LINK_CMD_STATUS_REFUSED && fabsf(p_gain - 1.1f) < 1e-6f);
    link_params_set(&p, 3, 0.7f, &value);
    assert(value.value == 1.0f && p_flag == 1);
    link_params_set(&p, 3, 0.2f, &value);
    assert(value.value == 0.0f && p_flag == 0);
    link_params_set(&p, 9, 1.0f, &value);
    assert(value.status == LINK_CMD_STATUS_REFUSED && value.value == 42.0f);
    link_params_set(&p, 77, 1.0f, &value);
    assert(value.status == LINK_CMD_STATUS_INVALID);
    puts("params: clamping, integer rounding, booleans, read-only refused, unknown id invalid");
}

/* ---- stray bytes into the console line ------------------------------------ */
static void feed_text(link_parser_t *parser, link_line_t *line, const char *s, int *lines, int *frames)
{
    link_frame_t frame;
    for (; *s; s++)
    {
        int r = link_parser_feed(parser, (uint8_t)*s, &frame);
        if (r == LINK_FEED_FRAME)
            (*frames)++;
        else if (r == LINK_FEED_STRAY && link_line_feed(line, (uint8_t)*s) > 0)
            (*lines)++;
    }
}

static void console_routing(void)
{
    char buffer[16];
    link_line_t line;
    link_parser_t parser;
    link_frame_t frame;
    uint8_t ping[LINK_FRAME_OVERHEAD + LINK_PING_SIZE];
    int lines = 0, frames = 0;
    size_t n;

    link_line_init(&line, buffer, sizeof buffer);
    link_parser_init(&parser);
    feed_text(&parser, &line, "help\r", &lines, &frames);
    assert(lines == 1 && strcmp(buffer, "help\r") == 0);
    /* A frame in the middle of a line is taken out of it. */
    n = link_frame_encode(ping, sizeof ping, LINK_FRAME_TYPE_PING, 1, &(link_ping_t){ 99 }, LINK_PING_SIZE);
    feed_text(&parser, &line, "he", &lines, &frames);
    for (size_t i = 0; i < n; i++)
    {
        int r = link_parser_feed(&parser, ping[i], &frame);
        assert(r != LINK_FEED_STRAY);
        if (r == LINK_FEED_FRAME)
            frames++;
    }
    assert(frames == 1 && frame.type == LINK_FRAME_TYPE_PING);
    feed_text(&parser, &line, "lp\r", &lines, &frames);
    assert(lines == 2 && strcmp(buffer, "help\r") == 0);
    /* Backspace deletes, line feed resets, the CR length includes the CR. */
    feed_text(&parser, &line, "nax\bv up\r", &lines, &frames);
    assert(lines == 3 && strcmp(buffer, "nav up\r") == 0);
    feed_text(&parser, &line, "garbage\nstatus", &lines, &frames);
    assert(link_line_feed(&line, '\r') == 7 && strcmp(buffer, "status\r") == 0);
    /* Overflow keeps room for the CR and the terminator. */
    feed_text(&parser, &line, "0123456789ABCDEFGHIJ\r", &lines, &frames);
    assert(strlen(buffer) == 15 && buffer[14] == '\r' && memcmp(buffer, "0123456789ABCD\r", 15) == 0);
    /* A frame with a corrupted CRC is dropped; its bytes never reach the console. */
    ping[5] ^= 0x01;
    lines = 0;
    for (size_t i = 0; i < n; i++)
        if (link_parser_feed(&parser, ping[i], &frame) == LINK_FEED_STRAY)
            lines++;
    assert(lines == 0 && parser.crc_errors == 1);
    feed_text(&parser, &line, "AT-AB ConnectionDown\r", &lines, &frames);
    assert(strncmp(buffer, "AT-AB Connecti", 14) == 0);
    puts("console: stray bytes rebuild the legacy line (CR ends, LF resets, BS deletes), frames removed");
}

int main(void)
{
    tring_retention_and_ack();
    tring_decimation();
    tring_resend_and_flush();
    ering_checks();
    fifo_checks();
    builders();
    led_sequence();
    led_blips();
    params();
    console_routing();
    puts("test_link: OK");
    return 0;
}
