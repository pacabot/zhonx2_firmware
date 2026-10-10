/*
 * link_ring.c
 *
 * Ariane Link streams, see link_ring.h. The consumer functions take a
 * short critical section (link_lock) because the telemetry producer runs
 * in the control interrupt; copying a batch of ten records costs about a
 * microsecond on the robot.
 */

#include <string.h>

#include "middleware/link/link_frame.h"
#include "middleware/link/link_port.h"
#include "middleware/link/link_ring.h"

/* ---- telemetry ---------------------------------------------------------- */

static uint16_t tring_slot(const link_tring_t *ring, unsigned index_from_oldest)
{
    unsigned oldest = (unsigned)ring->head + ring->capacity - ring->count;
    return (uint16_t)((oldest + index_from_oldest) % ring->capacity);
}

void link_tring_init(link_tring_t *ring, link_telemetry_record_t *slots, uint16_t capacity)
{
    memset(ring, 0, sizeof *ring);
    ring->slots = slots;
    ring->capacity = capacity;
    ring->base_seq = 1;
    ring->next_seq = 1;
}

void link_tring_reset(link_tring_t *ring)
{
    uint32_t state = link_lock();
    ring->head = 0;
    ring->count = 0;
    ring->sent = 0;
    ring->base_seq = ring->next_seq;
    ring->resend_left = 0;
    ring->dropped_unreported = 0;
    link_unlock(state);
}

void link_tring_push(link_tring_t *ring, const link_telemetry_record_t *rec)
{
    if (ring->capacity == 0)
        return;
    if (ring->count == ring->capacity)
    {
        /* Full: the oldest periodic record is decimated, sent or not. */
        ring->count--;
        ring->base_seq++;
        if (ring->sent > 0)
            ring->sent--;
        if (ring->resend_left > 0 && ring->resend_from < ring->base_seq)
        {
            ring->resend_from++;
            ring->resend_left--;
        }
        ring->dropped_total++;
        if (ring->dropped_unreported < 0xFFFFU)
            ring->dropped_unreported++;
    }
    ring->slots[ring->head] = *rec;
    ring->head = (uint16_t)((ring->head + 1U) % ring->capacity);
    ring->count++;
    ring->next_seq++;
}

unsigned link_tring_take(link_tring_t *ring, link_telemetry_record_t *out, unsigned max, uint32_t *first_seq)
{
    unsigned n = 0;
    uint32_t state = link_lock();

    if (ring->resend_left > 0)
    {
        unsigned start = (unsigned)(ring->resend_from - ring->base_seq);
        if (ring->resend_from < ring->base_seq || start >= ring->count)
        {
            ring->resend_left = 0;    /* the range left the ring meanwhile */
        }
        else
        {
            unsigned avail = ring->count - start;
            n = ring->resend_left < avail ? (unsigned)ring->resend_left : avail;
            if (n > max)
                n = max;
            for (unsigned i = 0; i < n; i++)
                out[i] = ring->slots[tring_slot(ring, start + i)];
            *first_seq = ring->resend_from;
            ring->resend_from += n;
            ring->resend_left -= n;
            /* Everything up to the resent range counts as sent. */
            if (start + n > ring->sent)
                ring->sent = (uint16_t)(start + n);
        }
    }
    if (n == 0 && ring->sent < ring->count)
    {
        unsigned avail = ring->count - ring->sent;
        n = avail < max ? avail : max;
        for (unsigned i = 0; i < n; i++)
            out[i] = ring->slots[tring_slot(ring, ring->sent + i)];
        *first_seq = ring->base_seq + ring->sent;
        ring->sent = (uint16_t)(ring->sent + n);
    }
    link_unlock(state);
    return n;
}

void link_tring_ack(link_tring_t *ring, uint32_t seq)
{
    uint32_t state = link_lock();
    uint32_t last_sent = ring->base_seq + ring->sent;   /* one past the last sent */

    if (seq >= ring->base_seq && ring->count > 0)
    {
        uint32_t k = seq - ring->base_seq + 1U;
        if (k > ring->sent)
            k = ring->sent;                /* never forget what was not sent */
        if (k > 0 && ring->base_seq + k <= last_sent)
        {
            ring->count = (uint16_t)(ring->count - k);
            ring->sent = (uint16_t)(ring->sent - k);
            ring->base_seq += k;
            if (ring->resend_left > 0 && ring->resend_from < ring->base_seq)
            {
                uint32_t gone = ring->base_seq - ring->resend_from;
                if (gone >= ring->resend_left)
                    ring->resend_left = 0;
                else
                {
                    ring->resend_left -= gone;
                    ring->resend_from = ring->base_seq;
                }
            }
        }
    }
    link_unlock(state);
}

unsigned link_tring_resend(link_tring_t *ring, uint32_t from, uint32_t to)
{
    unsigned n = 0;
    uint32_t state = link_lock();
    uint32_t end = ring->base_seq + ring->sent;    /* one past the newest record already sent */

    /* Only records already sent can be missing on the other side; the
     * others go out in order anyway. */
    if (to >= from && ring->sent > 0 && to >= ring->base_seq && from < end)
    {
        if (from < ring->base_seq)
            from = ring->base_seq;
        if (to >= end)
            to = end - 1U;
        ring->resend_from = from;
        ring->resend_left = to - from + 1U;
        n = (unsigned)ring->resend_left;
    }
    link_unlock(state);
    return n;
}

void link_tring_flush(link_tring_t *ring)
{
    uint32_t state = link_lock();
    ring->sent = 0;
    ring->resend_left = 0;
    link_unlock(state);
}

unsigned link_tring_pending(const link_tring_t *ring)
{
    return (unsigned)(ring->count - ring->sent) + (unsigned)ring->resend_left;
}

uint32_t link_tring_last_seq(const link_tring_t *ring)
{
    return ring->next_seq - 1U;
}

/* ---- events ------------------------------------------------------------- */

static uint16_t ering_slot(const link_ering_t *ring, unsigned index_from_oldest)
{
    unsigned oldest = (unsigned)ring->head + ring->capacity - ring->count;
    return (uint16_t)((oldest + index_from_oldest) % ring->capacity);
}

void link_ering_init(link_ering_t *ring, link_event_t *slots, uint16_t capacity)
{
    memset(ring, 0, sizeof *ring);
    ring->slots = slots;
    ring->capacity = capacity;
    ring->base_seq = 1;
    ring->next_seq = 1;
}

void link_ering_reset(link_ering_t *ring)
{
    uint32_t state = link_lock();
    ring->head = 0;
    ring->count = 0;
    ring->sent = 0;
    ring->base_seq = ring->next_seq;
    ring->dropped_unreported = 0;
    link_unlock(state);
}

uint32_t link_ering_push(link_ering_t *ring, uint8_t event, uint32_t t_us, const void *args, uint8_t len)
{
    uint32_t seq = 0;
    uint32_t state = link_lock();

    if (len > LINK_EVENT_ARGS_MAX)
        len = LINK_EVENT_ARGS_MAX;
    if (ring->count < ring->capacity && ring->capacity > 0)
    {
        link_event_t *slot = &ring->slots[ring->head];
        seq = ring->next_seq++;
        slot->seq = seq;
        slot->t_us = t_us;
        slot->event = event;
        slot->len = len;
        if (len > 0)
            memcpy(slot->args, args, len);
        ring->head = (uint16_t)((ring->head + 1U) % ring->capacity);
        ring->count++;
    }
    else
    {
        ring->dropped_total++;
        if (ring->dropped_unreported < 0xFFU)
            ring->dropped_unreported++;
    }
    link_unlock(state);
    return seq;
}

int link_ering_take(link_ering_t *ring, link_event_t *out)
{
    int taken = 0;
    uint32_t state = link_lock();

    if (ring->sent < ring->count)
    {
        *out = ring->slots[ering_slot(ring, ring->sent)];
        ring->sent++;
        taken = 1;
    }
    link_unlock(state);
    return taken;
}

void link_ering_ack(link_ering_t *ring, uint32_t seq)
{
    uint32_t state = link_lock();

    if (seq >= ring->base_seq && ring->count > 0)
    {
        uint32_t k = seq - ring->base_seq + 1U;
        if (k > ring->sent)
            k = ring->sent;
        ring->count = (uint16_t)(ring->count - k);
        ring->sent = (uint16_t)(ring->sent - k);
        ring->base_seq += k;
    }
    link_unlock(state);
}

unsigned link_ering_resend(link_ering_t *ring, uint32_t from, uint32_t to)
{
    unsigned n = 0;
    uint32_t state = link_lock();
    uint32_t end = ring->base_seq + ring->sent;    /* one past the newest event already sent */

    if (to >= from && ring->sent > 0 && to >= ring->base_seq && from < end)
    {
        if (from < ring->base_seq)
            from = ring->base_seq;
        /* Events are few: everything from the first missing one goes again. */
        ring->sent = (uint16_t)(from - ring->base_seq);
        n = (unsigned)(ring->count - ring->sent);
    }
    link_unlock(state);
    return n;
}

void link_ering_flush(link_ering_t *ring)
{
    uint32_t state = link_lock();
    ring->sent = 0;
    link_unlock(state);
}

unsigned link_ering_pending(const link_ering_t *ring)
{
    return (unsigned)(ring->count - ring->sent);
}

/* ---- byte FIFO ---------------------------------------------------------- */

void link_fifo_init(link_fifo_t *fifo, uint8_t *data, uint16_t capacity)
{
    memset(fifo, 0, sizeof *fifo);
    fifo->data = data;
    fifo->capacity = capacity;
}

unsigned link_fifo_free(const link_fifo_t *fifo)
{
    return (unsigned)(fifo->capacity - fifo->count);
}

int link_fifo_put(link_fifo_t *fifo, const uint8_t *data, unsigned len)
{
    if (len > link_fifo_free(fifo))
        return 0;
    for (unsigned i = 0; i < len; i++)
    {
        fifo->data[fifo->head] = data[i];
        fifo->head = (uint16_t)((fifo->head + 1U) % fifo->capacity);
    }
    fifo->count = (uint16_t)(fifo->count + len);
    return 1;
}

unsigned link_fifo_peek(const link_fifo_t *fifo, uint8_t *out, unsigned max)
{
    unsigned n = fifo->count < max ? fifo->count : max;
    for (unsigned i = 0; i < n; i++)
        out[i] = fifo->data[(fifo->tail + i) % fifo->capacity];
    return n;
}

void link_fifo_drop(link_fifo_t *fifo, unsigned len)
{
    if (len > fifo->count)
        len = fifo->count;
    fifo->tail = (uint16_t)((fifo->tail + len) % fifo->capacity);
    fifo->count = (uint16_t)(fifo->count - len);
}

/* ---- frame builders ----------------------------------------------------- */

size_t link_build_telemetry(link_tring_t *ring, uint8_t *frame, size_t cap, uint8_t frame_seq)
{
    uint8_t payload[LINK_TELEMETRY_BATCH_SIZE + LINK_BATCH_RECORDS * LINK_TELEMETRY_RECORD_SIZE];
    link_telemetry_record_t records[LINK_BATCH_RECORDS];
    link_telemetry_batch_t header;
    uint32_t first_seq = 0;
    unsigned n;

    if (cap < LINK_FRAME_OVERHEAD + sizeof payload)
        return 0;
    n = link_tring_take(ring, records, LINK_BATCH_RECORDS, &first_seq);
    if (n == 0)
        return 0;
    header.first_seq = first_seq;
    header.count = (uint8_t)n;
    header.record_size = LINK_TELEMETRY_RECORD_SIZE;
    memcpy(payload, &header, LINK_TELEMETRY_BATCH_SIZE);
    memcpy(payload + LINK_TELEMETRY_BATCH_SIZE, records, n * LINK_TELEMETRY_RECORD_SIZE);
    return link_frame_encode(frame, cap, LINK_FRAME_TYPE_TELEMETRY, frame_seq, payload,
                             (uint16_t)(LINK_TELEMETRY_BATCH_SIZE + n * LINK_TELEMETRY_RECORD_SIZE));
}

size_t link_build_event(link_ering_t *ring, uint8_t *frame, size_t cap, uint8_t frame_seq)
{
    uint8_t payload[LINK_EVENT_HEADER_SIZE + LINK_EVENT_ARGS_MAX];
    link_event_header_t header;
    link_event_t evt;

    if (cap < LINK_FRAME_OVERHEAD + sizeof payload)
        return 0;
    if (!link_ering_take(ring, &evt))
        return 0;
    header.seq = evt.seq;
    header.t_us = evt.t_us;
    header.event = evt.event;
    header.len = evt.len;
    memcpy(payload, &header, LINK_EVENT_HEADER_SIZE);
    memcpy(payload + LINK_EVENT_HEADER_SIZE, evt.args, evt.len);
    return link_frame_encode(frame, cap, LINK_FRAME_TYPE_EVENT, frame_seq, payload,
                             (uint16_t)(LINK_EVENT_HEADER_SIZE + evt.len));
}
