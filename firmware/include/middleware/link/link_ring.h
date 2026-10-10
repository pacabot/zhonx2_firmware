/*
 * link_ring.h
 *
 * Robot -> application streams of Ariane Link, pure C (host tested):
 *  - the telemetry ring: single producer (the 1 kHz control interrupt),
 *    single consumer (linkPoll), records kept until the application
 *    acknowledges them, the oldest records decimated when the ring is full;
 *  - the events ring: never decimated, an event is counted as dropped when
 *    the ring is full;
 *  - a byte FIFO for the frames that must not be lost (replies);
 *  - the frame builders: TELEMETRY batches of up to LINK_BATCH_RECORDS
 *    records and single EVENT frames, ready for the UART.
 * Sequence numbers are 32-bit per stream and start at 1, so that an ACK of
 * 0 acknowledges nothing.
 */

#ifndef LINK_RING_H_
#define LINK_RING_H_

#include <stddef.h>
#include <stdint.h>

#include "middleware/link/ariane_link_schema.h"

#define LINK_BATCH_RECORDS      10U
#define LINK_EVENT_ARGS_MAX     16U
#define LINK_STREAM_TELEMETRY   0U
#define LINK_STREAM_EVENTS      1U

typedef struct
{
    uint32_t seq;
    uint32_t t_us;
    uint8_t event;
    uint8_t len;
    uint8_t args[LINK_EVENT_ARGS_MAX];
} link_event_t;

typedef struct
{
    link_telemetry_record_t *slots;
    uint16_t capacity;
    uint16_t head;              /* slot written next */
    uint16_t count;             /* records retained, oldest first */
    uint16_t sent;              /* of the retained records, how many were sent at least once */
    uint32_t base_seq;          /* sequence number of the oldest retained record */
    uint32_t next_seq;          /* sequence number of the next record */
    uint32_t resend_from;       /* RESEND range still to send, when resend_left > 0 */
    uint32_t resend_left;
    uint32_t dropped_total;
    uint32_t dropped_unreported;
} link_tring_t;

typedef struct
{
    link_event_t *slots;
    uint16_t capacity;
    uint16_t head;
    uint16_t count;
    uint16_t sent;
    uint32_t base_seq;
    uint32_t next_seq;
    uint32_t dropped_total;
    uint32_t dropped_unreported;
} link_ering_t;

typedef struct
{
    uint8_t *data;
    uint16_t capacity;
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} link_fifo_t;

/* ---- telemetry ---------------------------------------------------------- */
void     link_tring_init(link_tring_t *ring, link_telemetry_record_t *slots, uint16_t capacity);
/* Producer side (interrupt): stores rec with the next sequence number. */
void     link_tring_push(link_tring_t *ring, const link_telemetry_record_t *rec);
/* Consumer side: copies up to max records to send (RESEND range first,
 * then the records never sent) and marks them sent; *first_seq is the
 * sequence number of out[0]. Returns the number of records copied. */
unsigned link_tring_take(link_tring_t *ring, link_telemetry_record_t *out, unsigned max, uint32_t *first_seq);
/* Forgets the records up to seq included. */
void     link_tring_ack(link_tring_t *ring, uint32_t seq);
/* Schedules [from, to] again; returns how many of them are still retained. */
unsigned link_tring_resend(link_tring_t *ring, uint32_t from, uint32_t to);
/* Everything retained goes out again (FLUSH). */
void     link_tring_flush(link_tring_t *ring);
unsigned link_tring_pending(const link_tring_t *ring);
/* Last sequence number produced, 0 before the first record. */
uint32_t link_tring_last_seq(const link_tring_t *ring);
void     link_tring_reset(link_tring_t *ring);

/* ---- events ------------------------------------------------------------- */
void     link_ering_init(link_ering_t *ring, link_event_t *slots, uint16_t capacity);
/* Returns the sequence number given, or 0 when the ring is full (dropped). */
uint32_t link_ering_push(link_ering_t *ring, uint8_t event, uint32_t t_us, const void *args, uint8_t len);
int      link_ering_take(link_ering_t *ring, link_event_t *out);
void     link_ering_ack(link_ering_t *ring, uint32_t seq);
unsigned link_ering_resend(link_ering_t *ring, uint32_t from, uint32_t to);
void     link_ering_flush(link_ering_t *ring);
unsigned link_ering_pending(const link_ering_t *ring);
void     link_ering_reset(link_ering_t *ring);

/* ---- byte FIFO ---------------------------------------------------------- */
void     link_fifo_init(link_fifo_t *fifo, uint8_t *data, uint16_t capacity);
unsigned link_fifo_free(const link_fifo_t *fifo);
int      link_fifo_put(link_fifo_t *fifo, const uint8_t *data, unsigned len);
/* Copies up to max bytes without removing them; link_fifo_drop removes. */
unsigned link_fifo_peek(const link_fifo_t *fifo, uint8_t *out, unsigned max);
void     link_fifo_drop(link_fifo_t *fifo, unsigned len);

/* ---- frame builders ----------------------------------------------------- */
/* TELEMETRY frame from the next batch of the ring, 0 when nothing to send.
 * cap must hold LINK_FRAME_OVERHEAD + LINK_TELEMETRY_BATCH_SIZE +
 * LINK_BATCH_RECORDS * LINK_TELEMETRY_RECORD_SIZE bytes. */
size_t   link_build_telemetry(link_tring_t *ring, uint8_t *frame, size_t cap, uint8_t frame_seq);
/* EVENT frame from the next event of the ring, 0 when nothing to send. */
size_t   link_build_event(link_ering_t *ring, uint8_t *frame, size_t cap, uint8_t frame_seq);

#endif /* LINK_RING_H_ */
