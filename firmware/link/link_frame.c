#include "link_frame.h"
#include <string.h>

enum { HUNT = 0, HEADER = 1, PAYLOAD = 2, CRC = 3 };

uint16_t link_crc16(const uint8_t *data, size_t len, uint16_t crc)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t link_frame_encode(uint8_t *dst, size_t cap, uint8_t type, uint8_t seq, const void *payload, uint16_t len)
{
    size_t total = LINK_FRAME_OVERHEAD + len;
    if (len > LINK_FRAME_MAX_PAYLOAD || cap < total)
        return 0;
    dst[0] = LINK_FRAME_SOF;
    dst[1] = type;
    dst[2] = seq;
    dst[3] = (uint8_t)(len & 0xFF);
    dst[4] = (uint8_t)(len >> 8);
    if (len)
        memcpy(dst + 5, payload, len);
    uint16_t crc = link_crc16(dst + 1, LINK_FRAME_HEADER + len, 0xFFFF);
    dst[5 + len] = (uint8_t)(crc & 0xFF);
    dst[6 + len] = (uint8_t)(crc >> 8);
    return total;
}

void link_parser_init(link_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

static void reset(link_parser_t *p)
{
    p->state = HUNT;
    p->pos = 0;
    p->need = 0;
}

int link_parser_feed(link_parser_t *p, uint8_t byte, link_frame_t *out)
{
    switch (p->state) {
    case HUNT:
        if (byte != LINK_FRAME_SOF)
            return LINK_FEED_STRAY;
        p->state = HEADER;
        p->pos = 0;
        return LINK_FEED_NONE;
    case HEADER:
        p->buf[p->pos++] = byte;
        if (p->pos == LINK_FRAME_HEADER) {
            p->need = (uint16_t)(p->buf[2] | (p->buf[3] << 8));
            if (p->need > LINK_FRAME_MAX_PAYLOAD) {
                p->length_errors++;
                reset(p);
                return LINK_FEED_NONE;
            }
            p->state = p->need ? PAYLOAD : CRC;
        }
        return LINK_FEED_NONE;
    case PAYLOAD:
        p->buf[p->pos++] = byte;
        if (p->pos == LINK_FRAME_HEADER + p->need)
            p->state = CRC;
        return LINK_FEED_NONE;
    case CRC:
        p->buf[p->pos++] = byte;
        if (p->pos == LINK_FRAME_HEADER + p->need + 2) {
            uint16_t got = (uint16_t)(p->buf[p->pos - 2] | (p->buf[p->pos - 1] << 8));
            uint16_t want = link_crc16(p->buf, LINK_FRAME_HEADER + p->need, 0xFFFF);
            int result = LINK_FEED_NONE;
            if (got == want) {
                out->type = p->buf[0];
                out->seq = p->buf[1];
                out->len = p->need;
                out->payload = p->buf + LINK_FRAME_HEADER;
                p->frames++;
                result = LINK_FEED_FRAME;
            } else {
                p->crc_errors++;
            }
            reset(p);
            return result;
        }
        return LINK_FEED_NONE;
    default:
        reset(p);
        return LINK_FEED_NONE;
    }
}
