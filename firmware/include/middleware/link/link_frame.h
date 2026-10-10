/* Ariane Link : tramage de référence en C, partagé robot / tests hôte.
 * Trame : SOF(0xA5) TYPE(1) SEQ(1) LEN(2 LE) PAYLOAD(LEN) CRC16(2 LE)
 * CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) sur TYPE..PAYLOAD.
 */
#ifndef ARIANE_LINK_FRAME_H
#define ARIANE_LINK_FRAME_H

#include <stddef.h>
#include <stdint.h>

#define LINK_FRAME_SOF 0xA5u
#define LINK_FRAME_MAX_PAYLOAD 512u
#define LINK_FRAME_HEADER 4u   /* type, seq, len */
#define LINK_FRAME_OVERHEAD 7u /* sof + header + crc */

typedef struct {
    uint8_t type;
    uint8_t seq;
    uint16_t len;
    const uint8_t *payload; /* pointe dans le tampon interne de l'analyseur, valide jusqu'au prochain feed */
} link_frame_t;

typedef struct {
    uint8_t state;
    uint16_t need;
    uint16_t pos;
    uint8_t buf[LINK_FRAME_HEADER + LINK_FRAME_MAX_PAYLOAD + 2];
    uint32_t frames;
    uint32_t crc_errors;
    uint32_t length_errors;
} link_parser_t;

#define LINK_FEED_NONE 0    /* octet consommé, pas encore de trame */
#define LINK_FEED_FRAME 1   /* trame complète et valide dans *out */
#define LINK_FEED_STRAY (-1) /* octet hors trame : à router vers la console texte */

uint16_t link_crc16(const uint8_t *data, size_t len, uint16_t crc);
size_t link_frame_encode(uint8_t *dst, size_t cap, uint8_t type, uint8_t seq, const void *payload, uint16_t len);
void link_parser_init(link_parser_t *p);
int link_parser_feed(link_parser_t *p, uint8_t byte, link_frame_t *out);

#endif
