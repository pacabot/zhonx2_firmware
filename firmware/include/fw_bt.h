#ifndef FW_BT_H
#define FW_BT_H
#include <stdint.h>
#include "fw_hm18.h"
/* HM-18 BLE module on USART1 (K1: PA9 TX, PA10 RX), see docs/ARIANE.md.
 * Reception: circular DMA2 stream 2. Transmission: DMA2 stream 7 in chunks,
 * paced by a byte budget because the module has no flow control and
 * silently drops what BLE cannot carry. No interrupt: fw_bt_poll() runs at
 * 1 kHz from the link (PendSV) and is the only caller of everything below,
 * except the read-only status getters. */
#define FW_BT_TX_RATE_DEFAULT 4000u   /* bytes/s, parameter bt_tx_bps */
#define FW_BT_TX_RATE_MIN 500u
#define FW_BT_TX_RATE_MAX 11000u
typedef struct {
    const char *state;
    unsigned baud, tx_rate;
    int ready, connected, quiet;
    uint32_t rx_bytes, tx_bytes, tx_dropped, tx_stalls, rx_errors, connects, losses;
    const char *name, *address;
} fw_bt_status_t;
void fw_bt_init(uint32_t now);
void fw_bt_poll(uint32_t now);
/* Next byte received in data mode; 0 when none. */
int fw_bt_read(uint8_t *byte);
/* Room for fw_bt_send(), 0 while the module cannot carry data. */
unsigned fw_bt_tx_free(void);
/* All or nothing; 0 when refused. */
int fw_bt_send(const void *data, unsigned length);
/* Connection state, learnt by the link from OK+CONN / OK+LOST and frames. */
void fw_bt_set_connected(int connected);
int fw_bt_connected(void);
int fw_bt_ready(void);
/* Holds transmission (autonomous trials without an Ariane session). */
void fw_bt_quiet(int quiet);
void fw_bt_set_rate(unsigned bytes_per_second);
unsigned fw_bt_rate(void);
void fw_bt_status(fw_bt_status_t *);
#endif
