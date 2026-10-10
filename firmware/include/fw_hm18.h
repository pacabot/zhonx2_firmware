#ifndef FW_HM18_H
#define FW_HM18_H
#include <stdint.h>
/* HM-18 (CC2640R2F, HM-10 command set) configuration, hardware independent.
 * Commands carry no terminator; the CR LF variant is tried when a module
 * does not answer the bare form. Replies end with a short silence.
 *
 * Boot: find the module baud (115200 first, then the 9600 factory setting
 * and the others), move it to 115200 (AT+BAUD4, AT+RESET), name it
 * FW_HM18_NAME, enable the OK+CONN / OK+LOST notifications (AT+NOTI1) and
 * read its BLE address. A module that never answers is probed again
 * every FW_HM18_RETRY_MS. While unconnected the module parses every byte
 * it receives as a command, so the driver must not transmit data then. */
#define FW_HM18_NAME "Zhonx-2"
#define FW_HM18_BAUD 115200u
#define FW_HM18_RETRY_MS 10000u
#define FW_HM18_REPLY_MS 600u
#define FW_HM18_SILENCE_MS 40u
#define FW_HM18_RESET_MS 1200u
enum {
    FW_HM18_PROBE, FW_HM18_SET_BAUD, FW_HM18_RESET, FW_HM18_NAME_GET, FW_HM18_NAME_SET,
    FW_HM18_NOTI_GET, FW_HM18_NOTI_SET, FW_HM18_ADDR_GET, FW_HM18_FINAL, FW_HM18_READY, FW_HM18_ABSENT
};
typedef struct {
    uint8_t state, baud_index, crlf, changed, baud_set_tried, after_baud, pending, reply_len;
    uint32_t sent_ms, last_rx_ms, wait_until;
    unsigned baud;          /* UART speed the driver must use now */
    char reply[40];
    char command[24];
    char name[16];          /* as read from the module before any change */
    char address[16];
    unsigned probes;        /* commands sent, for the diagnostic screen */
} fw_hm18_t;
void fw_hm18_start(fw_hm18_t *, uint32_t now);
/* Bytes received while configuring. */
void fw_hm18_rx(fw_hm18_t *, uint8_t byte, uint32_t now);
/* Returns 1 with *send set when a command must go out, at m->baud, after the
 * driver has applied that speed and dropped what it had received. */
int fw_hm18_step(fw_hm18_t *, uint32_t now, const char **send);
int fw_hm18_ready(const fw_hm18_t *);
const char *fw_hm18_state_name(const fw_hm18_t *);

/* Connection notifications in the data stream (bytes outside link frames). */
typedef struct { char tail[8]; } fw_hm18_watch_t;
/* 1 after "OK+CONN", -1 after "OK+LOST", 0 otherwise. */
int fw_hm18_watch(fw_hm18_watch_t *, uint8_t byte);
#endif
