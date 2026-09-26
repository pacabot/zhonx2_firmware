#ifndef FW_PROTOCOL_H
#define FW_PROTOCOL_H
#include "fw_update.h"
#define FW_PACKET_MAGIC UINT32_C(0x3257465a)
#define FW_PACKET_BYTES 1056
#define FW_REPLY_BYTES 32
enum { FW_INFO=1, FW_BEGIN, FW_DATA, FW_COMMIT, FW_BOOT };
/* Little endian words: magic, command, sequence, arg0, arg1, length, version, crc.
 * CRC covers the first 28 bytes followed by length payload bytes (not padding). */
typedef struct { uint32_t word[8]; uint8_t payload[1024]; } fw_packet_t;
typedef struct { uint32_t word[8]; } fw_reply_t;
/* Returns 1 to boot AFTER delivering the reply, 0 otherwise. */
int fw_protocol(const fw_flash_t *, fw_update_t *, const fw_packet_t *, fw_reply_t *);
#endif
