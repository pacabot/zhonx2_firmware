#ifndef FW_UPDATE_H
#define FW_UPDATE_H
#include "fw_flash.h"
typedef struct { uint32_t size, crc, received; int active; } fw_update_t;
int fw_update_begin(const fw_flash_t *, fw_update_t *, uint32_t size, uint32_t crc);
int fw_update_write(const fw_flash_t *, fw_update_t *, uint32_t offset, const void *, size_t length);
int fw_update_seal(const fw_flash_t *, fw_update_t *);
/* Repeats safely after reset if power was lost while replacing the application. */
int fw_update_install(const fw_flash_t *);
int fw_application_valid(const fw_flash_t *);
#endif
