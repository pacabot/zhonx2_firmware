#ifndef FW_FLASH_H
#define FW_FLASH_H
#include <stddef.h>
#include <stdint.h>
typedef struct {
    void *context;
    int (*read)(void *, uint32_t, void *, size_t);
    int (*erase)(void *, uint32_t, size_t);
    int (*program)(void *, uint32_t, const void *, size_t);
} fw_flash_t;
uint32_t fw_crc32(const void *data, size_t length);
uint32_t fw_crc32_more(uint32_t previous, const void *data, size_t length);
int fw_flash_crc(const fw_flash_t *f, uint32_t address, size_t length, uint32_t *crc);
extern const fw_flash_t fw_stm32_flash;
#endif
