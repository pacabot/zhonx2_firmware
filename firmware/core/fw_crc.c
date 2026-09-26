#include "fw_flash.h"
uint32_t fw_crc32_more(uint32_t previous, const void *data, size_t length)
{
    const unsigned char *p = data;
    uint32_t crc = ~previous;
    while (length--) {
        crc ^= *p++;
        for (unsigned i = 0; i < 8; ++i)
            crc = (crc >> 1) ^ (UINT32_C(0xedb88320) & (0u - (crc & 1u)));
    }
    return ~crc;
}
uint32_t fw_crc32(const void *data, size_t length)
{
    return fw_crc32_more(0, data, length);
}
int fw_flash_crc(const fw_flash_t *f, uint32_t address, size_t length, uint32_t *crc)
{
    unsigned char buf[256];
    *crc = 0;
    while (length) {
        size_t n = length < sizeof buf ? length : sizeof buf;
        if (f->read(f->context, address, buf, n)) return -1;
        *crc = fw_crc32_more(*crc, buf, n);
        address += (uint32_t)n;
        length -= n;
    }
    return 0;
}
