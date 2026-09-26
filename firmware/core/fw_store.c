#include "fw_store.h"
#include "fw_layout.h"
#include <string.h>
#define STORE_MAGIC UINT32_C(0x3254535a)
#define COMMITTED UINT32_C(0x434f4d54)
typedef struct {
    uint32_t magic, schema, sequence, size, payload_crc, header_crc, committed, reserved;
} header_t;
static int valid_any(const fw_flash_t *f, uint32_t addr, header_t *h)
{
    uint32_t crc;
    if (f->read(f->context, addr, h, sizeof *h)) return 0;
    if (h->magic != STORE_MAGIC || !h->size || h->size>FW_STORE_SIZE-sizeof *h || (h->size&3u) ||
        h->committed != COMMITTED || h->header_crc != fw_crc32(h, 20)) return 0;
    return !fw_flash_crc(f, addr + sizeof *h, h->size, &crc) && crc == h->payload_crc;
}
static int valid(const fw_flash_t *f, uint32_t addr, uint32_t schema, size_t size, header_t *h)
{
    return valid_any(f,addr,h) && h->schema==schema && h->size==size;
}
static int latest(const fw_flash_t *f, uint32_t schema, size_t size, header_t h[2])
{
    int a = valid(f, FW_STORE_A, schema, size, &h[0]);
    int b = valid(f, FW_STORE_B, schema, size, &h[1]);
    if (!a) return b ? 1 : -1;
    if (!b) return 0;
    return (int32_t)(h[1].sequence - h[0].sequence) > 0 ? 1 : 0;
}
int fw_store_load(const fw_flash_t *f, uint32_t schema, void *data, size_t size)
{
    header_t h[2];
    int index = latest(f, schema, size, h);
    if (index < 0) return -1;
    return f->read(f->context, (index ? FW_STORE_B : FW_STORE_A) + sizeof(header_t), data, size);
}
int fw_store_save(const fw_flash_t *f, uint32_t schema, const void *data, size_t size)
{
    header_t old[2], h;
    uint32_t verify;
    if (!data || !size || (size & 3u) || size > FW_STORE_SIZE - sizeof h) return -1;
    /* Preserve the newest committed snapshot even when migrating its schema. */
    int a=valid_any(f,FW_STORE_A,&old[0]), b=valid_any(f,FW_STORE_B,&old[1]);
    int index=!a?(b?1:-1):!b?0:((int32_t)(old[1].sequence-old[0].sequence)>0?1:0);
    /* Avoid erases when settings and map have not changed. */
    if (index >= 0 && old[index].schema==schema && old[index].size==size &&
        old[index].payload_crc == fw_crc32(data, size)) {
        unsigned char buf[256];
        uint32_t addr = (index ? FW_STORE_B : FW_STORE_A) + sizeof h;
        size_t i = 0;
        for (; i < size; i += sizeof buf) {
            size_t n = size - i < sizeof buf ? size - i : sizeof buf;
            if (f->read(f->context, addr + i, buf, n) || memcmp(buf, (const char *)data + i, n)) break;
        }
        if (i >= size) return 0;
    }
    uint32_t dest = index == 0 ? FW_STORE_B : FW_STORE_A;
    memset(&h, 0xff, sizeof h);
    h.magic = STORE_MAGIC; h.schema = schema;
    h.sequence = index < 0 ? 0 : old[index].sequence + 1;
    h.size = (uint32_t)size; h.payload_crc = fw_crc32(data, size);
    h.header_crc = fw_crc32(&h, 20);
    if (f->erase(f->context, dest, FW_STORE_SIZE) ||
        f->program(f->context, dest, &h, sizeof h) ||
        f->program(f->context, dest + sizeof h, data, size) ||
        fw_flash_crc(f, dest + sizeof h, size, &verify) || verify != h.payload_crc) return -1;
    uint32_t commit = COMMITTED;
    if (f->program(f->context, dest + offsetof(header_t, committed), &commit, sizeof commit)) return -1;
    return valid(f, dest, schema, size, &h) ? 0 : -1;
}
