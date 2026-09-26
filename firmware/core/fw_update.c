#include "fw_update.h"
#include "fw_layout.h"
#include <string.h>
#define IMAGE_MAGIC UINT32_C(0x32474d49)
#define IMAGE_COMMIT UINT32_C(0x434f4d54)
#define PENDING UINT32_C(0xfffffffe)
#define INSTALLED UINT32_C(0xfffffffc)
typedef struct {
    uint32_t magic, version, size, crc, header_crc, state, committed, reserved;
} manifest_t;
static int manifest(const fw_flash_t *f, manifest_t *m)
{
    if (f->read(f->context, FW_META_BASE, m, sizeof *m)) return -1;
    return m->magic == IMAGE_MAGIC && m->version == 1 && m->size >= 8 &&
        m->size <= FW_IMAGE_MAX && !(m->size & 3) && m->header_crc == fw_crc32(m,16) &&
        m->committed == IMAGE_COMMIT && (m->state == PENDING || m->state == INSTALLED) ? 0 : -1;
}
static int image(const fw_flash_t *f, uint32_t base, uint32_t size, uint32_t expected)
{
    uint32_t v[2], crc;
    if (f->read(f->context,base,v,sizeof v) || fw_flash_crc(f,base,size,&crc)) return -1;
    return crc == expected && v[0] > FW_SRAM_BASE && v[0] <= FW_SRAM_END && !(v[0] & 7u) &&
        (v[1] & 1u) && v[1] >= FW_APP_BASE && v[1] < FW_APP_BASE + size ? 0 : -1;
}
int fw_update_begin(const fw_flash_t *f, fw_update_t *u, uint32_t size, uint32_t crc)
{
    memset(u,0,sizeof *u);
    if (size < 8 || size > FW_IMAGE_MAX || (size & 3)) return -1;
    uint32_t erase_size=(size+0x1ffffu)&~0x1ffffu;
    if (f->erase(f->context, FW_STAGE_BASE, erase_size)) return -1;
    u->size = size; u->crc = crc; u->active = 1;
    return 0;
}
int fw_update_write(const fw_flash_t *f, fw_update_t *u, uint32_t off, const void *data, size_t n)
{
    unsigned char verify[256];
    if (!u->active || !data || !n || n > 1024 || (n & 3u) || (off & 3u) ||
        off > u->size || n > u->size - off || off > u->received) return -1;
    if (off < u->received && n > u->received - off) return -1;
    if (off == u->received && f->program(f->context, FW_STAGE_BASE + off, data, n)) return -1;
    for (size_t i = 0; i < n; i += sizeof verify) {
        size_t take = n - i < sizeof verify ? n - i : sizeof verify;
        if (f->read(f->context, FW_STAGE_BASE+off+i,verify,take) || memcmp(verify,(const char *)data+i,take)) return -1;
    }
    if (off == u->received) u->received += n;
    return 0;
}
int fw_update_seal(const fw_flash_t *f, fw_update_t *u)
{
    manifest_t m;
    if (!u->active || u->received != u->size || image(f,FW_STAGE_BASE,u->size,u->crc)) return -1;
    memset(&m,0xff,sizeof m);
    m.magic = IMAGE_MAGIC; m.version = 1; m.size = u->size; m.crc = u->crc;
    m.header_crc = fw_crc32(&m,16); m.state = PENDING;
    if (f->erase(f->context, FW_META_BASE, FW_STORE_SIZE) ||
        f->program(f->context, FW_META_BASE, &m, sizeof m)) return -1;
    uint32_t commit = IMAGE_COMMIT;
    if (f->program(f->context,FW_META_BASE+offsetof(manifest_t,committed),&commit,4)) return -1;
    u->active = 0;
    return manifest(f,&m);
}
int fw_update_install(const fw_flash_t *f)
{
    manifest_t m;
    unsigned char block[1024];
    if (manifest(f,&m)) return -1;
    if (m.state == INSTALLED) return fw_application_valid(f);
    if (image(f,FW_STAGE_BASE,m.size,m.crc)) return -1;
    uint32_t end=FW_APP_BASE+0x10000; /* First application sector is 64 KiB. */
    while (end<FW_APP_BASE+m.size) end+=0x20000;
    if (end>FW_APP_ERASE_END || f->erase(f->context,FW_APP_BASE,end-FW_APP_BASE)) return -1;
    for (uint32_t off = 0; off < m.size; off += sizeof block) {
        size_t n = m.size-off < sizeof block ? m.size-off : sizeof block;
        if (f->read(f->context,FW_STAGE_BASE+off,block,n) ||
            f->program(f->context,FW_APP_BASE+off,block,n)) return -1;
    }
    if (image(f,FW_APP_BASE,m.size,m.crc)) return -1;
    uint32_t state = INSTALLED;
    if (f->program(f->context,FW_META_BASE+offsetof(manifest_t,state),&state,4)) return -1;
    return fw_application_valid(f);
}
int fw_application_valid(const fw_flash_t *f)
{
    manifest_t m;
    if (manifest(f,&m) || m.state != INSTALLED) return -1;
    return image(f,FW_APP_BASE,m.size,m.crc);
}
