#include "fw_flash.h"
#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
#include <string.h>
static const uint32_t sectors[] = {
    0x08000000,0x08004000,0x08008000,0x0800c000,0x08010000,0x08020000,
    0x08040000,0x08060000,0x08080000,0x080a0000,0x080c0000,0x080e0000,0x08100000
};
static int range(uint32_t a, size_t n)
{
    return a >= sectors[0] && a < sectors[12] && n <= sectors[12] - a;
}
static int read_flash(void *ctx, uint32_t a, void *p, size_t n)
{
    (void)ctx;
    if (!range(a,n)) return -1;
    memcpy(p,(const void *)(uintptr_t)a,n); return 0;
}
static void invalidate_cache(void)
{
    FLASH_DataCacheCmd(DISABLE); FLASH_InstructionCacheCmd(DISABLE);
    FLASH_DataCacheReset(); FLASH_InstructionCacheReset();
    FLASH_DataCacheCmd(ENABLE); FLASH_InstructionCacheCmd(ENABLE);
    __DSB(); __ISB();
}
static int erase_flash(void *ctx, uint32_t a, size_t n)
{
    (void)ctx;
    /* Sector zero (the recovery loader) is never writable by this backend. */
    if (!range(a,n) || a < sectors[1] || !n) return -1;
    int first = -1, last = -1;
    for (int i = 1; i < 13; ++i) { if (a == sectors[i]) first = i; if (a+n == sectors[i]) last = i; }
    if (first < 1 || last <= first) return -1;
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    int result = 0;
    for (int i = first; i < last; ++i)
        if (FLASH_EraseSector((uint16_t)(i * 8),VoltageRange_3) != FLASH_COMPLETE) { result = -1; break; }
    FLASH_Lock(); invalidate_cache(); __set_PRIMASK(mask); return result;
}
static int program_flash(void *ctx, uint32_t a, const void *p, size_t n)
{
    (void)ctx;
    if (!range(a,n) || a < sectors[1] || (a & 3u) || (n & 3u)) return -1;
    uint32_t mask = __get_PRIMASK(); __disable_irq(); FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    int result = 0;
    for (size_t i = 0; i < n; i += 4) {
        uint32_t word; memcpy(&word,(const char *)p+i,4);
        if (FLASH_ProgramWord(a+i,word) != FLASH_COMPLETE) { result = -1; break; }
    }
    FLASH_Lock(); invalidate_cache(); __set_PRIMASK(mask); return result;
}
const fw_flash_t fw_stm32_flash = {0,read_flash,erase_flash,program_flash};
