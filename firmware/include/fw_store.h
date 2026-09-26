#ifndef FW_STORE_H
#define FW_STORE_H
#include "fw_flash.h"
/* Whole snapshots alternate between independent erase sectors. */
int fw_store_load(const fw_flash_t *flash, uint32_t schema, void *data, size_t size);
int fw_store_save(const fw_flash_t *flash, uint32_t schema, const void *data, size_t size);
#endif
