#ifndef FW_ARIANE_H
#define FW_ARIANE_H
#include <stdint.h>
#include "nimes.h"
/* ZHONX III frame of the Ariane traces (mazeCore.h): origin at the start cell,
 * NORTH = heading at the start, x east, y SOUTH, headings N0 E1 S2 W3.
 * nimes.c has north = +y and an absolute start heading. */
void fw_ariane_frame(const nm_map_t *, int x, int y, unsigned heading, int8_t *ox, int8_t *oy, uint8_t *oheading);
/* Walls seen at a reading (bit 0 front, 1 left, 2 right) from a cell entered
 * with `heading`, as N1 E2 S4 W8 bits of the Ariane frame. */
uint8_t fw_ariane_walls(const nm_map_t *, unsigned heading, uint8_t seen);
#endif
