/*
 * link_params.h
 *
 * Parameters the application can read and set (PARAM_LIST, PARAM_GET,
 * PARAM_SET). The generic part is pure C: lookup by id, clamping to the
 * bounds, rounding of integers and booleans, PARAM_INFO / PARAM_VALUE
 * filling. The robot table lives in link_params_table.c.
 * ParamValue.status carries a CmdStatus: DONE when the value is applied,
 * INVALID for an unknown id, REFUSED for a read-only parameter or a value
 * that is not a number.
 */

#ifndef LINK_PARAMS_H_
#define LINK_PARAMS_H_

#include <stdint.h>

#include "middleware/link/ariane_link_schema.h"

typedef struct
{
    uint16_t id;
    const char *name;       /* at most 15 characters */
    uint8_t type;           /* enum ParamType */
    uint8_t flags;          /* enum ParamFlags */
    float min_value;
    float max_value;
    float (*get)(void);
    void (*set)(float value);
} link_param_t;

typedef struct
{
    const link_param_t *table;
    unsigned count;
} link_params_t;

void link_params_init(link_params_t *params, const link_param_t *table, unsigned count);
unsigned link_params_count(const link_params_t *params);
const link_param_t *link_params_find(const link_params_t *params, uint16_t id);
/* Fills out for the parameter at index; returns 0 when index is out of range. */
int link_params_info(const link_params_t *params, unsigned index, link_param_info_t *out);
/* PARAM_GET: out->status tells the result. */
void link_params_get(const link_params_t *params, uint16_t id, link_param_value_t *out);
/* PARAM_SET: the value is clamped and rounded, out holds what was applied. */
void link_params_set(const link_params_t *params, uint16_t id, float value, link_param_value_t *out);
/* The value a parameter would take: clamped to the bounds, rounded for
 * integers, 0 or 1 for booleans. */
float link_params_clamp(const link_param_t *param, float value);

#endif /* LINK_PARAMS_H_ */
