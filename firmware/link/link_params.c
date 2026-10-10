/*
 * link_params.c
 *
 * Generic parameter handling, see link_params.h.
 */

#include <math.h>
#include <string.h>

#include "middleware/link/link_params.h"

void link_params_init(link_params_t *params, const link_param_t *table, unsigned count)
{
    params->table = table;
    params->count = count;
}

unsigned link_params_count(const link_params_t *params)
{
    return params->count;
}

const link_param_t *link_params_find(const link_params_t *params, uint16_t id)
{
    for (unsigned i = 0; i < params->count; i++)
    {
        if (params->table[i].id == id)
            return &params->table[i];
    }
    return NULL;
}

float link_params_clamp(const link_param_t *param, float value)
{
    if (value < param->min_value)
        value = param->min_value;
    if (value > param->max_value)
        value = param->max_value;
    if (param->type == LINK_PARAM_TYPE_BOOL)
        return value >= 0.5f ? 1.0f : 0.0f;
    if (param->type == LINK_PARAM_TYPE_I32)
        return (float)lroundf(value);
    return value;
}

int link_params_info(const link_params_t *params, unsigned index, link_param_info_t *out)
{
    const link_param_t *param;

    if (index >= params->count)
        return 0;
    param = &params->table[index];
    memset(out, 0, sizeof *out);
    out->id = param->id;
    out->type = param->type;
    out->flags = param->flags;
    strncpy(out->name, param->name, sizeof out->name - 1U);
    out->min_value = param->min_value;
    out->max_value = param->max_value;
    out->value = param->get != NULL ? param->get() : 0.0f;
    out->count = (uint16_t)params->count;
    out->index = (uint16_t)index;
    return 1;
}

void link_params_get(const link_params_t *params, uint16_t id, link_param_value_t *out)
{
    const link_param_t *param = link_params_find(params, id);

    memset(out, 0, sizeof *out);
    out->id = id;
    if (param == NULL)
    {
        out->status = LINK_CMD_STATUS_INVALID;
        return;
    }
    out->status = LINK_CMD_STATUS_DONE;
    out->type = param->type;
    out->value = param->get != NULL ? param->get() : 0.0f;
}

void link_params_set(const link_params_t *params, uint16_t id, float value, link_param_value_t *out)
{
    const link_param_t *param = link_params_find(params, id);

    memset(out, 0, sizeof *out);
    out->id = id;
    if (param == NULL)
    {
        out->status = LINK_CMD_STATUS_INVALID;
        return;
    }
    out->type = param->type;
    if ((param->flags & LINK_PARAM_FLAGS_READ_ONLY) != 0 || param->set == NULL || isnan(value))
    {
        out->status = LINK_CMD_STATUS_REFUSED;
        out->value = param->get != NULL ? param->get() : 0.0f;
        return;
    }
    value = link_params_clamp(param, value);
    param->set(value);
    out->status = LINK_CMD_STATUS_DONE;
    out->value = param->get != NULL ? param->get() : value;
}
