#include "wall_control.h"
void wall_control_reset(wall_control_t *c) { c->filtered = c->previous = c->output = 0; }
int wall_control_step(wall_control_t *c, uint8_t s)
{
    int left = !(s & 0x20), right = !(s & 0x01);
    int near_left = !(s & 0x10), near_right = !(s & 0x02);
    int error = 0;
    if (near_left != near_right) error = near_left ? 1024 : -1024;
    else if (left != right) error = left ? -256 : 256;
    /* No distance information in a doorway: decay rather than integrate noise. */
    if (!left && !right) error = 0;
    c->filtered += (error - c->filtered) / 4;
    int desired = (80 * c->filtered + 25 * (c->filtered - c->previous)) / 1024;
    c->previous = c->filtered;
    if (desired > 120) desired = 120;
    if (desired < -120) desired = -120;
    int delta = desired - c->output;
    if (delta > 8) delta = 8;
    if (delta < -8) delta = -8;
    c->output += delta;
    return c->output;
}
