#include "fw_start.h"
int fw_start_gate(fw_start_gate_t *g, uint32_t now, int present, int fresh)
{
    if (!fresh) { g->timing=0; return 0; }
    if (g->state==FW_START_READY) return 1;
    int expected=g->state==FW_WAIT_HAND;
    if (!!present!=expected) { g->timing=0; return 0; }
    if (!g->timing) { g->timing=1; g->since=now; }
    if (now-g->since >= (g->state==FW_WAIT_HAND ? 200u : 100u)) {
        ++g->state; g->timing=0;
    }
    return g->state==FW_START_READY;
}
