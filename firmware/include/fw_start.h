#ifndef FW_START_H
#define FW_START_H
#include <stdint.h>
enum { FW_WAIT_HAND, FW_WAIT_RELEASE, FW_START_READY };
typedef struct { uint32_t since; unsigned state; int timing; } fw_start_gate_t;
/* Call on fresh sensor scans; presence 200 ms, then clearance 100 ms. */
int fw_start_gate(fw_start_gate_t *gate, uint32_t now, int present, int fresh);
#endif
