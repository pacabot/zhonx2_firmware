#ifndef FW_LINK_H
#define FW_LINK_H
#include <stdint.h>
#include "nimes.h"
/* Ariane Link on ZHONX II (ariane-tools/link/PROTOCOL.md) over the HM-18 BLE
 * module, see docs/ARIANE.md. Contexts:
 *  - SysTick (1 kHz): fw_link_tick() samples the telemetry and pends PendSV;
 *  - PendSV (lowest priority): the link proper. It reads the module, answers
 *    the application, queues the long commands as a job and drains the rings
 *    into the module. ZHONX II has no main loop: every screen is a blocking
 *    loop, so the link cannot wait for the foreground;
 *  - foreground: the menu runs a queued job (fw_link_job_run) when it waits
 *    for keys, and the exploration / run code calls the hooks below.
 * The remote STOP sets fw_link_stop_flag (fw_buttons.h), which every
 * fw_cancel_pressed() checkpoint honours like the Escape button. */

/* MOVE_BEGIN kinds, as on ZHONX III: MoveKind for straight moves,
 * 0x10 | TurnKind for turns. */
#define FW_LINK_MOVE_CELLS 0x00
#define FW_LINK_MOVE_TURN  0x10
#define FW_LINK_MOVE_RESET_START 0x20
/* EVENT STOP reasons, as on ZHONX III: remote STOP, emergency (motion fault). */
#define FW_LINK_STOP_REMOTE 1
#define FW_LINK_STOP_EMERGENCY 2
/* fw_link_turn_begin(curve): in place, 90-degree curve, U-turn. */
enum { FW_LINK_IN_PLACE, FW_LINK_CURVE, FW_LINK_UTURN };

void fw_link_init(void);
void fw_link_tick(uint32_t now);
void PendSV_Handler(void);

/* Menu: idle while choose() waits for a key; remote jobs run only then. */
void fw_link_menu_idle(int idle);
int fw_link_job_pending(void);
void fw_link_job_run(void);

int fw_link_session(void);
int fw_link_connected(void);
/* Autonomous trials: the module stays silent unless a session is active. */
void fw_link_quiet(int quiet);

/* wait_hand(): light signal and events before a start. Returns -1 when a
 * STOP arrived, 1 when the remote job asked for no hand wave, 0 to wait for
 * the hand as usual, then call fw_link_hand_go(). */
int fw_link_start_gate(void);
void fw_link_hand_go(void);

/* Exploration and runs, in the ZHONX III frame (docs/ARIANE.md, "Repère"):
 * origin at the start cell, NORTH = heading at the start, x east, y SOUTH.
 * fw_link_entered(): the robot crossed the entry edge of this cell; the CELL
 * event of that cell is dated there, as ZHONX III reads its walls just before
 * the edge. fw_link_rebase(): the map origin moved by (dx, dy) cells.
 * fw_link_cell(): walls seen at that reading, bit 0 front, 1 left, 2 right.
 * fw_link_decision(): before a segment, heading before it and its last cell. */
void fw_link_entered(nm_pose_t cell);
void fw_link_rebase(int dx, int dy);
void fw_link_cell(const nm_map_t *, nm_pose_t, uint8_t seen);
void fw_link_decision(const nm_map_t *, unsigned from_heading, nm_pose_t target);
void fw_link_move_begin(uint8_t kind, int16_t param, uint16_t speed_mm_s);
/* Turn in place (90, -90, 180, clockwise positive), 90-degree curve, U-turn. */
void fw_link_turn_begin(int clockwise_degrees, int curve, unsigned speed_mm_s);
void fw_link_move_end(int stopped);
void fw_link_run_done(uint32_t ms, int failed);

/* Parameter accessors (fw_link_params.c). RUN 1 speed when the command says 0
 * (run1_max_mm_s, not saved: run 2 uses the Fast run setting); 0 until set =
 * the exploration speed, as the RUNS menu. */
extern int fw_link_run1_speed;
int fw_link_telemetry_rate(void);
void fw_link_set_telemetry_rate(int rate_hz);
int fw_link_blip_period(void);
void fw_link_set_blip_period(int period_ms);

/* Hardware tests > Bluetooth. */
typedef struct {
    int session, telemetry_hz;
    uint32_t frames_rx, crc_errors, replies_dropped, logs_dropped, telemetry_dropped, events_dropped, build_id;
} fw_link_status_t;
void fw_link_status(fw_link_status_t *);
#endif
