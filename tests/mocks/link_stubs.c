/* Host tests of the platform files without the Ariane link nor the sound
 * engine. Weak: a test that observes one of these defines its own. */
#include "fw_bt.h"
#include "fw_link.h"
#include "fw_motion.h"
#include "fw_sound.h"
#include "config/config.h"
#include <string.h>
#define WEAK __attribute__((weak))
WEAK robot_settings zhonxSettings;
WEAK void fw_link_menu_idle(int idle) {(void)idle;}
WEAK int fw_link_job_pending(void) {return 0;}
WEAK void fw_link_job_run(void) {}
WEAK int fw_link_session(void) {return 0;}
WEAK int fw_link_connected(void) {return 0;}
WEAK void fw_link_quiet(int quiet) {(void)quiet;}
WEAK int fw_link_start_gate(void) {return 0;}
WEAK void fw_link_hand_go(void) {}
WEAK void fw_link_entered(nm_pose_t c) {(void)c;}
WEAK void fw_link_rebase(int dx,int dy) {(void)dx;(void)dy;}
WEAK void fw_link_cell(const nm_map_t *m,nm_pose_t p,uint8_t s) {(void)m;(void)p;(void)s;}
WEAK void fw_link_decision(const nm_map_t *m,unsigned h,nm_pose_t t) {(void)m;(void)h;(void)t;}
WEAK void fw_link_move_begin(uint8_t k,int16_t p,uint16_t s) {(void)k;(void)p;(void)s;}
WEAK void fw_link_turn_begin(int d,int c,unsigned s) {(void)d;(void)c;(void)s;}
WEAK void fw_link_move_end(int s) {(void)s;}
WEAK void fw_link_run_done(uint32_t ms,int f) {(void)ms;(void)f;}
WEAK int fw_motion_path_segment(int *turn,uint32_t *length) {(void)turn;(void)length;return -1;}
WEAK void fw_link_status(fw_link_status_t *s) {memset(s,0,sizeof *s);}
WEAK void fw_bt_status(fw_bt_status_t *s) {memset(s,0,sizeof *s);s->state="absent";s->name=s->address="";}
WEAK void fw_sound_play(int success) {(void)success;}
WEAK void fw_sound_effect(unsigned sound) {(void)sound;}
WEAK unsigned fw_sound_level(void) {return FW_SOUND_HIGH;}
WEAK void fw_sound_set_level(unsigned level) {(void)level;}
