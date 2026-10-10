#ifndef FW_SOUND_H
#define FW_SOUND_H
#include <stdint.h>
/* Non-blocking tunes, advanced by fw_sound_tick() from SysTick. The buzzer
 * (SMT-1027-S-R, magnetic) resonates near 2.7 kHz: every tone stays within
 * 1.8-4.2 kHz, where it is 10 to 15 dB louder than below 1.5 kHz. */
enum { FW_SOUND_FAILURE, FW_SOUND_VICTORY, FW_SOUND_CLICK, FW_SOUND_SELECT, FW_SOUND_BACK,
       FW_SOUND_CONNECT, FW_SOUND_DISCONNECT, FW_SOUND_COUNT };
/* Volume levels shown in Settings > Sound. */
enum { FW_SOUND_OFF, FW_SOUND_LOW, FW_SOUND_MEDIUM, FW_SOUND_HIGH };
void fw_sound_play(int success);
/* Menu feedback never interrupts a tune (victory, failure, connection). */
void fw_sound_effect(unsigned sound);
void fw_sound_tick(uint32_t now);
int fw_sound_busy(void);
/* The level lives in zhonxSettings.beeper_enabled, saved with the settings:
 * 0 off, 1 high (every earlier snapshot holds 1), 2 medium, 3 low. */
unsigned fw_sound_level(void);
void fw_sound_set_level(unsigned level);
#endif
