#ifndef FW_SOUND_H
#define FW_SOUND_H
#include <stdint.h>
void fw_sound_play(int success);
void fw_sound_tick(uint32_t now);
#endif
