#include "fw_sound.h"
#include "app/app_def.h"
#include "config/config.h"
#include "hal/hal_beeper.h"
#include <assert.h>
#include <stdio.h>
app_config app_context;
robot_settings zhonxSettings;
static unsigned now,calls;
static long frequencies[16];
unsigned long hal_os_get_systicks(void) {return now;}
int hal_beeper_tone(HAL_BEEPER_HANDLE h,long freq) {(void)h;assert(calls<16);frequencies[calls++]=freq;return 0;}
static void play_out(void) {for(unsigned i=0;i<800;++i,++now)fw_sound_tick(now);}
int main(void)
{
    zhonxSettings.beeper_enabled=1;
    assert(fw_sound_level()==FW_SOUND_HIGH);
    fw_sound_play(1);
    for(now=0;now<800;++now)fw_sound_tick(now);
    assert(calls==6 && frequencies[0]==2093 && frequencies[3]==4186 && frequencies[5]==0);
    /* Every tone sits in the loud band of the SMT-1027 buzzer. */
    for(unsigned i=0;i<5;++i)assert(frequencies[i]==0 || (frequencies[i]>=1800 && frequencies[i]<=4200));
    calls=0;now=0xfffffff0u;fw_sound_play(0);play_out();
    assert(calls==5 && frequencies[0]==2794 && frequencies[2]==1865 && frequencies[4]==0);
    calls=0;fw_sound_play(1);fw_sound_tick(now++);fw_sound_play(0);fw_sound_tick(now);
    assert(calls==2 && frequencies[1]==2794);
    play_out();
    /* A menu click never cuts a tune, a tune replaces a click. */
    calls=0;fw_sound_play(1);fw_sound_tick(now++);fw_sound_effect(FW_SOUND_CLICK);fw_sound_tick(now+200);
    assert(calls==2 && frequencies[1]==2637);
    play_out();
    calls=0;fw_sound_effect(FW_SOUND_CLICK);fw_sound_tick(now++);fw_sound_effect(FW_SOUND_CONNECT);fw_sound_tick(now);
    assert(calls==2 && frequencies[0]==2700 && frequencies[1]==2093);
    play_out();
    /* Volume levels round-trip through the saved byte; earlier snapshots hold 1 = high. */
    for(unsigned level=FW_SOUND_OFF;level<=FW_SOUND_HIGH;++level) {fw_sound_set_level(level);assert(fw_sound_level()==level);}
    fw_sound_set_level(FW_SOUND_HIGH);assert(zhonxSettings.beeper_enabled==1);
    fw_sound_set_level(FW_SOUND_OFF);calls=0;fw_sound_effect(FW_SOUND_SELECT);fw_sound_play(1);play_out();
    assert(calls==0);
    puts("sound: tunes at the buzzer resonance, click/tune priority, volume levels, mute and tick wrap");
}
