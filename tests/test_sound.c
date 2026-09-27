#include "fw_sound.h"
#include "app/app_def.h"
#include "hal/hal_beeper.h"
#include <assert.h>
#include <stdio.h>
app_config app_context;
static unsigned now,calls;
static long frequencies[16];
unsigned long hal_os_get_systicks(void) {return now;}
int hal_beeper_tone(HAL_BEEPER_HANDLE h,long freq) {(void)h;assert(calls<16);frequencies[calls++]=freq;return 0;}
int main(void)
{
    fw_sound_play(1);
    for(now=0;now<800;++now)fw_sound_tick(now);
    assert(calls==6 && frequencies[0]==784 && frequencies[3]==1568 && frequencies[5]==0);
    calls=0;now=0xfffffff0u;fw_sound_play(0);
    for(unsigned i=0;i<800;++i,++now)fw_sound_tick(now);
    assert(calls==5 && frequencies[0]==587 && frequencies[2]==294 && frequencies[4]==0);
    calls=0;fw_sound_play(1);fw_sound_tick(now++);fw_sound_play(0);fw_sound_tick(now);
    assert(calls==2 && frequencies[1]==587);
    puts("sound: nonblocking victory/failure sequences, replacement and tick wrap");
}
