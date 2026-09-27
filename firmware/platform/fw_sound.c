#include "fw_sound.h"
#include "stm32f4xx.h"
#include "app/app_def.h"
#include "hal/hal_os.h"
#include "hal/hal_beeper.h"
typedef struct {unsigned hz,ms;} note_t;
static const note_t victory[]={{784,110},{988,110},{1175,110},{1568,300},{0,50}};
static const note_t failure[]={{587,180},{440,180},{294,320},{0,50}};
static const note_t *volatile song;
static unsigned index,count;
static uint32_t until;
void fw_sound_play(int success)
{
    uint32_t mask=__get_PRIMASK();__disable_irq();
    song=success?victory:failure;index=0;count=success?5:4;until=hal_os_get_systicks();
    __set_PRIMASK(mask);
}
void fw_sound_tick(uint32_t now)
{
    if(!song || (int32_t)(now-until)<0)return;
    if(index==count) {hal_beeper_tone(app_context.beeper,0);song=0;return;}
    hal_beeper_tone(app_context.beeper,song[index].hz);
    until=now+song[index++].ms;
}
