#include "fw_sound.h"
#include "stm32f4xx.h"
#include "app/app_def.h"
#include "config/config.h"
#include "hal/hal_os.h"
#include "hal/hal_beeper.h"
typedef struct {unsigned hz,ms;} note_t;
typedef struct {const note_t *notes;unsigned count,priority;} tune_t;
static const note_t failure[]={{2794,180},{2349,180},{1865,320},{0,50}};
static const note_t victory[]={{2093,110},{2637,110},{3136,110},{4186,300},{0,50}};
static const note_t click[]={{2700,12},{0,5}};
static const note_t select_[]={{2349,25},{3136,40},{0,5}};
static const note_t back[]={{3136,25},{2349,40},{0,5}};
static const note_t connect[]={{2093,70},{2794,70},{3520,120},{0,20}};
static const note_t disconnect[]={{3520,70},{2794,70},{2093,120},{0,20}};
#define TUNE(n,p) {n,sizeof n/sizeof *n,p}
static const tune_t tunes[FW_SOUND_COUNT]={TUNE(failure,2),TUNE(victory,2),TUNE(click,1),
    TUNE(select_,1),TUNE(back,1),TUNE(connect,2),TUNE(disconnect,2)};
static const tune_t *volatile song;
static unsigned index,count;
static uint32_t until;
void fw_sound_effect(unsigned sound)
{
    if(sound>=FW_SOUND_COUNT || !fw_sound_level())return;
    uint32_t mask=__get_PRIMASK();__disable_irq();
    if(!song || tunes[sound].priority>=song->priority) {
        song=&tunes[sound];index=0;count=song->count;until=hal_os_get_systicks();
    }
    __set_PRIMASK(mask);
}
void fw_sound_play(int success) {fw_sound_effect(success?FW_SOUND_VICTORY:FW_SOUND_FAILURE);}
int fw_sound_busy(void) {return song!=0;}
void fw_sound_tick(uint32_t now)
{
    if(!song || (int32_t)(now-until)<0)return;
    if(index==count) {hal_beeper_tone(app_context.beeper,0);song=0;return;}
    hal_beeper_tone(app_context.beeper,song->notes[index].hz);
    until=now+song->notes[index++].ms;
}
unsigned fw_sound_level(void)
{
    unsigned code=zhonxSettings.beeper_enabled;
    return code==0?FW_SOUND_OFF:code==2?FW_SOUND_MEDIUM:code==3?FW_SOUND_LOW:FW_SOUND_HIGH;
}
void fw_sound_set_level(unsigned level)
{
    zhonxSettings.beeper_enabled=(unsigned char)(level==FW_SOUND_OFF?0:level==FW_SOUND_LOW?3:level==FW_SOUND_MEDIUM?2:1);
}
