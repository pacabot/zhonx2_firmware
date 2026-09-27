/*---------------------------------------------------------------------------
 *
 *      hal_beeper.h
 *
 *---------------------------------------------------------------------------*/

#ifndef __HAL_BEEPER_H__
#define __HAL_BEEPER_H__

/* Module Identifier */
#define HAL_BEEPER_MODULE_ID    2

/* Error codes */
#define HAL_BEEPER_E_SUCCESS    0
#define HAL_BEEPER_E_ERROR      MAKE_ERROR(HAL_BEEPER_MODULE_ID, 1)
#define HAL_BEEPER_E_BAD_HANDLE MAKE_ERROR(HAL_BEEPER_MODULE_ID, 2)

/* Beeper States */
#define HAL_BEEPER_STATE_OFF    0
#define HAL_BEEPER_STATE_ON     1

/* HAL BEEPER handle */
typedef void *HAL_BEEPER_HANDLE;

/* Exported functions */
int hal_beeper_init(void);
int hal_beeper_terminate(void);

int hal_beeper_open(HAL_BEEPER_HANDLE *handle, void *params);
int hal_beeper_close(HAL_BEEPER_HANDLE handle);

int hal_beeper_get_state(HAL_BEEPER_HANDLE handle, int *state);
int hal_beeper_set_state(HAL_BEEPER_HANDLE handle, int state);
int hal_beeper_tone(HAL_BEEPER_HANDLE handle, long freq);
int hal_beeper_beep(HAL_BEEPER_HANDLE handle, long freq, long duration);

#endif // __HAL_BEEPER_H__
