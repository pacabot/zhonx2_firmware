#ifndef FW_BUTTONS_H
#define FW_BUTTONS_H
#include "stm32f4xx_gpio.h"
/* Joystick left is Back. Separate PC13 button is Escape / emergency stop. */
#define FW_BACK_PIN (1u<<8)
#define FW_UP_PIN (1u<<9)
#define FW_DOWN_PIN (1u<<10)
#define FW_SELECT_PINS ((1u<<11)|(1u<<12))
#define FW_ESCAPE_PIN (1u<<13)
/* Ariane STOP (fw_link.c): set from the link interrupt, cleared when a new
 * activity starts. Weak so that host tests without the link still link. */
__attribute__((weak)) volatile unsigned char fw_link_stop_flag;
static inline int fw_back_pressed(void) { return !(GPIOC->IDR&FW_BACK_PIN); }
static inline int fw_cancel_button(void)
{ return (GPIOC->IDR&(FW_BACK_PIN|FW_ESCAPE_PIN))!=(FW_BACK_PIN|FW_ESCAPE_PIN); }
/* Every stop checkpoint of motion, exploration, runs and tests. */
static inline int fw_cancel_pressed(void) { return fw_cancel_button() || fw_link_stop_flag; }
static inline int fw_select_pressed(void)
{ return (GPIOC->IDR&FW_SELECT_PINS)!=FW_SELECT_PINS; }
#endif
