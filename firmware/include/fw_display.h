#ifndef FW_DISPLAY_H
#define FW_DISPLAY_H
/* OLED text policy: 7 pixels wide, at least 12 pixels high. No tiny fallback. */
void fw_display_text(unsigned x,unsigned y,const char *text);
void fw_display_large(unsigned x,unsigned y,const char *text);
void fw_display_scroll(unsigned index,unsigned count);
#endif
