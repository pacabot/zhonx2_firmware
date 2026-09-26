#ifndef FW_TEXT_H
#define FW_TEXT_H
#include <stddef.h>
/* Word wrapping for small fixed-width OLED text areas. out has columns+1 bytes. */
const char *fw_text_line(const char *text, char *out, size_t columns);
#endif
