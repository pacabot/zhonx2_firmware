/*
 * link_line.h
 *
 * Text line assembly for the legacy Bluetooth console, with the semantics
 * the USART3 interrupt had before Ariane Link: CR ends the line (it is
 * kept in the buffer, followed by a zero), LF empties it, backspace deletes
 * the previous character, other bytes are stored while room remains. The
 * bytes received outside Ariane Link frames feed it.
 */

#ifndef LINK_LINE_H_
#define LINK_LINE_H_

#include <stddef.h>

typedef struct
{
    char *buffer;
    size_t capacity;
    size_t length;
} link_line_t;

#define LINK_LINE_CR 0x0D
#define LINK_LINE_LF 0x0A
#define LINK_LINE_BS 0x08

void link_line_init(link_line_t *line, char *buffer, size_t capacity);
/* Returns the length of the completed line, CR included, or 0. */
size_t link_line_feed(link_line_t *line, unsigned char byte);

#endif /* LINK_LINE_H_ */
