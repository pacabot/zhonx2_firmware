/*
 * link_line.c
 *
 * Console line assembly, see link_line.h.
 */

#include "middleware/link/link_line.h"

void link_line_init(link_line_t *line, char *buffer, size_t capacity)
{
    line->buffer = buffer;
    line->capacity = capacity;
    line->length = 0;
}

size_t link_line_feed(link_line_t *line, unsigned char byte)
{
    switch (byte)
    {
        case LINK_LINE_CR:
        {
            size_t length;
            if (line->length + 2U > line->capacity)
                line->length = line->capacity - 2U;
            line->buffer[line->length] = (char)LINK_LINE_CR;
            line->buffer[line->length + 1U] = '\0';
            length = line->length + 1U;
            line->length = 0;
            return length;
        }
        case LINK_LINE_LF:
            line->length = 0;
            return 0;
        case LINK_LINE_BS:
            if (line->length > 0)
                line->length--;
            return 0;
        default:
            if (line->length + 2U < line->capacity)
                line->buffer[line->length++] = (char)byte;
            return 0;
    }
}
