#include "fw_text.h"
#include <string.h>
const char *fw_text_line(const char *text, char *out, size_t columns)
{
    size_t n=0,last_space=0;
    if (!columns) { out[0]=0; return text; }
    while (*text==' ') ++text;
    while (text[n] && text[n]!='\n' && n<columns) {
        if (text[n]==' ') last_space=n;
        ++n;
    }
    size_t take=n;
    if (text[n] && text[n]!='\n' && text[n]!=' ' && last_space) take=last_space;
    memcpy(out,text,take); out[take]=0;
    text+=take;
    while (*text==' ') ++text;
    if (*text=='\n') ++text;
    return text;
}
