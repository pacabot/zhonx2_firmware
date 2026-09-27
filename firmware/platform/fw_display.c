#include "fw_display.h"
#include "oled/ssd1306.h"
static void draw(unsigned x,unsigned y,const char *text,unsigned height)
{
    for(;*text && x+7<=128 && y+height<=64;++text,x+=8) {
        unsigned c=(unsigned char)*text;
        if(c<32 || c>127)c='?';
        for(unsigned col=0;col<7;++col) {
            unsigned bits=Font_7x8.au8FontTable[(c-32)*7+col];
            for(unsigned row=0;row<height;++row)
                if(bits&(1u<<(row*8/height)))ssd1306DrawPixel(x+col,y+row);
        }
    }
}
void fw_display_text(unsigned x,unsigned y,const char *text) {draw(x,y,text,12);}
void fw_display_large(unsigned x,unsigned y,const char *text) {draw(x,y,text,16);}
void fw_display_scroll(unsigned index,unsigned count)
{
    if(count<2)return;
    unsigned h=48/count;if(h<5)h=5;
    if(index>=count)index=count-1;
    ssd1306DrawLine(126,16,126,63);
    ssd1306FillRect(125,16+index*(48-h)/(count-1),3,h);
}
