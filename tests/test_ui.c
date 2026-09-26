#include "fw_ui.h"
#include "oled/ssd1306.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
extern unsigned char buffer[1024];
int HAL_Delay(unsigned long ms) { (void)ms;return 0; }
static void image(const char *file)
{
    FILE *f=fopen(file,"wb");assert(f);fputs("P5\n128 64\n255\n",f);
    for(unsigned y=0;y<64;++y) for(unsigned x=0;x<128;++x)
        fputc(ssd1306GetPixel(x,y)?255:0,f);
    fclose(f);
}
int main(void)
{
    /* Long menu text at the right edge must not wrap at x=256 into the map. */
    char long_text[301];memset(long_text,'M',300);long_text[300]=0;
    ssd1306ClearScreen();ssd1306DrawString(110,0,long_text,&Font_5x8);
    for(unsigned y=0;y<64;++y) for(unsigned x=0;x<110;++x) assert(!ssd1306GetPixel(x,y));
    ssd1306ClearScreen();ssd1306DrawString(256,0,"BAD",&Font_5x8);
    for(unsigned i=0;i<1024;++i) assert(!buffer[i]);
    ssd1306DrawTextBox(6,18,116,30,"REGLAGES ET CARTE SAUVES SANS ERREUR",&Font_5x8);
    for(unsigned y=0;y<64;++y) for(unsigned x=0;x<128;++x)
        if(x<6 || x>=122 || y<18 || y>=48) assert(!ssd1306GetPixel(x,y));
    image("build/ui-prompt.pgm");
    nm_map_t map;nm_init(&map);
    assert(!nm_observe(&map,(nm_pose_t){0,0,NM_NORTH},6));
    assert(!nm_observe(&map,(nm_pose_t){0,1,NM_NORTH},6));
    assert(!nm_observe(&map,(nm_pose_t){0,2,NM_NORTH},3));
    assert(!nm_observe(&map,(nm_pose_t){1,2,NM_EAST},6));
    assert(!nm_edge(&map,20,NM_EAST,0));assert(!nm_edge(&map,20,NM_NORTH,0));
    assert(!nm_edge(&map,21,NM_NORTH,0));assert(!nm_edge(&map,29,NM_EAST,0));
    for(unsigned y=0;y<9;++y) for(unsigned x=0;x<9;++x)
        fw_ui_maze(&map,(nm_pose_t){x,y,(x+y)%4},"EXPLORATION",220,72000,0x37);
    fw_ui_maze(&map,(nm_pose_t){2,2,NM_NORTH},"EXPLORATION",220,72000,0x37);
    image("build/ui-maze.pgm");
    puts("OLED: actual renderer, clipped menu text, wrapped prompts, all 81 map positions");
    return 0;
}
