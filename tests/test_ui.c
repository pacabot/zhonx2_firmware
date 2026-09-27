#include "fw_ui.h"
#include "fw_battery.h"
#include "fw_menu.h"
#include "oled/ssd1306.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
extern unsigned char buffer[1024];
static unsigned adc=3000,now;
void fw_battery_poll(void) {fw_battery_sample(adc,now,0);}
static void charge(unsigned raw) {
    adc=raw;fw_battery_set_reference((fw_battery_reference_t){3000,8400});
    for(unsigned i=0;i<7;++i) {now+=1000;fw_battery_poll();}
}
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
    fw_ui_card("ZHONX II","Calibration","",FW_ICON_CALIBRATE,1,4);image("build/ui-menu-home.pgm");
    fw_ui_card("MAZE","Load maze","Library",FW_ICON_MAZE,1,4);image("build/ui-menu-maze.pgm");
    fw_ui_card("RUNS","Slow run","120 mm/s",FW_ICON_RUN,0,4);image("build/ui-menu-run.pgm");
    const char *cal_names[]={"Walls","Rotation","Left edge","Right edge"};
    const unsigned cal_icons[]={FW_ICON_WALL,FW_ICON_TURN,FW_ICON_EDGE_LEFT,FW_ICON_EDGE_RIGHT};
    for(unsigned i=0;i<4;++i) {
        char file[60];fw_ui_card("CALIBRATION",cal_names[i],"Calibrate",cal_icons[i],i,7);
        snprintf(file,sizeof file,"build/ui-calibration-icon-%u.pgm",i);image(file);
    }
    const int values[]={470,940,167,179,173,220,260,0,0};
    for(unsigned i=0;i<9;++i) {char file[60];snprintf(file,sizeof file,"build/ui-setting-%u.pgm",i);
        fw_ui_setting(i,values[i]);image(file);}
    fw_saved_maze_t saved={.map=map,.route={.direction={0,0,1,1},.length=4},.id=7};
    fw_ui_library(&saved,0,2,0);image("build/ui-library-off.pgm");
    unsigned char off[1024];memcpy(off,buffer,sizeof off);
    fw_ui_library(&saved,0,2,1);image("build/ui-library-on.pgm");
    assert(memcmp(off,buffer,sizeof off));
    fw_ui_card("HARDWARE TESTS","Telemeters","Live view",FW_ICON_TEST,1,7);
    image("build/ui-hardware-menu.pgm");
    unsigned char before_battery[1024];memcpy(before_battery,buffer,sizeof buffer);
    charge(3000);fw_ui_menu_refresh();image("build/ui-battery-full.pgm");
    charge(2357);fw_ui_menu_refresh();image("build/ui-battery-empty.pgm");
    for(unsigned y=0;y<64;++y)for(unsigned x=0;x<96;++x)
        assert(ssd1306GetPixel(x,y)==!!(before_battery[x+(y/8)*128]&(1u<<(y%8))));
    fw_battery_set_reference((fw_battery_reference_t){0});fw_ui_menu_refresh();image("build/ui-battery-unknown.pgm");
    fw_ui_card("METER VOLTAGE / 2S","Meter volts","8.40 V",FW_ICON_SETTINGS,0,1);
    fw_ui_hint("UP/DN  OK:SAVE");image("build/ui-battery-setup.pgm");
    fw_ui_card("ZHONX II","Maze","",FW_ICON_MAZE,0,5);
    for(unsigned y=0;y<10;++y)for(unsigned x=0;x<96;++x)assert(!ssd1306GetPixel(x,y));
    image("build/ui-menu-no-header.pgm");
    fw_ui_card("DELETE THIS MAZE?","Delete maze","Confirm",FW_ICON_REPORT,1,2);
    image("build/ui-confirm-delete.pgm");
    fw_battery_set_reference((fw_battery_reference_t){0});
    fw_ui_idle_start(0x12345678);
    fw_ui_idle(0);image("build/ui-idle-0.pgm");memcpy(off,buffer,sizeof off);
    charge(3000);fw_ui_idle(0);
    assert(!memcmp(off,buffer,sizeof off)); /* No percentage over the animation. */
    fw_ui_idle(600);assert(!memcmp(off,buffer,sizeof off));
    fw_ui_idle(2000);image("build/ui-idle-2000.pgm");assert(memcmp(off,buffer,sizeof off));
    fw_ui_idle(4699);unsigned char scattered[1024];memcpy(scattered,buffer,1024);
    fw_ui_idle(4700);assert(!memcmp(scattered,buffer,1024)); /* Symmetric poses: no jump when reforming. */
    fw_ui_idle(8000);image("build/ui-idle-assembled.pgm");assert(!memcmp(off,buffer,1024));
    extern unsigned fw_idle_test_choice(void);
    unsigned seen=0,previous=8;
    for(unsigned cycle=0;cycle<24;++cycle) {
        fw_ui_idle(cycle*10000+2000);
        unsigned selected=fw_idle_test_choice();assert(selected<8 && selected!=previous);
        assert(!(seen&(1u<<selected)));seen|=1u<<selected;previous=selected;
        if(cycle%8==7) {assert(seen==255);seen=0;}
        fw_ui_idle(cycle*10000+8000);assert(!memcmp(off,buffer,1024));
        fw_ui_idle(cycle*10000+9999);assert(!memcmp(off,buffer,1024));
        fw_ui_idle((cycle+1)*10000);assert(!memcmp(off,buffer,1024)); /* Seamless change of choreography. */
    }
    previous=fw_idle_test_choice();fw_ui_idle_start(73241);fw_ui_idle(0);
    assert(fw_idle_test_choice()!=previous); /* Bag survives wake-up, no immediate repeat. */
    assert(!memcmp(off,buffer,1024));
    FILE *frames=fopen("build/ui-idle-frames.raw","wb");assert(frames);
    for(unsigned frame=0;frame<2400;++frame) {
        unsigned time=frame*10000/300;
        fw_ui_idle(time);
        if(frame%300==60 || frame%300==120 || frame%300==180) {
            char path[80];snprintf(path,sizeof path,"build/ui-idle-variant-%u-%u.pgm",frame/300,frame%300);
            image(path);
        }
        assert(fwrite(buffer,1,sizeof buffer,frames)==sizeof buffer);
    }
    fclose(frames);
    puts("Idle: eight shuffled choreographies, no consecutive repeats, seamless transitions, full-screen frames");
    charge(2700);fw_ui_card("HOME","Maze","",FW_ICON_MAZE,0,5);image("build/ui-battery-partial.pgm");
    puts("OLED: large icon cards, nine illustrated settings, blinking stored route");
    return 0;
}
