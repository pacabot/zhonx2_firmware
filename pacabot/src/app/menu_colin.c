/*
 * menu_colin.c
 *
 *  Created on: 4 déc. 2014
 *      Author: colin
 */
#include <stdio.h>
#include <string.h>
#include "config/basetypes.h"
#include "hal/hal_os.h"
#include "hal/hal_adc.h"
#include "hal/hal_beeper.h"
#include "hal/hal_step_motor.h"
#include "hal/hal_pid.h"
#include "hal/hal_serial.h"
#include "hal/hal_sensor.h"
#include "hal/hal_led.h"
#include "hal/hal_nvm.h"
#include "hal/hal_ui.h"

#include "app/menu_colin.h"
#include "hal/hal_ui.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

#include "config/basetypes.h"
#include "config/errors.h"
#include "config/config.h"
#include "hal/hal_os.h"
#include "app/app_def.h"

#include "oled/ssd1306.h"
#include "oled/smallfonts.h"

#define JOYSTICK_LEFT       GPIOC, GPIO_Pin_8
#define JOYSTICK_UP         GPIOC, GPIO_Pin_9
#define JOYSTICK_DOWN       GPIOC, GPIO_Pin_10
#define JOYSTICK_RIGHT      GPIOC, GPIO_Pin_11
#define JOYSTICK_PUSH       GPIOC, GPIO_Pin_12
#define RETURN_BUTTON       GPIOC, GPIO_Pin_13


/* Extern functions */
extern void hal_ui_anti_rebonds (GPIO_TypeDef* gpio, uint16_t gpio_pin);
extern int test_hal_adc(void);
extern int test_hal_color_sensor(void);
extern int test_hal_sensor(void);
extern int test_hal_beeper(void);
extern int test_hal_led(void);
extern int test_hal_ui(void);
//extern int test_hal_nvm(void);
extern int test_hal_step_motor(void);
extern int test_oled1(void);
extern int test_oled2(void);
extern int test_oled3(void);
extern int test_step_motor_driver(void);
extern int test_motor_rotate(void);
extern int distance_cal(void);
extern int mazeBertrand(int i, int str);
static int start_maze(void) { return mazeBertrand(0, 0); }
extern void calibrateSimple(void);
extern void test1Bezier(void);
static int test_bezier_menu(void) { test1Bezier(); return 0; }

extern int sensor_calibrate(void);
extern void* calloc_s (size_t nombre, size_t taille);

extern app_config app_context;

menuItem maze_menu =
{
		"maze menu",
		{
				{"New maze",'f',			start_maze},
                {"wall know cost",'i',      (void*)&zhonxSettings.wall_know_cost},
                {"move cost",'i',           (void*)&zhonxSettings.cell_cost},
                {"direction",'i',           (void*)&zhonxSettings.start_orientation},
				{"x finish",'i',			(void*)&zhonxSettings.x_finish_maze},
				{"y finish",'i',			(void*)&zhonxSettings.y_finish_maze},
				{"color finish",'b',		(void*)&zhonxSettings.color_sensor_enabled},
				{"calib. enabled",'b',		(void*)&zhonxSettings.calibration_enabled},
				{"calibration",'f',			(void*)calibrateSimple},
				{(char*)NULL,	0,				NULL}
		}
};
menuItem motor_menu=
{
    "MOTOR MENU",
    {
		{"Random move",'f',				null},
		{"Rotate calib.",'f',			test_motor_rotate},
		{"Distance calib.",'f',			distance_cal},
		{"Stepper motor move",'f',		test_step_motor_driver},
		{"Test moteur",'f',		test_hal_step_motor},
		{(char*)NULL,	0,				NULL}
    }
};
float toto=10,titi=10,tata=10;

menuItem testGraphicMenu =
{
    "test graphic menu",
    {
		{"Default accel :",'l',			(void*)&toto},
		{"Max speed dist:",'l',			(void*)&titi},
		{"Default accel :",'l',			(void*)&tata},
		{"graphique",'g',null},
		{(char*)NULL,	0,				NULL}
    }
};

menuItem motion_settings =
{
    "MOTION SETTINGS",
    {
		{"Initial speed :",'l',			(void*)&zhonxSettings.initial_speed},
		{"Max speed dist:",'l',			(void*)&zhonxSettings.max_speed_distance},
		{"Default accel :",'l',			(void*)&zhonxSettings.default_accel},
		{"Rotate accel  :",'l',			(void*)&zhonxSettings.rotate_accel},
		{"Emerg decel   :",'l',			(void*)&zhonxSettings.emergency_decel},
		{(char*)NULL,	0,				NULL}
    }
};
menuItem PID_settings =
{
    "PID SETTINGS",
    {
		{"Proportional  :",'l',			(void*)&zhonxSettings.correction_p},
		{"Integral      :",'l',			(void*)&zhonxSettings.correction_i},
		{"Max correction:",'l',			(void*)&zhonxSettings.max_correction},
		{(char*)NULL,	0,				NULL}
    }
};
menuItem sensor_settings =
{
    "COLOR SENSOR SETTINGS",
    {
		{"Enabled?      :",'b',			(void*)&zhonxSettings.color_sensor_enabled},
		{"Calibrate color",'f',			sensor_calibrate},
		{"Threshold val :",'l',			(void*)&zhonxSettings.threshold_color},
		{"End Greater?  :",'b',			(void*)&zhonxSettings.threshold_greater},
		{(char*)NULL,	0,				NULL}
    }
};
menuItem paramters_menu=
{
		"parameters menu",
		{
			{"Motion settings",'m',			(void*)&motion_settings},
			{"PID settings",'m',			(void*)&PID_settings},
			{"Sensor settings",'m',			(void*)&sensor_settings},
			{(char*)NULL,	0,				NULL}
//			{"Save settings",'m',			&save_settings},
//			{"Restore settings",'m',		&restore_settings}
		}
};
static const menuItem oled_menu =
{
    "OLED MENU",
    {
		{"OLED Test 1",'f',				test_oled1},
		{"OLED Test 2",'f',				test_oled2},
		{"OLED Test 3",'f',				test_oled3},
		{(char*)NULL,	0,				NULL}
    }
};
menuItem tests_menu=
{
		"test menu",
		{
				{"test bezier",'f',				test_bezier_menu},
				{"Test ADC",'f',				test_hal_adc},
				{"Test beeper",'f',				test_hal_beeper},
				{"Test OLED",'m',				(void*)&oled_menu},
				{"Test motor",'m',				(void*)&motor_menu},
				{"Test sensor",'f',				test_hal_sensor},
				{"Test LEDs",'f',				test_hal_led},
				{"Test OLEDs",'f',				test_hal_ui},
				{"Test color sensor",'f',		test_hal_color_sensor},
				{(char*)NULL,	0,				NULL}
		}
};
menuItem menu_c =
{
		"ZHONX II                 V2.0",
		{
			{"Maze menu",'m',			(void*)&maze_menu },
			{"prameters",'m',			(void*)&paramters_menu},
			{"test menu",'m',			(void*)&tests_menu},
			{"beeper enabled?",'b',		(void*)&zhonxSettings.beeper_enabled},
			{"test graph",'m',			(void*)&testGraphicMenu},
			{(char*)NULL,	0,				NULL}
		}
};
int menu_colin(menuItem menu)
{
	signed char line_screen=1;
	signed char line_menu=0;
	affiche_menu(menu,line_menu);
	ssd1306InvertArea(0, 10, 120, 10);
	hal_ui_refresh(app_context.ui);
	while (true)
	{
		// Exit Button
		if (GPIO_ReadInputDataBit(JOYSTICK_LEFT) == Bit_RESET)
		{
			// Wait until button is released
			hal_ui_anti_rebonds (JOYSTICK_LEFT);
			return 1;
		}

		// Joystick down
		if (GPIO_ReadInputDataBit(JOYSTICK_DOWN) == Bit_RESET)
		{
			hal_ui_anti_rebonds (JOYSTICK_DOWN);
            hal_beeper_beep(app_context.beeper, 4000, 10);
			if(menu.line[line_menu+1].name!=null)
			{
				line_menu++;
				line_screen++;
				if(line_screen>MAX_LINE_SCREEN)
				{
					line_screen--;
					affiche_menu(menu,line_menu-(line_screen-1));
					ssd1306InvertArea(0, line_screen*10, 120, 10);
					hal_ui_refresh(app_context.ui);
				}
				else
				{
					menu_animate((line_screen-1)*10+1, (line_screen)*10);
				}
			}
		}
		// Joystick up
		if (GPIO_ReadInputDataBit(JOYSTICK_UP) == Bit_RESET)
		{
			hal_ui_anti_rebonds (JOYSTICK_UP);
            hal_beeper_beep(app_context.beeper, 4000, 10);
			if(line_screen==1)
			{
				if(line_menu>0)
				{
					line_menu--;
					affiche_menu(menu,line_menu);
					ssd1306InvertArea(0, 10, 120, 10);
					hal_ui_refresh(app_context.ui);
				}
			}
			else
			{
				line_menu--;
				line_screen--;
				menu_animate((line_screen+1)*10-1, (line_screen)*10);
			}

		}

		// Validate button
		if(GPIO_ReadInputDataBit(JOYSTICK_RIGHT) == Bit_RESET)
		{
			hal_ui_anti_rebonds (JOYSTICK_RIGHT);
            hal_beeper_beep(app_context.beeper, 4000, 10);
			switch(menu.line[line_menu].type)
			{
				case 'b':
					hal_ui_modify_bool_param(app_context.ui,menu.line[line_menu].name,(bool*) menu.line[line_menu].param);
					break;
				case 'i':
					hal_ui_modify_int_param(app_context.ui,menu.line[line_menu].name,(int*)menu.line[line_menu].param,1);
					break;
				case 'l':
					hal_ui_modify_long_param(app_context.ui,menu.line[line_menu].name,(long*)menu.line[line_menu].param,1);
					break;
				case 'm':
					menu_colin(*(menuItem*)menu.line[line_menu].param);
					break;
				case 'f':
					if (menu.line[line_menu].param!=null)
						menu.line[line_menu].param();
					break;
				case 'g':
					graphMotorSettings((float*)menu.line[line_menu-3].param,(float*)menu.line[line_menu-2].param,(float*)menu.line[line_menu-1].param);
					break;
				default:
					break;
			}
			affiche_menu(menu,line_menu-(line_screen-1));
			ssd1306InvertArea(0,10*line_screen,120,10);
			hal_ui_refresh(app_context.ui);
		}
	}
	return -1;
}
void menu_animate(unsigned char y, unsigned char max_y)
{

    if (max_y > y)
    {
        for ( ; y <= max_y; y++)
        {
            ssd1306InvertArea(0, y - 1, 120, 10);
            ssd1306InvertArea(0, y, 120, 10);
            ssd1306Refresh();
            HAL_Delay(10);
        }
    }
    else
    {
        for ( ; y >= max_y; y--)
        {
            ssd1306InvertArea(0, y + 1, 120, 10);
            ssd1306InvertArea(0, y, 120, 10);
            ssd1306Refresh();
            HAL_Delay(10);
        }
    }
}
void affiche_menu(menuItem menu,int line)
{
	hal_ui_clear_scr(app_context.ui);
	hal_ui_display_txt(app_context.ui,0,0,menu.name);
	ssd1306DrawLine(0, 9, 128, 9);
	for (int i=0;i<MAX_LINE_SCREEN;i++)
	{
		if(menu.line[i].name!=null)
			hal_ui_display_txt(app_context.ui,0,10*i+10,menu.line[line+i].name);
			switch (menu.line[line+i].type)
			{
				case 'b':
					if(*((bool*)menu.line[i+line].param)==true)
						hal_ui_display_txt(app_context.ui,90,10*i+10,"yes");
					else
						hal_ui_display_txt(app_context.ui,90,10*i+10,"no");
					break;
				case 'i':
					ssd1306PrintInt(90,10*i+10," ",*((unsigned int*)menu.line[i+line].param),&Font_3x6);
					break;
				case 'l':
					ssd1306PrintInt(90,10*i+10," ",*((unsigned long*)menu.line[i+line].param),&Font_3x6);
					break;
				case 'f':
					hal_ui_display_txt(app_context.ui,110,i*10+10,"->");
					break;
				case 'm':
					hal_ui_display_txt(app_context.ui,115,i*10+10,">");
					break;
			}
	}
	char nmbr_item=0;
	while(menu.line[nmbr_item].name!=null)
	{
		nmbr_item++;
	}
	if (nmbr_item>MAX_LINE_SCREEN)
	{
		int heightOneItem=54/nmbr_item;
		hal_ui_fill_rect(app_context.ui,123,heightOneItem*line+10,3,MAX_LINE_SCREEN*heightOneItem);
		hal_ui_refresh(app_context.ui);
	}
}

void graphMotorSettings (float *acceleration, float *maxSpeed, float *deceleration)
{
	int number_value=0;
	float* values[3]={acceleration,maxSpeed,deceleration};
	while(true)
	{
		printGraphMotor ( *acceleration, *maxSpeed, *deceleration);
		if (GPIO_ReadInputDataBit(JOYSTICK_LEFT) == Bit_RESET)
		{
			// Wait until button is released
			hal_ui_anti_rebonds (JOYSTICK_LEFT);
			if (number_value <= 0)
			{
				return;
			}
			else
			{
				number_value--;
			}
		}

		// Joystick down
		if (GPIO_ReadInputDataBit(JOYSTICK_DOWN) == Bit_RESET)
		{
			hal_ui_anti_rebonds (JOYSTICK_DOWN);
			*(values[number_value])-=0.5;
		}
		// Joystick up
		if (GPIO_ReadInputDataBit(JOYSTICK_UP) == Bit_RESET)
		{
			hal_ui_anti_rebonds (JOYSTICK_UP);
			*(values[number_value])+=0.5;

		}

		// Validate button
		if(GPIO_ReadInputDataBit(JOYSTICK_RIGHT) == Bit_RESET)
		{
			hal_ui_anti_rebonds (JOYSTICK_RIGHT);
			if (number_value >= 2)
			{
				return;
			}
			else
			{
				number_value++;
			}
		}
	}
	hal_ui_anti_rebonds(JOYSTICK_RIGHT);
}

void printGraphMotor (float acceleration, float maxSpeed, float deceleration)
{
	char str[10];
	ssd1306ClearScreen();
	char point1[2]={(char)((0-maxSpeed)/(0-acceleration)),64-(char)maxSpeed};
	char point2[2]={(char)(128-(0-maxSpeed)/(0-deceleration)),64-(char)(maxSpeed)};
	ssd1306DrawLine(0,64,point1[0],point1[1]);
	ssd1306DrawLine(point1[0],point1[1],point2[0],point2[1]);
	ssd1306DrawLine(point2[0],point1[1],128,64);

	sprintf(str,"%.2fM.S^2",acceleration);
	ssd1306DrawString((0+point1[0])/2+2,(64+point1[1])/2+2,str,&Font_3x6);

	sprintf(str,"%.2fM.S",maxSpeed);
	ssd1306DrawString((point1[0]+point2[0])/2,(point1[1]+point2[1])/2,str,&Font_3x6);

	sprintf(str,"%.2fM.S^2",deceleration);
	ssd1306DrawString((point2[0]+128)/2-27,(point2[1]+64)/2,str,&Font_3x6);
	ssd1306Refresh();
}
listDote* doteGraph (int time, int value)
{
	static listDote dotes={0};
	static listDote *actual_list=NULL;
	static int x=0;

	if(actual_list==NULL)
		actual_list=&dotes;

	actual_list->dotes[x]=value;

	x++;
	if (x%MAX_DOTES==0)
	{
		actual_list->next=calloc_s(1,sizeof(listDote));
		x=0;
	}
	return actual_list;
}
//void hal_ui_anti_rebonds (GPIO_TypeDef* gpio, uint16_t gpio_pin)
//{
//	unsigned long int time_wait=500;
//	unsigned long int time_base = hal_os_get_systicks();
//	do
//	{
//		if (GPIO_ReadInputDataBit(gpio,gpio_pin) == Bit_RESET)
//			time_base = hal_os_get_systicks();
//	}while (time_base>(hal_os_get_systicks()-time_wait));
//}

