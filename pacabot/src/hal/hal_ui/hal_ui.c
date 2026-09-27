/*---------------------------------------------------------------------------
 *
 *      hal_ui.c
 *
 *---------------------------------------------------------------------------*/

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

#include "hal/hal_os.h"

/* Declarations for this module */
#include "hal/hal_ui.h"
#include "fw_buttons.h"
#include "fw_menu.h"
#include "hal/hal_adc.h"
#include "hal/hal_led.h"
#include "hal/hal_beeper.h"

#include "util/common_functions.h"

/* ST Lib declarations */
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "misc.h"

#include "oled/ssd1306.h"
#include "oled/smallfonts.h"
#include "oled/pictures.h"
//#include "app/solver_maze.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* GPIO definitions */
#define JOYSTICK_LEFT_PIN    GPIO_Pin_8
#define JOYSTICK_UP_PIN      GPIO_Pin_9
#define JOYSTICK_DOWN_PIN    GPIO_Pin_10
#define JOYSTICK_RIGHT_PIN   GPIO_Pin_11
#define JOYSTICK_PUSH_PIN    GPIO_Pin_12
#define RETURN_BUTTON_PIN    GPIO_Pin_13

#define JOYSTICK_LEFT       GPIOC, GPIO_Pin_8
#define JOYSTICK_UP         GPIOC, GPIO_Pin_9
#define JOYSTICK_DOWN       GPIOC, GPIO_Pin_10
#define JOYSTICK_RIGHT      GPIOC, GPIO_Pin_11
#define JOYSTICK_PUSH       GPIOC, GPIO_Pin_12
#define RETURN_BUTTON       GPIOC, GPIO_Pin_13

#define UI_PORT             GPIOC

// Number of ms to wait when 'VALIDATE' button is pressed
#define VALIDATE_WAIT_TIME  300

/********************************* Macros ***************************************/
#define ITEM_HOVER      ssd1306ClearRect(item[items_nb].x, item[items_nb].y, \
                        item[items_nb].w, item[items_nb].h); \
                        ssd1306DrawBmp(item[items_nb].bmp_hover, \
                        item[items_nb].x, item[items_nb].y, \
                        item[items_nb].w, item[items_nb].h); \
                        ssd1306Refresh()

#define ITEM_RESTORE    ssd1306ClearRect(selected_item->x, selected_item->y, \
                                         selected_item->w, selected_item->h); \
                        ssd1306DrawBmp(selected_item->bmp, \
                        selected_item->x, selected_item->y, \
                        selected_item->w, selected_item->h); \
                        ssd1306Refresh()

/***************************** Static functions *******************************/
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void menu_animate(unsigned char y, unsigned char max_y);
void hal_ui_anti_rebonds (GPIO_TypeDef* gpio, uint16_t gpio_pin);

/* Static variables */
static HAL_BEEPER_HANDLE beeper;
static HAL_ADC_HANDLE adc;


int hal_ui_init(void)
{
    int i;

    RCC_Configuration();
    GPIO_Configuration();

    hal_beeper_open(&beeper, null);

    ssd1306Init(0);
    ssd1306Refresh();
    ssd1306ClearScreen();
    ssd1306DrawBmp(Pacabot_bmp, 1, 1, 128, 40);
    ssd1306Refresh();
    for (i = 0; i < 100; i += 2)
    {
        ssd1306ProgressBar(10, 55, i);
        ssd1306Refresh();
        HAL_Delay(10);
    }

    return HAL_UI_E_SUCCESS;
}


int hal_ui_terminate(void)
{
    return HAL_UI_E_SUCCESS;
}


int hal_ui_open(HAL_UI_HANDLE *handle, void *params)
{
    hal_led_open(handle, null);
    adc = (HAL_ADC_HANDLE)params;

    return HAL_UI_E_SUCCESS;
}


int hal_ui_close(HAL_UI_HANDLE handle)
{
    UNUSED(handle);

    return HAL_UI_E_SUCCESS;
}

int hal_ui_refresh(HAL_UI_HANDLE handle)
{
    UNUSED(handle);
    ssd1306Refresh();

    return HAL_UI_E_SUCCESS;
}

int hal_ui_clear_scr(HAL_UI_HANDLE handle)
{
    UNUSED(handle);
    ssd1306ClearScreen();
    fw_ui_battery();
    return HAL_UI_E_SUCCESS;
}

int hal_ui_display_txt(HAL_UI_HANDLE handle, int x, int y, const char *str)
{
    UNUSED(handle);
    ssd1306DrawString(x, y, str, &Font_5x8);

    return HAL_UI_E_SUCCESS;
}


int hal_ui_display_prompt(HAL_UI_HANDLE handle,
                          const char *title, const char *str)
{
    UNUSED(handle);
    ssd1306ClearScreen();

    // Draw the title
    ssd1306DrawRect(4, 2, 120, 60);
    ssd1306DrawString(5, 3, touppercase((char *)title, strlen(title)), &Font_3x6);
    ssd1306InvertArea(5, 3, 118, 8);
    // Draw prompt string
    ssd1306DrawTextBox(6, 18, 116, 30, str, &Font_5x8);
    // Draw instructions string
    ssd1306DrawString(6, 53, "OK / LEFT:BACK / ESC", &Font_3x6);
    ssd1306Refresh();

    // Wait until 'Return' Button is pressed
    while(!fw_select_pressed() && !fw_cancel_pressed()) __WFI();
    while(fw_select_pressed() || fw_cancel_pressed()) __WFI();
    HAL_Delay(VALIDATE_WAIT_TIME);

    return 0;
}


static void menu_animate(unsigned char y, unsigned char max_y)
{
    if (max_y > y)
    {
        for ( ; y <= max_y; y++)
        {
            ssd1306InvertArea(0, y - 1, 128, 10);
            ssd1306InvertArea(0, y, 128, 10);
            ssd1306Refresh();
            HAL_Delay(2);
        }
    }
    else
    {
        for ( ; y >= max_y; y--)
        {
            ssd1306InvertArea(0, y + 1, 128, 10);
            ssd1306InvertArea(0, y, 128, 10);
            ssd1306Refresh();
            HAL_Delay(2);
        }
    }
}


int hal_ui_display_menu(HAL_UI_HANDLE handle, void *menu,
                        int *selected_index, unsigned char exit_on_select)
{
    menu_item       *item;
    unsigned int    items_nb = 0;
    int             j;
    int             k;
    char            tmp_items[MAX_PAGE_ITEMS][40];

    UNUSED(handle);
    if (menu == null)
    {
        return HAL_UI_E_ERROR;
    }
    item = (menu_item *)menu;
    item++;

    // Compute the number of menu items
    while (item->str != null)
    {
        items_nb++;
        item++;
    }

    // Limit the number of items to the maximum number
    if (items_nb > MAX_PAGE_ITEMS)
    {
        items_nb = MAX_PAGE_ITEMS;
    }

    while (1)
    {
        // Re-initialize the item pointer
        item = (menu_item *)menu;

        hal_ui_clear_scr(handle);

        // Write the menu title
        ssd1306DrawString(0, 0, item[0].str, &Font_3x6);
        ssd1306DrawLine(0, 10, 128, 10);
        item++;

        // Write the menu items
        for (j = 15, k = 0; k < (items_nb); j += 10, k++)
        {
            if (item[k].str[0] == '[')
            {
                // Copy from the 4th character
                memcpy(tmp_items[k], &item[k].str[3], strlen(item[k].str) - 2);
                switch (item[k].str[1])
                {
                    case 'b':   // Show boolean
                        if (*((bool *)item[k].param) == true)
                        {
                            strcat(tmp_items[k], "YES");
                        }
                        else
                        {
                            strcat(tmp_items[k], "NO");
                        }
                        break;
                    case 'l':   // Show long
                        sprintf(&(tmp_items[k][strlen(tmp_items[k])]), "%i",
                                                      *((int *)item[k].param));
                        break;
                }
                ssd1306DrawString(1, j, tmp_items[k], &Font_5x8);
            }
            else
            {
                ssd1306DrawString(1, j, item[k].str, &Font_5x8);
            }
        }

        // Mark the first item as default one
        ssd1306InvertArea(0, 14, 128, 10);
        ssd1306Refresh();

        j = 14;
        (*selected_index) = 1;

        while (1)
        {
            // Exit Button
            if (GPIO_ReadInputDataBit(JOYSTICK_LEFT) == Bit_RESET)
            {
            	hal_ui_anti_rebonds(JOYSTICK_LEFT);
                (*selected_index) = 0;
                return HAL_UI_E_SUCCESS;
            }

            // Joystick down
            if (GPIO_ReadInputDataBit(JOYSTICK_DOWN) == Bit_RESET)
            {
                hal_ui_anti_rebonds(JOYSTICK_DOWN);
                if ((*selected_index) < items_nb)
                {
                    menu_animate(j + 1, j + 10);
                    j += 10;

                    hal_beeper_beep(beeper, 4000, 10);
                    (*selected_index)++;
                }
            }
            // Joystick up
            if (GPIO_ReadInputDataBit(JOYSTICK_UP) == Bit_RESET)
            {
                hal_ui_anti_rebonds(JOYSTICK_UP);
                if ((*selected_index) > 1)
                {
                    menu_animate(j - 1, j - 10);
                    j -= 10;

                    hal_beeper_beep(beeper, 4000, 10);
                    (*selected_index)--;
                }
            }

            // Validate button
            if(GPIO_ReadInputDataBit(JOYSTICK_RIGHT) == Bit_RESET)
            {
                // Wait until button is released
                hal_ui_anti_rebonds(JOYSTICK_RIGHT);
                HAL_Delay(VALIDATE_WAIT_TIME);

                if (item[(*selected_index) - 1].handler != null)
                {
                    item[(*selected_index) - 1].handler();
                }
                else if (item[(*selected_index) - 1].param != null)
                {
                    switch (item[(*selected_index) - 1].str[1])
                    {
                        case 'b':
                            hal_ui_modify_bool_param(handle,
                                                     tmp_items[(*selected_index) - 1],
                                                     (bool *)item[(*selected_index) - 1].param);
                            break;
                        case 'l':
                            hal_ui_modify_long_param(handle,
                                                     tmp_items[(*selected_index) - 1],
                                                     (unsigned long *)item[(*selected_index) - 1].param,
                                                     item[(*selected_index) - 1].step);
                            break;
                    }
                }
                if (exit_on_select == true)
                {
                    return HAL_UI_E_SUCCESS;
                }
                break;
            }
        }
    }
    return HAL_UI_E_ERROR;
}


int hal_ui_display_bmp_menu(HAL_UI_HANDLE handle, void *menu, void *param)
{
    bmp_menu_item   *item;
    unsigned int    items_nb = 0;
    bmp_menu_item   *selected_item;

    UNUSED(handle);
    if (menu == null)
    {
        return HAL_UI_E_ERROR;
    }
    item = (bmp_menu_item *)menu;
    item++;

    //ssd1306ClearScreen();
    hal_ui_clear_scr(handle);

    // Draw the first item as selected by default
    ssd1306DrawBmp(item->bmp_hover, item->x, item->y, item->w, item->h);
    item++;

    // Draw the menu items
    while (item->bmp != null)
    {
        ssd1306DrawBmp(item->bmp, item->x, item->y, item->w, item->h);
        items_nb++;
        item++;
        if (items_nb == 3)
        {
            break;
        }
    }
    ssd1306Refresh();
    item = (bmp_menu_item *)menu;
    item++;

    // Choose the first item by default
    selected_item = item;

    while (1)
    {
        // Exit Button
        if (GPIO_ReadInputDataBit(JOYSTICK_LEFT) == Bit_RESET)
        {
            // Wait until button is released
            while (GPIO_ReadInputDataBit(JOYSTICK_LEFT) == Bit_RESET);
            HAL_Delay(VALIDATE_WAIT_TIME);
            return HAL_UI_E_SUCCESS;
        }

        // Joystick down
        if (GPIO_ReadInputDataBit(JOYSTICK_DOWN) == Bit_RESET)
        {
            if (selected_item->select_direction == MENU_DIRECTION_DOWN)
            {
                continue;
            }

            // Restore the previously selected item to its initial state
            ITEM_RESTORE;

            for (items_nb = 0; items_nb < 4; items_nb++)
            {
                if (item[items_nb].select_direction == MENU_DIRECTION_DOWN)
                {
                    ITEM_HOVER;
                    selected_item = &item[items_nb];
                    hal_beeper_beep(beeper, 4000, 10);
                    HAL_Delay(VALIDATE_WAIT_TIME);
                }
            }
        }

        // Joystick up
        if (GPIO_ReadInputDataBit(JOYSTICK_UP) == Bit_RESET)
        {
            if (selected_item->select_direction == MENU_DIRECTION_UP)
            {
                continue;
            }

            // Restore the previously selected item to its initial state
            ITEM_RESTORE;

            for (items_nb = 0; items_nb < 4; items_nb++)
            {
                if (item[items_nb].select_direction == MENU_DIRECTION_UP)
                {
                    ITEM_HOVER;
                    selected_item = &item[items_nb];
                    hal_beeper_beep(beeper, 4000, 10);
                    HAL_Delay(VALIDATE_WAIT_TIME);
                }
            }
        }

        // Joystick left
        if (GPIO_ReadInputDataBit(JOYSTICK_LEFT) == Bit_RESET)
        {
            if (selected_item->select_direction == MENU_DIRECTION_LEFT)
            {
                continue;
            }

            // Restore the previously selected item to its initial state
            ITEM_RESTORE;

            for (items_nb = 0; items_nb < 4; items_nb++)
            {
                if (item[items_nb].select_direction == MENU_DIRECTION_LEFT)
                {
                    ITEM_HOVER;
                    selected_item = &item[items_nb];
                    hal_beeper_beep(beeper, 4000, 10);
                    HAL_Delay(VALIDATE_WAIT_TIME);
                }
            }
        }

        // Joystick right
        if (GPIO_ReadInputDataBit(JOYSTICK_RIGHT) == Bit_RESET)
        {
            if (selected_item->select_direction == MENU_DIRECTION_RIGHT)
            {
                continue;
            }

            // Restore the previously selected item to its initial state
            ITEM_RESTORE;

            for (items_nb = 0; items_nb < 4; items_nb++)
            {
                if (item[items_nb].select_direction == MENU_DIRECTION_RIGHT)
                {
                    ITEM_HOVER;
                    selected_item = &item[items_nb];
                    hal_beeper_beep(beeper, 4000, 10);
                    HAL_Delay(VALIDATE_WAIT_TIME);
                }
            }
        }

        if(GPIO_ReadInputDataBit(JOYSTICK_RIGHT) == Bit_RESET)
        {
            // Wait until button is released
            while (GPIO_ReadInputDataBit(JOYSTICK_RIGHT) == Bit_RESET);
            HAL_Delay(VALIDATE_WAIT_TIME);

            if (selected_item->handler != null)
            {
                hal_ui_clear_scr(handle);
                ssd1306Refresh();
                return selected_item->handler(param);
            }
            break;
        }
    }

    return HAL_UI_E_SUCCESS;
}


int hal_ui_modify_bool_param(HAL_UI_HANDLE handle,
                             char *param_name, unsigned char *param)
{
    char str[4];
    bool param_copy = (bool)*param;

    hal_ui_clear_scr(handle);

    // Write the parameter name
    ssd1306DrawString(0, 0,
                      touppercase(param_name, strlen(param_name)), &Font_3x6);
    ssd1306DrawLine(0, 10, 128, 10);

    if (param_copy == true)
    {
        sprintf(str, "YES");
    }
    else
    {
        sprintf(str, "NO");
    }
    ssd1306DrawString(0, 28, str, &Font_8x8);
    ssd1306DrawString(0, 50, "PRESS 'RIGHT' TO VALIDATE", &Font_3x6);
    ssd1306DrawString(0, 57, "      'LEFT'  TO RETURN.", &Font_3x6);
    ssd1306Refresh();

    while (1)
    {
        // Exit Button
        if (GPIO_ReadInputDataBit(JOYSTICK_LEFT) == Bit_RESET)
        {
        	hal_ui_anti_rebonds(JOYSTICK_LEFT);
            break;
        }
        else if ((GPIO_ReadInputDataBit(JOYSTICK_UP) == Bit_RESET) ||
                 (GPIO_ReadInputDataBit(JOYSTICK_DOWN) == Bit_RESET))
        {
        	hal_ui_anti_rebonds(JOYSTICK_UP);
        	hal_ui_anti_rebonds(JOYSTICK_DOWN);
            if (param_copy == true)
            {
                param_copy = false;
                sprintf(str, "NO");
            }
            else
            {
                param_copy = true;
                sprintf(str, "YES");
            }
            ssd1306ClearRect(0, 28, 164, 8);
            ssd1306DrawString(0, 28, str, &Font_8x8);
            ssd1306Refresh();
            HAL_Delay(VALIDATE_WAIT_TIME);
        }
        else if (GPIO_ReadInputDataBit(JOYSTICK_RIGHT) == Bit_RESET)
        {
        	hal_ui_anti_rebonds(JOYSTICK_RIGHT);

            *param = param_copy;
            hal_ui_clear_scr(handle);
            ssd1306Refresh();
            break;
        }
    }
    return HAL_UI_E_SUCCESS;
}


int hal_ui_modify_long_param(HAL_UI_HANDLE handle, char *param_name,
                             unsigned long *param, int step)
{
		char collone=0;
	    char str[40];
	    unsigned long param_copy = *param;

	    hal_ui_clear_scr(handle);

	    // Write the parameter name
	    ssd1306DrawString(0, 0,
	                      touppercase(param_name, strlen(param_name)), &Font_3x6);
	    ssd1306DrawLine(0, 10, 128, 10);

	    sprintf(str, "%10d", param_copy);
	    ssd1306DrawString(0, 28, str, &Font_8x8);
	    ssd1306DrawString(0, 50, "PRESS 'RIGHT' TO VALIDATE", &Font_3x6);
	    ssd1306DrawString(0, 57, "      'LEFT'  TO RETURN.", &Font_3x6);
		ssd1306DrawString((10-collone)*8,20,"^",&Font_8x8);
		ssd1306DrawString((10-collone)*8,36,"v",&Font_8x8);
	    ssd1306Refresh();

	    while (1)
	    {
	        // Exit Button
	        if (GPIO_ReadInputDataBit(JOYSTICK_LEFT) == Bit_RESET)
	        {
	        	hal_ui_anti_rebonds(JOYSTICK_LEFT);
	            if(collone==9)
	            	break;
	            else
	            {
	            	collone++;
					ssd1306ClearRect(0,20,128,8);
					ssd1306ClearRect(0,36,128,8);
					ssd1306DrawString((9-collone)*9,20,"^",&Font_8x8);
					ssd1306DrawString((9-collone)*9,36,"v",&Font_8x8);
					ssd1306Refresh();
	            }
	        }
	        else if (GPIO_ReadInputDataBit(JOYSTICK_UP) == Bit_RESET)
	        {
	        	hal_ui_anti_rebonds(JOYSTICK_UP);
	            param_copy += (step*pow(10,collone));
	            ssd1306ClearRect(0, 28, 164, 8);
	            sprintf(str, "%10d", param_copy);
	            ssd1306DrawString(0, 28, str, &Font_8x8);
	            ssd1306Refresh();
	            HAL_Delay(100);
	        }
	        else if (GPIO_ReadInputDataBit(JOYSTICK_DOWN) == Bit_RESET)
	        {
	        	hal_ui_anti_rebonds(JOYSTICK_DOWN);
	            param_copy -= (step*pow(10,collone));
	            ssd1306ClearRect(0, 28, 164, 8);
	            sprintf(str, "%10i", (int)param_copy);
	            ssd1306DrawString(0, 28, str, &Font_8x8);
	            ssd1306Refresh();
	            HAL_Delay(100);
	        }
	        else if (GPIO_ReadInputDataBit(JOYSTICK_RIGHT) == Bit_RESET)
	        {
	        	hal_ui_anti_rebonds(JOYSTICK_RIGHT);
	            if(collone==0)
	            {
					*param = param_copy;
					hal_ui_clear_scr(handle);
					ssd1306Refresh();
					break;
	            }
	            else
	            {
	            	collone--;
					ssd1306ClearRect(0,20,128,8);
					ssd1306ClearRect(0,36,128,8);
	            	ssd1306DrawString((9-collone)*9,20,"^",&Font_8x8);
	            	ssd1306DrawString((9-collone)*9,36,"v",&Font_8x8);
					ssd1306Refresh();
	            }
	        }
	    }

	    return HAL_UI_E_SUCCESS;
}

int hal_ui_modify_int_param(HAL_UI_HANDLE handle, char *param_name,
                             unsigned int *param, int step)
{
	char collone=0;
    char str[40];
    unsigned int param_copy = *param;

    hal_ui_clear_scr(handle);

	// Write the parameter name
	ssd1306DrawString(0, 0,
					  touppercase(param_name, strlen(param_name)), &Font_3x6);
	ssd1306DrawLine(0, 10, 128, 10);

	sprintf(str, "%10i", (int)param_copy);
	ssd1306DrawString(0, 28, str, &Font_8x8);
	ssd1306DrawString(0, 50, "PRESS 'RIGHT' TO VALIDATE", &Font_3x6);
	ssd1306DrawString(0, 57, "      'LEFT'  TO RETURN.", &Font_3x6);
	ssd1306DrawString((10-collone)*8,20,"^",&Font_8x8);
	ssd1306DrawString((10-collone)*8,36,"v",&Font_8x8);
	ssd1306Refresh();

	while (1)
	{
		// Exit Button
		if (GPIO_ReadInputDataBit(JOYSTICK_LEFT) == Bit_RESET)
		{
			hal_ui_anti_rebonds(JOYSTICK_LEFT);
			if(collone==9)
				break;
			else
			{
				collone++;
				ssd1306ClearRect(0,20,128,8);
				ssd1306ClearRect(0,36,128,8);
				ssd1306DrawString((9-collone)*9,20,"^",&Font_8x8);
				ssd1306DrawString((9-collone)*9,36,"v",&Font_8x8);
				ssd1306Refresh();
			}
		}
		else if (GPIO_ReadInputDataBit(JOYSTICK_UP) == Bit_RESET)
		{
			hal_ui_anti_rebonds(JOYSTICK_UP);
			param_copy += (step*pow(10,collone));
			ssd1306ClearRect(0, 28, 164, 8);
			sprintf(str, "%10i", (int)param_copy);
			ssd1306DrawString(0, 28, str, &Font_8x8);
			ssd1306Refresh();
			HAL_Delay(100);
		}
		else if (GPIO_ReadInputDataBit(JOYSTICK_DOWN) == Bit_RESET)
		{
			hal_ui_anti_rebonds(JOYSTICK_DOWN);
			param_copy -= (step*pow(10,collone));
			ssd1306ClearRect(0, 28, 164, 8);
			sprintf(str, "%10i", (int)param_copy);
			ssd1306DrawString(0, 28, str, &Font_8x8);
			ssd1306Refresh();
			HAL_Delay(100);
		}
		else if (GPIO_ReadInputDataBit(JOYSTICK_RIGHT) == Bit_RESET)
		{
			hal_ui_anti_rebonds(JOYSTICK_RIGHT);
			if(collone==0)
			{
				HAL_Delay(VALIDATE_WAIT_TIME);
				*param = param_copy;
				hal_ui_clear_scr(handle);
				ssd1306Refresh();
				break;
			}
			else
			{
				collone--;
				ssd1306ClearRect(0,20,128,8);
				ssd1306ClearRect(0,36,128,8);
				ssd1306DrawString((9-collone)*9,20,"^",&Font_8x8);
				ssd1306DrawString((9-collone)*9,36,"v",&Font_8x8);
				ssd1306Refresh();
			}
		}
	}

	return HAL_UI_E_SUCCESS;
}

int hal_ui_draw_line(HAL_UI_HANDLE handle,
                     unsigned char x0, unsigned char y0,
                     unsigned char x1, unsigned char y1)
{
    UNUSED(handle);
    ssd1306DrawLine(x0, y0, x1, y1);

    return HAL_UI_E_SUCCESS;
}


int hal_ui_fill_rect(HAL_UI_HANDLE handle,
                     unsigned char x, unsigned char y,
                     unsigned char w, unsigned char h)
{
    UNUSED(handle);
    ssd1306FillRect(x, y, w, h);

    return HAL_UI_E_SUCCESS;
}


int hal_ui_fill_circle(HAL_UI_HANDLE handle,
                       unsigned char x0, unsigned char y0,
                       unsigned char r)
{
    UNUSED(handle);
    ssd1306FillCircle(x0, y0, r);

    return HAL_UI_E_SUCCESS;
}

void hal_ui_anti_rebonds (GPIO_TypeDef* gpio, uint16_t gpio_pin)
{
	unsigned long int time_wait=50;
	unsigned long int time_base = hal_os_get_systicks();
	do
	{
		if (GPIO_ReadInputDataBit(gpio,gpio_pin) == Bit_RESET)
			time_base = hal_os_get_systicks();
	}while (time_base>(hal_os_get_systicks()-time_wait));
}

static void RCC_Configuration(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
}


static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin   = JOYSTICK_DOWN_PIN | JOYSTICK_UP_PIN |
                                    JOYSTICK_LEFT_PIN | JOYSTICK_RIGHT_PIN |
                                    JOYSTICK_PUSH_PIN |
                                    RETURN_BUTTON_PIN;
    GPIO_Init(UI_PORT, &GPIO_InitStructure);
}
