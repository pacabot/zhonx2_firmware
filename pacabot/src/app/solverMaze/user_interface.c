/*
 * user_interface.c
 *
 *  Created on: Apr 17, 2017
 *      Author: colin
 */

#include "oled/ssd1306.h"
#include <string.h>
#include "app/solverMaze/user_interface.h"

#include "app/solverMaze/solverMaze.h"

/* todo : change the joystick architecture*/
#include "stm32f4xx_gpio.h"

#define JOYSTICK_LEFT       GPIOC, GPIO_Pin_8
#define JOYSTICK_UP         GPIOC, GPIO_Pin_9
#define JOYSTICK_DOWN       GPIOC, GPIO_Pin_10
#define JOYSTICK_RIGHT      GPIOC, GPIO_Pin_11
#define JOYSTICK_PUSH       GPIOC, GPIO_Pin_12
#define RETURN_BUTTON       GPIOC, GPIO_Pin_13


walls ask_cell_state()
{
    walls cell_state;
    memset(&cell_state, NO_KNOWN, sizeof(walls));
    while (GPIO_ReadInputDataBit(JOYSTICK_UP) != Bit_RESET || GPIO_ReadInputDataBit(RETURN_BUTTON) != Bit_RESET)
    {
        if (GPIO_ReadInputDataBit(JOYSTICK_DOWN) == Bit_RESET)
        {
            if (cell_state.front == WALL_PRESENCE)
            {
                cell_state.front = NO_WALL;
            }
            else
            {
                cell_state.front = WALL_PRESENCE;
            }
        }
        if (GPIO_ReadInputDataBit(JOYSTICK_RIGHT) == Bit_RESET)
        {
            if (cell_state.left == WALL_PRESENCE)
            {
                cell_state.left = NO_WALL;
            }
            else
            {
                cell_state.left = WALL_PRESENCE;
            }
        }
        if (GPIO_ReadInputDataBit(JOYSTICK_LEFT) == Bit_RESET)
        {
            if (cell_state.right == WALL_PRESENCE)
            {
                cell_state.right = NO_WALL;
            }
            else
            {
                cell_state.right = WALL_PRESENCE;
            }
        }
        print_cell_state(cell_state);
        ssd1306Refresh();
    }
    return cell_state;
}


void print_cell_state(walls cell_state)
{
    ssd1306ClearRect(64, HEAD_MARGIN, 54, 5);
    ssd1306ClearRect(64, HEAD_MARGIN, 5, 54);
    ssd1306ClearRect(113, HEAD_MARGIN, 5, 54);

    if (cell_state.front == WALL_PRESENCE)
    {
        ssd1306FillRect(64, HEAD_MARGIN, 54, 5);
    }
    if (cell_state.left == WALL_PRESENCE)
    {
        ssd1306FillRect(64, HEAD_MARGIN, 5, 54);
    }
    if (cell_state.right == WALL_PRESENCE)
    {
        ssd1306FillRect(113, HEAD_MARGIN, 5, 54);
    }
}
void printMaze(labyrinthe maze, coordinate robot_coordinate)
{
    ssd1306ClearRect(0, 0, 64, 64);
    int size_cell_on_oled = ((63) / MAZE_SIZE);
    int x, y;
    for (y = 0; y < MAZE_SIZE; y++)
    {
        for (x = 0; x < MAZE_SIZE; x++)
        {
            if (maze.cell[x][y].wall_north == WALL_PRESENCE)
            {
                ssd1306DrawLine(x * size_cell_on_oled,
                        y * size_cell_on_oled + HEAD_MARGIN,
                        x * size_cell_on_oled + size_cell_on_oled + 1,
                        y * size_cell_on_oled + HEAD_MARGIN);
            }
            else if (maze.cell[x][y].wall_north == NO_KNOWN)
            {
                ssd1306DrawDashedLine(x * size_cell_on_oled,
                        y * size_cell_on_oled + HEAD_MARGIN,
                        x * size_cell_on_oled + size_cell_on_oled + 1,
                        y * size_cell_on_oled + HEAD_MARGIN);
            }
            if (maze.cell[x][y].wall_west == WALL_PRESENCE)
            {
                ssd1306DrawLine(x * size_cell_on_oled,
                        y * size_cell_on_oled + HEAD_MARGIN,
                        x * size_cell_on_oled,
                        (y + 1) * size_cell_on_oled
                        + HEAD_MARGIN + 1);
            }
            else if (maze.cell[x][y].wall_west == NO_KNOWN)
            {
                ssd1306DrawDashedLine(x * size_cell_on_oled,
                        y * size_cell_on_oled+ HEAD_MARGIN,
                        x * size_cell_on_oled,
                        (y + 1) * size_cell_on_oled
                        + HEAD_MARGIN + 1);
            }
            if (maze.cell[x][y].wall_south == WALL_PRESENCE)
            {
                ssd1306DrawLine(x * size_cell_on_oled,
                        (y+ 1) * size_cell_on_oled+ HEAD_MARGIN,
                        size_cell_on_oled + x * size_cell_on_oled,
                        (y+ 1) * size_cell_on_oled + HEAD_MARGIN);
            }
            else if (maze.cell[x][y].wall_south == NO_KNOWN)
            {
                ssd1306DrawDashedLine(x * size_cell_on_oled,
                        (y + 1) * size_cell_on_oled + HEAD_MARGIN,
                        size_cell_on_oled + x * size_cell_on_oled,
                        (y + 1) * size_cell_on_oled + HEAD_MARGIN);
            }
            if (maze.cell[x][y].wall_east == WALL_PRESENCE)
            {
                ssd1306DrawLine((x + 1) * size_cell_on_oled,
                        y * size_cell_on_oled + HEAD_MARGIN,
                        (x + 1) * size_cell_on_oled,
                        (y + 1) * size_cell_on_oled
                        + HEAD_MARGIN + 1);
            }
            else if (maze.cell[x][y].wall_east == NO_KNOWN)
            {
                ssd1306DrawDashedLine((x + 1) * size_cell_on_oled,
                        y * size_cell_on_oled + HEAD_MARGIN,
                        (x + 1) * size_cell_on_oled,
                        (y + 1) * size_cell_on_oled
                        + HEAD_MARGIN + 1);
            }
        }
    }
    printLength(maze, robot_coordinate.x, robot_coordinate.y);
    ssd1306DrawRect((robot_coordinate.x * size_cell_on_oled) + 1,
            (robot_coordinate.y * size_cell_on_oled + HEAD_MARGIN) + 1, 2,
            2);
    ssd1306Refresh();
}

void printLength(const labyrinthe maze, const int x_robot, const int y_robot)
{
#if DEBUG > 2 && !defined ZHONX2
    bluetoothWaitReady();
    bluetoothPrintf("zhonx : %d; %d\n", x_robot, y_robot);
    bluetoothPrintf("  ");
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        bluetoothPrintf("%5d", i);
    }
    bluetoothPrintf("\n");
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        bluetoothPrintf("    ");
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            if (maze.cell[j][i].wall_north == NO_KNOWN)
            {
                bluetoothPrintf("- - +");
            }
            else if (maze.cell[j][i].wall_north == WALL_PRESENCE)
            {
                bluetoothPrintf("----+");
            }
            else
            {
                bluetoothPrintf("    +");
            }
        }
        bluetoothPrintf("\n ");

        bluetoothPrintf("%2d ", i);
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            if (maze.cell[j][i].length != CANT_GO)
            {
                bluetoothPrintf("%4d", maze.cell[j][i].length);
            }
            else
            {
                bluetoothPrintf("    ");
            }
            if (maze.cell[j][i].wall_east == NO_KNOWN)
            {
                bluetoothPrintf("!");
            }
            else if (maze.cell[j][i].wall_east == WALL_PRESENCE)
            {
                bluetoothPrintf("|");
            }
            else
            {
                bluetoothPrintf(" ");
            }
        }
        bluetoothPrintf("\n");
    }
    bluetoothPrintf("\n");
#endif
}

