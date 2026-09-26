/*
 * run.c
 *
 *  Created on: 4 juin 2015
 *      Author: Colin
 */

#include "config/basetypes.h"
#include "stm32f4xx_gpio.h"
#include "config/config.h"
#include "hal/hal_os.h"
#include "hal/hal_step_motor.h"
/* peripherale inlcudes*/
#include "stm32f4xx.h"
#include "oled/ssd1306.h"
/* meddleware include */

/*application include */
#include "app/solverMaze/solverMaze.h"
#include "app/solverMaze/robotInterface.h"
#include "app/solverMaze/run.h"
#include "app/solverMaze/user_interface.h"
void run1(labyrinthe *maze, positionRobot *positionZhonx, coordinate start_oordinate, coordinate end_coordinate)
{
	coordinate way[MAZE_SIZE*MAZE_SIZE];
	char choice;
	do
	{
		choice = -1;
		clearMazelength(maze);
		poids(maze, end_coordinate, false, false);
		moveVirtualZhonx(*maze, *positionZhonx, way, end_coordinate);
		waitStart ();
		start_navigation();
		moveRealZhonxArc(maze, positionZhonx, way);
		HAL_Delay(1000);
		goToPosition(maze,positionZhonx,start_oordinate);
		doUTurn (positionZhonx);
		end_navigation();

		ssd1306ClearScreen ();
		ssd1306DrawString (10, 10, "presse \"RIGHT\" to ", &Font_5x8);
		ssd1306DrawString (10, 18, "do a new run 1", &Font_5x8);
		ssd1306Refresh ();
		while (choice == -1)
		{
			 if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11) != Bit_SET)
			{
				choice = 1;
			}

			if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) != Bit_SET)
			{
				choice = 0;
			}
		}
	}while (choice == 1);
}
void run2(labyrinthe *maze, positionRobot *positionZhonx, coordinate start_oordinate, coordinate end_coordinate)
{
	coordinate way[MAZE_SIZE*MAZE_SIZE];
	char choice;
	do
	{
		choice = -1;
		clearMazelength(maze);
		poids(maze, end_coordinate, false, false);
		printMaze(*maze,positionZhonx->coordinate_robot);
		moveVirtualZhonx (*maze, *positionZhonx, way, end_coordinate);
		waitStart ();
		start_navigation();
		moveRealZhonxArc (maze, positionZhonx, way);
		if (zhonxSettings.calibration_enabled == true)
			calibrateSimple ();
		HAL_Delay(2000);
		goToPosition(maze,positionZhonx,start_oordinate);
		if (zhonxSettings.calibration_enabled == true)
			calibrateSimple ();
		doUTurn (positionZhonx);
		ssd1306ClearScreen ();
		ssd1306DrawString (10, 10, "presse \"RIGHT\" to ", &Font_5x8);
		ssd1306DrawString (10, 18, "do a new run 2", &Font_5x8);
		ssd1306Refresh ();
		while (choice == -1)
		{
			 if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11) != Bit_SET)
			{
				choice = 1;
			}

			if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) != Bit_SET)
			{
				choice = 0;
			}
		}
	}while (choice == 1);
}
