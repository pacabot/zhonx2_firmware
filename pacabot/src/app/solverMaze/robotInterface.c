#include "config/basetypes.h"
#include "config/config.h"
/* peripherale inlcudes*/
#include "drivers/step_motor_driver.h"
#include "hal/hal_ui.h"
#include "oled/ssd1306.h"
#include "hal/hal_sensor.h"
#include "hal/hal_step_motor.h"
#include "app/app_def.h"
#include "hal/hal_os.h"
#include "oled/ssd1306.h"

/*application include */
#include "app/solverMaze/solverMaze.h"

/* Middleware declarations */
#include "app/solverMaze/robotInterface.h"

void goOrientation(char *orientationZhonx, char directionToGo)
{
	int turn = (4 + directionToGo - *orientationZhonx) % 4;
	*orientationZhonx = directionToGo;
	switch (turn)
	{
		case FORWARD :
			break;
		case RIGHT :
				step_motors_rotate_in_place(-90);
				break;
		case UTURN :
				step_motors_rotate_in_place(180);
				break;
		case LEFT :
				step_motors_rotate_in_place(90);

			break;
	}
}

void move_zhonx (int direction_to_go, positionRobot *positionZhonx, int numberOfCase)
{
	int turn = (4 + direction_to_go- positionZhonx->orientation) % 4;
	positionZhonx->orientation = direction_to_go;
	switch (turn)
	{
		case FORWARD :
			break;
		case RIGHT :
				//step_motors_rotate(-90, 90, CHAIN_BEFORE | CHAIN_AFTER);
	//			step_motors_rotate(-90, 90, 0);
				step_motors_rotate_in_place(-90);
				break;
		case UTURN :
				step_motors_rotate_in_place(180);
				break;
		case LEFT :
//				step_motors_rotate(90, 90, 0);
				step_motors_rotate_in_place(90);

			break;
	}
//	step_motors_move(CELL_LENGTH*numberOfCase, 0, CHAIN_BEFORE | CHAIN_AFTER);
	step_motors_move(CELL_LENGTH*numberOfCase, 0, 0);
}

void move_zhonx_arc (int direction_to_go, positionRobot *positionZhonx, int numberOfCase, char endMidOfCase, char chain)
{
    endMidOfCase = true;
	int distanceToMove=CELL_LENGTH*numberOfCase;
	int turn=(4+direction_to_go-positionZhonx->orientation)%4;

	positionZhonx->orientation=direction_to_go;
	switch (turn)
	{
		case FORWARD :
			break;
		case RIGHT :
				if(positionZhonx->midOfCell==true)
					step_motors_rotate_in_place(-90);
				else
				{
					step_motors_rotate(90, CELL_LENGTH/2, chain);
					distanceToMove-=CELL_LENGTH;
				}
				break;
		case UTURN :
				step_motors_rotate_in_place(180);
				break;
		case LEFT :
				if(positionZhonx->midOfCell==true)
					step_motors_rotate_in_place(90);
				else
				{
					step_motors_rotate(-90, CELL_LENGTH/2, chain);
					distanceToMove-=CELL_LENGTH;
				}

			break;
	}
	if (positionZhonx->midOfCell == endMidOfCase)
	{
		/*
		 * distanceToMove-=CELL_LENGTH/2;
		 * distanceToMove+=CELL_LENGTH/2;
		 */
	}
	else if(positionZhonx->midOfCell == true) // so endMidOfCase=false
	{
		distanceToMove -= CELL_LENGTH/2;
	}
	else // so positionZhonx->midOfCel = false && endMidOfCase = true
	{
		distanceToMove += CELL_LENGTH/2;
	}
	chain=0;

	if (positionZhonx->midOfCell == false)
	{
		chain=chain | CHAIN_BEFORE;
	}
	if (endMidOfCase == false)
	{

		chain=chain | CHAIN_AFTER;
	}

	step_motors_move(distanceToMove, 0, chain);
	positionZhonx->midOfCell=endMidOfCase;
}
void doUTurn(positionRobot *positionZhonx)
{
	hal_step_motor_enable();
	goOrientation(&positionZhonx->orientation, (positionZhonx->orientation+2)%4);
	hal_step_motor_disable();
}

void newCell(walls new_walls, labyrinthe *maze, positionRobot positionZhonx)

{
#ifdef DEBUG
	/*print walls position*/
	static char i=1;
	i++;
	ssd1306ClearRect(64,0,64,64);
	if (new_walls.front == WALL_PRESENCE)
	{
		hal_ui_fill_rect(app_context.ui,64,0,54,5);
	}
	if (new_walls.left == WALL_PRESENCE)
	{
		hal_ui_fill_rect(app_context.ui,64,0,5,54);
	}
	if (new_walls.right == WALL_PRESENCE)
	{
		hal_ui_fill_rect(app_context.ui,113,0,5,54);
	}
	/*end print wall position*/
#endif
	switch (positionZhonx.orientation)
	{
		case NORTH :
			if(positionZhonx.midOfCell == true)
			{
				maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_east = new_walls.right;
				maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_west = new_walls.left;

				if (positionZhonx.coordinate_robot.x < (MAZE_SIZE - 1))
					maze->cell[(int) (positionZhonx.coordinate_robot.x + 1)][(int) (positionZhonx.coordinate_robot.y)].wall_west = new_walls.right;
				if (positionZhonx.coordinate_robot.x > 0)
					maze->cell[(int) (positionZhonx.coordinate_robot.x - 1)][(int) (positionZhonx.coordinate_robot.y)].wall_east = new_walls.left;
			}
			if (positionZhonx.coordinate_robot.y > 0)
				maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y - 1)].wall_south = new_walls.front;

			maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_north = new_walls.front;
			break;

		case EAST :

				if(positionZhonx.midOfCell == true)
				{
					maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_south = new_walls.right;
					maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_north = new_walls.left;

					if (positionZhonx.coordinate_robot.y < (MAZE_SIZE - 1))
						maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y + 1)].wall_north = new_walls.right;
					if (positionZhonx.coordinate_robot.y > 0)
						maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y - 1)].wall_south = new_walls.left;

				}
			if (positionZhonx.coordinate_robot.x < (MAZE_SIZE - 1) )
				maze->cell[(int) (positionZhonx.coordinate_robot.x + 1)][(int) (positionZhonx.coordinate_robot.y)].wall_west = new_walls.front;
			maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_east = new_walls.front;
			break;

		case SOUTH :

			if(positionZhonx.midOfCell == true)
			{
				maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_west = new_walls.right;
				maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_east = new_walls.left;

				if (positionZhonx.coordinate_robot.x > 0)
					maze->cell[(int) (positionZhonx.coordinate_robot.x - 1)][(int) (positionZhonx.coordinate_robot.y)].wall_east = new_walls.right;
				if (positionZhonx.coordinate_robot.x < (MAZE_SIZE - 1))
					maze->cell[(int) (positionZhonx.coordinate_robot.x + 1)][(int) (positionZhonx.coordinate_robot.y)].wall_west = new_walls.left;
			}
			if (positionZhonx.coordinate_robot.y > 0)
				maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y + 1)].wall_north = new_walls.front;
			maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_south =
					new_walls.front;
			break;

		case WEST :
			if(positionZhonx.midOfCell == true)
			{
				maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_north = new_walls.right;
				maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_south = new_walls.left;


				if (positionZhonx.coordinate_robot.y > 0)
					maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y - 1)].wall_south = new_walls.right;
				if (positionZhonx.coordinate_robot.y < (MAZE_SIZE - 1))
					maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y + 1)].wall_north = new_walls.left;
			}
			if (positionZhonx.coordinate_robot.x > 0)
				maze->cell[(int) (positionZhonx.coordinate_robot.x - 1)][(int) (positionZhonx.coordinate_robot.y)].wall_east = new_walls.front;
			maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_west = new_walls.front;
			break;
	}
}

walls getCellState ()
{
	unsigned char   sensors_state = hal_sensor_get_state(app_context.sensors);
		walls cell_condition= {NO_WALL,NO_WALL,NO_WALL};
		if (check_bit(sensors_state, SENSOR_L10_POS) == false)
		   {
			   cell_condition.left = WALL_PRESENCE;
		   }
		if (check_bit(sensors_state, SENSOR_R10_POS) == false)
		   {
			   cell_condition.right = WALL_PRESENCE;
		   }
		if (check_bit(sensors_state, SENSOR_F10_POS) == false)
		   {
			   cell_condition.front = WALL_PRESENCE;
		   }

		return cell_condition;
}
void calibrateSimple()
{
	hal_step_motor_enable();
	char orientation=0;
	unsigned char sensors_state = 0;
	for(int i=0; i<2;i++)
	{
		sensors_state = hal_sensor_get_state(app_context.sensors);
		if (check_bit(sensors_state, SENSOR_L10_POS) == false)
		{
			goOrientation(&orientation,orientation-1);
		}
		else if (check_bit(sensors_state, SENSOR_R10_POS) == false)
		{
			goOrientation(&orientation,orientation+1);
		}
 		step_motors_basic_move(70);
		HAL_Delay(500);
		step_motors_basic_move(-((CELL_LENGTH/2)-46));
	}
	goOrientation(&orientation,0);
	HAL_Delay(100);
	hal_step_motor_disable();
}

void waitStart()
{
	unsigned char sensors_state = hal_sensor_get_state(app_context.sensors);
	while(check_bit(sensors_state, SENSOR_F10_POS)==true)
		sensors_state = hal_sensor_get_state(app_context.sensors);
	HAL_Delay(200);
	while(check_bit(sensors_state, SENSOR_F10_POS)==false)
		sensors_state = hal_sensor_get_state(app_context.sensors);
}

void start_navigation()
{
	hal_step_motor_enable();
}

void end_navigation()
{
	HAL_Delay(300);
	hal_step_motor_disable();
}
