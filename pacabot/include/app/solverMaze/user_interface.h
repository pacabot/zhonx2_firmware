/*
 * user_interface.h
 *
 *  Created on: Apr 17, 2017
 *      Author: colin
 */

#ifndef PACABOT_INCLUDE_APP_SOLVERMAZE_USER_INTERFACE_H_
#define PACABOT_INCLUDE_APP_SOLVERMAZE_USER_INTERFACE_H_

#include "app/solverMaze/solverMaze.h"

walls ask_cell_state();
void print_cell_state(walls cell_state);
void printMaze(labyrinthe maze, coordinate robot_coordinate);
void printLength(const labyrinthe maze,const int x_robot, const int y_robot);

#endif /* PACABOT_INCLUDE_APP_SOLVERMAZE_USER_INTERFACE_H_ */
