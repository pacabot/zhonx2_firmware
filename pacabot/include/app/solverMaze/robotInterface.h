/*
 * robotInterface.h
 *
 *  Created on: 4 juin 2015
 *      Author: zhonx
 */

#ifndef ROBOTINTERFACE_H_
#define ROBOTINTERFACE_H_

#include "solverMaze.h"
void goOrientation(char *orientationZhonx, char directionToGo);
void doUTurn(positionRobot *positionZhonx);
void moveZhonxArc (int direction_to_go, positionRobot *positionZhonx, int numberOfCase, char end_mid_of_case, char chain);
int waitValidation(unsigned long timeout);
void newCell(walls new_walls, labyrinthe *maze, positionRobot positionZhonx);
walls getCellState (void);
void move_zhonx_arc (int direction_to_go, positionRobot *positionZhonx, int numberOfCell, char end_mid_of_case, char chain);
extern positionRobot *pt_zhonx_position;
int rotate90WithCal(int rotation_type, float max_speed, float end_speed);
void calibrateSimple();
void move_zhonx (int direction_to_go, positionRobot *positionZhonx, int numberOfCase);
void start_navigation();
void end_navigation();

#endif /* ROBOTINTERFACE_H_ */
