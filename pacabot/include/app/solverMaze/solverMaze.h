/*
 * resolution_maze.h
 *
 *  Created on: 27 sept. 2014
 *      Author: Colin
 */

#ifndef RESOLUTION_MAZE_H_
#define RESOLUTION_MAZE_H_

#define ZHONX2

#define NAND(a,b) (!a && !b)

/* Error codes */
#define MAZE_SOLVER_E_SUCCESS  0
#define MAZE_SOLVER_E_ERROR    1

//Define for the maze

#define MAZE_SIZE 17

//definition for numerotation function
#define CANT_GO 2147483647

//orientation define
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3
//action define
#define FORWARD 0
#define RIGHT 1
#define UTURN 2
#define LEFT 3
//wall state define
#define NO_KNOWN 0
#define WALL_PRESENCE 1
#define NO_WALL 2

#define NO_END				0
#define END_FIND			1
#define POSSIBLE_END_FIND	2

#define MAX_SPEED_ROTATION		(300)
#define MAX_SPEED_TRANSLATION   (300)
#define END_SPEED_TRANSLATION	(300)

#define DEBUG 3

#ifdef DEBUG
//#undef END_SPEED_TRANSLATION
//#define END_SPEED_TRANSLATION 10
#endif
#define END_OF_LIST 255
#include <stdlib.h>

//Structures typedef
typedef struct
{
  char wall_north;
  char wall_south;
  char wall_east;
  char wall_west;
  int length;
}one_cell;

typedef struct
{
  one_cell cell[MAZE_SIZE][MAZE_SIZE];
}labyrinthe;

typedef struct
{
	char front;
	char left;
	char right;
}walls;

typedef struct coordinate
{
  int x;
  int y;
}coordinate;
typedef struct
{
    coordinate coordinate_robot;
    char orientation;
    char midOfCell;
} positionRobot;


// fonctions
extern int maze(void);
int exploration(labyrinthe *maze, positionRobot* positionZhonx,const positionRobot *start_coordinates,
        coordinate *end_coordinate);
int goToPosition(labyrinthe *maze, positionRobot* positionZhonx,  coordinate end_coordinate);
int moveVirtualZhonx(labyrinthe maze, positionRobot positionZhonxVirtuel,
		coordinate way[], coordinate end_coordinate);
void poids(labyrinthe *maze, coordinate end_coordinate, char wallNoKnow, char contournKnownCell);
void mazeInit (labyrinthe *maze);
void clearMazelength(labyrinthe* maze);
char miniwayFind(labyrinthe *maze, coordinate start_coordinate, coordinate end_coordinate);
int moveRealZhonxArc(labyrinthe *maze, positionRobot *positionZhonx,
                     coordinate way[]);
void waitStart(void);
char diffway(coordinate way1[], coordinate way2[]);
coordinate findEndCoordinate (coordinate coordinate_tab[]);
int findArrival (labyrinthe maze, coordinate *end_coordinate);
#endif /* RESOLUTION_MAZE_H_ */
