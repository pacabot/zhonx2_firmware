#ifndef NIMES_H
#define NIMES_H
#include <stdint.h>
#define NM_SIDE 9
#define NM_CELLS (NM_SIDE * NM_SIDE)
#define NM_SEARCH_MS UINT32_C(300000)
enum { NM_NORTH, NM_EAST, NM_SOUTH, NM_WEST };
typedef struct { uint8_t known, walls, visited, reserved; } nm_cell_t;
typedef struct { nm_cell_t cell[NM_CELLS]; } nm_map_t;
typedef struct { uint8_t x, y, heading; } nm_pose_t;
typedef struct { uint8_t direction[NM_CELLS]; uint16_t length; uint32_t cost; } nm_route_t;
void nm_init(nm_map_t *map);
int nm_neighbour(int cell, unsigned direction);
int nm_edge(nm_map_t *map, int cell, unsigned direction, int wall);
/* Observe front, left, right at a cell centre; bit 0=front, 1=left, 2=right. */
int nm_observe(nm_map_t *map, nm_pose_t pose, uint8_t walls);
int nm_goal(const nm_map_t *map, uint8_t targets[NM_CELLS]);
int nm_route(const nm_map_t *map, nm_pose_t from, const uint8_t targets[NM_CELLS],
             int allow_unknown, nm_route_t *route);
int nm_frontier(const nm_map_t *map, nm_pose_t from, nm_route_t *route);
/* Compare fully known path with the optimistic bound, then target its first unknown edge. */
int nm_refine(const nm_map_t *map, nm_pose_t from, nm_pose_t start,
              const uint8_t goals[NM_CELLS], nm_route_t *route);
int nm_valid(const nm_map_t *map);
#endif
