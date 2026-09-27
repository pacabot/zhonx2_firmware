#ifndef FW_LIBRARY_H
#define FW_LIBRARY_H
#include "nimes.h"
#define FW_MAZE_SLOTS 8u
typedef struct {
    nm_map_t map;
    nm_route_t route;
    uint32_t id, corner, heading, search_ms;
} fw_saved_maze_t;
typedef struct { uint32_t count, next_id; fw_saved_maze_t item[FW_MAZE_SLOTS]; } fw_library_t;
nm_pose_t fw_maze_origin(unsigned corner,unsigned heading);
int fw_maze_certify(const nm_map_t *,unsigned corner,unsigned heading,nm_route_t *);
int fw_library_valid(const fw_library_t *);
/* Deduplicate identical learned maps; never silently evict an older maze. */
int fw_library_put(fw_library_t *,const nm_map_t *,unsigned corner,unsigned heading,uint32_t ms);
int fw_library_remove(fw_library_t *,unsigned index);
/* Five first distinct route motifs; display only, no flash format change. */
void fw_maze_name(const fw_saved_maze_t *,char name[6]);
#endif
