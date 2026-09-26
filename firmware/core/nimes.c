#include "nimes.h"
#include <limits.h>
#include <string.h>
#define STATES (NM_CELLS * 4)
int nm_neighbour(int cell, unsigned direction)
{
    if (cell < 0 || cell >= NM_CELLS || direction > 3) return -1;
    int x = cell % NM_SIDE, y = cell / NM_SIDE;
    switch (direction) {
    case NM_NORTH: return y + 1 < NM_SIDE ? cell + NM_SIDE : -1;
    case NM_EAST: return x + 1 < NM_SIDE ? cell + 1 : -1;
    case NM_SOUTH: return y ? cell - NM_SIDE : -1;
    default: return x ? cell - 1 : -1;
    }
}
void nm_init(nm_map_t *map)
{
    memset(map, 0, sizeof *map);
    for (int c = 0; c < NM_CELLS; ++c)
        for (unsigned d = 0; d < 4; ++d)
            if (nm_neighbour(c, d) < 0) {
                map->cell[c].known |= 1u << d;
                map->cell[c].walls |= 1u << d;
            }
}
int nm_edge(nm_map_t *map, int c, unsigned d, int wall)
{
    if (c < 0 || c >= NM_CELLS || d > 3) return -1;
    int n = nm_neighbour(c, d);
    unsigned bit = 1u << d, opposite = 1u << ((d + 2) % 4);
    if (n < 0 && !wall) return -1;
    if ((map->cell[c].known & bit) && !!(map->cell[c].walls & bit) != !!wall) return -1;
    if (n >= 0 && (map->cell[n].known & opposite) && !!(map->cell[n].walls & opposite) != !!wall) return -1;
    map->cell[c].known |= bit;
    if (wall) map->cell[c].walls |= bit;
    if (n >= 0) {
        map->cell[n].known |= opposite;
        if (wall) map->cell[n].walls |= opposite;
    }
    return 0;
}
int nm_observe(nm_map_t *map, nm_pose_t p, uint8_t walls)
{
    if (p.x >= NM_SIDE || p.y >= NM_SIDE || p.heading > 3) return -1;
    nm_map_t copy = *map; /* No partial observation when a sensor contradicts a known edge. */
    unsigned dirs[3] = {p.heading, (p.heading + 3u) % 4u, (p.heading + 1u) % 4u};
    int c = p.y * NM_SIDE + p.x;
    for (unsigned i = 0; i < 3; ++i)
        if (nm_edge(&copy, c, dirs[i], walls & (1u << i))) return -1;
    copy.cell[c].visited = 1;
    *map = copy;
    return 0;
}
static int open_edge(const nm_map_t *map, int c, unsigned d)
{
    unsigned b = 1u << d;
    return (map->cell[c].known & b) && !(map->cell[c].walls & b);
}
int nm_goal(const nm_map_t *map, uint8_t targets[NM_CELLS])
{
    int rooms = 0;
    memset(targets, 0, NM_CELLS);
    for (int y = 0; y < NM_SIDE - 1; ++y) for (int x = 0; x < NM_SIDE - 1; ++x) {
        int c = y * NM_SIDE + x;
        if (open_edge(map,c,NM_EAST) && open_edge(map,c,NM_NORTH) &&
            open_edge(map,c+1,NM_NORTH) && open_edge(map,c+NM_SIDE,NM_EAST)) {
            ++rooms;
            targets[c] = targets[c+1] = targets[c+NM_SIDE] = targets[c+NM_SIDE+1] = 1;
        }
    }
    return rooms;
}
int nm_route(const nm_map_t *map, nm_pose_t from, const uint8_t targets[NM_CELLS],
             int unknown, nm_route_t *out)
{
    uint32_t dist[STATES];
    int16_t prev[STATES];
    uint8_t used[STATES] = {0};
    out->length = 0; out->cost = UINT32_MAX;
    if (from.x >= NM_SIDE || from.y >= NM_SIDE || from.heading > 3) return -1;
    for (int i = 0; i < STATES; ++i) { dist[i] = UINT32_MAX; prev[i] = -1; }
    int start = (from.y * NM_SIDE + from.x) * 4 + from.heading, end = -1;
    dist[start] = 0;
    for (int count = 0; count < STATES; ++count) {
        int s = -1;
        for (int i = 0; i < STATES; ++i)
            if (!used[i] && dist[i] != UINT32_MAX && (s < 0 || dist[i] < dist[s])) s = i;
        if (s < 0) break;
        if (targets[s/4]) { end = s; break; }
        used[s] = 1;
        for (unsigned d = 0; d < 4; ++d) {
            int c = s / 4, n = nm_neighbour(c,d);
            if (n < 0 || (map->cell[c].walls & (1u << d)) || (!unknown && !open_edge(map,c,d))) continue;
            unsigned turn = (d + 4u - s % 4) % 4u;
            if (turn == 3) turn = 1;
            /* Cell count first, quarter-turns second. Max simple route < 1024 turns. */
            uint32_t cost = dist[s] + 1024u + turn;
            if (cost < dist[n*4+d]) { dist[n*4+d] = cost; prev[n*4+d] = s; }
        }
    }
    if (end < 0) return -1;
    out->cost = dist[end];
    for (int s = end; s != start; s = prev[s]) {
        if (s < 0 || out->length >= NM_CELLS) return -1;
        out->direction[out->length++] = s % 4;
    }
    for (unsigned i = 0; i < out->length / 2; ++i) {
        uint8_t t = out->direction[i];
        out->direction[i] = out->direction[out->length-1-i];
        out->direction[out->length-1-i] = t;
    }
    return 0;
}
int nm_frontier(const nm_map_t *map, nm_pose_t from, nm_route_t *route)
{
    uint8_t targets[NM_CELLS];
    for (int c = 0; c < NM_CELLS; ++c) targets[c] = !map->cell[c].visited || map->cell[c].known != 15;
    return nm_route(map, from, targets, 0, route);
}
int nm_refine(const nm_map_t *map, nm_pose_t from, nm_pose_t start,
              const uint8_t goals[NM_CELLS], nm_route_t *route)
{
    nm_route_t known, optimistic;
    if (nm_route(map, start, goals, 1, &optimistic)) return -1;
    if (!nm_route(map,start,goals,0,&known) && known.cost == optimistic.cost) return 1;
    int c = start.y * NM_SIDE + start.x;
    for (unsigned i = 0; i < optimistic.length; ++i) {
        unsigned d = optimistic.direction[i];
        if (!open_edge(map,c,d)) {
            uint8_t target[NM_CELLS] = {0}; target[c] = 1;
            return nm_route(map,from,target,0,route);
        }
        c = nm_neighbour(c,d);
    }
    return -1;
}
int nm_valid(const nm_map_t *map)
{
    for (int c = 0; c < NM_CELLS; ++c) for (unsigned d = 0; d < 4; ++d) {
        const nm_cell_t *p = &map->cell[c];
        if (p->known > 15 || (p->walls & ~p->known) || p->visited > 1) return 0;
        unsigned b = 1u << d, ob = 1u << ((d+2)%4);
        int n = nm_neighbour(c,d);
        if (n < 0) { if (!(p->known & p->walls & b)) return 0; }
        else if (!!(p->known & b) != !!(map->cell[n].known & ob) ||
                 !!(p->walls & b) != !!(map->cell[n].walls & ob)) return 0;
    }
    return 1;
}
