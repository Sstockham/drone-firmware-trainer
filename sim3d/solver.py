"""Coarse-grid A* solvability check for World3D layouts.

Used by build_world3d to filter out impossible procedurally-generated worlds
before the firmware ever runs against them. Intentionally an under-approximation
of true reachability: the 0.5 m grid is coarser than the drone (0.3 m cube), so
a True result means "there's room for a 0.5 m-resolution path" — the firmware
still has to fly it.
"""

import heapq
import math

from sim3d.world import (
    ARENA_W, ARENA_D, CEILING,
    SPAWN, GOAL_CENTER, GOAL_RADIUS,
    Rect3D, World3D,
)

GRID_RES = 0.5
NX = int(ARENA_W / GRID_RES)
NY = int(ARENA_D / GRID_RES)
NZ = int(CEILING / GRID_RES)


def _cell_center(i: int, j: int, k: int) -> tuple[float, float, float]:
    return (
        (i + 0.5) * GRID_RES,
        (j + 0.5) * GRID_RES,
        (k + 0.5) * GRID_RES,
    )


def _cell_of(px: float, py: float, pz: float) -> tuple[int, int, int]:
    return (
        max(0, min(NX - 1, int(px / GRID_RES))),
        max(0, min(NY - 1, int(py / GRID_RES))),
        max(0, min(NZ - 1, int(pz / GRID_RES))),
    )


def _blocked(i: int, j: int, k: int, rects: list[Rect3D]) -> bool:
    cx, cy, cz = _cell_center(i, j, k)
    return any(r.contains_point3d(cx, cy, cz) for r in rects)


def _is_goal_cell(i: int, j: int, k: int) -> bool:
    cx, cy, cz = _cell_center(i, j, k)
    gx, gy, gz = GOAL_CENTER
    return math.sqrt((cx - gx) ** 2 + (cy - gy) ** 2 + (cz - gz) ** 2) <= GOAL_RADIUS


def has_path(world: World3D) -> bool:
    rects = world.obstacles
    start = _cell_of(*world.spawn)
    if _blocked(*start, rects):
        return False

    open_set: list[tuple[int, int, tuple[int, int, int]]] = []
    counter = 0
    heapq.heappush(open_set, (0, counter, start))
    g_score: dict[tuple[int, int, int], int] = {start: 0}

    neighbors = (
        (1, 0, 0), (-1, 0, 0),
        (0, 1, 0), (0, -1, 0),
        (0, 0, 1), (0, 0, -1),
    )

    while open_set:
        _, _, (i, j, k) = heapq.heappop(open_set)
        if _is_goal_cell(i, j, k):
            return True
        for di, dj, dk in neighbors:
            ni, nj, nk = i + di, j + dj, k + dk
            if not (0 <= ni < NX and 0 <= nj < NY and 0 <= nk < NZ):
                continue
            if _blocked(ni, nj, nk, rects):
                continue
            tentative = g_score[(i, j, k)] + 1
            if tentative >= g_score.get((ni, nj, nk), 10 ** 9):
                continue
            g_score[(ni, nj, nk)] = tentative
            gx, gy, gz = GOAL_CENTER
            cx, cy, cz = _cell_center(ni, nj, nk)
            heur = int((abs(cx - gx) + abs(cy - gy) + abs(cz - gz)) / GRID_RES)
            counter += 1
            heapq.heappush(open_set, (tentative + heur, counter, (ni, nj, nk)))

    return False
