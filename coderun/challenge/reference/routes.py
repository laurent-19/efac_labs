"""
ORGANIZERS ONLY. Route planning of the reference solution (pure numpy/scipy, no ROS), also
used by tools/gen_maps.py to check that random goals are always solvable.

All coordinates are in the start frame (the frame of /odom, /goals and /map).
A route is a list of (x, y, goal_index_or_None): drive to each point in turn along a
straight leg; at points with a goal index also turn to that goal's yaw.
"""

import heapq
import math

import numpy as np
from scipy import ndimage


def manhattan_route(start_xy, goals_xy):
    """EASY: for each goal drive along x, then along y (straight lines + 90 deg turns)."""
    route, cur = [], start_xy
    for i, (gx, gy) in enumerate(goals_xy):
        if abs(gx - cur[0]) > 1e-6 and abs(gy - cur[1]) > 1e-6:
            route.append((gx, cur[1], None))
        route.append((gx, gy, i))
        cur = (gx, gy)
    return route


# ---------------------------------------------------------------- grid planner --
# `occ` is a boolean grid, row index = y, column index = x, row 0 = the lowest y (this is the
# layout of nav_msgs/OccupancyGrid); cell (r, c) has its lower-left corner at
# origin + (c, r) * res.

def free_space(occ, res, inflate):
    """Cells whose centre is more than `inflate` metres away from any obstacle."""
    return ndimage.distance_transform_edt(~occ) * res > inflate


def _cell(origin, res, x, y):
    return int((y - origin[1]) / res), int((x - origin[0]) / res)


def _world(origin, res, cell):
    return origin[0] + (cell[1] + 0.5) * res, origin[1] + (cell[0] + 0.5) * res


def astar(free, s, g):
    """8-connected A* on the boolean grid `free` from cell s to cell g -> list of cells or None."""
    h, w = free.shape
    if not (free[s] and free[g]):
        return None
    inf = float('inf')
    cost = np.full(free.shape, inf)
    prev = {}
    cost[s] = 0.0
    heap = [(0.0, s)]

    def heur(c):
        dr, dc = abs(c[0] - g[0]), abs(c[1] - g[1])
        return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)

    while heap:
        _, cur = heapq.heappop(heap)
        if cur == g:
            path = [cur]
            while cur in prev:
                cur = prev[cur]
                path.append(cur)
            return path[::-1]
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                n = (cur[0] + dr, cur[1] + dc)
                if not (0 <= n[0] < h and 0 <= n[1] < w) or not free[n]:
                    continue
                if dr and dc and not (free[cur[0] + dr, cur[1]] and free[cur[0], cur[1] + dc]):
                    continue        # no corner cutting
                nc = cost[cur] + (math.sqrt(2) if dr and dc else 1.0)
                if nc < cost[n]:
                    cost[n] = nc
                    prev[n] = cur
                    heapq.heappush(heap, (nc + heur(n), n))
    return None


def _line_free(free, a, b):
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
    for t in np.linspace(0.0, 1.0, n + 1):
        r = int(round(a[0] + t * (b[0] - a[0])))
        c = int(round(a[1] + t * (b[1] - a[1])))
        if not free[r, c]:
            return False
    return True


def smooth(free, path):
    """Greedy line-of-sight shortcutting of a cell path."""
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _line_free(free, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def plan_route(occ, res, origin, start_xy, goals_xy, inflate=0.4):
    """A* between consecutive goals on the inflated map -> route [(x, y, goal_index_or_None)].
    Raises RuntimeError if a goal cannot be reached."""
    free = free_space(occ, res, inflate)
    route, cur = [], start_xy
    for i, goal in enumerate(goals_xy):
        path = astar(free, _cell(origin, res, *cur), _cell(origin, res, *goal))
        if path is None:
            raise RuntimeError(f'no path to goal {i + 1} at {goal}')
        wps = smooth(free, path)[1:]
        if not wps:
            route.append((goal[0], goal[1], i))
        for k, cell in enumerate(wps):
            x, y = _world(origin, res, cell)
            last = k == len(wps) - 1
            route.append((goal[0], goal[1], i) if last else (x, y, None))
        cur = goal
    return route
