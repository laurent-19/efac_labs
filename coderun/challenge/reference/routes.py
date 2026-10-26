"""
ORGANIZERS ONLY. Routes of the reference solution, shared with tools/gen_maps.py so the
maps are checked to be drivable along them.

A route is a list of (x, y, goal_index_or_None): drive to each point in turn along an
axis-aligned leg; at points with a goal index also turn to that goal's yaw.
"""


def manhattan_route(start_xy, goals):
    """EASY: for each goal drive along x, then along y (straight lines + 90 deg turns)."""
    route, cur = [], start_xy
    for i, g in enumerate(goals):
        if abs(g['x'] - cur[0]) > 1e-6 and abs(g['y'] - cur[1]) > 1e-6:
            route.append((g['x'], cur[1], None))
        route.append((g['x'], g['y'], i))
        cur = (g['x'], g['y'])
    return route


# HARD: hand-made axis-aligned route through the gaps of the staggered walls
HARD_ROUTE = [
    (4.5, -5.2, None),
    (4.5, -4.5, 0),
    (4.5, -0.5, 1),
    (4.5, 0.2, None),
    (-4.5, 0.2, None),
    (-4.5, 2.2, 2),
    (4.5, 2.2, None),
    (4.5, 5.0, None),
    (-4.5, 5.0, 3),
]
