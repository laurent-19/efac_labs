"""
Generate the CodeRun arena bitmaps (deterministic).

Stage reads dark pixels as obstacles. Resolution is 0.02 m/px, image row 0 is the
top of the world (+y), column 0 is the left (-x). The scorer reads the very same
PNGs to detect collisions and compute reachable area, so the maps live in exactly
one place.

Usage:
    python3 tools/gen_maps.py            # writes src/coderun_challenge/bitmaps/*.png
"""

import random
from collections import deque
from pathlib import Path

from PIL import Image, ImageDraw

RES = 0.02          # m / pixel
WALL = 0.2          # wall thickness (m)
ROBOT_R = 0.30      # inflation used for the reachability checks (m)

OUT = Path(__file__).resolve().parent.parent / "src" / "coderun_challenge" / "bitmaps"


class Arena:
    def __init__(self, size_m):
        self.size = size_m
        self.px = int(round(size_m / RES))
        self.img = Image.new("L", (self.px, self.px), 255)
        self.draw = ImageDraw.Draw(self.img)

    def to_px(self, x, y):
        return (x + self.size / 2) / RES, (self.size / 2 - y) / RES

    def rect(self, cx, cy, w, h):
        x0, y0 = self.to_px(cx - w / 2, cy + h / 2)
        x1, y1 = self.to_px(cx + w / 2, cy - h / 2)
        self.draw.rectangle([x0, y0, x1 - 1, y1 - 1], fill=0)

    def border(self):
        s, t = self.size, WALL
        self.rect(0, s / 2 - t / 2, s, t)
        self.rect(0, -s / 2 + t / 2, s, t)
        self.rect(-s / 2 + t / 2, 0, t, s)
        self.rect(s / 2 - t / 2, 0, t, s)

    def save(self, name):
        OUT.mkdir(parents=True, exist_ok=True)
        self.img.save(OUT / name)

    # -- analysis ------------------------------------------------------------
    def reachable(self, start_xy, radius=ROBOT_R):
        """Boolean grid of cells the robot centre can reach from start_xy."""
        import numpy as np
        from scipy import ndimage

        occ = np.array(self.img) < 128
        dist = ndimage.distance_transform_edt(~occ) * RES
        free = dist > radius
        c, r = self.to_px(*start_xy)
        c, r = int(c), int(r)
        assert free[r, c], f"start {start_xy} is not free"
        seen = np.zeros_like(free)
        q = deque([(r, c)])
        seen[r, c] = True
        while q:
            y, x = q.popleft()
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < free.shape[0] and 0 <= nx < free.shape[1] and free[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    q.append((ny, nx))
        return seen

    def is_reachable(self, start_xy, goal_xy):
        seen = self.reachable(start_xy)
        c, r = self.to_px(*goal_xy)
        return bool(seen[int(r), int(c)])


# ---------------------------------------------------------------------------
# EASY: 12 x 12 m arena with scattered obstacles, 3 waypoints given to students
# ---------------------------------------------------------------------------
EASY_START = (-5.0, -5.0)
EASY_WAYPOINTS = [(4.0, -4.0), (4.0, 4.0), (-4.0, 4.0)]
EASY_OBSTACLES = [  # cx, cy, w, h
    (-2.5, -4.0, 1.0, 1.0),
    (0.0, -2.5, 0.6, 3.0),
    (2.5, -5.0, 1.0, 1.0),
    (-3.0, 0.0, 3.0, 0.6),
    (2.0, 0.5, 3.0, 0.6),
    (0.0, 3.0, 0.6, 3.0),
    (-2.5, 3.5, 1.0, 1.0),
    (4.5, 2.0, 1.0, 1.0),
]


def make_easy():
    a = Arena(12.0)
    a.border()
    for o in EASY_OBSTACLES:
        a.rect(*o)
    pts = [EASY_START] + EASY_WAYPOINTS
    for p, q in zip(pts, pts[1:]):
        assert a.is_reachable(p, q), f"easy: {p} -> {q} not reachable"
    a.save("easy.png")
    return a


# ---------------------------------------------------------------------------
# HARD: 16 x 16 m, 4x4 grid of rooms, doors placed pseudo-randomly, pillars
# ---------------------------------------------------------------------------
HARD_START = (-6.0, -6.0)
DOOR = 1.2


def make_hard(seed=7):
    rng = random.Random(seed)
    a = Arena(16.0)
    a.border()
    lines = [-4.0, 0.0, 4.0]
    edges = [-8.0, -4.0, 0.0, 4.0, 8.0]

    # every wall line is cut into four 4 m sides; each side gets a door unless closed
    def side(fixed, lo, hi, vertical):
        length = hi - lo
        if rng.random() < 0.25:                       # closed side -> forces detours
            door = None
        else:
            door = lo + rng.uniform(0.6 + DOOR / 2, length - 0.6 - DOOR / 2)
        segs = [(lo, hi)] if door is None else [(lo, door - DOOR / 2), (door + DOOR / 2, hi)]
        for s0, s1 in segs:
            mid, ln = (s0 + s1) / 2, s1 - s0
            if vertical:
                a.rect(fixed, mid, WALL, ln)
            else:
                a.rect(mid, fixed, ln, WALL)

    for f in lines:
        for lo, hi in zip(edges, edges[1:]):
            side(f, lo, hi, vertical=True)
            side(f, lo, hi, vertical=False)

    # pillars in the middle of some rooms
    centres = [(-6, -2), (-2, -6), (2, -2), (6, 2), (-2, 6), (2, 6), (-6, 2), (6, -6)]
    for cx, cy in centres:
        a.rect(cx, cy, 0.8, 0.8)

    # if a closed side disconnected the maze, open the seed until it is connected
    seen = a.reachable(HARD_START)
    import numpy as np
    free_all = _free_cells(a)
    frac = seen.sum() / max(free_all.sum(), 1)
    return a, frac


def _free_cells(a):
    import numpy as np
    from scipy import ndimage
    occ = np.array(a.img) < 128
    return ndimage.distance_transform_edt(~occ) * RES > ROBOT_R


if __name__ == "__main__":
    make_easy()
    print("easy.png written")
    for seed in range(1, 200):
        arena, frac = make_hard(seed)
        if frac > 0.98:          # fully connected maze
            arena.save("hard.png")
            print(f"hard.png written (seed={seed}, reachable={frac:.3f})")
            break
    else:
        raise SystemExit("no fully connected hard maze found")
