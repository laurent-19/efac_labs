"""
Generate the CodeRun arena bitmaps (deterministic) and check them against the goal files.

Stage reads dark pixels as obstacles. Resolution is 0.02 m/px, image row 0 is the top of
the world (+y), column 0 is the left (-x). The scorer reads the very same PNGs to detect
collisions, and the very same config/*_goals.yaml for start and goals.

Checks (the script fails loudly if one is violated):
  * start and every goal are free and connected, in order (robot radius inflation)
  * every goal pose is at least GOAL_CLEARANCE from any wall
  * the reference route stays at least ROUTE_CLEARANCE from any wall

Usage:
    python3 tools/gen_maps.py          # writes src/coderun_challenge/bitmaps/*.png
"""

import sys
from collections import deque
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage

RES = 0.02              # m / pixel
WALL = 0.2              # wall thickness (m)
ROBOT_R = 0.30          # inflation used for the reachability checks (m)
GOAL_CLEARANCE = 0.5    # goals must be this far from any wall (m)
ROUTE_CLEARANCE = 0.45  # reference route must stay this far from any wall (m)

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / 'src' / 'coderun_challenge'
OUT = PKG / 'bitmaps'
sys.path.insert(0, str(ROOT / 'reference'))
from routes import HARD_ROUTE, manhattan_route  # noqa: E402


class Arena:
    def __init__(self, size_m):
        self.size = size_m
        self.px = int(round(size_m / RES))
        self.img = Image.new('L', (self.px, self.px), 255)
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

    def hwall(self, y, x0, x1):
        self.rect((x0 + x1) / 2, y, x1 - x0, WALL)

    def save(self, name):
        OUT.mkdir(parents=True, exist_ok=True)
        self.img.save(OUT / name)

    # -- analysis ----------------------------------------------------------
    def dist(self):
        occ = np.array(self.img) < 128
        return ndimage.distance_transform_edt(~occ) * RES

    def wall_dist(self, d, x, y):
        c, r = self.to_px(x, y)
        return float(d[int(r), int(c)])

    def reachable(self, d, start_xy):
        free = d > ROBOT_R
        c, r = self.to_px(*start_xy)
        c, r = int(c), int(r)
        assert free[r, c], f'start/goal {start_xy} is not free'
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


def load(task):
    return yaml.safe_load((PKG / 'config' / f'{task}_goals.yaml').read_text())


def check(arena, task, cfg, route):
    d = arena.dist()
    start = (cfg['start']['x'], cfg['start']['y'])
    pts = [start] + [(g['x'], g['y']) for g in cfg['goals']]
    for p, q in zip(pts, pts[1:]):
        seen = arena.reachable(d, p)
        c, r = arena.to_px(*q)
        assert seen[int(r), int(c)], f'{task}: {p} -> {q} not connected'
    for g in cfg['goals']:
        w = arena.wall_dist(d, g['x'], g['y'])
        assert w >= GOAL_CLEARANCE, f"{task}: goal {g} only {w:.2f} m from a wall"
    prev = start
    for x, y, _ in route:
        n = int(max(abs(x - prev[0]), abs(y - prev[1])) / 0.05) + 1
        for t in np.linspace(0, 1, n):
            px, py = prev[0] + t * (x - prev[0]), prev[1] + t * (y - prev[1])
            w = arena.wall_dist(d, px, py)
            assert w >= ROUTE_CLEARANCE, f'{task}: reference route {w:.2f} m from a wall at ({px:.2f}, {py:.2f})'
        prev = (x, y)
    print(f'{task}: ok ({len(cfg["goals"])} goals, route of {len(route)} legs)')


# ---------------------------------------------------------------------------
# EASY: sparse obstacles, kept away from the axis-aligned (x then y) routes
# ---------------------------------------------------------------------------
EASY_OBSTACLES = [  # cx, cy, w, h
    (-3.0, -2.0, 1.0, 1.0),
    (2.5, -3.0, 1.2, 1.2),
    (-2.0, 1.75, 0.6, 2.5),
    (2.0, 2.5, 1.0, 1.0),
    (-5.0, 1.0, 0.8, 0.8),
]


def make_easy():
    cfg = load('easy')
    a = Arena(cfg['world_size'])
    a.border()
    for o in EASY_OBSTACLES:
        a.rect(*o)
    start = (cfg['start']['x'], cfg['start']['y'])
    check(a, 'easy', cfg, manhattan_route(start, cfg['goals']))
    a.save('easy.png')


# ---------------------------------------------------------------------------
# HARD: staggered walls (gaps alternate right / left / right) and a few pillars
# ---------------------------------------------------------------------------
HARD_WALLS = [  # y, x0, x1
    (-2.5, -5.9, 3.0),   # gap on the right
    (1.0, -3.0, 5.9),    # gap on the left
    (3.5, -5.9, 3.0),    # gap on the right
]
HARD_PILLARS = [(-1.5, -4.3, 0.8, 0.8), (0.5, -0.8, 0.8, 0.8)]


def make_hard():
    cfg = load('hard')
    a = Arena(cfg['world_size'])
    a.border()
    for y, x0, x1 in HARD_WALLS:
        a.hwall(y, x0, x1)
    for p in HARD_PILLARS:
        a.rect(*p)
    check(a, 'hard', cfg, HARD_ROUTE)
    a.save('hard.png')


if __name__ == '__main__':
    make_easy()
    make_hard()
    print('bitmaps written to', OUT)
