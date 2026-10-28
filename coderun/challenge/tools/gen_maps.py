"""
Generate the CodeRun arena bitmaps (deterministic) and stress-test the random goals.

Frames
  * map frame   : centre of the bitmap = (0, 0). Obstacles below are drawn in it.
  * start frame : the robot starts at (0, 0, 0); /odom, /ground_truth, /goals and /map are in
                  it. start = map - map_origin, `map_origin` (in config/*_goals.yaml) being the
                  map centre expressed in the start frame.

Stage reads dark pixels as obstacles. Resolution is 0.02 m/px, image row 0 is the top of
the map (+y), column 0 is the left (-x). The scorer reads the very same PNGs.

The world files are NOT generated here: the goals are random, so the launch file writes the
world (with the goal markers) at every launch (coderun_challenge/worldgen.py).

Checks (the script fails loudly if one is violated):
  * the start yaw is 0 (the floorplan is only translated)
  * static goals are free, away from walls and connected, in order
  * for SEEDS random seeds: goals are sampled under the rules (clearance, reachable, separation,
    blocked line for HARD, clear x-then-y route for EASY) and the reference planner finds a
    route to every goal

Usage:
    python3 tools/gen_maps.py [--seeds N]   # writes bitmaps/*.png, then runs the checks
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

RES = 0.02              # m / pixel
WALL = 0.2              # wall thickness (m)
GOAL_CLEARANCE = 0.5    # goals must be this far from any wall (m)

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / 'src' / 'coderun_challenge'
sys.path.insert(0, str(ROOT / 'reference'))
sys.path.insert(0, str(PKG))
from coderun_challenge import sampler  # noqa: E402
from routes import manhattan_route, plan_route  # noqa: E402


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

    def save(self, task):
        out = PKG / 'bitmaps'
        out.mkdir(parents=True, exist_ok=True)
        self.img.save(out / f'{task}.png')


def load(task):
    cfg = yaml.safe_load((PKG / 'config' / f'{task}_goals.yaml').read_text())
    assert abs(cfg['start']['x']) + abs(cfg['start']['y']) + abs(cfg['start']['yaw_deg']) == 0, \
        f'{task}: the start pose must be (0, 0, 0) in the start frame'
    return cfg


def planner_grid(task, cfg):
    """The /map the students get (0.04 m cells, row 0 = lowest y) and its origin."""
    occ = np.array(Image.open(PKG / 'bitmaps' / f'{task}.png').convert('L')) < 128
    f = 2
    h, w = occ.shape[0] // f * f, occ.shape[1] // f * f
    occ = np.flipud(occ[:h, :w].reshape(h // f, f, w // f, f).max(axis=(1, 3)))
    s = cfg['world_size']
    return occ, RES * f, (cfg['map_origin']['x'] - s / 2, cfg['map_origin']['y'] - s / 2)


def check(task, cfg, seeds):
    bitmap = str(PKG / 'bitmaps' / f'{task}.png')
    grid = sampler.MapGrid(bitmap, cfg['world_size'], (cfg['map_origin']['x'], cfg['map_origin']['y']))

    # static goals
    prev = (0.0, 0.0)
    for g in cfg.get('goals') or []:
        p = (g['x'], g['y'])
        assert grid.connected(prev, p), f'{task}: static goal {p} not connected to {prev}'
        w = grid.wall_dist(*p)
        assert w >= GOAL_CLEARANCE, f'{task}: static goal {g} only {w:.2f} m from a wall'
        prev = p

    # random goals: sample, re-verify independently, plan with the reference planner
    occ, res, origin = planner_grid(task, cfg)
    lengths = []
    for seed in range(seeds):
        goals = sampler.sample_goals(cfg, bitmap, seed)
        assert len(goals) == len(cfg.get('goals') or []) + int(cfg.get('random_goals', 0))
        xy = [(g['x'], g['y']) for g in goals]
        pts = [(0.0, 0.0)] + xy
        for a, b in zip(pts, pts[1:]):
            assert grid.connected(a, b), f'{task} seed {seed}: {b} not connected to {a}'
            assert grid.wall_dist(*b) >= GOAL_CLEARANCE, f'{task} seed {seed}: goal {b} too close to a wall'
        rules = cfg.get('sampling') or {}
        if rules.get('manhattan'):
            r = manhattan_route((0.0, 0.0), xy)
            prev = (0.0, 0.0)
            for x, y, _ in r:
                assert grid.segment_min_dist(prev, (x, y)) >= sampler.MANHATTAN_CLEARANCE, \
                    f'{task} seed {seed}: x-then-y route too close to a wall near {(x, y)}'
                prev = (x, y)
        route = plan_route(occ, res, origin, (0.0, 0.0), xy)
        length, prev = 0.0, (0.0, 0.0)
        for x, y, _ in route:
            length += math.hypot(x - prev[0], y - prev[1])
            prev = (x, y)
        lengths.append(length)
    print(f'{task}: ok - static goals fine, {seeds} random seeds sampled and planned '
          f'(route length {min(lengths):.0f}-{max(lengths):.0f} m, mean {sum(lengths) / len(lengths):.0f} m)')


# ---------------------------------------------------------------------------
# EASY: sparse obstacles (map frame), kept away from the axis-aligned (x then y) routes
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
    a.save('easy')
    return cfg


# ---------------------------------------------------------------------------
# HARD: staggered walls (gaps alternate right / left / right) and a few pillars (map frame)
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
    a.save('hard')
    return cfg


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, default=30, help='random seeds to stress-test per task')
    args = ap.parse_args()
    easy, hard = make_easy(), make_hard()
    check('easy', easy, args.seeds)
    check('hard', hard, args.seeds)
    print('bitmaps written to', PKG / 'bitmaps')
