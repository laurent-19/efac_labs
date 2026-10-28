"""
Random goal sampling (pure numpy/scipy, no ROS) - used by the launch file and by the
stress test in tools/gen_maps.py.

All coordinates are in the start frame (robot starts at (0, 0, 0)); the bitmap is placed
by `map_origin` (the map centre in that frame).
"""

import math
import random
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

RES = 0.02                 # bitmap resolution (m / px)
ROBOT_R = 0.30             # robot radius used for reachability / line-of-sight (m)
GOAL_CLEARANCE = 0.6       # a goal centre is at least this far from any wall (m)
MANHATTAN_CLEARANCE = 0.45 # x-then-y route must keep this far from walls (m)
EDGE_MARGIN = 0.8          # keep goals away from the map border (m)
SEP_FROM_OTHERS = 1.5      # goals / start are at least this far from each other (m)


class MapGrid:
    def __init__(self, bitmap, size, origin):
        self.occ = np.array(Image.open(bitmap).convert('L')) < 128
        self.dist = ndimage.distance_transform_edt(~self.occ) * RES
        self.size = float(size)
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.labels, _ = ndimage.label(self.dist > ROBOT_R)     # connected free areas

    def _px(self, x, y):
        c = int((x - self.ox + self.size / 2) / RES)
        r = int((self.size / 2 - (y - self.oy)) / RES)
        return r, c

    def _inside(self, r, c):
        return 0 <= r < self.dist.shape[0] and 0 <= c < self.dist.shape[1]

    def wall_dist(self, x, y):
        r, c = self._px(x, y)
        return float(self.dist[r, c]) if self._inside(r, c) else 0.0

    def component(self, x, y):
        r, c = self._px(x, y)
        return int(self.labels[r, c]) if self._inside(r, c) else 0

    def connected(self, a, b):
        la = self.component(*a)
        return la != 0 and la == self.component(*b)

    def segment_min_dist(self, a, b, step=0.05):
        n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / step) + 1
        return min(self.wall_dist(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
                   for t in np.linspace(0.0, 1.0, n))

    def line_free(self, a, b):
        return self.segment_min_dist(a, b) > ROBOT_R

    def manhattan_clear(self, a, b):
        corner = (b[0], a[1])
        return (self.segment_min_dist(a, corner) >= MANHATTAN_CLEARANCE
                and self.segment_min_dist(corner, b) >= MANHATTAN_CLEARANCE)


def sample_goals(cfg, bitmap, seed):
    """Static goals of the config followed by `random_goals` sampled ones (list of dicts)."""
    grid = MapGrid(bitmap, cfg['world_size'], (cfg['map_origin']['x'], cfg['map_origin']['y']))
    rng = random.Random(seed)
    rules = cfg.get('sampling') or {}
    goals = [dict(g) for g in (cfg.get('goals') or [])]
    pts = [(0.0, 0.0)] + [(g['x'], g['y']) for g in goals]
    half = cfg['world_size'] / 2 - EDGE_MARGIN

    for k in range(int(cfg.get('random_goals', 0))):
        prev = pts[-1]
        for _ in range(20000):
            x = round(grid.ox + rng.uniform(-half, half), 2)
            y = round(grid.oy + rng.uniform(-half, half), 2)
            if grid.wall_dist(x, y) < GOAL_CLEARANCE:
                continue
            if not grid.connected(prev, (x, y)):
                continue
            if math.hypot(x - prev[0], y - prev[1]) < rules.get('min_separation', 3.0):
                continue
            if any(math.hypot(x - p[0], y - p[1]) < SEP_FROM_OTHERS for p in pts):
                continue
            if rules.get('blocked_line') and grid.line_free(prev, (x, y)):
                continue
            if rules.get('manhattan') and not grid.manhattan_clear(prev, (x, y)):
                continue
            goals.append({'x': x, 'y': y, 'yaw_deg': float(rng.choice(range(0, 360, 45)))})
            pts.append((x, y))
            break
        else:
            raise RuntimeError(f'could not sample random goal {k + 1}: rules too strict for this map')
    return goals


def bitmap_path(share, task):
    return str(Path(share) / 'bitmaps' / f'{task}.png')
