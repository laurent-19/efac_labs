"""
CodeRun automatic scorer.

Listens to the simulator ground truth (/ground_truth), detects collisions against
the very same bitmap Stage uses, tracks task progress and writes a JSON result.

Point system (from the CodeRun guidelines):

            max   baseline   requirements   time bonus
    EASY    110      10           90        0.5 / min saved, max 10
    HARD    220      20          180        1.0 / min saved, max 20

Time bonus is only awarded when >= 50 % of the requirements are solved.
Every collision costs `collision_penalty` points of the requirement score.

EASY progress: waypoints reached in order / number of waypoints.
HARD progress: explored fraction of the reachable floor / `coverage_target`.
"""

import json
import math
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from scipy import ndimage

RES = 0.02            # bitmap resolution (m / px), must match the world files
ROBOT_R = 0.30        # inflation for the reachable-floor computation
COLLIDE_DIST = 0.28   # pose-to-wall distance below which the robot is touching
CLEAR_DIST = 0.34     # hysteresis: distance above which a new collision may count
CELL = 1.0            # coverage grid cell size (m)

MODES = {
    #        base  req_max  time_rate  time_max  penalty
    'easy': (10.0, 90.0, 0.5, 10.0, 5.0),
    'hard': (20.0, 180.0, 1.0, 20.0, 10.0),
}


class Scorer(Node):
    def __init__(self):
        super().__init__('coderun_scorer')
        share = get_package_share_directory('coderun_challenge')
        self.declare_parameter('mode', 'easy')
        self.declare_parameter('team', 'unnamed')
        self.declare_parameter('time_limit', 1200.0)      # seconds (20 min)
        self.declare_parameter('coverage_target', 0.8)
        self.declare_parameter('waypoint_radius', 0.5)
        self.declare_parameter('waypoints', [4.0, -4.0, 4.0, 4.0, -4.0, 4.0])
        self.declare_parameter('start', [-5.0, -5.0])
        self.declare_parameter('world_size', 12.0)
        self.declare_parameter('bitmap', '')
        self.declare_parameter('result_file', '/tmp/coderun_result.json')

        self.mode = self.get_parameter('mode').value
        if self.mode not in MODES:
            raise ValueError(f"mode must be one of {list(MODES)}")
        self.base, self.req_max, self.time_rate, self.time_max, self.penalty = MODES[self.mode]
        self.team = self.get_parameter('team').value
        self.time_limit = float(self.get_parameter('time_limit').value)
        self.cov_target = float(self.get_parameter('coverage_target').value)
        self.wp_radius = float(self.get_parameter('waypoint_radius').value)
        wp = list(self.get_parameter('waypoints').value)
        self.waypoints = list(zip(wp[0::2], wp[1::2]))
        self.size = float(self.get_parameter('world_size').value)
        self.result_file = self.get_parameter('result_file').value
        bitmap = self.get_parameter('bitmap').value or f'{share}/bitmaps/{self.mode}.png'
        start = list(self.get_parameter('start').value)

        # --- map ---------------------------------------------------------
        occ = np.array(Image.open(bitmap).convert('L')) < 128
        self.dist = ndimage.distance_transform_edt(~occ) * RES
        reach = self._flood(self.dist > ROBOT_R, start)
        n = int(round(self.size / CELL))
        self.cells = np.zeros((n, n), bool)             # reachable cells (counted)
        self.visited = np.zeros((n, n), bool)
        px_per_cell = int(round(CELL / RES))
        for i in range(n):
            for j in range(n):
                blk = reach[i * px_per_cell:(i + 1) * px_per_cell, j * px_per_cell:(j + 1) * px_per_cell]
                self.cells[i, j] = blk.mean() > 0.2
        self.get_logger().info(
            f'[{self.team}] mode={self.mode} bitmap={Path(bitmap).name} '
            f'reachable cells={int(self.cells.sum())} time_limit={self.time_limit:.0f}s')

        # --- state -------------------------------------------------------
        self.t0 = None
        self.collisions = 0
        self.in_collision = False
        self.next_wp = 0
        self.finished = False
        self.last_log = 0.0
        self.sub = self.create_subscription(Odometry, 'ground_truth', self.on_gt, 10)

    # ---------------------------------------------------------------- map --
    def _flood(self, free, start_xy):
        c, r = self._to_px(*start_xy)
        seen = np.zeros_like(free)
        if not free[r, c]:
            raise ValueError(f'start {start_xy} is not free in the bitmap')
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

    def _to_px(self, x, y):
        return int((x + self.size / 2) / RES), int((self.size / 2 - y) / RES)

    def _wall_dist(self, x, y):
        c, r = self._to_px(x, y)
        if not (0 <= r < self.dist.shape[0] and 0 <= c < self.dist.shape[1]):
            return 0.0
        return float(self.dist[r, c])

    # ----------------------------------------------------------- scoring ---
    def progress(self):
        if self.mode == 'easy':
            return self.next_wp / max(len(self.waypoints), 1)
        return min(1.0, self.coverage() / self.cov_target)

    def coverage(self):
        return float(self.visited[self.cells].sum()) / max(int(self.cells.sum()), 1)

    def score(self, elapsed, done):
        prog = self.progress()
        req = max(0.0, self.req_max * prog - self.penalty * self.collisions)
        time_pts = 0.0
        if prog >= 0.5 and done:
            minutes_saved = max(0.0, self.time_limit - elapsed) / 60.0
            time_pts = min(self.time_max, self.time_rate * minutes_saved)
        return {
            'baseline': self.base,
            'requirements': round(req, 2),
            'time_bonus': round(time_pts, 2),
            'total': round(self.base + req + time_pts, 2),
            'max_total': self.base + self.req_max + self.time_max,
        }

    # ----------------------------------------------------------- callback --
    def on_gt(self, msg):
        if self.finished:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = stamp
        elapsed = stamp - self.t0
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y

        # collisions (rising edge with hysteresis)
        d = self._wall_dist(x, y)
        if d < COLLIDE_DIST and not self.in_collision:
            self.in_collision = True
            self.collisions += 1
            self.get_logger().warn(f'collision #{self.collisions} at ({x:.2f}, {y:.2f}) t={elapsed:.0f}s')
        elif d > CLEAR_DIST:
            self.in_collision = False

        done = False
        if self.mode == 'easy':
            if self.next_wp < len(self.waypoints):
                wx, wy = self.waypoints[self.next_wp]
                if math.hypot(x - wx, y - wy) < self.wp_radius:
                    self.next_wp += 1
                    self.get_logger().info(f'waypoint {self.next_wp}/{len(self.waypoints)} reached at t={elapsed:.0f}s')
            done = self.next_wp >= len(self.waypoints)
        else:
            i = int((self.size / 2 - y) / CELL)
            j = int((x + self.size / 2) / CELL)
            if 0 <= i < self.visited.shape[0] and 0 <= j < self.visited.shape[1]:
                self.visited[i, j] = True
            done = self.coverage() >= self.cov_target

        if time.monotonic() - self.last_log > 5.0:
            self.last_log = time.monotonic()
            s = self.score(elapsed, done)
            extra = (f'wp={self.next_wp}/{len(self.waypoints)}' if self.mode == 'easy'
                     else f'coverage={self.coverage() * 100:.0f}%')
            self.get_logger().info(
                f't={elapsed:5.0f}s {extra} collisions={self.collisions} score~{s["total"]:.1f}')

        if done or elapsed >= self.time_limit:
            self.finish(elapsed, done)

    def finish(self, elapsed, done):
        self.finished = True
        s = self.score(elapsed, done)
        result = {
            'team': self.team,
            'mode': self.mode,
            'completed': bool(done),
            'elapsed_s': round(elapsed, 1),
            'collisions': self.collisions,
            'progress': round(self.progress(), 3),
            'coverage': round(self.coverage(), 3) if self.mode == 'hard' else None,
            'waypoints_reached': self.next_wp if self.mode == 'easy' else None,
            'score': s,
        }
        Path(self.result_file).write_text(json.dumps(result, indent=2))
        self.get_logger().info(
            f'FINISHED team={self.team} mode={self.mode} completed={done} '
            f'score={s["total"]}/{s["max_total"]} -> {self.result_file}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Scorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
