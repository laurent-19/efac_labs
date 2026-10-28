"""
CodeRun automatic scorer.

Listens to the simulator ground truth (/ground_truth), detects collisions against the
very same bitmap Stage uses, tracks the goal poses reached and writes a JSON result.

A goal counts when, in order, the robot is within `goal_radius` of it AND its heading is
within `yaw_tolerance_deg` of the goal yaw (both read from config/<task>_goals.yaml).

Point system (from the CodeRun guidelines):

            max   baseline   requirements   time bonus
    EASY    110      10           90        0.5 / min saved, max 10
    HARD    220      20          180        1.0 / min saved, max 20

Requirements = max_req * goals_reached / goals_total - penalty * collisions (min 0).
Time bonus = rate * minutes between the finish and the time limit, only awarded when the
run is complete and >= 50 % of the requirements are solved. The faster, the better.
"""

import json
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy import ndimage

RES = 0.02            # bitmap resolution (m / px), must match the world files
COLLIDE_DIST = 0.28   # pose-to-wall distance below which the robot is touching
CLEAR_DIST = 0.34     # hysteresis: distance above which a new collision may count

MODES = {
    #        base  req_max  time_rate  time_max  penalty
    'easy': (10.0, 90.0, 0.5, 10.0, 5.0),
    'hard': (20.0, 180.0, 1.0, 20.0, 10.0),
}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class Scorer(Node):
    def __init__(self):
        super().__init__('coderun_scorer')
        share = get_package_share_directory('coderun_challenge')
        self.declare_parameter('mode', 'easy')
        self.declare_parameter('team', 'unnamed')
        self.declare_parameter('time_limit', 1200.0)      # seconds (20 min)
        self.declare_parameter('goals_file', '')
        self.declare_parameter('goals', Parameter.Type.DOUBLE_ARRAY)   # [x, y, yaw_rad, ...] of this run
        self.declare_parameter('seed', '')
        self.declare_parameter('bitmap', '')
        self.declare_parameter('result_file', '/tmp/coderun_result.json')

        self.mode = self.get_parameter('mode').value
        if self.mode not in MODES:
            raise ValueError(f'mode must be one of {list(MODES)}')
        self.base, self.req_max, self.time_rate, self.time_max, self.penalty = MODES[self.mode]
        self.team = self.get_parameter('team').value
        self.time_limit = float(self.get_parameter('time_limit').value)
        self.result_file = self.get_parameter('result_file').value

        goals_file = self.get_parameter('goals_file').value or f'{share}/config/{self.mode}_goals.yaml'
        cfg = yaml.safe_load(Path(goals_file).read_text())
        self.size = float(cfg['world_size'])
        # map centre in the start frame (the frame of the goals and of /ground_truth)
        self.ox = float(cfg['map_origin']['x'])
        self.oy = float(cfg['map_origin']['y'])
        self.radius = float(cfg['goal_radius'])
        self.yaw_tol = math.radians(cfg['yaw_tolerance_deg'])
        # goals of THIS run (random ones included) come from the launch file; without them only
        # the static goals of the YAML are used
        flat = list(self.get_parameter('goals').value or [])
        if flat:
            self.goals = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]
        else:
            self.goals = [(g['x'], g['y'], math.radians(g['yaw_deg'])) for g in cfg['goals']]
        self.seed = self.get_parameter('seed').value

        bitmap = self.get_parameter('bitmap').value or f'{share}/bitmaps/{self.mode}.png'
        occ = np.array(Image.open(bitmap).convert('L')) < 128
        self.dist = ndimage.distance_transform_edt(~occ) * RES
        self.get_logger().info(
            f'[{self.team}] mode={self.mode} goals={len(self.goals)} bitmap={Path(bitmap).name} '
            f'time_limit={self.time_limit:.0f}s')

        self.t0 = None
        self.collisions = 0
        self.in_collision = False
        self.next_goal = 0
        self.goal_times = []
        self.finished = False
        self.last_log = -10.0
        self.create_subscription(Odometry, 'ground_truth', self.on_gt, 10)

    # ----------------------------------------------------------- helpers ---
    def _wall_dist(self, x, y):
        c = int((x - self.ox + self.size / 2) / RES)
        r = int((self.size / 2 - (y - self.oy)) / RES)
        if not (0 <= r < self.dist.shape[0] and 0 <= c < self.dist.shape[1]):
            return 0.0
        return float(self.dist[r, c])

    def progress(self):
        return self.next_goal / max(len(self.goals), 1)

    def score(self, elapsed, done):
        prog = self.progress()
        req = max(0.0, self.req_max * prog - self.penalty * self.collisions)
        time_pts = 0.0
        if done and prog >= 0.5:
            minutes_saved = max(0.0, self.time_limit - elapsed) / 60.0
            time_pts = min(self.time_max, self.time_rate * minutes_saved)
        return {
            'baseline': self.base,
            'requirements': round(req, 2),
            'time_bonus': round(time_pts, 2),
            'total': round(self.base + req + time_pts, 2),
            'max_total': self.base + self.req_max + self.time_max,
        }

    # ---------------------------------------------------------- callback ---
    def on_gt(self, msg):
        if self.finished:
            return
        # Stage publishes ground_truth for ALL models; filter by the robot's frame
        if 'robot' not in msg.child_frame_id:
            return
        # use wall clock time (Stage's sim time persists between runs)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        elapsed = now - self.t0
        p = msg.pose.pose
        x, y, yaw = p.position.x, p.position.y, yaw_of(p.orientation)

        # collisions (rising edge with hysteresis)
        d = self._wall_dist(x, y)
        if d < COLLIDE_DIST and not self.in_collision:
            self.in_collision = True
            self.collisions += 1
            self.get_logger().warn(f'collision #{self.collisions} at ({x:.2f}, {y:.2f}) t={elapsed:.0f}s')
        elif d > CLEAR_DIST:
            self.in_collision = False

        # goals, strictly in order
        if self.next_goal < len(self.goals):
            gx, gy, gyaw = self.goals[self.next_goal]
            if math.hypot(x - gx, y - gy) < self.radius and angle_diff(yaw, gyaw) < self.yaw_tol:
                self.next_goal += 1
                self.goal_times.append(round(elapsed, 1))
                self.get_logger().info(f'goal {self.next_goal}/{len(self.goals)} reached at t={elapsed:.1f}s')
        done = self.next_goal >= len(self.goals)

        if elapsed - self.last_log >= 5.0:
            self.last_log = elapsed
            s = self.score(elapsed, done)
            self.get_logger().info(
                f't={elapsed:5.0f}s goals={self.next_goal}/{len(self.goals)} '
                f'collisions={self.collisions} score~{s["total"]:.1f}')

        if done or elapsed >= self.time_limit:
            self.finish(elapsed, done)

    def finish(self, elapsed, done):
        self.finished = True
        s = self.score(elapsed, done)
        result = {
            'team': self.team,
            'mode': self.mode,
            'seed': self.seed,
            'goals': [[round(x, 2), round(y, 2), round(math.degrees(a))] for x, y, a in self.goals],
            'completed': bool(done),
            'elapsed_s': round(elapsed, 1),
            'collisions': self.collisions,
            'goals_reached': self.next_goal,
            'goals_total': len(self.goals),
            'goal_times_s': self.goal_times,
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
