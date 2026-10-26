"""
ORGANIZERS ONLY - do not hand out. Reference solution used to validate the scorer.

Only straight lines and 90 degree turns:
  * EASY: for each goal drive along x, then along y (see routes.manhattan_route)
  * HARD: follow the hand-made axis-aligned route routes.HARD_ROUTE through the wall gaps
At every goal it turns to the goal yaw before moving on.

    python3 reference_solution.py --ros-args -p task:=easy
    python3 reference_solution.py --ros-args -p task:=hard

It reads /odom (assumed to be in the world frame, i.e. to start at the start pose) and
stops while something is within 0.3 m in front of it.
"""

import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from routes import HARD_ROUTE, manhattan_route

V_MAX = 0.6
W_MAX = 0.7


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Reference(Node):
    def __init__(self):
        super().__init__('reference_solution')
        self.declare_parameter('task', 'easy')
        task = self.get_parameter('task').value
        cfg = yaml.safe_load((Path(get_package_share_directory('coderun_challenge'))
                              / 'config' / f'{task}_goals.yaml').read_text())
        self.goal_yaw = [math.radians(g['yaw_deg']) for g in cfg['goals']]
        start = (cfg['start']['x'], cfg['start']['y'])
        self.route = manhattan_route(start, cfg['goals']) if task == 'easy' else HARD_ROUTE

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'base_scan', self.on_scan, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(0.1, self.loop)
        self.scan = None
        self.have_odom = False
        self.x = self.y = self.yaw = 0.0
        self.idx = 0
        self.phase = 'turn'          # turn -> drive -> arrive -> (next point)

    def on_scan(self, m):
        self.scan = m

    def on_odom(self, m):
        p = m.pose.pose
        self.x, self.y = p.position.x, p.position.y
        q = p.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.have_odom = True

    def front_blocked(self):
        if self.scan is None:
            return False
        s = self.scan
        r = np.nan_to_num(np.array(s.ranges), nan=s.range_max, posinf=s.range_max)
        ang = s.angle_min + np.arange(len(r)) * s.angle_increment
        sel = np.abs(ang) < math.radians(20)
        return bool(sel.any() and r[sel].min() < 0.3)

    @staticmethod
    def turn_cmd(err):
        w = float(np.clip(1.2 * err, -W_MAX, W_MAX))
        return math.copysign(max(abs(w), 0.12), w)

    def loop(self):
        cmd = Twist()
        if not self.have_odom or self.idx >= len(self.route):
            self.pub.publish(cmd)
            return
        px, py, gi = self.route[self.idx]
        dx, dy = px - self.x, py - self.y
        dist = math.hypot(dx, dy)
        # legs are axis-aligned: snap the leg heading to a multiple of 90 deg
        leg = round(math.atan2(dy, dx) / (math.pi / 2)) * (math.pi / 2) if dist > 0.05 else self.yaw

        if self.phase == 'turn':
            err = wrap(leg - self.yaw)
            if dist <= 0.06 or abs(err) < 0.03:
                self.phase = 'drive'
            else:
                cmd.angular.z = self.turn_cmd(err)
        elif self.phase == 'drive':
            if dist < 0.06:
                self.phase = 'arrive'
            elif not self.front_blocked():
                cmd.linear.x = float(np.clip(0.8 * dist, 0.1, V_MAX))
                cmd.angular.z = float(np.clip(2.0 * wrap(leg - self.yaw), -0.5, 0.5))
        if self.phase == 'arrive':
            if gi is None:
                self.idx, self.phase = self.idx + 1, 'turn'
            else:
                err = wrap(self.goal_yaw[gi] - self.yaw)
                if abs(err) < 0.05:
                    self.get_logger().info(f'goal {gi + 1} done')
                    self.idx, self.phase = self.idx + 1, 'turn'
                else:
                    cmd.angular.z = self.turn_cmd(err)
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = Reference()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
