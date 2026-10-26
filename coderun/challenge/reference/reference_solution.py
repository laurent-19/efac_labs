"""
ORGANIZERS ONLY - do not hand out. Reference solution used to validate the scorer
and to check that both tasks are actually solvable with just /base_scan + /odom.

    python3 reference_solution.py --ros-args -p mode:=easy
    python3 reference_solution.py --ros-args -p mode:=hard

A tiny VFH-style reactive controller: pick the heading with enough free space that
is closest to the wanted direction (waypoint bearing in EASY, a slowly changing
random heading in HARD).
"""

import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

WAYPOINTS = [(4.0, -4.0), (4.0, 4.0), (-4.0, 4.0)]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Reference(Node):
    def __init__(self):
        super().__init__('reference_solution')
        self.declare_parameter('mode', 'easy')
        self.declare_parameter('seed', 1)
        self.mode = self.get_parameter('mode').value
        random.seed(self.get_parameter('seed').value)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'base_scan', self.on_scan, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(0.1, self.loop)
        self.scan = None
        self.x = self.y = self.yaw = 0.0
        self.wp = 0
        self.wander = 0.0          # preferred heading offset in HARD (rad, robot frame)
        self.wander_until = 0.0
        self.tick = 0

    def on_scan(self, m):
        self.scan = m

    def on_odom(self, m):
        p = m.pose.pose
        self.x, self.y = p.position.x, p.position.y
        q = p.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def loop(self):
        self.tick += 1
        t = self.tick * 0.1
        cmd = Twist()
        if self.scan is None:
            self.pub.publish(cmd)
            return
        s = self.scan
        r = np.nan_to_num(np.array(s.ranges), nan=s.range_max, posinf=s.range_max)
        r = np.clip(r, 0.0, s.range_max)
        ang = s.angle_min + np.arange(len(r)) * s.angle_increment

        # wanted heading (robot frame)
        if self.mode == 'easy':
            if self.wp < len(WAYPOINTS):
                wx, wy = WAYPOINTS[self.wp]
                if math.hypot(wx - self.x, wy - self.y) < 0.4:
                    self.wp += 1
            wx, wy = WAYPOINTS[min(self.wp, len(WAYPOINTS) - 1)]
            want = wrap(math.atan2(wy - self.y, wx - self.x) - self.yaw)
        else:
            if t > self.wander_until:
                self.wander = random.uniform(-1.2, 1.2)
                self.wander_until = t + random.uniform(4.0, 10.0)
            want = self.wander

        # candidate headings every 5 deg in +-110 deg, free space = min range in a +-22 deg window
        best, best_cost = None, 1e9
        for a in np.radians(np.arange(-110, 111, 5)):
            win = np.abs(ang - a) < math.radians(22)
            if not win.any():
                continue
            free = float(r[win].min())
            if free < 0.75:
                continue
            cost = abs(wrap(a - want)) + 0.4 * max(0.0, 2.0 - free)
            if cost < best_cost:
                best, best_cost = a, cost

        front = float(r[np.abs(ang) < math.radians(20)].min())
        if best is None or front < 0.45:
            # boxed in: turn on the spot towards the more open side
            left = float(r[(ang > 0) & (ang < math.radians(100))].mean())
            right = float(r[(ang < 0) & (ang > -math.radians(100))].mean())
            cmd.angular.z = 0.7 if left >= right else -0.7
            if front > 0.35:
                cmd.linear.x = 0.05
        else:
            cmd.angular.z = float(np.clip(1.6 * best, -0.75, 0.75))
            turn_penalty = max(0.0, 1.0 - abs(best) / math.radians(70))
            cmd.linear.x = float(0.6 * turn_penalty * min(1.0, (front - 0.45) / 1.0))
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
