"""
ORGANIZERS ONLY - do not hand out. Reference solution used to validate the scorer and to
check that every random run is solvable. Straight lines and turns in place only.

    python3 reference_solution.py --ros-args -p task:=easy
    python3 reference_solution.py --ros-args -p task:=hard

It reads the goals from /goals (they are random, nothing is hardcoded) and:
  * EASY: for each goal drives along x, then along y (straight lines and 90 deg turns)
  * HARD: plans an A* path on /map (inflated by 0.4 m), smooths it into straight legs
At every leg it turns in place towards the next point, then drives; at every goal it turns to
the goal yaw before moving on. It stops while something is within 0.3 m in front of it.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from routes import manhattan_route, plan_route

V_MAX = 0.6
W_MAX = 0.7


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Reference(Node):
    def __init__(self):
        super().__init__('reference_solution')
        self.declare_parameter('task', 'easy')
        self.task = self.get_parameter('task').value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(PoseArray, 'goals', self.on_goals, latched)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, latched)
        self.create_subscription(LaserScan, 'base_scan', self.on_scan, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(0.1, self.loop)

        self.goals = None            # [(x, y, yaw)]
        self.map = None
        self.scan = None
        self.have_odom = False
        self.x = self.y = self.yaw = 0.0
        self.route = None
        self.idx = 0
        self.phase = 'turn'          # turn -> drive -> arrive -> (next point)

    # ---------------------------------------------------------- callbacks --
    def on_goals(self, m):
        self.goals = [(p.position.x, p.position.y, yaw_of(p.orientation)) for p in m.poses]

    def on_map(self, m):
        self.map = m

    def on_scan(self, m):
        self.scan = m

    def on_odom(self, m):
        p = m.pose.pose
        self.x, self.y, self.yaw = p.position.x, p.position.y, yaw_of(p.orientation)
        self.have_odom = True

    # ------------------------------------------------------------ helpers --
    def make_route(self):
        xy = [(g[0], g[1]) for g in self.goals]
        if self.task == 'easy':
            self.route = manhattan_route((self.x, self.y), xy)
        else:
            m = self.map
            occ = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width) > 50
            origin = (m.info.origin.position.x, m.info.origin.position.y)
            self.route = plan_route(occ, m.info.resolution, origin, (self.x, self.y), xy)
        self.get_logger().info(f'{len(self.goals)} goals, route of {len(self.route)} legs')

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

    # --------------------------------------------------------------- loop --
    def loop(self):
        cmd = Twist()
        ready = self.have_odom and self.goals and (self.task == 'easy' or self.map is not None)
        if not ready:
            self.pub.publish(cmd)
            return
        if self.route is None:
            self.make_route()
        if self.idx >= len(self.route):
            self.pub.publish(cmd)
            return

        px, py, gi = self.route[self.idx]
        dx, dy = px - self.x, py - self.y
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) if dist > 0.05 else self.yaw

        if self.phase == 'turn':
            err = wrap(bearing - self.yaw)
            if dist <= 0.06 or abs(err) < 0.04:
                self.phase = 'drive'
            else:
                cmd.angular.z = self.turn_cmd(err)
        elif self.phase == 'drive':
            err = wrap(bearing - self.yaw)
            if dist < 0.06:
                self.phase = 'arrive'
            elif abs(err) > 0.5:
                self.phase = 'turn'          # drifted off the leg: realign
            elif not self.front_blocked():
                cmd.linear.x = float(np.clip(0.8 * dist, 0.1, V_MAX))
                cmd.angular.z = float(np.clip(2.0 * err, -0.5, 0.5))
        if self.phase == 'arrive':
            if gi is None:
                self.idx, self.phase = self.idx + 1, 'turn'
            else:
                err = wrap(self.goals[gi][2] - self.yaw)
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
