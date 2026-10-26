"""
CodeRun - your solution node (starting point, based on Lab 2 "wander").

TASK: the judge gives you a list of goal poses (x, y, yaw). Make the robot reach them
IN ORDER, as FAST as you can, without touching anything. A goal counts when the robot is
within 0.3 m of it AND its heading is within 20 deg of the goal yaw.

You have:
    /odom       nav_msgs/Odometry       robot pose estimate (starts at the START pose)
    /base_scan  sensor_msgs/LaserScan   270 deg field of view, 1 deg steps, 5 m range
and you publish:
    /cmd_vel    geometry_msgs/Twist     linear.x <= 1.0 m/s, angular.z <= 0.78 rad/s

The goal poses are in config/<task>_goals.yaml of the `coderun_challenge` package
(world frame, metres / degrees). Look at the map, then decide on waypoints / a path
that gets you from goal to goal.

Run it with:
    ros2 run coderun_student solution --ros-args -p task:=easy
"""

import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class SolutionNode(Node):
    def __init__(self):
        super().__init__('solution_node')
        self.declare_parameter('task', 'easy')
        task = self.get_parameter('task').value
        cfg_path = Path(get_package_share_directory('coderun_challenge')) / 'config' / f'{task}_goals.yaml'
        cfg = yaml.safe_load(cfg_path.read_text())
        # list of (x, y, yaw in radians)
        self.goals = [(g['x'], g['y'], math.radians(g['yaw_deg'])) for g in cfg['goals']]

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'base_scan', self.scan_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_timer(0.1, self.control_loop)          # 10 Hz

        self.scan = None
        self.x = self.y = self.yaw = 0.0

        # TODO: turn self.goals into a list of waypoints (intermediate points, if needed)
        self.waypoints = list(self.goals)

    def scan_callback(self, msg):
        # msg.ranges[i] is the distance at angle msg.angle_min + i * msg.angle_increment
        # (0 rad = straight ahead, positive = counter-clockwise / left)
        self.scan = msg

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def control_loop(self):
        cmd = Twist()
        # TODO: compute the velocity that follows self.waypoints
        #       (turn towards the next waypoint, drive, turn to the goal yaw, ...)
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SolutionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
