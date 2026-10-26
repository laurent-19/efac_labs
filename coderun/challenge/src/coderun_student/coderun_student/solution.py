"""
CodeRun - your solution node (starting point, based on Lab 2 "wander").

The simulator gives you:
    /base_scan  sensor_msgs/LaserScan   270 deg field of view, 1 deg steps, 5 m range
    /odom       nav_msgs/Odometry       robot pose estimate (starts at the START pose)
and listens on:
    /cmd_vel    geometry_msgs/Twist     linear.x  <= 1.0 m/s,  angular.z <= 0.78 rad/s

The /ground_truth topic is used by the judge only - do not rely on it.

EASY: pass through the 3 waypoints, in order, without touching anything.
HARD: explore as much of the maze as you can, without touching anything.
(see the README for scoring)

Run it with:
    ros2 run coderun_student solution
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# EASY waypoints in the world frame (x, y) in metres - visit them in this order
WAYPOINTS = [(4.0, -4.0), (4.0, 4.0), (-4.0, 4.0)]


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SolutionNode(Node):
    def __init__(self):
        super().__init__('solution_node')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'base_scan', self.scan_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_timer(0.1, self.control_loop)          # 10 Hz

        self.scan = None
        self.x = self.y = self.yaw = 0.0

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
        if self.scan is None:
            self.cmd_pub.publish(cmd)
            return

        # TODO: look at self.scan.ranges and decide where it is safe to go
        # TODO: EASY - steer towards the next waypoint; HARD - explore the maze
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SolutionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
