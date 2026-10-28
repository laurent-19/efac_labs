"""
CodeRun - your solution node (starting point, based on Lab 2 "wander").

TASK: the judge gives you a list of goal poses (x, y, yaw). Make the robot reach them
IN ORDER, as FAST as you can, without touching anything. A goal counts when the robot is
within 0.3 m of it AND its heading is within 20 deg of the goal yaw.

The goals are RANDOM at every run (in EASY only the last one, in HARD all of them), so you
must read them at runtime - hardcoded positions will not work.

You have:
    /goals      geometry_msgs/PoseArray  the goal poses, in order (latched / transient local)
    /map        nav_msgs/OccupancyGrid   the arena, 0.04 m cells, 100 = wall (latched)
    /odom       nav_msgs/Odometry        robot pose estimate (starts at (0, 0, 0))
    /base_scan  sensor_msgs/LaserScan    270 deg field of view, 1 deg steps, 5 m range
and you publish:
    /cmd_vel    geometry_msgs/Twist      linear.x <= 1.0 m/s, angular.z <= 0.78 rad/s

/goals, /map and /odom are all in the same frame (the robot starts at its origin).
/ground_truth is for the judge only - do not rely on it.

Look at the goals and at the map, then decide on waypoints / a path that gets you from goal
to goal around the walls. In HARD a straight line to the next goal is blocked.

Run it with:
    ros2 run coderun_student solution
"""

import math

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class SolutionNode(Node):
    def __init__(self):
        super().__init__('solution_node')
        # /goals and /map are published once and latched: ask for transient-local durability
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(PoseArray, 'goals', self.goals_callback, latched)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, latched)
        self.create_subscription(LaserScan, 'base_scan', self.scan_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_timer(0.1, self.control_loop)          # 10 Hz

        self.goals = None          # list of (x, y, yaw in radians), in order
        self.map = None            # nav_msgs/OccupancyGrid
        self.scan = None
        self.x = self.y = self.yaw = 0.0
        self.waypoints = None

    def goals_callback(self, msg):
        self.goals = [(p.position.x, p.position.y, yaw_from_quaternion(p.orientation)) for p in msg.poses]
        self.get_logger().info(f'received {len(self.goals)} goals')

    def map_callback(self, msg):
        # msg.data[row * msg.info.width + col] is 100 for a wall, 0 for free space.
        # Cell (row, col) covers x in msg.info.origin.position.x + col * msg.info.resolution (+ one cell)
        # and y likewise with row. Row 0 is the lowest y.
        self.map = msg

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
        if self.goals is None or self.map is None:
            self.cmd_pub.publish(cmd)          # wait for the goals and the map
            return

        if self.waypoints is None:
            # TODO: turn self.goals (and self.map) into a list of waypoints: intermediate
            #       points that lead around the walls from one goal to the next
            self.waypoints = list(self.goals)

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
