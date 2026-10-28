"""
Publish the goals of this run (latched) so that students can read them at runtime:

    /goals         geometry_msgs/PoseArray       the goal poses, IN ORDER (frame odom)
    /map           nav_msgs/OccupancyGrid        the arena, 0.04 m cells (frame odom)
    /goal_markers  visualization_msgs/MarkerArray  rectangles / arrows / rings / numbers for RViz

Goals are random (see sampler.py), so they are NOT in a file students can read: they are
passed to this node by the launch file as the `goals` parameter [x, y, yaw_rad, x, y, ...].
Frame: `frame_id` (default `odom`). /odom and /ground_truth both start at (0, 0, 0) and
everything here is in that frame. Subscribers must use a transient-local QoS (latched).
"""

import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

RES = 0.02               # bitmap resolution (m / px)
MAP_DOWNSAMPLE = 2       # /map cell = 0.04 m
RECT = (0.5, 0.36)       # rectangle footprint (m): length along the heading, width
CYAN = (0.1, 0.8, 0.9, 0.7)
GREEN = (0.2, 0.9, 0.2, 0.7)
WHITE = (1.0, 1.0, 1.0, 1.0)


class GoalServer(Node):
    def __init__(self):
        super().__init__('goal_server')
        self.declare_parameter('task', 'easy')
        self.declare_parameter('goals', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('frame_id', 'odom')
        task = self.get_parameter('task').value
        share = Path(get_package_share_directory('coderun_challenge'))
        cfg = yaml.safe_load((share / 'config' / f'{task}_goals.yaml').read_text())
        self.frame = self.get_parameter('frame_id').value
        self.radius = float(cfg['goal_radius'])

        flat = list(self.get_parameter('goals').value or [])
        if not flat:        # fallback: static goals of the YAML only
            flat = [v for g in cfg['goals'] for v in (g['x'], g['y'], math.radians(g['yaw_deg']))]
        self.goals = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.goals_pub = self.create_publisher(PoseArray, 'goals', qos)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', qos)
        self.marker_pub = self.create_publisher(MarkerArray, 'goal_markers', qos)

        self.goals_msg = self._goals_msg()
        self.map_msg = self._map_msg(share / 'bitmaps' / f'{task}.png', cfg)
        self.markers = self._markers(cfg['start'])
        self.create_timer(1.0, self.publish)       # also re-send for late subscribers
        self.publish()
        self.get_logger().info(f'{len(self.goals)} goal(s) published on /goals, map on /map: ' +
                               ', '.join(f'({x:.2f}, {y:.2f}, {math.degrees(a):.0f})' for x, y, a in self.goals))

    # ----------------------------------------------------------- messages --
    @staticmethod
    def _quat(msg_pose, yaw):
        msg_pose.orientation.z, msg_pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)

    def _goals_msg(self):
        msg = PoseArray()
        msg.header.frame_id = self.frame
        for x, y, yaw in self.goals:
            p = Pose()
            p.position.x, p.position.y = float(x), float(y)
            self._quat(p, yaw)
            msg.poses.append(p)
        return msg

    def _map_msg(self, bitmap, cfg):
        occ = np.array(Image.open(bitmap).convert('L')) < 128          # row 0 = top (+y)
        f = MAP_DOWNSAMPLE
        h, w = occ.shape[0] // f * f, occ.shape[1] // f * f
        occ = occ[:h, :w].reshape(h // f, f, w // f, f).max(axis=(1, 3))
        occ = np.flipud(occ)                                           # row 0 = bottom (-y)
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame
        msg.info.resolution = RES * f
        msg.info.height, msg.info.width = occ.shape
        size, ox, oy = cfg['world_size'], cfg['map_origin']['x'], cfg['map_origin']['y']
        msg.info.origin.position.x = float(ox - size / 2)              # lower-left corner
        msg.info.origin.position.y = float(oy - size / 2)
        msg.info.origin.orientation.w = 1.0
        msg.data = (occ.astype(np.int8) * 100).flatten().tolist()
        return msg

    def _base(self, ns, mid, mtype, x, y, z=0.0, yaw=0.0):
        m = Marker()
        m.header.frame_id = self.frame
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), z
        self._quat(m.pose, yaw)
        return m

    @staticmethod
    def _color(m, rgba):
        m.color.r, m.color.g, m.color.b, m.color.a = rgba

    def _pose_markers(self, out, ns, mid, x, y, yaw, rgba, label):
        rect = self._base(f'{ns}_rect', mid, Marker.CUBE, x, y, 0.01, yaw)
        rect.scale.x, rect.scale.y, rect.scale.z = RECT[0], RECT[1], 0.02
        self._color(rect, rgba)

        arrow = self._base(f'{ns}_heading', mid, Marker.ARROW, x, y, 0.04, yaw)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = RECT[0] * 0.9, 0.06, 0.06
        self._color(arrow, WHITE)

        ring = self._base(f'{ns}_radius', mid, Marker.LINE_STRIP, x, y, 0.02)
        ring.scale.x = 0.02
        self._color(ring, rgba[:3] + (1.0,))
        for k in range(33):
            a = 2 * math.pi * k / 32
            ring.points.append(Point(x=self.radius * math.cos(a), y=self.radius * math.sin(a), z=0.0))

        text = self._base(f'{ns}_label', mid, Marker.TEXT_VIEW_FACING, x, y, 0.45)
        text.scale.z = 0.35
        self._color(text, WHITE)
        text.text = label
        out += [rect, arrow, ring, text]

    def _markers(self, start):
        out = []
        self._pose_markers(out, 'start', 0, start['x'], start['y'], math.radians(start['yaw_deg']), GREEN, 'S')
        for i, (x, y, yaw) in enumerate(self.goals, start=1):
            self._pose_markers(out, 'goal', i, x, y, yaw, CYAN, str(i))
        return out

    def publish(self):
        now = self.get_clock().now().to_msg()
        self.goals_msg.header.stamp = now
        self.map_msg.header.stamp = now
        for m in self.markers:
            m.header.stamp = now
        self.goals_pub.publish(self.goals_msg)
        self.map_pub.publish(self.map_msg)
        self.marker_pub.publish(MarkerArray(markers=self.markers))


def main(args=None):
    rclpy.init(args=args)
    node = GoalServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
