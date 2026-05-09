#!/usr/bin/env python3
"""Publishes the static workspace (table, two coloured bins, QR marker)
into the MoveIt planning scene. Cubes are spawned dynamically by main_node."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from moveit_msgs.msg import PlanningScene, CollisionObject, ObjectColor
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA, Header


def make_box(object_id, frame_id, size_xyz, position_xyz):
    co = CollisionObject()
    co.header = Header(frame_id=frame_id)
    co.id = object_id
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(size_xyz)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, position_xyz)
    pose.orientation.w = 1.0
    co.primitives.append(box)
    co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    return co


def make_color(object_id, r, g, b, a=1.0):
    oc = ObjectColor()
    oc.id = object_id
    oc.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
    return oc


class ScenePublisher(Node):
    FRAME = 'panda_link0'

    def __init__(self):
        super().__init__('scene_publisher')
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PlanningScene, '/planning_scene', qos)
        self.scene_msg = self._build_scene()
        self._left = 5
        self.timer = self.create_timer(1.0, self._tick)
        self._tick()
        self.get_logger().info('Scene publisher started (table, bins, QR marker).')

    def _build_scene(self):
        s = PlanningScene()
        s.is_diff = True
        s.robot_state.is_diff = True
        s.world.collision_objects = [
            make_box('table',     self.FRAME, (1.20, 1.60, 0.05), (0.45,  0.00, -0.025)),
            make_box('bin_A',     self.FRAME, (0.16, 0.16, 0.10), (0.594, -0.358, 0.05)),
            make_box('bin_B',     self.FRAME, (0.16, 0.16, 0.10), (0.594,  0.358, 0.05)),
            make_box('qr_marker', self.FRAME, (0.001, 0.10, 0.10), (0.78, -0.10, 0.45)),
        ]
        s.object_colors = [
            make_color('table',     0.55, 0.45, 0.30),
            make_color('bin_A',     0.85, 0.10, 0.10),
            make_color('bin_B',     0.10, 0.30, 0.85),
            make_color('qr_marker', 0.05, 0.05, 0.05),
        ]
        return s

    def _tick(self):
        if self._left <= 0:
            self.timer.cancel()
            return
        self.pub.publish(self.scene_msg)
        self._left -= 1


def main():
    rclpy.init()
    node = ScenePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
