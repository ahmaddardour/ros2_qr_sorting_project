#!/usr/bin/env python3
"""Multi-object pick-scan-place demo with smooth animations.

At startup both cubes appear:
  - yellow object_A at the pick station
  - green  object_B at a waiting spot next to it

Triggers (publish a String to /qr_data):
  'A'    -> run cycle for object_A only           (yellow -> RED bin)
  'B'    -> slide object_B to pick, run cycle B   (green  -> BLUE bin)
  'demo' / 'AB' / anything else  -> full sequence (A then B)

All placements use a smooth descent animation so the cube settles into the bin
instead of teleporting from gripper height."""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Header, ColorRGBA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (PlanningScene, CollisionObject,
                             AttachedCollisionObject, ObjectColor)


def _build_cube(frame_id, position_xyz, object_id, size=0.05):
    co = CollisionObject()
    co.header = Header(frame_id=frame_id)
    co.id = object_id
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [size, size, size]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, position_xyz)
    pose.orientation.w = 1.0
    co.primitives.append(box)
    co.primitive_poses.append(pose)
    return co


class MainNode(Node):
    WORLD_FRAME = 'panda_link0'
    GRIPPER_LINK = 'panda_hand'

    PICK_XYZ        = (0.796, -0.026, 0.025)
    WAIT_B_XYZ      = (0.796,  0.100, 0.025)
    BIN_A_DROP_XYZ  = (0.594, -0.358, 0.13)
    BIN_B_DROP_XYZ  = (0.594,  0.358, 0.13)
    PLACE_A_GRIPPER = (0.594, -0.358, 0.28)   # FK of place_A joint pose
    PLACE_B_GRIPPER = (0.594,  0.358, 0.28)   # FK of place_B joint pose

    OBJECTS = {
        'A': dict(id='object_A', color=(1.00, 0.85, 0.10), bin='RED'),
        'B': dict(id='object_B', color=(0.20, 0.80, 0.20), bin='BLUE'),
    }

    def __init__(self):
        super().__init__('main_node')
        self.get_logger().info("=== Multi-object QR sorting (animated) ===")

        self.publisher = self.create_publisher(
            JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)

        scene_qos = QoSProfile(depth=1,
                               reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', scene_qos)

        self.subscription = self.create_subscription(
            String, '/qr_data', self.qr_callback, 10)

        self.joint_names = ['panda_joint1','panda_joint2','panda_joint3',
                            'panda_joint4','panda_joint5','panda_joint6','panda_joint7']
        self.home    = [ 0.0000, -0.7850,  0.0000, -2.3560,  0.0000, 1.5710, 0.7850]
        self.front   = [-0.1524,  0.6769, -0.1615, -0.8525,  0.1009, 1.5235, 0.5111]
        self.down    = [-0.0005,  1.4391, -0.0957, -0.5492,  0.1038, 1.9856, 0.7300]
        self.up      = [-0.0245,  1.0064, -0.1193, -0.5954,  0.1008, 1.5985, 0.6936]
        self.scan    = [-0.3706,  0.4568, -0.2590, -1.1386,  0.1133, 1.5821, 0.1795]
        self.place_A = [-0.3548,  0.6153, -0.2256, -1.5970,  0.1597, 2.1941, 0.1512]
        self.place_B = [ 0.3548,  0.6153,  0.2256, -1.5970, -0.1597, 2.1941, -0.1512]

        self._busy = False
        self._b_at_pick = False
        self._init_timer = self.create_timer(2.0, self._on_init_timer)

    def _on_init_timer(self):
        self._init_timer.cancel()
        self.get_logger().info("Spawning both cubes at the workstation...")
        self._spawn_in_world('object_A', self.PICK_XYZ,   self.OBJECTS['A']['color'])
        self._spawn_in_world('object_B', self.WAIT_B_XYZ, self.OBJECTS['B']['color'])
        self.get_logger().info("Ready. Publish to /qr_data: 'A', 'B', or 'demo' for full run.")

    def move(self, pose, name):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = pose
        pt.time_from_start.sec = 5
        msg.points.append(pt)
        self.publisher.publish(msg)
        self.get_logger().info(f"-> {name}")
        time.sleep(6)

    def _color(self, rgb):
        return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=1.0)

    def _spawn_in_world(self, obj_id, xyz, rgb):
        s = PlanningScene()
        s.is_diff = True
        s.robot_state.is_diff = True
        co = _build_cube(self.WORLD_FRAME, xyz, obj_id)
        co.operation = CollisionObject.ADD
        s.world.collision_objects.append(co)
        s.object_colors.append(ObjectColor(id=obj_id, color=self._color(rgb)))
        self.scene_pub.publish(s)
        time.sleep(0.3)

    def _slide(self, obj_id, from_xyz, to_xyz, rgb, duration=1.5, steps=20):
        dt = duration / steps
        for i in range(1, steps + 1):
            t = i / steps
            pos = tuple(from_xyz[k] + (to_xyz[k] - from_xyz[k]) * t for k in range(3))
            s = PlanningScene()
            s.is_diff = True
            s.robot_state.is_diff = True
            cube = _build_cube(self.WORLD_FRAME, pos, obj_id)
            cube.operation = CollisionObject.ADD
            s.world.collision_objects.append(cube)
            s.object_colors.append(ObjectColor(id=obj_id, color=self._color(rgb)))
            self.scene_pub.publish(s)
            time.sleep(dt)

    def _attach_to_gripper(self, obj_id, rgb):
        s = PlanningScene()
        s.is_diff = True
        s.robot_state.is_diff = True
        rm = CollisionObject()
        rm.header = Header(frame_id=self.WORLD_FRAME)
        rm.id = obj_id
        rm.operation = CollisionObject.REMOVE
        s.world.collision_objects.append(rm)
        cube = _build_cube(self.GRIPPER_LINK, (0.0, 0.0, 0.05), obj_id)
        cube.operation = CollisionObject.ADD
        aco = AttachedCollisionObject()
        aco.link_name = self.GRIPPER_LINK
        aco.object = cube
        aco.touch_links = ['panda_hand','panda_leftfinger','panda_rightfinger']
        s.robot_state.attached_collision_objects.append(aco)
        s.object_colors.append(ObjectColor(id=obj_id, color=self._color(rgb)))
        self.scene_pub.publish(s)
        self.get_logger().info(f"   {obj_id} attached to gripper")

    def _release_into_bin(self, obj_id, gripper_xyz, drop_xyz, rgb, bin_label):
        # Detach cube into the world at gripper position
        s = PlanningScene()
        s.is_diff = True
        s.robot_state.is_diff = True
        det = AttachedCollisionObject()
        det.link_name = self.GRIPPER_LINK
        det_obj = CollisionObject()
        det_obj.id = obj_id
        det_obj.operation = CollisionObject.REMOVE
        det.object = det_obj
        s.robot_state.attached_collision_objects.append(det)
        cube = _build_cube(self.WORLD_FRAME, gripper_xyz, obj_id)
        cube.operation = CollisionObject.ADD
        s.world.collision_objects.append(cube)
        s.object_colors.append(ObjectColor(id=obj_id, color=self._color(rgb)))
        self.scene_pub.publish(s)
        self.get_logger().info(f"   releasing {obj_id} above {bin_label} bin...")
        time.sleep(0.3)
        # Smooth descent into the bin
        self._slide(obj_id, gripper_xyz, drop_xyz, rgb, duration=1.5, steps=20)
        self.get_logger().info(f"   {obj_id} placed in {bin_label} bin")

    def _run_cycle(self, qr):
        spec = self.OBJECTS[qr]
        obj_id = spec['id']
        rgb = spec['color']
        bin_label = spec['bin']
        place_pose = self.place_A if qr == 'A' else self.place_B
        gripper_xyz = self.PLACE_A_GRIPPER if qr == 'A' else self.PLACE_B_GRIPPER
        drop_xyz = self.BIN_A_DROP_XYZ if qr == 'A' else self.BIN_B_DROP_XYZ

        if qr == 'B' and not self._b_at_pick:
            self.get_logger().info("Sliding object_B from waiting spot to pick station...")
            self._slide(obj_id, self.WAIT_B_XYZ, self.PICK_XYZ, rgb)
            self._b_at_pick = True

        self.move(self.home,  "HOME")
        self.move(self.front, "APPROACH PICK")
        self.move(self.down,  f"PICK {obj_id}")
        self._attach_to_gripper(obj_id, rgb)

        self.move(self.up,    "LIFT")
        self.move(self.scan,  "SCAN")
        self.get_logger().info(
            f"   >>> SCAN RESULT: code='{qr}' -> route to {bin_label} bin <<<")

        self.move(place_pose, f"PLACE {qr}")
        self._release_into_bin(obj_id, gripper_xyz, drop_xyz, rgb, bin_label)

        self.move(self.home, "RETURN HOME")
        self.get_logger().info(f"Cycle for '{qr}' complete.\n")

    def qr_callback(self, msg):
        if self._busy:
            self.get_logger().warn("Pipeline busy, ignoring trigger.")
            return
        qr = (msg.data or '').strip().upper()
        self.get_logger().info(f"Trigger received: '{qr}'")
        self._busy = True
        try:
            if qr == 'A':
                self._run_cycle('A')
            elif qr == 'B':
                self._run_cycle('B')
            else:
                self.get_logger().info("Running full demo: A then B...")
                self._run_cycle('A')
                self._run_cycle('B')
        finally:
            self._busy = False


def main():
    rclpy.init()
    node = MainNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
