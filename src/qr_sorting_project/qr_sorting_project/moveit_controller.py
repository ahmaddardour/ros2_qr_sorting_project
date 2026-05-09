import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from moveit.planning import MoveItPy
from moveit.planning import PlanningComponent

class MoveItController(Node):
    def __init__(self):
        super().__init__('moveit_controller')

        self.get_logger().info("MoveIt Controller Started")

        # Initialize MoveIt
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = PlanningComponent("panda_arm", self.moveit)

        # Subscribe to QR
        self.subscription = self.create_subscription(
            String,
            '/qr_data',
            self.qr_callback,
            10
        )

    def qr_callback(self, msg):
        qr_value = msg.data
        self.get_logger().info(f"QR detected: {qr_value}")

        if qr_value == 'A':
            self.go_to_pose([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        elif qr_value == 'B':
            self.go_to_pose([-0.5, -0.3, -0.2, -1.5, 0.0, 1.2, -0.5])

    def go_to_pose(self, joint_positions):
        self.get_logger().info(f"Planning to: {joint_positions}")

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(joint_positions=joint_positions)

        plan = self.arm.plan()

        if plan:
            self.get_logger().info("Executing plan...")
            self.arm.execute()
        else:
            self.get_logger().error("Planning failed!")

def main(args=None):
    rclpy.init(args=args)
    node = MoveItController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
