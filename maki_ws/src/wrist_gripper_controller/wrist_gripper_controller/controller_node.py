import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class WristGripperController(Node):
    def __init__(self):
        super().__init__('wrist_gripper_controller')

        # Publishers
        self.pub_wrist = self.create_publisher(Float32, "wrist/position", 10)
        self.pub_gripper = self.create_publisher(Float32, "gripper/position", 10)
        self.pub_velocity = self.create_publisher(Float32, "motion/velocity", 10)

        self.get_logger().info("Wrist + Gripper controller started")

    def set_wrist(self, angle):
        msg = Float32()
        msg.data = float(angle)
        self.pub_wrist.publish(msg)

    def set_gripper(self, angle):
        msg = Float32()
        msg.data = float(angle)
        self.pub_gripper.publish(msg)

    def set_velocity(self, vel):
        msg = Float32()
        msg.data = float(vel)
        self.pub_velocity.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WristGripperController()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
