import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class LeftHandTestPublisher(Node):
    def __init__(self):
        super().__init__("left_hand_test_publisher")
        self.pub = self.create_publisher(Int32MultiArray, "/control_robot_hand_position", 10)
        self.timer = self.create_timer(2.0, self.publish_command)
        self.closed = False

    def publish_command(self):
        msg = Int32MultiArray()

        open_left = [0, 100, 0, 0, 0, 0]
        close_left = [69, 99, 42, 54, 61, 60]

        left = close_left if self.closed else open_left
        right = [0, 100, 0, 0, 0, 0]

        msg.data = left + right
        self.pub.publish(msg)

        self.get_logger().info(f"Published left hand: {left}")
        self.closed = not self.closed


def main():
    rclpy.init()
    node = LeftHandTestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
