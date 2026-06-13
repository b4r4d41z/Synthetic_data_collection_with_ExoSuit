from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from .hand_command_mapper import HandCommandMapper


TOPIC_NAME = "/control_robot_hand_position"


class Ros2HandCommandSubscriber(Node):
    def __init__(self):
        super().__init__("ros2_hand_command_subscriber")
        self.mapper = HandCommandMapper()

        self.sub = self.create_subscription(
            Int32MultiArray,
            TOPIC_NAME,
            self.callback,
            10,
        )

        self.get_logger().info(f"Listening to {TOPIC_NAME}")

    def callback(self, msg: Int32MultiArray):
        if len(msg.data) != 12:
            self.get_logger().warn(f"Expected 12 values, got {len(msg.data)}")
            return

        left = list(msg.data[:6])
        right = list(msg.data[6:12])

        self.get_logger().info(f"left={left}, right={right}")

        # Пока только проверяем mapper без Isaac articulation.
        left_targets = self.mapper.map_left_hand(left)
        self.get_logger().info(f"left_targets={left_targets}")


def main():
    rclpy.init()
    node = Ros2HandCommandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()