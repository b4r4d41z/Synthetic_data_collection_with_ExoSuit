#!/usr/bin/env python3

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState


INPUT_TOPIC = os.getenv("ARM_INPUT_TOPIC", "/kuavo_arm_traj")
OUTPUT_TOPIC = os.getenv("ARM_OUTPUT_TOPIC", "/robot_arm_joint_command")

# Данные из экзоскелета сейчас выглядят как градусы.
INPUT_IS_DEGREES = os.getenv("ARM_INPUT_IS_DEGREES", "1") == "1"

ARM_JOINTS = {
    "zarm_r1_joint",
    "zarm_r2_joint",
    "zarm_r3_joint",
    "zarm_r4_joint",
    "zarm_r5_joint",
    "zarm_r6_joint",
    "zarm_r7_joint",
    "zarm_l1_joint",
    "zarm_l2_joint",
    "zarm_l3_joint",
    "zarm_l4_joint",
    "zarm_l5_joint",
    "zarm_l6_joint",
    "zarm_l7_joint",
}


def swap_left_right_joint_name(name):
    if name.startswith("zarm_l"):
        return name.replace("zarm_l", "zarm_r", 1)

    if name.startswith("zarm_r"):
        return name.replace("zarm_r", "zarm_l", 1)

    return name


class ArmControlNode(Node):
    def __init__(self):
        super().__init__("isaac_arm_control_node")

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.sub = self.create_subscription(
            JointState,
            INPUT_TOPIC,
            self.arm_callback,
            input_qos,
        )

        self.pub = self.create_publisher(
            JointState,
            OUTPUT_TOPIC,
            output_qos,
        )

        self.get_logger().info(f"Subscribing: {INPUT_TOPIC}")
        self.get_logger().info(f"Publishing:  {OUTPUT_TOPIC}")
        self.get_logger().info(f"Input is degrees: {INPUT_IS_DEGREES}")
        self.get_logger().info("Left/right arm joint names are swapped before publishing")

    def convert_position(self, value):
        value = float(value)

        if INPUT_IS_DEGREES:
            return math.radians(value)

        return value

    def arm_callback(self, msg):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id

        for name, position in zip(msg.name, msg.position):
            if name not in ARM_JOINTS:
                continue

            output_name = swap_left_right_joint_name(name)

            out.name.append(output_name)
            out.position.append(self.convert_position(position))

        if len(out.name) != 14:
            self.get_logger().warning(
                f"Expected 14 arm joints, got {len(out.name)}"
            )

        self.pub.publish(out)

        self.get_logger().info(
            f"Published arm joints: {len(out.name)}",
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)

    node = ArmControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()