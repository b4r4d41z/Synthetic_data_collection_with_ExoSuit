#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from dynamic_biped.msg import RobotHandPosition
from sensor_msgs.msg import JointState


PUBLISH_HZ = 50.0

GUIDE_FINGER_RATE_PER_SEC = 300.0
THUMB_RATE_PER_SEC = 250.0

THUMB_START_AFTER_GUIDE_VALUE = 40.0

BASE_MAX_ANGLE_RAD = math.pi / 2.0

INDEX_BASE_MAX_ANGLE_RAD = BASE_MAX_ANGLE_RAD + math.radians(10.0)

# Thumb MCP flexion is reduced by 40 degrees: 90 -> 50 degrees.
THUMB_MAX_ANGLE_RAD = BASE_MAX_ANGLE_RAD - math.radians(40.0)

# Thumb CMC_FE should still be able to bend up to 90 degrees.
THUMB_CMC_FE_MAX_ANGLE_RAD = BASE_MAX_ANGLE_RAD

# Thumb CMC_AA is fixed at -20 degrees.
THUMB_CMC_AA_TARGET_RAD = math.radians(-20.0)

# Thumb tip joint bends slightly more than thumb MCP.
THUMB_TIP_MAX_ANGLE_RAD = math.radians(65.0)

DISTAL_EXTRA_ANGLE_RAD = math.radians(10.0)
DISTAL_MAX_ANGLE_RAD = BASE_MAX_ANGLE_RAD + DISTAL_EXTRA_ANGLE_RAD


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def move_towards(current, target, max_step):
    if current < target:
        return min(current + max_step, target)
    if current > target:
        return max(current - max_step, target)
    return current


def command_to_angle(value, min_angle=0.0, max_angle=math.pi / 2.0):
    value = clamp(float(value), 0.0, 100.0)
    return min_angle + value / 100.0 * (max_angle - min_angle)


class IsaacEffControlNode(Node):
    def __init__(self):
        super().__init__("isaac_eff_control_node")

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
            RobotHandPosition,
            "/control_robot_hand_position",
            self.hand_callback,
            input_qos,
        )

        self.pub = self.create_publisher(
            JointState,
            "/hand_joint_command",
            output_qos,
        )

        self.get_logger().info("isaac_eff_control_node started")
        self.get_logger().info("subscribing: /control_robot_hand_position")
        self.get_logger().info("publishing:  /hand_joint_command")
        self.get_logger().info("current scene: left hand only")

        self.target_left_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_left_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.target_right_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.left_mapping = {
            # left_hand_position[0] -> thumb MCP flexion.
            # Thumb MCP is reduced by 40 degrees: 90 -> 50 degrees.
            # Thumb IP bends slightly more: 65 degrees.
            0: [
                ("left_thumb_MCP_FE", 0.0, THUMB_MAX_ANGLE_RAD),
                ("left_thumb_IP", 0.0, THUMB_TIP_MAX_ANGLE_RAD),
            ],

            # left_hand_position[1] -> thumb CMC positioning.
            # CMC_FE bends up to 90 degrees.
            # CMC_AA is fixed at -20 degrees.
            1: [
                ("left_thumb_CMC_FE", 0.0, THUMB_CMC_FE_MAX_ANGLE_RAD),
                ("left_thumb_CMC_AA", THUMB_CMC_AA_TARGET_RAD, THUMB_CMC_AA_TARGET_RAD),
            ],

            # left_hand_position[2] -> index finger flexion.
            # Index MCP base joint is increased by 10 degrees: 90 -> 100 degrees.
            # PIP and DIP also close 10 degrees more than the standard base angle.
            2: [
                ("left_index_MCP_FE", 0.0, INDEX_BASE_MAX_ANGLE_RAD),
                ("left_index_PIP", 0.0, DISTAL_MAX_ANGLE_RAD),
                ("left_index_DIP", 0.0, DISTAL_MAX_ANGLE_RAD),
            ],

            # left_hand_position[3] -> middle finger flexion.
            # PIP and DIP close 10 degrees more than MCP_FE.
            3: [
                ("left_middle_MCP_FE", 0.0, BASE_MAX_ANGLE_RAD),
                ("left_middle_PIP", 0.0, DISTAL_MAX_ANGLE_RAD),
                ("left_middle_DIP", 0.0, DISTAL_MAX_ANGLE_RAD),
            ],

            # left_hand_position[4] -> ring finger flexion.
            # PIP and DIP close 10 degrees more than MCP_FE.
            4: [
                ("left_ring_MCP_FE", 0.0, BASE_MAX_ANGLE_RAD),
                ("left_ring_PIP", 0.0, DISTAL_MAX_ANGLE_RAD),
                ("left_ring_DIP", 0.0, DISTAL_MAX_ANGLE_RAD),
            ],

            # left_hand_position[5] -> pinky finger flexion.
            # PIP and DIP close 10 degrees more than MCP_FE.
            5: [
                ("left_pinky_MCP_FE", 0.0, BASE_MAX_ANGLE_RAD),
                ("left_pinky_PIP", 0.0, DISTAL_MAX_ANGLE_RAD),
                ("left_pinky_DIP", 0.0, DISTAL_MAX_ANGLE_RAD),
            ],
        }

        # In the current Isaac Sim scene only the left hand exists:
        # /World/left_hand_s40
        # Therefore right_hand_position is intentionally ignored here.
        self.right_mapping = {}

        self.timer = self.create_timer(
            1.0 / PUBLISH_HZ,
            self.timer_callback,
        )

    def hand_callback(self, msg):
        left_values = list(msg.left_hand_position)
        right_values = list(msg.right_hand_position)

        for i in range(min(6, len(left_values))):
            self.target_left_values[i] = clamp(float(left_values[i]), 0.0, 100.0)

        for i in range(min(6, len(right_values))):
            self.target_right_values[i] = clamp(float(right_values[i]), 0.0, 100.0)

    def update_current_left_values(self):
        dt = 1.0 / PUBLISH_HZ

        guide_step = GUIDE_FINGER_RATE_PER_SEC * dt
        thumb_step = THUMB_RATE_PER_SEC * dt

        # 1. Thumb CMC follows directly because it is a pre-bent thumb positioning joint.
        self.current_left_values[1] = move_towards(
            self.current_left_values[1],
            self.target_left_values[1],
            thumb_step,
        )

        # 2. Four guide fingers close first.
        guide_indices = [2, 3, 4, 5]

        for index in guide_indices:
            self.current_left_values[index] = move_towards(
                self.current_left_values[index],
                self.target_left_values[index],
                guide_step,
            )

        guide_average = sum(self.current_left_values[i] for i in guide_indices) / len(guide_indices)

        # 3. Thumb MCP/IP starts closing only after guide fingers are already partially closed.
        thumb_mcp_index = 0
        thumb_is_opening = self.target_left_values[thumb_mcp_index] < self.current_left_values[thumb_mcp_index]
        guide_is_ready = guide_average >= THUMB_START_AFTER_GUIDE_VALUE

        if thumb_is_opening or guide_is_ready:
            self.current_left_values[thumb_mcp_index] = move_towards(
                self.current_left_values[thumb_mcp_index],
                self.target_left_values[thumb_mcp_index],
                thumb_step,
            )

    def append_hand_targets(self, names, positions, values, mapping):
        for command_index, joint_targets in mapping.items():
            if command_index >= len(values):
                continue

            raw_value = values[command_index]

            for joint_name, min_angle, max_angle in joint_targets:
                names.append(joint_name)
                positions.append(command_to_angle(raw_value, min_angle, max_angle))

    def timer_callback(self):
        self.update_current_left_values()

        names = []
        positions = []

        self.append_hand_targets(
            names,
            positions,
            self.current_left_values,
            self.left_mapping,
        )

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = names
        joint_msg.position = positions

        self.pub.publish(joint_msg)

        self.get_logger().info(
            f"target_left={self.target_left_values} current_left={self.current_left_values} published_joints={len(names)}",
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)

    node = IsaacEffControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()