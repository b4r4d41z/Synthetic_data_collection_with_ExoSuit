#!/usr/bin/env python3

import math
import os
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.clock import Clock as ROSClock
from rclpy.clock_type import ClockType

from dynamic_biped.msg import RobotHandPosition
from sensor_msgs.msg import JointState

from .flow_status import FlowStatus

from .bfm_contract import HELD_HAND_TOPIC, collection_enabled


BFM_COLLECTION_MODE = collection_enabled()
HAND_INPUT_TOPIC = os.getenv("HAND_INPUT_TOPIC", "/robot/command/hand_target")

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

# Left thumb CMC_AA is fixed at -20 degrees.
THUMB_CMC_AA_TARGET_RAD = math.radians(-20.0)

# Right thumb CMC_AA uses the opposite sign to get the same visual start pose
# on the mirrored right hand model.
RIGHT_THUMB_CMC_AA_TARGET_RAD = math.radians(20.0)

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
            reliability=(ReliabilityPolicy.RELIABLE if HAND_INPUT_TOPIC == HELD_HAND_TOPIC else
                         ReliabilityPolicy.BEST_EFFORT),
        )

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.sub = self.create_subscription(
            RobotHandPosition,
            HAND_INPUT_TOPIC,
            self.hand_callback,
            input_qos,
        )

        # Retain the internal mapped JointState topics used by the Isaac graph;
        # the semantic bridge input carries RobotHandPosition, not joint angles.
        self.pub_left = self.create_publisher(
            JointState,
            "/left_hand_joint_command",
            output_qos,
        )

        self.pub_right = self.create_publisher(
            JointState,
            "/right_hand_joint_command",
            output_qos,
        )

        self.flow = FlowStatus(self.get_logger(), 'HAND')
        self.flow_timer = self.create_timer(0.2, self.flow.check)
        # Full-rate validated logical controller targets, separate from samples.
        self.logical_pub = (self.create_publisher(RobotHandPosition, HELD_HAND_TOPIC, input_qos)
                            if BFM_COLLECTION_MODE and HAND_INPUT_TOPIC != HELD_HAND_TOPIC else None)

        self.target_left_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_left_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.target_right_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_right_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.left_mapping = {
            0: [
                ("left_thumb_MCP_FE", 0.0, THUMB_MAX_ANGLE_RAD),
                ("left_thumb_IP", 0.0, THUMB_TIP_MAX_ANGLE_RAD),
            ],

            1: [
                ("left_thumb_CMC_FE", 0.0, THUMB_CMC_FE_MAX_ANGLE_RAD),
                ("left_thumb_CMC_AA", THUMB_CMC_AA_TARGET_RAD, THUMB_CMC_AA_TARGET_RAD),
            ],

            2: [
                ("left_index_MCP_FE", 0.0, INDEX_BASE_MAX_ANGLE_RAD),
                ("left_index_PIP", 0.0, DISTAL_MAX_ANGLE_RAD),
                ("left_index_DIP", 0.0, DISTAL_MAX_ANGLE_RAD),
            ],

            3: [
                ("left_middle_MCP_FE", 0.0, BASE_MAX_ANGLE_RAD),
                ("left_middle_PIP", 0.0, DISTAL_MAX_ANGLE_RAD),
                ("left_middle_DIP", 0.0, DISTAL_MAX_ANGLE_RAD),
            ],

            4: [
                ("left_ring_MCP_FE", 0.0, BASE_MAX_ANGLE_RAD),
                ("left_ring_PIP", 0.0, DISTAL_MAX_ANGLE_RAD),
                ("left_ring_DIP", 0.0, DISTAL_MAX_ANGLE_RAD),
            ],

            5: [
                ("left_pinky_MCP_FE", 0.0, BASE_MAX_ANGLE_RAD),
                ("left_pinky_PIP", 0.0, DISTAL_MAX_ANGLE_RAD),
                ("left_pinky_DIP", 0.0, DISTAL_MAX_ANGLE_RAD),
            ],
        }

        self.right_mapping = {
            0: [
                ("right_thumb_MCP_FE", 0.0, -THUMB_MAX_ANGLE_RAD),
                ("right_thumb_IP", 0.0, -THUMB_TIP_MAX_ANGLE_RAD),
            ],

            1: [
                ("right_thumb_CMC_FE", 0.0, -THUMB_CMC_FE_MAX_ANGLE_RAD),
                ("right_thumb_CMC_AA", RIGHT_THUMB_CMC_AA_TARGET_RAD, RIGHT_THUMB_CMC_AA_TARGET_RAD),
            ],

            2: [
                ("right_index_MCP_FE", 0.0, -INDEX_BASE_MAX_ANGLE_RAD),
                ("right_index_PIP", 0.0, -DISTAL_MAX_ANGLE_RAD),
                ("right_index_DIP", 0.0, -DISTAL_MAX_ANGLE_RAD),
            ],

            3: [
                ("right_middle_MCP_FE", 0.0, -BASE_MAX_ANGLE_RAD),
                ("right_middle_PIP", 0.0, -DISTAL_MAX_ANGLE_RAD),
                ("right_middle_DIP", 0.0, -DISTAL_MAX_ANGLE_RAD),
            ],

            4: [
                ("right_ring_MCP_FE", 0.0, -BASE_MAX_ANGLE_RAD),
                ("right_ring_PIP", 0.0, -DISTAL_MAX_ANGLE_RAD),
                ("right_ring_DIP", 0.0, -DISTAL_MAX_ANGLE_RAD),
            ],

            5: [
                ("right_pinky_MCP_FE", 0.0, -BASE_MAX_ANGLE_RAD),
                ("right_pinky_PIP", 0.0, -DISTAL_MAX_ANGLE_RAD),
                ("right_pinky_DIP", 0.0, -DISTAL_MAX_ANGLE_RAD),
            ],
        }

        # Explicit steady clock: even use_sim_time=true cannot throttle fingers.
        self.wall_clock = ROSClock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.timer_callback,
                                       clock=self.wall_clock)

    def hand_callback(self, msg):
        values = list(msg.left_hand_position) + list(msg.right_hand_position)
        valid = (len(msg.left_hand_position) == len(msg.right_hand_position) == 6
                 and all(math.isfinite(v) and 0 <= v <= 100 for v in values))
        self.flow.receive(valid)
        if not valid:
            self.get_logger().warning('Rejected hand input: need six finite logical values in 0..100 per side')
            return
        self.target_left_values = [float(v) for v in msg.left_hand_position]
        self.target_right_values = [float(v) for v in msg.right_hand_position]
        if self.logical_pub is not None:
            self.logical_pub.publish(deepcopy(msg))

    def update_current_left_values(self):
        dt = 1.0 / PUBLISH_HZ

        guide_step = GUIDE_FINGER_RATE_PER_SEC * dt
        thumb_step = THUMB_RATE_PER_SEC * dt

        self.current_left_values[1] = move_towards(
            self.current_left_values[1],
            self.target_left_values[1],
            thumb_step,
        )

        guide_indices = [2, 3, 4, 5]

        for index in guide_indices:
            self.current_left_values[index] = move_towards(
                self.current_left_values[index],
                self.target_left_values[index],
                guide_step,
            )

        guide_average = sum(self.current_left_values[i] for i in guide_indices) / len(guide_indices)

        thumb_mcp_index = 0
        thumb_is_opening = self.target_left_values[thumb_mcp_index] < self.current_left_values[thumb_mcp_index]
        guide_is_ready = guide_average >= THUMB_START_AFTER_GUIDE_VALUE

        if thumb_is_opening or guide_is_ready:
            self.current_left_values[thumb_mcp_index] = move_towards(
                self.current_left_values[thumb_mcp_index],
                self.target_left_values[thumb_mcp_index],
                thumb_step,
            )

    def update_current_right_values(self):
        dt = 1.0 / PUBLISH_HZ

        guide_step = GUIDE_FINGER_RATE_PER_SEC * dt
        thumb_step = THUMB_RATE_PER_SEC * dt

        self.current_right_values[1] = move_towards(
            self.current_right_values[1],
            self.target_right_values[1],
            thumb_step,
        )

        guide_indices = [2, 3, 4, 5]

        for index in guide_indices:
            self.current_right_values[index] = move_towards(
                self.current_right_values[index],
                self.target_right_values[index],
                guide_step,
            )

        guide_average = sum(self.current_right_values[i] for i in guide_indices) / len(guide_indices)

        thumb_mcp_index = 0
        thumb_is_opening = self.target_right_values[thumb_mcp_index] < self.current_right_values[thumb_mcp_index]
        guide_is_ready = guide_average >= THUMB_START_AFTER_GUIDE_VALUE

        if thumb_is_opening or guide_is_ready:
            self.current_right_values[thumb_mcp_index] = move_towards(
                self.current_right_values[thumb_mcp_index],
                self.target_right_values[thumb_mcp_index],
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

    def make_joint_state_msg(self, names, positions):
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = names
        joint_msg.position = positions
        return joint_msg

    def timer_callback(self):
        self.update_current_left_values()
        self.update_current_right_values()

        left_names = []
        left_positions = []

        right_names = []
        right_positions = []

        self.append_hand_targets(
            left_names,
            left_positions,
            self.current_left_values,
            self.left_mapping,
        )

        self.append_hand_targets(
            right_names,
            right_positions,
            self.current_right_values,
            self.right_mapping,
        )

        left_msg = self.make_joint_state_msg(left_names, left_positions)
        right_msg = self.make_joint_state_msg(right_names, right_positions)

        self.pub_left.publish(left_msg)
        self.pub_right.publish(right_msg)



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
