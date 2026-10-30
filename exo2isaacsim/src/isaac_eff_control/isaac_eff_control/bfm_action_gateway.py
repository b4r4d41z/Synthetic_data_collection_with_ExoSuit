"""10 Hz sample-and-hold after arm mapping and before hand interpolation.

Validated source callbacks forward high-rate controls; simulation ticks sample actions.
Run with the default single-threaded executor so a tick is one snapshot.
"""

from copy import deepcopy
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from dynamic_biped.msg import BfmAction, RobotHandPosition
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

from .sim_policy_clock import SimPolicyClock

from .bfm_contract import (
    ACTION_TOPIC, ARM_JOINTS, ARM_SOURCE_TOPIC, HELD_ARM_TOPIC, HELD_HAND_TOPIC,
    RAW_HAND_TOPIC, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, collection_enabled, arm_target_processing,
)


class BfmActionGateway(Node):
    def __init__(self):
        super().__init__("bfm_action_gateway")
        self.enabled = self.declare_parameter(
            "bfm_collection_mode", collection_enabled()).value
        self.arm = None
        self.hand = None
        if not self.enabled:
            self.get_logger().info("BFM gateway disabled; normal direct control is unchanged")
            return
        self.arm_limits = arm_target_processing()["joint_limits"]
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        hand_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.action_pub = self.create_publisher(BfmAction, ACTION_TOPIC, reliable)
        self.arm_pub = self.create_publisher(JointState, HELD_ARM_TOPIC, reliable)
        self.hand_pub = self.create_publisher(RobotHandPosition, HELD_HAND_TOPIC, reliable)
        self.create_subscription(JointState, ARM_SOURCE_TOPIC, self.arm_callback, reliable)
        self.create_subscription(
            RobotHandPosition, os.getenv("HAND_INPUT_TOPIC", RAW_HAND_TOPIC),
            self.hand_callback, hand_qos,
        )
        self.policy_clock = SimPolicyClock()
        self.create_subscription(Clock, '/clock', self.clock_callback,
                                 QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1000,
                                            reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info(
            "BFM 10 Hz gateway: bounded arm + raw hand -> held commands + /bfm/action"
        )

    def clock_callback(self, msg):
        stamp = msg.clock
        if self.policy_clock.advance(stamp.sec * 1_000_000_000 + stamp.nanosec):
            self.policy_tick(stamp)

    def arm_callback(self, msg):
        if (len(msg.name) != 14 or len(msg.position) != 14 or len(set(msg.name)) != 14
                or set(msg.name) != ARM_JOINTS
                or not all(math.isfinite(value) for value in msg.position)):
            self.arm = None
            self.get_logger().warning(
                "BFM rejected arm input: need 14 unique finite mapped joints",
                throttle_duration_sec=2.0,
            )
            return
        self.arm = deepcopy(msg)
        bounded = deepcopy(self.arm)
        bounded.position = [min(max(value, self.arm_limits[name][0]), self.arm_limits[name][1])
                             for name, value in zip(msg.name, msg.position)]
        self.arm_pub.publish(bounded)

    def hand_callback(self, msg):
        values = [*msg.left_hand_position, *msg.right_hand_position]
        if (len(msg.left_hand_position) != 6 or len(msg.right_hand_position) != 6
                or any(not math.isfinite(value) or not 0 <= value <= 100 for value in values)):
            self.hand = None
            self.get_logger().warning(
                "BFM rejected hand input: need six values per hand in 0..100",
                throttle_duration_sec=2.0,
            )
            return
        self.hand = deepcopy(msg)
        self.hand_pub.publish(deepcopy(self.hand))

    def policy_tick(self, simulation_stamp=None):
        if not self.enabled:
            return
        if self.arm is None or self.hand is None:
            missing = [name for name, value in (("arm", self.arm), ("hand", self.hand))
                       if value is None]
            self.get_logger().warning(
                f"BFM waiting for valid {' and '.join(missing)} input; no decision forwarded",
                throttle_duration_sec=2.0,
            )
            return
        arm, hand = deepcopy(self.arm), deepcopy(self.hand)
        # Reapplying the same saturation is idempotent; raw input is never mutated.
        arm.position = [min(max(value, self.arm_limits[name][0]), self.arm_limits[name][1])
                        for name, value in zip(arm.name, arm.position)]
        positions = dict(zip(arm.name, arm.position))
        action = BfmAction()
        action.header.stamp = (deepcopy(simulation_stamp) if simulation_stamp is not None
                               else self.get_clock().now().to_msg())
        action.left_arm_joint_names = list(LEFT_ARM_JOINTS)
        action.left_arm_target = [positions[name] for name in LEFT_ARM_JOINTS]
        action.left_hand_target = [float(value) for value in hand.left_hand_position]
        action.right_arm_joint_names = list(RIGHT_ARM_JOINTS)
        action.right_arm_target = [positions[name] for name in RIGHT_ARM_JOINTS]
        action.right_hand_target = [float(value) for value in hand.right_hand_position]
        # Sample the target values forwarded by the high-rate callbacks.
        # The decision timestamp is not a controller execution acknowledgement.
        self.action_pub.publish(action)


def main(args=None):
    rclpy.init(args=args)
    node = BfmActionGateway()
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
