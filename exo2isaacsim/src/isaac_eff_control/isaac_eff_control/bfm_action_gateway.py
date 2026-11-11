"""15 Hz canonical decisions after arm mapping and before hand interpolation.

Source callbacks only buffer; simulation ticks publish canonical decisions.
Run with the default single-threaded executor so a tick is one snapshot.
"""

from copy import deepcopy
import math
import os
import json
from std_msgs.msg import String
import time
from uuid import uuid4
from .bfm_contract import executor_metadata

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
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
        self.producer_session_id = uuid4().hex
        self.action_id = 0
        self.received = {}
        self.playing = False
        self.boundary = None
        self.source_timeout = executor_metadata()["configuration"]["source_timeout_wall_sec"]
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
        self.create_subscription(JointState, ARM_SOURCE_TOPIC, self.arm_callback, reliable)
        self.create_subscription(
            RobotHandPosition, os.getenv("HAND_INPUT_TOPIC", RAW_HAND_TOPIC),
            self.hand_callback, hand_qos,
        )
        self.policy_clock = SimPolicyClock()
        self.create_subscription(String, '/bfm/episode', self.episode_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Clock, '/clock', self.clock_callback,
                                 QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1000,
                                            reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info(
            "BFM v4: buffered sources -> 15 Hz /bfm/action -> executor"
        )

    def episode_callback(self, msg):
        event = json.loads(msg.data)
        if 'phase' not in event:
            return
        if event.get('boundary_id') != self.boundary or event['phase'] in ('stopped', 'failed', 'closed'):
            self.arm = self.hand = None
            self.received.clear()
            self.policy_clock = SimPolicyClock()
        self.boundary = event.get('boundary_id')
        self.playing = event['phase'] == 'playing'

    def clock_callback(self, msg):
        stamp = msg.clock
        now = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.policy_clock.last_ns is not None and now < self.policy_clock.last_ns:
            self.arm = self.hand = None
            self.received.clear()
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
        self.received["arm"] = time.monotonic()

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
        self.received["hand"] = time.monotonic()

    def policy_tick(self, simulation_stamp=None):
        if not self.enabled or not self.playing:
            return
        if self.arm is None or self.hand is None:
            missing = [name for name, value in (("arm", self.arm), ("hand", self.hand))
                       if value is None]
            self.get_logger().warning(
                f"BFM waiting for valid {' and '.join(missing)} input; no decision forwarded",
                throttle_duration_sec=2.0,
            )
            return
        if any(time.monotonic() - self.received.get(k, -float("inf")) > self.source_timeout
               for k in ("arm", "hand")):
            return
        if self.count_publishers(ACTION_TOPIC) > 1:
            raise RuntimeError("Competing /bfm/action publisher")
        arm, hand = deepcopy(self.arm), deepcopy(self.hand)
        # Reapplying the same saturation is idempotent; raw input is never mutated.
        arm.position = [min(max(value, self.arm_limits[name][0]), self.arm_limits[name][1])
                        for name, value in zip(arm.name, arm.position)]
        positions = dict(zip(arm.name, arm.position))
        action = BfmAction()
        self.action_id += 1
        action.producer_session_id = self.producer_session_id
        action.action_id = self.action_id
        action.header.stamp = (deepcopy(simulation_stamp) if simulation_stamp is not None
                               else self.get_clock().now().to_msg())
        action.left_arm_joint_names = list(LEFT_ARM_JOINTS)
        action.left_arm_target = [positions[name] for name in LEFT_ARM_JOINTS]
        action.left_hand_target = [float(value) for value in hand.left_hand_position]
        action.right_arm_joint_names = list(RIGHT_ARM_JOINTS)
        action.right_arm_target = [positions[name] for name in RIGHT_ARM_JOINTS]
        action.right_hand_target = [float(value) for value in hand.right_hand_position]
        # Only canonical actions enter the common executor.
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
