"""Policy-only direct target translation. No trajectory, clock gate or ACK.

Never run alongside ExoSuit collection: the launchers enforce exclusive control.
The passive collection sampler has no connection to this node in teleoperation.
"""
import math
from copy import deepcopy

from .bfm_contract import LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, arm_target_processing


def validate_action(message):
    """Validate the existing 26D schema; timestamps have no acceptance deadline."""
    if (not isinstance(message.producer_session_id, str)
            or not message.producer_session_id.strip()
            or type(message.action_id) is not int or not 0 < message.action_id < 2**64):
        raise ValueError('Invalid producer_session_id/action_id')
    if (list(message.left_arm_joint_names) != list(LEFT_ARM_JOINTS)
            or list(message.right_arm_joint_names) != list(RIGHT_ARM_JOINTS)):
        raise ValueError('Invalid canonical joint names/order')
    for group, size in (('left_arm', 7), ('left_hand', 6), ('right_arm', 7), ('right_hand', 6)):
        values = getattr(message, group + '_target')
        if len(values) != size or any(not math.isfinite(v) for v in values):
            raise ValueError('Invalid dimensions/nonfinite values: ' + group)
    limits = arm_target_processing()['joint_limits']
    names = [*LEFT_ARM_JOINTS, *RIGHT_ARM_JOINTS]
    q = [*message.left_arm_target, *message.right_arm_target]
    if any(not limits[n][0] <= v <= limits[n][1] for n, v in zip(names, q)):
        raise ValueError('Arm target outside mechanical limits')
    if any(not 0 <= v <= 100 for v in (*message.left_hand_target, *message.right_hand_target)):
        raise ValueError('Hand target outside 0..100')
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('Invalid decision timestamp')
    return names, q


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from dynamic_biped.msg import BfmAction, RobotHandPosition
    from sensor_msgs.msg import JointState

    class PolicyBridge(Node):
        def __init__(self):
            super().__init__('bfm_policy_bridge')
            self.watermarks = {}
            self.arm = self.create_publisher(JointState, '/robot_arm_joint_command', 1)
            self.hand = self.create_publisher(RobotHandPosition, '/bfm/command/hand_target', 1)
            self.create_subscription(BfmAction, '/bfm/action', self.action, 1)

        def action(self, message):
            try:
                names, q = validate_action(message)
                if message.action_id <= self.watermarks.get(message.producer_session_id, 0):
                    raise ValueError('Duplicate/backward action identity')
            except (ValueError, TypeError, AttributeError, OverflowError) as error:
                self.get_logger().warning(f'Rejected malformed policy target: {error}')
                return
            if any(self.count_publishers(topic) > 1 for topic in
                   ('/robot_arm_joint_command', '/bfm/command/hand_target', '/bfm/action')):
                raise RuntimeError('Competing command producer; stop collection before policy control')
            arm, hand = JointState(), RobotHandPosition()
            arm.header, hand.header = deepcopy(message.header), deepcopy(message.header)
            arm.name, arm.position = names, q
            hand.left_hand_position = [round(v) for v in message.left_hand_target]
            hand.right_hand_position = [round(v) for v in message.right_hand_target]
            self.arm.publish(arm)
            self.hand.publish(hand)
            self.watermarks[message.producer_session_id] = message.action_id

    rclpy.init(args=args)
    node = PolicyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
