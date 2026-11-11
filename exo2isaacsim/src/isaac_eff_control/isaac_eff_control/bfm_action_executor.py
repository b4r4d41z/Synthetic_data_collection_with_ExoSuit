"""v4 acceptance and deterministic simulation-time execution (no raw inputs)."""
import json
import math
import time
from copy import deepcopy

from .bfm_contract import (LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS,
                           arm_target_processing, executor_metadata)
from .sim_policy_clock import SimPolicyClock

NAMES = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS


class ActionExecutor:
    """ROS-independent state machine. Identity watermarks survive scene resets."""
    def __init__(self):
        metadata = executor_metadata()
        self.config = metadata['configuration']
        self.digest = metadata['controller_config_sha256']
        c = self.config
        if (c['schema_version'] != 1 or c['algorithm'] != 'bounded_velocity_acceleration_position_v1'
                or c['arm_control_hz'] != 100
                or c['hand_control_hz'] != 50 or c['physics_hz'] != 300
                or c['policy_hz'] != 15):
            raise ValueError('Unsupported executor configuration')
        for key, size in [('max_velocity', 14), ('max_acceleration', 14)]:
            if len(c[key]) != size or any(not math.isfinite(v) or v <= 0 for v in c[key]):
                raise ValueError('Invalid trajectory limits: ' + key)
        for key in ('response_time', 'measurement_timeout_sim_sec',
                    'measurement_timeout_wall_sec', 'action_max_age_sim_sec', 'source_timeout_wall_sec'):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError('Invalid configuration: ' + key)
        self.bounds = arm_target_processing()['joint_limits']
        self.watermarks = {}
        self.reset()

    def reset(self):
        self.inhibit()
        self.owner = None
        self.last_stamp = None

    def inhibit(self):
        self.q = self.target = None
        self.v = [0.] * 14
        self.measured_at = None
        self.measured_wall = None

    def measure(self, names, positions, now_ns, wall):
        if (len(set(names)) != len(names) or len(names) != len(positions)
                or not set(NAMES).issubset(names)):
            self.inhibit()
            return False
        q = [positions[names.index(n)] for n in NAMES]
        if any(not math.isfinite(v) or not self.bounds[n][0] <= v <= self.bounds[n][1]
               for n, v in zip(NAMES, q)):
            self.inhibit()
            return False
        if self.q is None:
            self.q = q
        self.measured_at, self.measured_wall = now_ns, wall
        return True

    def fresh(self, now_ns, wall):
        return (self.q is not None and self.measured_at is not None
                and 0 <= now_ns - self.measured_at <= self.config['measurement_timeout_sim_sec'] * 1e9
                and 0 <= wall - self.measured_wall <= self.config['measurement_timeout_wall_sec'])

    def accept(self, action, now_ns, wall):
        try:
            session, aid = action.producer_session_id, action.action_id
            if not isinstance(session, str) or not session.strip() or len(session) > 128:
                raise ValueError('invalid_producer_session')
            if type(aid) is not int or not 0 < aid < 2**64:
                raise ValueError('invalid_action_id')
            if aid <= self.watermarks.get(session, 0):
                raise ValueError('duplicate_or_out_of_order_id')
            if self.owner not in (None, session):
                raise ValueError('producer_conflict_until_reset')
            if list(action.left_arm_joint_names) != list(LEFT_ARM_JOINTS) or list(action.right_arm_joint_names) != list(RIGHT_ARM_JOINTS):
                raise ValueError('invalid_joint_order')
            for values, size in ((action.left_arm_target, 7), (action.right_arm_target, 7),
                                 (action.left_hand_target, 6), (action.right_hand_target, 6)):
                if len(values) != size or any(not math.isfinite(v) for v in values):
                    raise ValueError('invalid_action_dimensions_or_values')
            q = [*action.left_arm_target, *action.right_arm_target]
            if any(not self.bounds[n][0] <= v <= self.bounds[n][1] for n, v in zip(NAMES, q)):
                raise ValueError('arm_target_out_of_bounds')
            if any(not 0 <= v <= 100 for v in (*action.left_hand_target, *action.right_hand_target)):
                raise ValueError('hand_target_out_of_bounds')
            stamp = action.header.stamp.sec * 1_000_000_000 + action.header.stamp.nanosec
            if not 0 <= action.header.stamp.nanosec < 1_000_000_000:
                raise ValueError('invalid_stamp')
            if stamp < 0 or not 0 <= now_ns - stamp <= self.config['action_max_age_sim_sec'] * 1e9:
                raise ValueError('stale_or_future_decision')
            if self.last_stamp is not None and stamp <= self.last_stamp:
                raise ValueError('duplicate_or_backward_stamp')
            if not self.fresh(now_ns, wall):
                raise ValueError('measurement_unavailable_or_stale')
        except (ValueError, AttributeError, TypeError, OverflowError) as error:
            return False, str(error)
        self.owner, self.last_stamp = session, stamp
        self.watermarks[session] = aid
        self.target = q
        return True, 'accepted_not_attained'

    def step(self, now_ns, wall):
        if not self.fresh(now_ns, wall):
            # Fault inhibits output and requires measured reinitialization + a new action.
            self.inhibit()
            return None
        if self.target is None:
            return None
        for i, name in enumerate(NAMES):
            vmax, amax = self.config['max_velocity'][i], self.config['max_acceleration'][i]
            desired = max(-vmax, min(vmax, (self.target[i] - self.q[i]) / self.config['response_time']))
            v = max(self.v[i] - amax * .01, min(self.v[i] + amax * .01, desired))
            lo, hi = self.bounds[name]
            q = max(lo, min(hi, self.q[i] + v * .01))
            self.q[i], self.v[i] = q, v
        return list(self.q)


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from dynamic_biped.msg import BfmAction, BfmExecutionStatus, RobotHandPosition

    class ExecutorNode(Node):
        def __init__(self):
            super().__init__('bfm_action_executor')
            self.engine = ActionExecutor()
            self.scheduler = SimPolicyClock(100)
            self.stamp = None
            self.playing = False
            self.boundary = None
            self.arm = self.create_publisher(JointState, '/robot_arm_joint_command', 10)
            self.hand = self.create_publisher(RobotHandPosition, '/bfm/command/hand_target', 10)
            self.status = self.create_publisher(BfmExecutionStatus, '/bfm/execution_status', 100)
            self.create_subscription(BfmAction, '/bfm/action', self.action, 100)
            self.create_subscription(JointState, '/robot/state/joint_states', self.measure,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(Clock, '/clock', self.clock,
                QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(String, '/bfm/episode', self.episode,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def ns(self):
            return self.stamp.sec * 1_000_000_000 + self.stamp.nanosec

        def episode(self, msg):
            event = json.loads(msg.data)
            if 'phase' not in event:  # recorder outcome event, not lifecycle control
                return
            boundary = event.get('boundary_id')
            if boundary != self.boundary or event['phase'] in ('stopped', 'failed', 'closed'):
                self.engine.reset()
                self.scheduler = SimPolicyClock(100)
            self.boundary = boundary
            self.playing = event['phase'] == 'playing'

        def measure(self, msg):
            if self.stamp is not None and self.playing:
                self.engine.measure(list(msg.name), list(msg.position), self.ns(), time.monotonic())

        def clock(self, msg):
            now = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
            if self.scheduler.last_ns is not None and now < self.scheduler.last_ns:
                self.engine.reset()
            self.stamp = msg.clock
            if self.scheduler.advance(now) and self.playing:
                for topic in ('/robot_arm_joint_command', '/bfm/command/hand_target'):
                    if self.count_publishers(topic) > 1:
                        raise RuntimeError('Competing executor publisher: ' + topic)
                q = self.engine.step(now, time.monotonic())
                if q is not None:
                    out = JointState()
                    out.header.stamp = deepcopy(self.stamp)
                    out.name, out.position = list(NAMES), q
                    self.arm.publish(out)

        def action(self, msg):
            accepted, reason = (self.engine.accept(msg, self.ns(), time.monotonic())
                                if self.stamp is not None and self.playing
                                else (False, 'not_playing_or_no_clock'))
            status = BfmExecutionStatus()
            if self.stamp is not None:
                status.header.stamp = deepcopy(self.stamp)
            status.producer_session_id, status.action_id = msg.producer_session_id, msg.action_id
            status.accepted, status.reason = accepted, reason
            status.controller_config_sha256 = self.engine.digest
            self.status.publish(status)
            if accepted:
                hand = RobotHandPosition()
                hand.header.stamp = deepcopy(self.stamp)
                # Existing message uses integers; preserve its execution quantization.
                hand.left_hand_position = [round(v) for v in msg.left_hand_target]
                hand.right_hand_position = [round(v) for v in msg.right_hand_target]
                self.hand.publish(hand)

    rclpy.init(args=args)
    node = ExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
