#!/usr/bin/env python3

import os
import sys
import threading
from time import monotonic

import rclpy
import zmq

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from dynamic_biped.msg import RobotHandPosition
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


ROS2_HAND_TOPIC = os.getenv(
    "ROS2_HAND_TOPIC",
    "/robot/command/hand_target",
)

ROS2_ARM_TOPIC = os.getenv(
    "ROS2_ARM_TOPIC",
    "/robot/command/arm_joint_target",
)

ROS2_RECORDING_TOPIC = os.getenv(
    "ROS2_RECORDING_TOPIC",
    "/data_collection/recording",
)

ZMQ_ENDPOINT = os.getenv(
    "ZMQ_BIND_ENDPOINT",
    "tcp://0.0.0.0:5555",
)


class Ros2Publisher(Node):
    def __init__(self):
        super().__init__("exo_ros2_publisher")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.hand_publisher = self.create_publisher(
            RobotHandPosition,
            ROS2_HAND_TOPIC,
            qos,
        )

        self.arm_publisher = self.create_publisher(
            JointState,
            ROS2_ARM_TOPIC,
            qos,
        )

        recording_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.recording_publisher = self.create_publisher(
            Bool,
            ROS2_RECORDING_TOPIC,
            recording_qos,
        )

        self._stop = threading.Event()
        self._ready = threading.Event()
        self._input_lock = threading.Lock()
        self._destroyed = False
        self.receiver_error = None
        self.input_counts = {kind: 0 for kind in ('arm', 'hand', 'recording')}
        self.forwarded_counts = dict(self.input_counts)
        self.invalid_counts = dict(self.input_counts)
        self.decode_errors = 0
        self.last_processing_error = None
        self.input_valid = {}
        self.input_times = {}
        self.input_health = {}
        self._reported_input = {}
        self._reported_error = None
        self.diagnostics_timer = self.create_timer(1.0, self.report_input)
        self.thread = threading.Thread(
            target=self.receive_loop,
            name='exo-zmq-receiver',
        )
        self.thread.start()
        if not self._ready.wait(5.0) or self.receiver_error:
            self._stop.set()
            self.thread.join()
            super().destroy_node()
            raise RuntimeError(f'Bridge receiver startup failed: {self.receiver_error or "startup timeout"}')

        self.get_logger().info(
            f"Publishing hand {ROS2_HAND_TOPIC}, arm {ROS2_ARM_TOPIC}, "
            f"and recording state {ROS2_RECORDING_TOPIC}; "
            f"waiting for bridge data on {ZMQ_ENDPOINT}"
        )

    def receive_loop(self):
        # PUSH/PULL are not thread-safe: this worker owns the entire lifecycle.
        # The private context never terminates a context used by another node.
        context = socket = None
        try:
            context = zmq.Context()
            socket = context.socket(zmq.PULL)
            socket.setsockopt(zmq.RCVHWM, 1)
            socket.bind(ZMQ_ENDPOINT)
            self._ready.set()
            while not self._stop.is_set() and rclpy.ok():
                if not socket.poll(100, zmq.POLLIN) or self._stop.is_set():
                    continue
                try:
                    self.process_payload(socket.recv_json(flags=zmq.NOBLOCK))
                except zmq.Again:
                    continue
                except (ValueError, TypeError, UnicodeError) as error:
                    with self._input_lock:
                        self.decode_errors += 1
                        self.last_processing_error = str(error)
        except Exception as error:
            with self._input_lock:
                self.receiver_error = str(error)
        finally:
            if socket is not None:
                socket.close(linger=0)
            if context is not None:
                context.term()
            self._ready.set()

    def process_payload(self, payload):
        if not isinstance(payload, dict):
            raise ValueError('Bridge payload must be a JSON object')
        kind = payload.get('kind', 'hand')
        if kind not in self.input_counts:
            raise ValueError(f'Unknown bridge kind: {kind}')
        with self._input_lock:
            self.input_counts[kind] += 1
            self.input_times[kind] = monotonic()
        try:
            valid = {'hand': self.publish_hand, 'arm': self.publish_arm,
                     'recording': self.publish_recording}[kind](payload)
        except Exception as error:
            valid = False
            with self._input_lock:
                self.last_processing_error = f'{kind}: {error}'
        with self._input_lock:
            self.input_valid[kind] = bool(valid)
            (self.forwarded_counts if valid else self.invalid_counts)[kind] += 1

    def report_input(self):
        """Diagnostics only: never drop, gate or synthesize bridge messages."""
        now = monotonic()
        with self._input_lock:
            counts, times, valid = dict(self.input_counts), dict(self.input_times), dict(self.input_valid)
            forwarded, invalid = dict(self.forwarded_counts), dict(self.invalid_counts)
            error = self.receiver_error or self.last_processing_error
            decode_errors = self.decode_errors
        # No socket, ROS publish or logging operation is performed under this lock.
        try:
            if error != self._reported_error:
                self.get_logger().error(f'[EXO BRIDGE] processing/receiver error: {error}; '
                                        f'decode_errors={decode_errors}; invalid={invalid}')
                self._reported_error = error
            self._report_streams(now, counts, times, valid, forwarded, invalid)
        except Exception as error:
            # A diagnostic backend failure must not terminate spin/command delivery.
            try:
                print(f'[EXO BRIDGE] diagnostic logging failed: {error}', file=sys.stderr)
            except OSError:
                pass  # A closed diagnostic stderr must not stop command delivery.

    def _report_streams(self, now, counts, times, valid, forwarded, invalid):
        for kind, source in (('arm', '/kuavo_arm_traj'), ('hand', '/control_robot_hand_position')):
            received = times.get(kind)
            active = received is not None and now - received <= 1.0
            state = (active, valid.get(kind, True))
            if self._reported_input.get(kind) != state:
                self.input_health[kind] = active
                message = (f'[EXO BRIDGE] {kind} {"ACTIVE" if active else "NO RECENT INPUT"}; '
                           f'ZMQ received counts={counts}; forwarded={forwarded}; invalid={invalid}; '
                           f'last_payload_valid={state[1]}. '
                           f'Inactivity is not proof of bridge failure; check ROS1 {source} '
                           'and ros1_receiver before the ROS2 mapper.')
                # Jazzy caches severity by call site: never dynamically select a method.
                if active and state[1]:
                    self.get_logger().info(message)
                else:
                    self.get_logger().warning(message)
                self._reported_input[kind] = state

    def publish_hand(self, payload):
        left = payload.get("left_hand_position", [])
        right = payload.get("right_hand_position", [])

        if len(left) != 6 or len(right) != 6:
            self.get_logger().warning(
                "Dropped hand command with invalid array length"
            )
            return False

        msg = RobotHandPosition()

        msg.header.stamp.sec = int(payload.get("stamp_sec", 0))
        msg.header.stamp.nanosec = int(payload.get("stamp_nanosec", 0))
        msg.header.frame_id = str(payload.get("frame_id", ""))

        msg.left_hand_position = [
            int(value) for value in left
        ]
        msg.right_hand_position = [
            int(value) for value in right
        ]

        self.hand_publisher.publish(msg)
        return True

    def publish_arm(self, payload):
        msg = JointState()

        msg.header.stamp.sec = int(payload.get("stamp_sec", 0))
        msg.header.stamp.nanosec = int(payload.get("stamp_nanosec", 0))
        msg.header.frame_id = str(payload.get("frame_id", ""))

        msg.name = [
            str(value) for value in payload.get("name", [])
        ]
        msg.position = [
            float(value) for value in payload.get("position", [])
        ]
        msg.velocity = [
            float(value) for value in payload.get("velocity", [])
        ]
        msg.effort = [
            float(value) for value in payload.get("effort", [])
        ]

        if len(msg.name) != len(msg.position):
            self.get_logger().warning(
                f"Dropped arm command: names={len(msg.name)} "
                f"positions={len(msg.position)}"
            )
            return False

        self.arm_publisher.publish(msg)
        return True

    def publish_recording(self, payload):
        enabled = payload.get("enabled")
        if not isinstance(enabled, bool):
            self.get_logger().warning(
                "Dropped recording state: enabled must be a boolean"
            )
            return False
        self.recording_publisher.publish(Bool(data=enabled))
        return True

    def destroy_node(self):
        if self._destroyed:
            return
        self._stop.set()
        self.thread.join()  # poll is bounded; close/term happen only in its owner.
        self._destroyed = True
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = Ros2Publisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
