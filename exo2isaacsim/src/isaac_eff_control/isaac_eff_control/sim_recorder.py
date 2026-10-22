#!/usr/bin/env python3
"""Record selected ROS2 topics when the retained recording state is true.

Run in the existing Isaac Sim environment with ``ros2 run isaac_eff_control
sim_recorder_node``. Configure output_root, topics, and stop_timeout_sec through
ROS2 parameters at startup. rosbag2 inherits this process's ROS environment.
"""

from datetime import datetime
import math
import os
from pathlib import Path
import signal
import subprocess
from uuid import uuid4

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


RECORDING_TOPIC = "/data_collection/recording"
DEFAULT_TOPICS = [
    "/robot/command/hand_target",
    "/robot/command/arm_joint_target",
    RECORDING_TOPIC,
    "/robot/state/joint_states",
    "/sensors/camera/head/image_raw",
    "/sensors/camera/left_wrist/image_raw",
    "/sensors/camera/right_wrist/image_raw",
    "/clock",
]


class SimRecorder(Node):
    """Manage one rosbag2 process; repeated state messages are idempotent."""

    def __init__(self):
        super().__init__("sim_recorder")
        output_root = self.declare_parameter(
            "output_root",
            "/workspace/Synthetic_data_collection_with_ExoSuit/records",
        ).value
        self.topics = list(self.declare_parameter("topics", DEFAULT_TOPICS).value)
        self.stop_timeout_sec = float(
            self.declare_parameter("stop_timeout_sec", 15.0).value
        )
        if not self.topics or any(
            not isinstance(topic, str)
            or not topic.startswith("/")
            or any(char.isspace() for char in topic)
            for topic in self.topics
        ):
            raise ValueError("topics must be a nonempty list of absolute ROS topic names")
        if not math.isfinite(self.stop_timeout_sec) or self.stop_timeout_sec <= 0:
            raise ValueError("stop_timeout_sec must be finite and positive")
        if not output_root or not str(output_root).strip():
            raise ValueError("output_root must not be empty")

        self.output_root = Path(output_root).expanduser().resolve()
        self.output_root.mkdir(parents=True, exist_ok=True)
        self._recording_enabled = False
        self._process = None
        self._bag_path = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._subscription = self.create_subscription(
            Bool, RECORDING_TOPIC, self.recording_callback, qos
        )
        self._monitor = self.create_timer(1.0, self.check_process)
        self.get_logger().info(
            f"Recording control: {RECORDING_TOPIC} (reliable, transient local); "
            f"output root: {self.output_root}; topics: {self.topics}"
        )

    def recording_callback(self, msg):
        """Treat the Bool as desired state, not a repeated start/stop event."""
        if msg.data == self._recording_enabled:
            return
        if msg.data:
            # On startup failure, require a false/true cycle rather than retrying
            # indefinitely on repeated true messages.
            self._recording_enabled = True
            self.start_recording()
        elif self.stop_recording():
            self._recording_enabled = False

    def start_recording(self):
        """Start a new bag without reusing an existing output directory."""
        if self._process is not None:
            self.get_logger().warning("Previous recorder still exists; refusing another")
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self._bag_path = self.output_root / f"record_{stamp}_{uuid4().hex[:8]}"
        while self._bag_path.exists():
            self._bag_path = self.output_root / f"record_{stamp}_{uuid4().hex[:8]}"
        command = [
            "ros2", "bag", "record", "--output", str(self._bag_path), *self.topics
        ]
        try:
            # Keep stdout/stderr in the tmux pane. A separate session lets us
            # interrupt the recorder and any children without signalling tmux.
            self._process = subprocess.Popen(command, start_new_session=True)
        except OSError as error:
            self.get_logger().error(
                f"Cannot start ros2 bag: {error}. Toggle recording false/true to retry."
            )
            return
        self.get_logger().info(
            f"Started ros2 bag pid={self._process.pid}: {self._bag_path}; "
            f"topics: {self.topics}"
        )

    def stop_recording(self):
        """Interrupt rosbag2 to finalize metadata, with bounded escalation."""
        process = self._process
        if process is None:
            return True
        for stop_signal, timeout in (
            (signal.SIGINT, self.stop_timeout_sec),
            (signal.SIGTERM, 5.0),
            (signal.SIGKILL, 5.0),
        ):
            return_code = process.poll()
            if return_code is not None:
                break
            if stop_signal == signal.SIGINT:
                self.get_logger().info(f"Stopping bag with SIGINT: {self._bag_path}")
            else:
                self.get_logger().warning(
                    f"Escalating recorder shutdown to {stop_signal.name}; "
                    "the bag may not be finalized"
                )
            try:
                os.killpg(process.pid, stop_signal)
            except ProcessLookupError:
                pass  # The child may have exited between poll() and killpg().
            except OSError as error:
                self.get_logger().error(f"Cannot signal recorder: {error}")
                return False
            try:
                return_code = process.wait(timeout=timeout)
                break
            except subprocess.TimeoutExpired:
                self.get_logger().warning(
                    f"Recorder did not exit within {timeout}s after {stop_signal.name}"
                )
        else:
            # Retain the handle so another true cannot spawn a second recorder.
            self.get_logger().error("Recorder is still running after SIGKILL")
            return False

        self._process = None
        self.get_logger().info(
            f"Recorder stopped (exit code {return_code}); bag: {self._bag_path}"
        )
        return True

    def check_process(self):
        """Report early exits without silently restarting into many empty bags."""
        if self._process is not None:
            return_code = self._process.poll()
            if return_code is not None:
                self._process = None
                self.get_logger().error(
                    f"ros2 bag exited unexpectedly (code {return_code}); "
                    f"output: {self._bag_path}. Toggle recording false/true to retry."
                )

    def destroy_node(self):
        """Finalize an active bag on Ctrl-C, SIGTERM, or normal node shutdown."""
        self.stop_recording()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    # Closing/replacing a tmux pane sends SIGHUP. The bag's separate session
    # would otherwise survive the recorder, so route hangup through cleanup.
    previous_hup = signal.signal(signal.SIGHUP, lambda *_: rclpy.try_shutdown())
    node = None
    try:
        node = SimRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGHUP, previous_hup)


if __name__ == "__main__":
    main()
