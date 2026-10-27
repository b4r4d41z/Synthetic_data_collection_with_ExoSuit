#!/usr/bin/env python3
"""Record selected ROS2 topics when the retained recording state is true.

Run in the existing Isaac Sim environment with ``ros2 run isaac_eff_control
sim_recorder_node``. Each timestamped episode contains episode.json and a bag/
directory using MCAP. Task parameters are captured once per episode. Message
headers retain their source-native times; rosbag record time is the cross-topic
synchronization contract for downstream conversion.
"""

from datetime import datetime
import json
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

from .bfm_episode import ACK_TOPIC, CONTROL_TOPIC, EPISODE_TOPIC

from .bfm_contract import (
    collection_enabled, collection_metadata, scene_config_default,
    scene_configuration, state_defaults,
)


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
    "/robot_arm_joint_command",
    "/right_hand_joint_command",
    "/bfm/action",
    "/bfm/state",
]

EPISODE_PARAMETERS = {
    "source": "sim",
    "task_id": "sorting_food",
    "instruction": "Sort the food items into their corresponding containers.",
    "robot": "kuavo_s40_sharpa",
    "scene": "s40_sharpa_two_cameras.usd",
}
CAMERA_TOPICS = {
    "head": "/sensors/camera/head/image_raw",
    "left_wrist": "/sensors/camera/left_wrist/image_raw",
    "right_wrist": "/sensors/camera/right_wrist/image_raw",
}


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
        for name, default in EPISODE_PARAMETERS.items():
            self.declare_parameter(name, default)
        self.declare_parameter(
            "camera_metadata_path", str(Path(__file__).with_name("camera_metadata.json"))
        )
        self.declare_parameter("bfm_collection_mode", collection_enabled())
        self._scene_config_path = self.declare_parameter(
            "scene_config_path", scene_config_default()).value
        defaults = state_defaults(self._scene_config_path)
        self._state_parameter_names = tuple(defaults)
        self._scene_configuration = scene_configuration(self._scene_config_path)
        for name, default in defaults.items():
            self.declare_parameter(name, default)
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
        self._episode_status = None
        self._recorder_id = uuid4().hex
        self._recording_enabled = False
        self._process = None
        self._episode_path = None
        self._bag_path = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        if self.get_parameter("bfm_collection_mode").value:
            from std_msgs.msg import String
            self._episode_string = String
            self._ack_pub = self.create_publisher(String, ACK_TOPIC, 10)
            self._episode_subscription = self.create_subscription(
                String, CONTROL_TOPIC, self.episode_callback, qos)
            if EPISODE_TOPIC not in self.topics:
                self.topics.append(EPISODE_TOPIC)
        self._subscription = self.create_subscription(
            Bool, RECORDING_TOPIC, self.recording_callback, qos
        )
        self._monitor = self.create_timer(1.0, self.check_process)
        self.get_logger().info(
            f"Recording control: {RECORDING_TOPIC} (reliable, transient local); "
            f"output root: {self.output_root}; topics: {self.topics}"
        )

    def episode_callback(self, msg):
        """Fail closed at a timeline boundary before acknowledging reset safety."""
        try:
            status = json.loads(msg.data)
            phase = status["phase"]
            if phase == "playing":
                episode = status["episode"]
                if not episode.get("episode_id") or not episode.get("object_prim_path"):
                    raise ValueError("Playing episode lacks identity")
            previous = self._episode_status
            changed = (previous is not None and
                       (previous.get("episode") or {}).get("episode_id") !=
                       (status.get("episode") or {}).get("episode_id"))
            if phase != "playing" or changed:
                # A retained true is never carried over to a new task episode.
                # Leave the Bool latch true until the operator sends false.
                if not self.stop_recording():
                    raise ValueError("Recorder has not finalized; refusing reset acknowledgement")
            self._episode_status = status
            if phase == "stopped":
                ack = self._episode_string()
                ack.data = json.dumps(dict(boundary_id=status["boundary_id"],
                                           recorder_id=self._recorder_id))
                self._ack_pub.publish(ack)
        except (ValueError, KeyError, TypeError, AttributeError) as error:
            self._episode_status = None
            self.stop_recording()
            self.get_logger().error(f"Invalid BFM episode control: {error}")

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

    def episode_metadata(self):
        """Snapshot task parameters and camera configuration for the next episode."""
        fields = {name: self.get_parameter(name).value for name in EPISODE_PARAMETERS}
        if any(not isinstance(value, str) or not value.strip() for value in fields.values()):
            raise ValueError(
                "Episode source, task_id, instruction, robot, and scene "
                "must be nonempty strings"
            )
        path = Path(self.get_parameter("camera_metadata_path").value).expanduser()
        snapshot = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(snapshot, dict) or not isinstance(snapshot.get("provenance"), dict):
            raise ValueError("Camera metadata must contain a provenance object")
        cameras = snapshot.get("cameras")
        if not isinstance(cameras, dict) or set(cameras) != set(CAMERA_TOPICS):
            raise ValueError("Camera metadata must describe head, left_wrist, and right_wrist")
        for name, topic in CAMERA_TOPICS.items():
            if not isinstance(cameras[name], dict) or cameras[name].get("topic") != topic:
                raise ValueError(f"Camera metadata topic does not match {name}: {topic}")
        bfm = collection_metadata(
            self.get_parameter("bfm_collection_mode").value,
            {key: self.get_parameter(key).value for key in self._state_parameter_names},
            self._scene_configuration,
        )
        if bfm["enabled"]:
            if self._episode_status is None or self._episode_status["phase"] != "playing":
                raise ValueError("BFM recording requires a prepared episode and live PhysX measurements")
            episode = json.loads(json.dumps(self._episode_status["episode"], allow_nan=False))
            bfm["episode"] = episode
            bfm["task_state"]["object_prim_path"] = episode["object_prim_path"]
        return {
            "schema_version": 1,
            **fields,
            "success": None,
            "created_at": datetime.now().astimezone().isoformat(),
            "time": {
                "synchronization_source": "rosbag_record_timestamp",
                "header_stamp": "source_native",
            },
            "recording": {
                "storage_id": "mcap",
                "bag_directory": "bag",
                "topics": list(self.topics),
            },
            "camera_configuration": snapshot["provenance"],
            "cameras": cameras,
            "bfm": bfm,
        }

    def start_recording(self):
        """Start a new bag without reusing an existing output directory."""
        if self._process is not None:
            self.get_logger().warning("Previous recorder still exists; refusing another")
            return
        try:
            metadata = self.episode_metadata()
            serialized_metadata = json.dumps(
                metadata, indent=2, ensure_ascii=False, allow_nan=False
            ) + "\n"
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            while True:
                self._episode_path = self.output_root / f"record_{stamp}_{uuid4().hex[:8]}"
                try:
                    self._episode_path.mkdir()
                    break
                except FileExistsError:
                    continue
            # rosbag2 must create its own output directory. The episode parent
            # owns the sidecar, which is saved before the bag process is started.
            self._bag_path = self._episode_path / "bag"
            with (self._episode_path / "episode.json").open("x", encoding="utf-8") as output:
                output.write(serialized_metadata)
            command = [
                "ros2", "bag", "record", "--output", str(self._bag_path),
                "--storage", "mcap", *self.topics,
            ]
            # Keep stdout/stderr in the tmux pane. A separate session lets us
            # interrupt the recorder and any children without signalling tmux.
            self._process = subprocess.Popen(command, start_new_session=True)
        except (OSError, ValueError) as error:
            self.get_logger().error(
                f"Cannot start MCAP episode: {error}. Toggle recording false/true to retry."
            )
            return
        self.get_logger().info(
            f"Started MCAP ros2 bag pid={self._process.pid}: {self._bag_path}; "
            f"task: {metadata['task_id']}; instruction: {metadata['instruction']}; "
            f"metadata: {self._episode_path / 'episode.json'}; topics: {self.topics}"
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
