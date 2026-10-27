"""Recording relay and process lifecycle tests without ROS or Isaac Sim."""

import importlib.util
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch


ROOT = Path(__file__).resolve().parents[1]


class Bool:
    def __init__(self, data=False):
        self.data = data


class Message:
    def __init__(self):
        self.name, self.position, self.velocity, self.effort = [], [], [], []
        self.left_hand_position, self.right_hand_position = [], []
        self.header = SimpleNamespace(
            stamp=SimpleNamespace(sec=0, nanosec=0, secs=0, nsecs=0),
            frame_id="",
        )


def load_module(name, path):
    if "isaac_eff_control/isaac_eff_control/" in path:
        name = f"isaac_eff_control.{name}"
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def ros_stubs(overrides):
    """Provide only the ROS surface needed by the actual production classes."""
    class Node:
        def __init__(self, name, **kwargs):
            self.logger = Mock()
            self.publishers = {}
            self.subscriptions = {}
            self.parameters = {}
            self.timers = []
            self.destroyed = False

        def declare_parameter(self, name, default):
            self.parameters[name] = overrides.get(name, default)
            return SimpleNamespace(value=self.parameters[name])

        def get_parameter(self, name):
            return SimpleNamespace(value=overrides.get(name, self.parameters[name]))

        def create_publisher(self, msg_type, topic, qos):
            publisher = Mock(qos=qos, msg_type=msg_type)
            self.publishers[topic] = publisher
            return publisher

        def create_subscription(self, msg_type, topic, callback, qos):
            subscription = SimpleNamespace(callback=callback, qos=qos)
            self.subscriptions[topic] = subscription
            return subscription

        def create_timer(self, period, callback):
            timer = SimpleNamespace(period=period, callback=callback)
            self.timers.append(timer)
            return timer

        def get_logger(self):
            return self.logger

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(
                to_msg=lambda: SimpleNamespace(sec=0, nanosec=0)))

        def destroy_node(self):
            self.destroyed = True
            return True

    modules = {
        name: ModuleType(name) for name in (
            "rclpy", "rclpy.node", "rclpy.qos", "rclpy.executors",
            "rospy", "zmq", "std_msgs", "std_msgs.msg",
            "sensor_msgs", "sensor_msgs.msg", "dynamic_biped", "dynamic_biped.msg",
        )
    }
    modules["rclpy.node"].Node = Node
    for name in ("init", "spin", "shutdown", "try_shutdown", "ok"):
        setattr(modules["rclpy"], name, Mock(return_value=True))
    modules["rclpy.executors"].ExternalShutdownException = type(
        "ExternalShutdownException", (Exception,), {}
    )
    qos = modules["rclpy.qos"]
    qos.QoSProfile = SimpleNamespace
    qos.ReliabilityPolicy = SimpleNamespace(RELIABLE="reliable", BEST_EFFORT="best_effort")
    qos.DurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL="transient_local")
    qos.HistoryPolicy = SimpleNamespace(KEEP_LAST="keep_last")
    modules["std_msgs.msg"].Bool = Bool
    modules["std_msgs.msg"].String = Message
    modules["sensor_msgs.msg"].JointState = Message
    modules["dynamic_biped.msg"].RobotHandPosition = Message
    modules["dynamic_biped.msg"].BfmAction = Message
    modules["dynamic_biped.msg"].BfmState = Message
    modules["dynamic_biped.msg"].robotHandPosition = Message
    for name in ("Subscriber", "loginfo", "logwarn_throttle"):
        setattr(modules["rospy"], name, Mock())
    zmq = modules["zmq"]
    zmq.Context = Mock()
    zmq.PUSH, zmq.PULL, zmq.SNDHWM, zmq.RCVHWM = range(4)
    zmq.error = SimpleNamespace(ContextTerminated=type("ContextTerminated", (Exception,), {}))
    return modules


class RecordingPipelineTests(unittest.TestCase):
    def setUp(self):
        env_patch = patch.dict(os.environ, {"BFM_COLLECTION_MODE": "0"})
        env_patch.start()
        self.addCleanup(env_patch.stop)
        path_patch = patch.object(sys, "path", [
            str(ROOT / "exo2isaacsim/src/isaac_eff_control"), *sys.path,
        ])
        path_patch.start()
        self.addCleanup(path_patch.stop)
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.overrides = {"output_root": str(Path(temporary.name) / "records")}
        self.modules = ros_stubs(self.overrides)
        modules_patch = patch.dict(sys.modules, self.modules)
        modules_patch.start()
        self.addCleanup(modules_patch.stop)
        self.recorder = load_module(
            "test_sim_recorder",
            "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/sim_recorder.py",
        )
        self.ros1 = load_module(
            "test_ros1_receiver", "docker/exo_ros_bridge/ros1/bridge/ros1_receiver.py"
        )
        self.ros2 = load_module(
            "test_ros2_publisher", "docker/exo_ros_bridge/ros2/bridge/ros2_publisher.py"
        )
        self.original_popen = subprocess.Popen
        for module, attribute, target in (
            (self.recorder.subprocess, "Popen", "popen"),
            (self.recorder.os, "killpg", "killpg"),
            (self.ros2.threading, "Thread", "thread"),
        ):
            patcher = patch.object(module, attribute)
            setattr(self, target, patcher.start())
            self.addCleanup(patcher.stop)
        self.process = Mock(pid=12345)
        self.process.poll.return_value = None
        self.process.wait.return_value = 0
        self.popen.return_value = self.process

    def test_default_topics_include_actions_trigger_and_sim_observations(self):
        expected = [
            "/robot/command/hand_target",
            "/robot/command/arm_joint_target",
            "/data_collection/recording",
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
        node = self.recorder.SimRecorder()
        self.assertEqual(node.topics, expected)
        node.recording_callback(Bool(True))
        self.assertEqual(self.popen.call_args.args[0][7:], expected)

    def test_state_transitions_are_idempotent_and_each_bag_is_new(self):
        node = self.recorder.SimRecorder()
        node.recording_callback(Bool(False))
        self.popen.assert_not_called()
        node.recording_callback(Bool(True))
        node.recording_callback(Bool(True))
        self.popen.assert_called_once()
        command = self.popen.call_args.args[0]
        self.assertEqual(command[:4], ["ros2", "bag", "record", "--output"])
        self.assertEqual(command[5:7], ["--storage", "mcap"])
        self.assertEqual(command[7:], self.recorder.DEFAULT_TOPICS)
        self.assertEqual(self.popen.call_args.kwargs, {"start_new_session": True})
        self.assertEqual(Path(command[4]).name, "bag")
        self.assertTrue(Path(command[4]).parent.name.startswith("record_"))
        self.assertTrue(node.output_root.is_dir())
        node.recording_callback(Bool(False))
        node.recording_callback(Bool(False))
        self.killpg.assert_called_once_with(12345, signal.SIGINT)
        self.process.wait.assert_called_once_with(timeout=15.0)
        node.recording_callback(Bool(True))
        self.assertEqual(self.popen.call_count, 2)
        self.assertNotEqual(command[4], self.popen.call_args.args[0][4])

    def test_mcap_episode_metadata_is_saved_before_rosbag_starts(self):
        def start(command, **kwargs):
            bag = Path(command[4])
            self.assertFalse(bag.exists())  # rosbag2 owns this output directory.
            metadata = json.loads((bag.parent / "episode.json").read_text())
            self.assertEqual(command[5:7], ["--storage", "mcap"])
            self.assertEqual(metadata["recording"], {
                "storage_id": "mcap", "bag_directory": "bag",
                "topics": self.recorder.DEFAULT_TOPICS,
            })
            self.assertEqual(metadata["source"], "sim")
            self.assertEqual(metadata["task_id"], "sorting_food")
            self.assertEqual(metadata["robot"], "kuavo_s40_sharpa")
            self.assertEqual(metadata["scene"], "s40_sharpa_two_cameras.usd")
            self.assertIsNone(metadata["success"])
            self.assertEqual(metadata["time"], {
                "synchronization_source": "rosbag_record_timestamp",
                "header_stamp": "source_native",
            })
            return self.process

        self.popen.side_effect = start
        node = self.recorder.SimRecorder()
        node.recording_callback(Bool(True))
        node.recording_callback(Bool(True))
        self.popen.assert_called_once()
        self.assertEqual(len(list(node.output_root.glob("record_*/episode.json"))), 1)

    def test_task_parameters_are_preserved_and_snapshotted_per_episode(self):
        instruction = "把食物分类。\nSort the food into the matching containers."
        self.overrides.update(task_id="sorting_food_v2", instruction=instruction)
        node = self.recorder.SimRecorder()
        node.recording_callback(Bool(True))
        first_path = node._episode_path / "episode.json"
        first = json.loads(first_path.read_text(encoding="utf-8"))
        self.assertEqual(first["task_id"], "sorting_food_v2")
        self.assertEqual(first["instruction"], instruction)
        self.overrides.update(task_id="another_task", instruction="A different task.")
        node.recording_callback(Bool(True))
        self.assertEqual(json.loads(first_path.read_text())["instruction"], instruction)
        node.recording_callback(Bool(False))
        node.recording_callback(Bool(True))
        second = json.loads((node._episode_path / "episode.json").read_text())
        self.assertEqual(second["task_id"], "another_task")
        self.assertEqual(second["instruction"], "A different task.")
        self.assertNotEqual(first_path.parent, node._episode_path)

    def test_episode_camera_metadata_has_known_values_and_explicit_unknowns(self):
        node = self.recorder.SimRecorder()
        node.recording_callback(Bool(True))
        metadata = json.loads((node._episode_path / "episode.json").read_text())
        self.assertFalse(metadata["camera_configuration"]["runtime_verified"])
        self.assertEqual(set(metadata["cameras"]), {"head", "left_wrist", "right_wrist"})
        for name, camera in metadata["cameras"].items():
            self.assertEqual(camera["topic"], f"/sensors/camera/{name}/image_raw")
            self.assertEqual(camera["resolution"], [640, 480])
            self.assertIsNone(camera["frame"])
            self.assertIsNone(camera["intrinsics"]["camera_matrix"])
            self.assertIsNone(camera["intrinsics"]["distortion_coefficients"])
            self.assertGreater(camera["intrinsics"]["focal_length_mm"], 0)
            self.assertEqual(len(camera["extrinsics"]["translation_m"]), 3)
            self.assertEqual(len(camera["extrinsics"]["quaternion_wxyz"]), 4)

    def test_custom_camera_snapshot_and_episode_identifiers_are_copied(self):
        snapshot = json.loads(Path(self.recorder.__file__).with_name(
            "camera_metadata.json").read_text())
        snapshot["cameras"]["head"]["frame"] = "custom_head_optical"
        path = Path(self.overrides["output_root"]).parent / "custom_cameras.json"
        path.write_text(json.dumps(snapshot))
        self.overrides.update(
            camera_metadata_path=str(path), source="real",
            robot="custom_robot", scene="custom_scene",
        )
        metadata = self.recorder.SimRecorder().episode_metadata()
        self.assertEqual(metadata["source"], "real")
        self.assertEqual(metadata["robot"], "custom_robot")
        self.assertEqual(metadata["scene"], "custom_scene")
        self.assertEqual(metadata["cameras"], snapshot["cameras"])

    def test_camera_snapshot_identifies_the_versioned_scene_and_model(self):
        snapshot = json.loads(Path(self.recorder.__file__).with_name(
            "camera_metadata.json").read_text())
        provenance = snapshot["provenance"]
        for name in ("scene", "model"):
            content = (ROOT / provenance[name + "_path"]).read_bytes()
            self.assertEqual(hashlib.sha256(content).hexdigest(), provenance[name + "_sha256"])
        self.assertEqual(len(provenance["repository_commit"]), 40)

    def test_metadata_errors_prevent_an_unlabelled_bag(self):
        path = Path(self.overrides["output_root"]).parent / "invalid_cameras.json"
        path.write_text("{}")
        for override in ({"instruction": " "}, {"camera_metadata_path": str(path)}):
            with self.subTest(override=override):
                self.overrides.update(override)
                node = self.recorder.SimRecorder()
                node.recording_callback(Bool(True))
                self.assertTrue(node.logger.error.called)
                self.popen.assert_not_called()
                for name in override:
                    del self.overrides[name]

    @unittest.skipUnless(importlib.util.find_spec("pxr"), "Requires OpenUSD (Isaac Sim)")
    def test_camera_snapshot_poses_and_optics_match_the_saved_usd(self):
        from pxr import Usd, UsdGeom

        snapshot = json.loads(Path(self.recorder.__file__).with_name(
            "camera_metadata.json").read_text())
        stage = Usd.Stage.Open(str(ROOT / snapshot["provenance"]["scene_path"]))
        scale = UsdGeom.GetStageMetersPerUnit(stage)
        cache = UsdGeom.XformCache()
        for camera in snapshot["cameras"].values():
            prim = stage.GetPrimAtPath(camera["usd_camera_prim"])
            optics = UsdGeom.Camera(prim)
            self.assertAlmostEqual(camera["intrinsics"]["focal_length_mm"],
                                   optics.GetFocalLengthAttr().Get() * scale * 100)
            reference = stage.GetPrimAtPath(camera["extrinsics"]["reference_prim"])
            matrix, reset = cache.ComputeRelativeTransform(prim, reference)
            self.assertFalse(reset)
            for actual, expected in zip(matrix.ExtractTranslation() * scale,
                                        camera["extrinsics"]["translation_m"]):
                self.assertAlmostEqual(actual, expected)
            quat = matrix.ExtractRotationQuat().GetNormalized()
            actual = [quat.GetReal(), *quat.GetImaginary()]
            expected = camera["extrinsics"]["quaternion_wxyz"]
            self.assertAlmostEqual(abs(sum(a * b for a, b in zip(actual, expected))), 1.0)

    def test_semantic_bridge_topics_reach_controllers_without_changing_ros1_or_units(self):
        arm_module = load_module(
            "test_arm_topics",
            "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/arm_control_node.py",
        )
        hand_module = load_module(
            "test_hand_topics",
            "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/eff_control_node.py",
        )
        bridge = self.ros2.Ros2Publisher()
        arm = arm_module.ArmControlNode()
        hand = hand_module.IsaacEffControlNode()
        self.assertEqual(self.ros1.ROS1_ARM_TOPIC, "/kuavo_arm_traj")
        self.assertEqual(self.ros1.ROS1_HAND_TOPIC, "/control_robot_hand_position")
        self.assertIn("/robot/command/arm_joint_target", arm.subscriptions)
        self.assertIn("/robot/command/hand_target", hand.subscriptions)
        self.assertEqual(hand.subscriptions["/robot/command/hand_target"].qos.reliability,
                         "best_effort")
        self.assertIn(arm_module.INPUT_TOPIC, bridge.publishers)
        self.assertIn(hand_module.HAND_INPUT_TOPIC, bridge.publishers)
        self.assertEqual(len(bridge.publishers), 3)  # No duplicate alias publishers.
        raw = Message()
        raw.name, raw.position = ["zarm_l4_joint"], [-30.0]
        arm.arm_callback(raw)
        converted = arm.pub.publish.call_args.args[0]
        self.assertEqual(converted.name, ["zarm_r4_joint"])
        self.assertAlmostEqual(converted.position[0], -0.5235987755982988)
        self.assertIn("/robot_arm_joint_command", arm.publishers)
        self.assertIn("/left_hand_joint_command", hand.publishers)
        self.assertIn("/right_hand_joint_command", hand.publishers)

    def test_legacy_bridge_and_controller_inputs_can_be_selected_explicitly(self):
        # Legacy names are opt-in ROS2 configuration; ROS1 publishers never change.
        with patch.dict(self.recorder.os.environ, {
            "ROS2_ARM_TOPIC": "/kuavo_arm_traj",
            "ARM_INPUT_TOPIC": "/kuavo_arm_traj",
            "ROS2_HAND_TOPIC": "/control_robot_hand_position",
            "HAND_INPUT_TOPIC": "/control_robot_hand_position",
        }):
            bridge_module = load_module(
                "test_legacy_bridge", "docker/exo_ros_bridge/ros2/bridge/ros2_publisher.py"
            )
            arm_module = load_module(
                "test_legacy_arm",
                "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/arm_control_node.py",
            )
            hand_module = load_module(
                "test_legacy_hand",
                "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/eff_control_node.py",
            )
            bridge = bridge_module.Ros2Publisher()
            self.assertIn(arm_module.INPUT_TOPIC, bridge.publishers)
            self.assertIn(hand_module.HAND_INPUT_TOPIC, bridge.publishers)
            self.assertIn("/kuavo_arm_traj", arm_module.ArmControlNode().subscriptions)
            self.assertIn("/control_robot_hand_position",
                          hand_module.IsaacEffControlNode().subscriptions)

    @unittest.skipUnless(importlib.util.find_spec("pxr"), "Requires OpenUSD (Isaac Sim)")
    def test_saved_isaac_graph_uses_semantic_observation_topics(self):
        from pxr import Sdf

        layer = Sdf.Layer.FindOrOpen(str(
            ROOT / "models/robots/s40_sharpa/assembly/s40_sharpa_two_cameras.usd"))
        root = "/biped_s40_with_sharpa_wave_hands/"
        expected = {
            "Control_ActionGraph/ros2_camera_helper": "/sensors/camera/head/image_raw",
            "Control_ActionGraph/ros2_camera_helper_01": "/sensors/camera/right_wrist/image_raw",
            "Control_ActionGraph/ros2_camera_helper_02": "/sensors/camera/left_wrist/image_raw",
            "Data_collection_ActionGraph/ros2_publish_joint_state": "/robot/state/joint_states",
            "Control_ActionGraph/ros2_subscribe_joint_state": "right_hand_joint_command",
            "Control_ActionGraph/ros2_subscribe_joint_state_01": "left_hand_joint_command",
            "Control_ActionGraph/ros2_subscribe_joint_state_02": "robot_arm_joint_command",
        }
        for node, topic in expected.items():
            attr = layer.GetAttributeAtPath(root + node + ".inputs:topicName")
            self.assertIsNotNone(attr, node)
            self.assertEqual(attr.default, topic)
        clock = layer.GetAttributeAtPath(
            root + "Data_collection_ActionGraph/ros2_publish_clock.inputs:topicName")
        self.assertTrue(clock is None or clock.default in (None, "clock", "/clock"))

    def test_shutdown_stops_an_active_bag(self):
        node = self.recorder.SimRecorder()
        node.recording_callback(Bool(True))
        node.destroy_node()
        self.killpg.assert_called_once_with(12345, signal.SIGINT)
        self.process.wait.assert_called_once_with(timeout=15.0)
        self.assertTrue(node.destroyed)

    def test_main_cleans_up_on_ros_shutdown_and_restores_hangup_handler(self):
        node = self.recorder.SimRecorder()
        node.recording_callback(Bool(True))
        self.modules["rclpy"].spin.side_effect = (
            self.modules["rclpy.executors"].ExternalShutdownException()
        )
        with patch.object(self.recorder, "SimRecorder", return_value=node), \
                patch.object(self.recorder.signal, "signal", return_value="previous") as handler:
            self.recorder.main()
            installed_handler = handler.call_args_list[0].args[1]
            installed_handler(signal.SIGHUP, None)
            self.modules["rclpy"].try_shutdown.assert_called_once()
            handler.assert_called_with(signal.SIGHUP, "previous")
        self.assertTrue(node.destroyed)
        self.killpg.assert_called_once_with(12345, signal.SIGINT)

    def test_timeout_escalates_to_terminate_and_kill(self):
        node = self.recorder.SimRecorder()
        node.recording_callback(Bool(True))
        self.process.wait.side_effect = [
            subprocess.TimeoutExpired("ros2", 15),
            subprocess.TimeoutExpired("ros2", 5),
            -signal.SIGKILL,
        ]
        node.recording_callback(Bool(False))
        self.assertEqual(self.killpg.call_args_list, [
            call(12345, signal.SIGINT), call(12345, signal.SIGTERM),
            call(12345, signal.SIGKILL),
        ])
        self.assertEqual(self.process.wait.call_args_list, [
            call(timeout=15.0), call(timeout=5.0), call(timeout=5.0),
        ])
        self.assertIsNone(node._process)
        self.assertTrue(node.logger.warning.called)

    def test_failed_stop_blocks_duplicate_process_and_can_be_retried(self):
        node = self.recorder.SimRecorder()
        node.recording_callback(Bool(True))
        self.process.wait.side_effect = subprocess.TimeoutExpired("ros2", 5)
        node.recording_callback(Bool(False))
        node.recording_callback(Bool(True))
        self.popen.assert_called_once()
        self.assertIs(node._process, self.process)
        self.assertTrue(node.logger.error.called)
        self.process.wait.side_effect = None
        node.recording_callback(Bool(False))
        self.assertIsNone(node._process)

    def test_unexpected_exit_requires_a_new_false_true_cycle(self):
        node = self.recorder.SimRecorder()
        node.recording_callback(Bool(True))
        self.process.poll.return_value = 1
        node.check_process()
        self.assertIsNone(node._process)
        self.assertTrue(node.logger.error.called)
        node.recording_callback(Bool(True))
        self.popen.assert_called_once()
        node.recording_callback(Bool(False))
        node.recording_callback(Bool(True))
        self.assertEqual(self.popen.call_count, 2)
        self.killpg.assert_not_called()

    def test_start_failure_is_reported_without_repeated_spawning(self):
        self.popen.side_effect = FileNotFoundError("ros2")
        node = self.recorder.SimRecorder()
        node.recording_callback(Bool(True))
        node.recording_callback(Bool(True))
        self.popen.assert_called_once()
        self.assertIsNone(node._process)
        self.assertTrue(node.logger.error.called)

    def test_parameters_configure_topics_output_and_stop_timeout(self):
        self.overrides.update(topics=["/robot/command/arm_joint_target"], stop_timeout_sec=2.0)
        node = self.recorder.SimRecorder()
        node.recording_callback(Bool(True))
        self.assertEqual(self.popen.call_args.args[0][7:], ["/robot/command/arm_joint_target"])
        node.recording_callback(Bool(False))
        self.process.wait.assert_called_once_with(timeout=2.0)

    def test_empty_topics_options_and_invalid_timeouts_are_rejected(self):
        for name, value in (
            ("topics", []), ("topics", ["-a"]), ("topics", ["/bad topic"]),
            ("stop_timeout_sec", 0), ("stop_timeout_sec", float("inf")),
        ):
            with self.subTest(name=name, value=value):
                self.overrides[name] = value
                with self.assertRaises(ValueError):
                    self.recorder.SimRecorder()
                del self.overrides[name]
        self.popen.assert_not_called()

    def test_recording_round_trip_uses_existing_socket_and_dispatch(self):
        receiver = self.ros1.Ros1Receiver()
        publisher = self.ros2.Ros2Publisher()
        socket = receiver.socket
        self.assertEqual(self.modules["rospy"].Subscriber.call_count, 3)
        for enabled in (False, True, False):
            receiver.recording_callback(Bool(enabled))
        payloads = [c.args[0] for c in socket.send_json.call_args_list]
        self.assertEqual(payloads, [
            {"kind": "recording", "enabled": enabled}
            for enabled in (False, True, False)
        ])
        publisher.socket.recv_json.side_effect = payloads
        self.modules["rclpy"].ok.side_effect = [True, True, True, False]
        publisher.receive_loop()
        messages = publisher.recording_publisher.publish.call_args_list
        self.assertEqual([c.args[0].data for c in messages], [False, True, False])
        self.assertEqual(self.modules["zmq"].Context.instance.call_count, 2)

    def test_recording_qos_is_retained_reliable_and_arm_hand_remain_best_effort(self):
        publisher = self.ros2.Ros2Publisher()
        recorder = self.recorder.SimRecorder()
        for qos in (
            publisher.recording_publisher.qos,
            recorder.subscriptions[self.recorder.RECORDING_TOPIC].qos,
        ):
            self.assertEqual(vars(qos), dict(
                reliability="reliable", durability="transient_local",
                history="keep_last", depth=1,
            ))
        for output in (publisher.hand_publisher, publisher.arm_publisher):
            self.assertEqual(output.qos.reliability, "best_effort")
            self.assertEqual(output.qos.depth, 1)

    def test_malformed_recording_state_is_not_coerced_to_true_or_false(self):
        publisher = self.ros2.Ros2Publisher()
        for payload in ({}, {"enabled": "false"}, {"enabled": 0}, {"enabled": None}):
            publisher.publish_recording(payload)
        publisher.recording_publisher.publish.assert_not_called()

    def test_hand_and_arm_values_still_round_trip_unchanged(self):
        receiver = self.ros1.Ros1Receiver()
        publisher = self.ros2.Ros2Publisher()
        msg = Message()
        msg.header.frame_id = "exo"
        msg.header.stamp.secs, msg.header.stamp.nsecs = 12, 345
        msg.left_hand_position = [1, 2, 3, 4, 5, 6]
        msg.right_hand_position = [6, 5, 4, 3, 2, 1]
        receiver.hand_callback(msg)
        publisher.publish_hand(receiver.socket.send_json.call_args.args[0])
        hand = publisher.hand_publisher.publish.call_args.args[0]
        self.assertEqual(hand.left_hand_position, msg.left_hand_position)
        self.assertEqual(hand.right_hand_position, msg.right_hand_position)
        msg.name, msg.position = ["zarm_r4_joint"], [-30.0]
        msg.velocity, msg.effort = [0.2], [1.0]
        receiver.arm_callback(msg)
        publisher.publish_arm(receiver.socket.send_json.call_args.args[0])
        arm = publisher.arm_publisher.publish.call_args.args[0]
        for name in ("name", "position", "velocity", "effort"):
            self.assertEqual(getattr(arm, name), getattr(msg, name))
        self.assertEqual(arm.header.stamp.sec, 12)
        self.assertEqual(arm.header.stamp.nanosec, 345)
        self.assertEqual(arm.header.frame_id, "exo")


    def make_gateway(self):
        self.overrides["bfm_collection_mode"] = True
        module = load_module("test_bfm_gateway",
            "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/bfm_action_gateway.py")
        return module, module.BfmActionGateway()

    def arm_source(self):
        message = Message()
        # Interleave sides and reverse order; array positions are not the contract.
        message.name = [f"zarm_{side}{i}_joint" for i in range(7, 0, -1) for side in "lr"]
        message.position = [i / 10 for i in range(14)]
        message.header.stamp.sec = 1234
        return message

    def hand_source(self, right=None):
        message = Message()
        message.left_hand_position = [6, 5, 4, 3, 2, 1]
        message.right_hand_position = right if right is not None else [0, 100, 42, 44, 61, 60]
        message.header.stamp.sec = 1792639761
        return message

    def test_bfm_is_opt_in_and_normal_controller_rates_are_unchanged(self):
        module = load_module("test_disabled_gateway",
            "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/bfm_action_gateway.py")
        gateway = module.BfmActionGateway()
        gateway.policy_tick()
        self.assertFalse(gateway.enabled)
        self.assertEqual(gateway.publishers, {})
        arm_module = load_module("test_normal_arm",
            "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/arm_control_node.py")
        hand_module = load_module("test_normal_hand",
            "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/eff_control_node.py")
        self.assertIn("/robot_arm_joint_command", arm_module.ArmControlNode().publishers)
        hand = hand_module.IsaacEffControlNode()
        self.assertIn("/robot/command/hand_target", hand.subscriptions)
        self.assertEqual(hand.timer.period, .02)
        self.assertEqual(hand_module.PUBLISH_HZ, 50.)
        self.assertFalse(self.recorder.SimRecorder().episode_metadata()["bfm"]["enabled"])

    def test_bfm_routing_reuses_mapping_and_cannot_bypass_hold_via_output_override(self):
        with patch.dict(os.environ, {"BFM_COLLECTION_MODE": "1", "ARM_OUTPUT_TOPIC": "/bypass"}):
            arm_module = load_module("test_bfm_arm",
                "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/arm_control_node.py")
            hand_module = load_module("test_bfm_hand",
                "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/eff_control_node.py")
        arm, hand = arm_module.ArmControlNode(), hand_module.IsaacEffControlNode()
        self.assertIn("/bfm/source/arm_joint_target_rad", arm.publishers)
        self.assertNotIn("/robot_arm_joint_command", arm.publishers)
        self.assertNotIn("/bypass", arm.publishers)
        self.assertIn("/bfm/command/hand_target", hand.subscriptions)
        self.assertEqual(hand.subscriptions["/bfm/command/hand_target"].qos.reliability,
                         "reliable")
        self.assertNotIn("/robot/command/hand_target", hand.subscriptions)
        _, gateway = self.make_gateway()
        source = self.arm_source()
        raw_positions = list(source.position)
        arm.arm_callback(source)
        mapped = arm.pub.publish.call_args.args[0]
        gateway.arm_callback(mapped)
        gateway.hand_callback(self.hand_source())
        gateway.policy_tick()
        executed = gateway.arm_pub.publish.call_args.args[0]
        action = gateway.action_pub.publish.call_args.args[0]
        by_name = dict(zip(executed.name, executed.position))
        expected = [math.radians(dict(zip(source.name, source.position))[f"zarm_l{i}_joint"])
                    for i in range(1, 8)]
        self.assertEqual(action.right_arm_target, expected)
        self.assertEqual(action.right_arm_target, [by_name[n] for n in action.right_arm_joint_names])
        self.assertEqual(source.position, raw_positions)  # Raw degrees were not rewritten.

    def test_bfm_gateway_honors_configured_raw_hand_input(self):
        with patch.dict(os.environ, {"HAND_INPUT_TOPIC": "/configured/raw_hand"}):
            _, gateway = self.make_gateway()
        self.assertIn("/configured/raw_hand", gateway.subscriptions)

    def test_gateway_waits_for_both_sources_and_only_timer_can_forward(self):
        _, gateway = self.make_gateway()
        gateway.policy_tick()
        gateway.arm_callback(self.arm_source())
        gateway.policy_tick()
        for publisher in (gateway.action_pub, gateway.arm_pub, gateway.hand_pub):
            publisher.publish.assert_not_called()
        self.assertTrue(gateway.logger.warning.called)
        gateway.hand_callback(self.hand_source())
        gateway.action_pub.publish.assert_not_called()
        gateway.policy_tick()
        gateway.action_pub.publish.assert_called_once()

    def test_gateway_canonical_order_and_executed_13d_targets_match_exactly(self):
        _, gateway = self.make_gateway()
        arm, hand = self.arm_source(), self.hand_source()
        gateway.arm_callback(arm)
        gateway.hand_callback(hand)
        gateway.policy_tick()
        action = gateway.action_pub.publish.call_args.args[0]
        self.assertEqual(action.right_arm_joint_names, [f"zarm_r{i}_joint" for i in range(1, 8)])
        positions = dict(zip(arm.name, arm.position))
        self.assertEqual(action.right_arm_target, [positions[n] for n in action.right_arm_joint_names])
        self.assertEqual(len(action.right_arm_target) + len(action.right_hand_target), 13)
        self.assertEqual(action.right_hand_target, [0, 100, 42, 44, 61, 60])
        executed_hand = gateway.hand_pub.publish.call_args.args[0]
        executed_arm = gateway.arm_pub.publish.call_args.args[0]
        self.assertEqual(executed_hand.right_hand_position, action.right_hand_target)
        self.assertEqual(executed_hand.left_hand_position, hand.left_hand_position)
        self.assertEqual(executed_arm.position, arm.position)
        self.assertEqual(executed_arm.header.stamp.sec, 1234)
        self.assertEqual(executed_hand.header.stamp.sec, 1792639761)

    def test_high_rate_sources_change_only_the_next_policy_snapshot(self):
        _, gateway = self.make_gateway()
        gateway.arm_callback(self.arm_source())
        gateway.hand_callback(self.hand_source())
        self.assertEqual(gateway.timer.period, .1)
        gateway.policy_tick()
        first = gateway.hand_pub.publish.call_args.args[0]
        arm = self.arm_source()
        hand = self.hand_source()
        for value in (10, 20, 30, 40):
            arm.position = [value / 100] * 14
            hand.right_hand_position = [value] * 6
            gateway.arm_callback(arm)
            gateway.hand_callback(hand)
        # Mutating source objects after callbacks cannot mutate buffered values.
        arm.position[0] = 999.
        hand.right_hand_position[0] = 99
        for publisher in (gateway.action_pub, gateway.arm_pub, gateway.hand_pub):
            publisher.publish.assert_called_once()
        self.assertEqual(first.right_hand_position, [0, 100, 42, 44, 61, 60])
        gateway.policy_tick()
        action = gateway.action_pub.publish.call_args.args[0]
        self.assertEqual(action.right_arm_target, [.4] * 7)
        self.assertEqual(action.right_hand_target, [40] * 6)
        self.assertEqual(gateway.action_pub.publish.call_count, 2)

    def test_low_level_hand_interpolation_runs_five_times_per_policy_interval(self):
        with patch.dict(os.environ, {"BFM_COLLECTION_MODE": "1"}):
            hand_module = load_module("test_hand_policy_rate",
                "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/eff_control_node.py")
        hand = hand_module.IsaacEffControlNode()
        _, gateway = self.make_gateway()
        gateway.arm_callback(self.arm_source())
        gateway.hand_callback(self.hand_source([80] * 6))
        gateway.policy_tick()
        hand.hand_callback(gateway.hand_pub.publish.call_args.args[0])
        for _ in range(5):
            gateway.hand_callback(self.hand_source([10] * 6))
            hand.timer_callback()
            self.assertEqual(hand.target_right_values, [80] * 6)
        gateway.action_pub.publish.assert_called_once()
        self.assertEqual(hand.pub_right.publish.call_count, 5)
        self.assertEqual(hand.timer.period, .02)

    def test_gateway_rejects_invalid_arm_and_hand_sources(self):
        _, gateway = self.make_gateway()
        gateway.hand_callback(self.hand_source())
        for kind in ("missing", "duplicate", "nan"):
            arm = self.arm_source()
            if kind == "missing":
                arm.name.pop()
                arm.position.pop()
            elif kind == "duplicate":
                arm.name[0] = arm.name[1]
            else:
                arm.position[0] = float("nan")
            gateway.arm_callback(arm)
            gateway.policy_tick()
        gateway.arm_callback(self.arm_source())
        for values in ([], [1] * 5, [101] * 6, [float("nan")] * 6):
            gateway.hand_callback(self.hand_source(values))
            gateway.policy_tick()
        gateway.action_pub.publish.assert_not_called()

    def test_bfm_recorder_blocks_until_measured_play_and_finalizes_before_stop_ack(self):
        self.overrides["bfm_collection_mode"] = True
        node = self.recorder.SimRecorder()
        node.recording_callback(Bool(True))
        self.popen.assert_not_called()
        episode = dict(episode_id="one", asset="orange2.usd", object_prim_path="/World/Task/Fruits/orange2")
        def status(phase, episode=episode):
            node.episode_callback(SimpleNamespace(data=json.dumps(dict(
                boundary_id="stop-one", phase=phase, episode=episode))))
        status("prepared")
        node.recording_callback(Bool(False)); node.recording_callback(Bool(True))
        self.popen.assert_not_called()
        status("playing")
        # No implicit restart from a retained recording=true.
        self.popen.assert_not_called()
        node.recording_callback(Bool(False)); node.recording_callback(Bool(True))
        self.popen.assert_called_once()
        order = []
        self.process.wait.side_effect = lambda **_: order.append("finalized") or 0
        node._ack_pub.publish.side_effect = lambda _: order.append("ack")
        status("stopped", None)
        self.assertEqual(order, ["finalized", "ack"])
        self.assertIsNone(node._process)
        status("playing", dict(episode, episode_id="two"))
        node.recording_callback(Bool(True))
        self.popen.assert_called_once()
        node.recording_callback(Bool(False)); node.recording_callback(Bool(True))
        self.assertEqual(self.popen.call_count, 2)
        metadata = json.loads((node._episode_path / "episode.json").read_text())
        self.assertEqual(metadata["bfm"]["episode"]["episode_id"], "two")

    def test_bfm_recorder_never_acknowledges_a_failed_bag_shutdown(self):
        self.overrides["bfm_collection_mode"] = True
        node = self.recorder.SimRecorder()
        with patch.object(node, "stop_recording", return_value=False):
            node.episode_callback(SimpleNamespace(data=json.dumps(dict(
                phase="stopped", boundary_id="wait", episode=None))))
        node._ack_pub.publish.assert_not_called()
        self.assertIsNone(node._episode_status)

    def test_bfm_metadata_extends_existing_camera_and_timestamp_metadata(self):
        self.overrides.update(bfm_collection_mode=True, task_id="sort_v2",
                              instruction="Sort items with all six Sharpa channels.",
                              state_frame="fixture", reference_prim_path="/World/Fixture")
        node = self.recorder.SimRecorder()
        node.episode_callback(SimpleNamespace(data=json.dumps(dict(phase="playing", episode=dict(
            episode_id="one", asset="lemon1.usd", object_prim_path="/World/Task/Fruits/lemon1")))))
        node.recording_callback(Bool(True))
        metadata = json.loads((node._episode_path / "episode.json").read_text())
        self.assertEqual(metadata["bfm"]["task_state"]["object_prim_path"], "/World/Task/Fruits/lemon1")
        self.assertEqual(metadata["bfm"]["episode"]["episode_id"], "one")
        self.assertEqual(metadata["task_id"], "sort_v2")
        self.assertEqual(metadata["instruction"], self.overrides["instruction"])
        self.assertEqual(metadata["recording"]["storage_id"], "mcap")
        self.assertEqual(metadata["time"], {"synchronization_source": "rosbag_record_timestamp",
                                            "header_stamp": "source_native"})
        snapshot = json.loads(Path(self.recorder.__file__).with_name("camera_metadata.json").read_text())
        self.assertEqual(metadata["cameras"], snapshot["cameras"])
        self.assertEqual(metadata["camera_configuration"], snapshot["provenance"])
        bfm = metadata["bfm"]
        self.assertTrue(bfm["enabled"])
        self.assertEqual((bfm["policy_hz"], bfm["policy_dt"], bfm["action_dim"]), (10, .1, 13))
        self.assertEqual(bfm["action_semantics"]["right_arm"]["unit"], "rad")
        self.assertEqual(bfm["action_semantics"]["right_hand"]["range"], [0, 100])
        self.assertEqual(bfm["action_semantics"]["right_hand"]["value_order"],
                         [f"right_hand_position[{i}]" for i in range(6)])
        self.assertEqual(bfm["task_state"]["frame"], "fixture")
        self.assertEqual(bfm["task_state"]["position_unit"], "m")
        self.assertEqual(bfm["task_state"]["velocity_unit"], "m/s")
        self.assertTrue(bfm["task_state"]["configured"])
        self.assertEqual(bfm["task_state"]["goal_representation"], "oriented_bounding_box")
        self.assertFalse(bfm["task_state"]["runtime_verified"])

    def test_task_state_native_updates_are_independent_of_policy_rate(self):
        self.overrides["bfm_collection_mode"] = True
        module = load_module("test_task_state",
            "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/bfm_task_state.py")
        measured = {
            "right_eef_position": [1., 2., 3.], "object_position": [4., 5., 6.],
            "object_velocity": [.1, .2, .3], "goal_center": [7., 8., 9.],
            "goal_half_extents": [.1, .2, .3], "goal_orientation": [0., 0., 0., 1.],
            "goal_linear_velocity": [0., 0., 0.], "goal_angular_velocity": [0., 0., 0.],
        }
        sampler = Mock(read=Mock(return_value=measured))
        node = module.BfmTaskStatePublisher("stage", sampler_factory=Mock(return_value=sampler))
        for t in (.02, .04, .06, .08, .1):
            node.publish_at(t)
        self.assertEqual(node.publisher.publish.call_count, 5)
        node.publish_at(.1)
        self.assertEqual(node.publisher.publish.call_count, 5)
        message = node.publisher.publish.call_args.args[0]
        self.assertEqual(message.object_velocity, [.1, .2, .3])
        self.assertEqual(message.object_position, [4., 5., 6.])
        self.assertEqual(message.header.frame_id, "world")
        self.assertEqual(message.header.stamp.nanosec, 100000000)

    def test_task_state_is_disabled_by_default_and_rejects_invalid_measurements(self):
        module = load_module("test_task_state_invalid",
            "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/bfm_task_state.py")
        factory = Mock()
        disabled = module.BfmTaskStatePublisher("stage", sampler_factory=factory)
        disabled.publish_at(.02)
        factory.assert_not_called()
        self.assertEqual(disabled.publishers, {})
        self.overrides["bfm_collection_mode"] = True
        with self.assertRaisesRegex(ValueError, "missing prim"):
            module.BfmTaskStatePublisher("stage", sampler_factory=Mock(
                side_effect=ValueError("missing prim")))
        sampler = Mock(read=Mock(return_value=[[float("nan")] * 3] * 4))
        node = module.BfmTaskStatePublisher("stage", sampler_factory=Mock(return_value=sampler))
        with self.assertRaises(ValueError):
            node.publish_at(.02)
        node.publisher.publish.assert_not_called()

    @unittest.skipUnless(importlib.util.find_spec("pxr"), "Requires OpenUSD, not Isaac GUI")
    def test_sampler_uses_measured_velocity_and_converts_reference_frame_and_si_units(self):
        from pxr import Usd, UsdGeom, UsdPhysics

        module = load_module("test_task_sampler",
            "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/bfm_task_state.py")
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, .01)
        UsdGeom.Xform.Define(stage, "/World")
        reference = UsdGeom.Xform.Define(stage, "/World/Reference")
        reference.AddTranslateOp().Set((10., 0., 0.))
        reference.AddRotateZOp().Set(90.)
        for name in ("Eef", "Object", "Container"):
            prim = UsdGeom.Xform.Define(stage, "/World/" + name).GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(prim)
        config = dict(state_frame="fixture", reference_prim_path="/World/Reference",
                      eef_prim_path="/World/Eef", object_prim_path="/World/Object",
                      container_prim_path="/World/Container",
                      goal_local_center=[0., 0., 0.], goal_half_extents=[.1, .05, .02],
                      goal_local_orientation=[0., 0., 0., 1.])
        eef = SimpleNamespace(count=1, get_transforms=lambda: [[10., 100., 0., 0., 0., 0., 1.]])
        obj = SimpleNamespace(count=1, get_transforms=lambda: [[10., 200., 0., 0., 0., 0., 1.]],
                              get_velocities=lambda: [[0., 100., 0., 0., 0., 0.]],
                              get_coms=lambda: [[0., 0., 0., 0., 0., 0., 1.]])
        container = SimpleNamespace(count=1, get_transforms=lambda: [[10., 50., 0., 0., 0., 0., 1.]],
                              get_velocities=lambda: [[0., 0., 0., 0., 0., 0.]],
                              get_coms=lambda: [[0., 0., 0., 0., 0., 0., 1.]])
        simulation = Mock(create_rigid_body_view=Mock(side_effect=[eef, obj, container]))
        values = module.IsaacTaskSampler(stage, config, simulation).read()
        for name, expected in zip(["right_eef_position", "object_position", "object_velocity",
                                   "goal_center"], [[1, 0, 0], [2, 0, 0], [1, 0, 0], [.5, 0, 0]]):
            actual = values[name]
            for a, b in zip(actual, expected):
                self.assertAlmostEqual(a, b)
        for key, value in (("object_prim_path", ""), ("container_prim_path", "/World/Missing"),
                           ("reference_prim_path", "/World/Eef"),
                           ("eef_prim_path", "/World/Reference")):
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                module.IsaacTaskSampler(stage, {**config, key: value}, simulation)
        invalid = Mock(create_rigid_body_view=Mock(return_value=SimpleNamespace(count=0)))
        with self.assertRaises(ValueError):
            module.IsaacTaskSampler(stage, config, invalid).read()

    def test_launcher_adds_gateway_only_in_bfm_mode_and_passes_configuration(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        directory = Path(temporary.name)
        fake_tmux = directory / "tmux"
        fake_tmux.write_text(
            f"#!{sys.executable}\nimport json, os, sys\n"
            "with open(os.environ['TEST_TMUX_LOG'], 'a') as out:\n"
            "    out.write(json.dumps(sys.argv[1:]) + '\\n')\nprint('%1')\n"
        )
        fake_tmux.chmod(0o755)
        for enabled in ("0", "1"):
            log = directory / f"tmux_{enabled}.jsonl"
            # The fake tmux logs commands; it starts no controller or ROS process.
            with patch.object(self.recorder.subprocess, "Popen", self.original_popen):
                subprocess.run(["bash", str(ROOT / "start_isaac_control.sh")], check=True,
                    stdout=subprocess.DEVNULL, env={**os.environ,
                        "PATH": str(directory) + os.pathsep + os.environ["PATH"],
                        "TEST_TMUX_LOG": str(log), "BFM_COLLECTION_MODE": enabled,
                        "BFM_STATE_FRAME": "fixture", "BFM_OBJECT_PRIM_PATH": "/World/Object"})
            calls = [json.loads(line) for line in log.read_text().splitlines()]
            commands = [c[3] for c in calls if c[0] == "send-keys"]
            self.assertEqual(len(commands), 4 if enabled == "1" else 3)
            self.assertEqual(any("bfm_action_gateway_node" in c for c in commands), enabled == "1")
            for command in commands:
                self.assertIn(f"BFM_COLLECTION_MODE={enabled}", command)
                self.assertIn("BFM_STATE_FRAME=fixture", command)
                self.assertNotIn("colcon build", command)
        legacy_log = directory / "legacy.jsonl"
        with patch.object(self.recorder.subprocess, "Popen", self.original_popen):
            result = subprocess.run(["bash", str(ROOT / "start_isaac_control.sh")],
                capture_output=True, text=True, env={**os.environ,
                    "PATH": str(directory) + os.pathsep + os.environ["PATH"],
                    "TEST_TMUX_LOG": str(legacy_log), "BFM_GOAL_PRIM_PATH": "/Old/Goal"})
        self.assertEqual(result.returncode, 1)
        self.assertIn("BFM_CONTAINER_PRIM_PATH", result.stderr)
        self.assertFalse(legacy_log.exists())  # Fail before touching a running session.


if __name__ == "__main__":
    unittest.main()
