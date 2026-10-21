"""Recording relay and process lifecycle tests without ROS or Isaac Sim."""

import importlib.util
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
        self.header = SimpleNamespace(
            stamp=SimpleNamespace(sec=0, nanosec=0, secs=0, nsecs=0),
            frame_id="",
        )


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def ros_stubs(overrides):
    """Provide only the ROS surface needed by the actual production classes."""
    class Node:
        def __init__(self, name):
            self.logger = Mock()
            self.publishers = {}
            self.subscriptions = {}
            self.destroyed = False

        def declare_parameter(self, name, default):
            return SimpleNamespace(value=overrides.get(name, default))

        def create_publisher(self, msg_type, topic, qos):
            publisher = Mock(qos=qos, msg_type=msg_type)
            self.publishers[topic] = publisher
            return publisher

        def create_subscription(self, msg_type, topic, callback, qos):
            subscription = SimpleNamespace(callback=callback, qos=qos)
            self.subscriptions[topic] = subscription
            return subscription

        def create_timer(self, period, callback):
            return SimpleNamespace(period=period, callback=callback)

        def get_logger(self):
            return self.logger

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
    modules["sensor_msgs.msg"].JointState = Message
    modules["dynamic_biped.msg"].RobotHandPosition = Message
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

    def test_state_transitions_are_idempotent_and_each_bag_is_new(self):
        node = self.recorder.SimRecorder()
        node.recording_callback(Bool(False))
        self.popen.assert_not_called()
        node.recording_callback(Bool(True))
        node.recording_callback(Bool(True))
        self.popen.assert_called_once()
        command = self.popen.call_args.args[0]
        self.assertEqual(command[:4], ["ros2", "bag", "record", "--output"])
        self.assertEqual(command[5:], self.recorder.DEFAULT_TOPICS)
        self.assertEqual(self.popen.call_args.kwargs, {"start_new_session": True})
        self.assertTrue(Path(command[4]).name.startswith("record_"))
        self.assertTrue(node.output_root.is_dir())
        node.recording_callback(Bool(False))
        node.recording_callback(Bool(False))
        self.killpg.assert_called_once_with(12345, signal.SIGINT)
        self.process.wait.assert_called_once_with(timeout=15.0)
        node.recording_callback(Bool(True))
        self.assertEqual(self.popen.call_count, 2)
        self.assertNotEqual(command[4], self.popen.call_args.args[0][4])

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
        self.overrides.update(topics=["/kuavo_arm_traj"], stop_timeout_sec=2.0)
        node = self.recorder.SimRecorder()
        node.recording_callback(Bool(True))
        self.assertEqual(self.popen.call_args.args[0][5:], ["/kuavo_arm_traj"])
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


if __name__ == "__main__":
    unittest.main()
