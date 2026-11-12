"""Real libzmq transport/lifecycle tests with ROS messages/nodes stubbed.

These tests do not claim ROS DDS, Docker or native-crash elimination in production.
"""
import inspect
from pathlib import Path
import queue
import sys
import tempfile
import threading
from time import monotonic, sleep
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import test_recording_pipeline as harness

try:
    import zmq as real_zmq
except ImportError:
    real_zmq = None


class SeverityLogger:
    """Model Jazzy's fixed severity per caller; catch the original alias bug."""
    def __init__(self):
        self.sites, self.messages = {}, []

    def write(self, severity, message, caller):
        site = (caller.f_code, caller.f_lineno)
        if self.sites.setdefault(site, severity) != severity:
            raise ValueError('Logger severity cannot be changed between calls.')
        self.messages.append((severity, message))

    def info(self, message):
        self.write('info', message, inspect.currentframe().f_back)

    def warning(self, message):
        self.write('warning', message, inspect.currentframe().f_back)

    def error(self, message):
        self.write('error', message, inspect.currentframe().f_back)


@unittest.skipUnless(real_zmq, 'Requires the bridge existing dependency pyzmq')
class BridgeStabilityTests(unittest.TestCase):
    def setUp(self):
        self.modules = harness.ros_stubs({})
        self.modules['zmq'] = real_zmq
        module_patch = patch.dict(sys.modules, self.modules)
        module_patch.start(); self.addCleanup(module_patch.stop)
        self.ros1 = harness.load_module('stable_ros1', 'docker/exo_ros_bridge/ros1/bridge/ros1_receiver.py')
        self.ros2 = harness.load_module('stable_ros2', 'docker/exo_ros_bridge/ros2/bridge/ros2_publisher.py')
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.endpoint = 'ipc://' + str(Path(temporary.name)/'bridge.sock')
        self.ros1.ZMQ_ENDPOINT = self.ros2.ZMQ_ENDPOINT = self.endpoint
        self.contexts = []
        self.ownership_errors = []
        self.main_thread = threading.get_ident()
        self.install_ownership_tracking()

    def install_ownership_tracking(self):
        contexts, errors = self.contexts, self.ownership_errors
        class Socket:
            def __init__(self, socket):
                self.socket, self.owner, self.closes = socket, threading.get_ident(), 0

            def __getattr__(self, name):
                method = getattr(self.socket, name)
                def invoke(*args, **kwargs):
                    if threading.get_ident() != self.owner:
                        errors.append(('cross-thread', name))
                    if name == 'close':
                        self.closes += 1
                        if self.closes > 1:
                            errors.append(('double-close', name))
                    return method(*args, **kwargs)
                return invoke
        class Context:
            def __init__(self):
                self.context = real_zmq.Context()
                self.owner, self.sockets, self.terminated = threading.get_ident(), [], False
                contexts.append(self)

            def socket(self, kind):
                if threading.get_ident() != self.owner:
                    errors.append(('cross-thread', 'socket'))
                socket = Socket(self.context.socket(kind))
                self.sockets.append(socket)
                return socket

            def term(self):
                if threading.get_ident() != self.owner or self.terminated:
                    errors.append(('context owner/duplicate term',))
                if any(s.closes != 1 for s in self.sockets):
                    errors.append(('context terminated before socket close',))
                self.context.term()
                self.terminated = True
        facade = SimpleNamespace(**{name:getattr(real_zmq, name) for name in
                                  ('PUSH','PULL','SNDHWM','RCVHWM','IMMEDIATE',
                                   'POLLIN','POLLOUT','NOBLOCK','Again')}, Context=Context)
        self.ros1.zmq = self.ros2.zmq = facade

    def receiver(self):
        receiver = self.ros1.Ros1Receiver()
        self.addCleanup(receiver.close)
        return receiver

    def publisher(self):
        publisher = self.ros2.Ros2Publisher()
        self.addCleanup(publisher.destroy_node)
        return publisher

    def wait_for(self, predicate, timeout=3):
        deadline = monotonic()+timeout
        while monotonic() < deadline:
            if predicate():
                return
            sleep(.005)
        self.fail('Timed out waiting for real ZeroMQ transport')

    def arm_message(self, value=1.):
        msg = harness.Message()
        msg.name, msg.position, msg.velocity, msg.effort = ['joint'], [value], [.2], [.3]
        msg.header.stamp.secs, msg.header.stamp.nsecs = 12, 345
        msg.header.frame_id = 'exo'
        return msg

    def assert_closed(self):
        self.assertFalse(self.ownership_errors)
        for context in self.contexts:
            self.assertNotEqual(context.owner, self.main_thread)
            self.assertTrue(context.terminated)
            self.assertEqual([s.closes for s in context.sockets], [1])

    def test_real_transport_preserves_all_kinds_fields_order_and_has_no_rate_gate(self):
        publisher, receiver = self.publisher(), self.receiver()
        hand = harness.Message()
        hand.left_hand_position, hand.right_hand_position = list(range(6)), list(range(6,12))
        hand.header.stamp.secs, hand.header.stamp.nsecs = 22, 456
        hand.header.frame_id = 'hands'
        for i in range(20):
            receiver.arm_callback(self.arm_message(float(i)))
            self.wait_for(lambda:publisher.arm_publisher.publish.call_count == i+1)
        receiver.hand_callback(hand)
        for enabled in (False, True, False):
            receiver.recording_callback(harness.Bool(enabled))
        self.wait_for(lambda:publisher.recording_publisher.publish.call_count == 3)
        self.assertEqual([c.args[0].position for c in publisher.arm_publisher.publish.call_args_list],
                         [[float(i)] for i in range(20)])
        arm = publisher.arm_publisher.publish.call_args.args[0]
        self.assertEqual((arm.header.stamp.sec, arm.header.stamp.nanosec, arm.header.frame_id), (12,345,'exo'))
        self.assertEqual((arm.name, arm.velocity, arm.effort), (['joint'],[.2],[.3]))
        result = publisher.hand_publisher.publish.call_args.args[0]
        self.assertEqual((result.left_hand_position,result.right_hand_position),
                         (hand.left_hand_position,hand.right_hand_position))
        self.assertEqual((result.header.stamp.sec,result.header.stamp.nanosec,result.header.frame_id), (22,456,'hands'))
        self.assertEqual([c.args[0].data for c in publisher.recording_publisher.publish.call_args_list], [False,True,False])
        receiver.close(); publisher.destroy_node()
        self.assert_closed()

    def test_jazzy_severity_transitions_and_independent_health(self):
        publisher = self.publisher()
        logger = SeverityLogger(); publisher.logger = logger
        with patch.object(self.ros2, 'monotonic', return_value=10.):
            publisher.report_input()  # Both absent.
            publisher.process_payload({'kind':'arm','name':['a'],'position':[1.]})
            publisher.process_payload({'kind':'hand','left_hand_position':[1]*6,'right_hand_position':[2]*6})
            publisher.report_input()  # Both active.
        with patch.object(self.ros2, 'monotonic', return_value=12.):
            publisher.process_payload({'kind':'hand','left_hand_position':[1]*6,'right_hand_position':[2]*6})
            publisher.report_input()  # Arm idle, hand continues.
            self.assertFalse(publisher.input_health['arm'])
            self.assertTrue(publisher.input_health['hand'])
            publisher.process_payload({'kind':'arm','name':['a'],'position':[2.]})
            publisher.report_input()  # Arm recovers.
            count = len(logger.messages)
            publisher.report_input()
            self.assertEqual(len(logger.messages), count)  # No per-frame/steady log spam.
        self.assertTrue(any(level=='warning' and 'arm NO RECENT INPUT' in m for level,m in logger.messages))
        self.assertTrue(any(level=='info' and 'arm ACTIVE' in m for level,m in logger.messages))
        self.assertEqual(publisher.arm_publisher.publish.call_count, 2)

    def test_diagnostic_failure_or_slow_logger_does_not_gate_real_delivery(self):
        publisher, receiver = self.publisher(), self.receiver()
        publisher.logger.warning.side_effect = ValueError('injected logging backend failure')
        with patch('sys.stderr'):
            publisher.report_input()  # Does not propagate to spin.
        entered, release = threading.Event(), threading.Event()
        def slow(message):
            entered.set(); release.wait(2)
        publisher.logger.warning.side_effect = slow
        thread = threading.Thread(target=publisher.report_input)
        thread.start(); self.addCleanup(thread.join); self.addCleanup(release.set)
        self.assertTrue(entered.wait(1))
        receiver.arm_callback(self.arm_message())
        self.wait_for(lambda:publisher.arm_publisher.publish.call_count == 1)
        release.set(); thread.join()

    def test_missing_upstream_idle_shutdown_and_repeated_restart(self):
        for _ in range(8):
            publisher = self.publisher()  # No sender; poll must not block shutdown.
            start = monotonic()
            publisher.destroy_node(); publisher.destroy_node()
            self.assertLess(monotonic()-start, 1.)
            self.assertFalse(publisher.thread.is_alive())
            receiver = self.receiver()  # No bound receiver; nonblocking callback/close.
            for _ in range(100):
                receiver.arm_callback(self.arm_message())
            self.assertLessEqual(receiver.pending.qsize(), 32)
            self.assertTrue(self.modules['rospy'].logwarn_throttle.called)
            start = monotonic()
            receiver.close(); receiver.close()
            self.assertLess(monotonic()-start, 1.)
            self.assertFalse(receiver.thread.is_alive())
        self.assert_closed()

    def test_sender_before_receiver_and_receiver_reconnect(self):
        receiver = self.receiver()
        receiver.arm_callback(self.arm_message(1.))
        publisher = self.publisher()
        self.wait_for(lambda:publisher.arm_publisher.publish.call_count == 1)
        publisher.destroy_node()
        publisher = self.publisher()
        # libzmq reconnect is asynchronous, not an application retry/rewrite.
        receiver.arm_callback(self.arm_message(2.))
        self.wait_for(lambda:publisher.arm_publisher.publish.call_count >= 1)
        self.assertEqual(publisher.arm_publisher.publish.call_args.args[0].position, [2.])
        receiver.close(); publisher.destroy_node()
        self.assert_closed()

    def test_tcp_delivery_and_sender_restart(self):
        # Same transport as Compose, on an ephemeral loopback port.
        with real_zmq.Context() as context:
            with context.socket(real_zmq.PULL) as socket:
                port = socket.bind_to_random_port('tcp://127.0.0.1')
        self.ros1.ZMQ_ENDPOINT = self.ros2.ZMQ_ENDPOINT = f'tcp://127.0.0.1:{port}'
        publisher = self.publisher()
        for i in range(3):
            receiver = self.receiver()
            receiver.arm_callback(self.arm_message(i))
            self.wait_for(lambda:publisher.arm_publisher.publish.call_count == i+1)
            receiver.close()
        publisher.destroy_node()
        self.assert_closed()

    def test_bind_failure_reports_error_and_cleans_resources(self):
        with real_zmq.Context() as context:
            with context.socket(real_zmq.PULL) as socket:
                port = socket.bind_to_random_port('tcp://127.0.0.1')
        self.ros2.ZMQ_ENDPOINT = f'tcp://127.0.0.1:{port}'
        publisher = self.publisher()
        with self.assertRaisesRegex(RuntimeError, 'startup failed'):
            self.publisher()  # Same TCP endpoint already bound; cleanup even if assertion fails.
        publisher.destroy_node()
        self.assert_closed()

    def test_context_failure_is_reported_without_startup_wait_or_leaked_worker(self):
        with patch.object(self.ros2.zmq, 'Context', side_effect=RuntimeError('injected context failure')):
            for constructor in (self.ros1.Ros1Receiver, self.ros2.Ros2Publisher):
                start = monotonic()
                with self.assertRaisesRegex(RuntimeError, 'injected context failure'):
                    constructor()
                self.assertLess(monotonic()-start, 1.)

    def test_malformed_wire_payload_does_not_kill_receiver_or_mask_source_activity(self):
        publisher = self.publisher()
        with real_zmq.Context() as context:
            with context.socket(real_zmq.PUSH) as sender:
                sender.setsockopt(real_zmq.LINGER, 0)
                sender.connect(self.endpoint)
                sender.send(b'not-json')
                self.wait_for(lambda:publisher.decode_errors == 1)
                sender.send_json({'kind':'arm','name':['a'],'position':[]})
                self.wait_for(lambda:publisher.invalid_counts['arm'] == 1)
                sender.send_json({'kind':'arm','name':['a'],'position':[3.]})
                self.wait_for(lambda:publisher.arm_publisher.publish.call_count == 1)
        self.assertEqual(publisher.input_counts['arm'], 2)
        self.assertEqual(publisher.forwarded_counts['arm'], 1)
        self.assertTrue(publisher.input_valid['arm'])
        self.assertIsNone(publisher.receiver_error)

    def test_concurrent_ros1_callbacks_have_one_socket_owner(self):
        publisher, receiver = self.publisher(), self.receiver()
        threads = [threading.Thread(target=receiver.arm_callback, args=(self.arm_message(i),)) for i in range(16)]
        for thread in threads:thread.start()
        for thread in threads:thread.join()
        self.wait_for(lambda:publisher.arm_publisher.publish.call_count == 16)
        self.assertEqual(sorted(c.args[0].position[0] for c in publisher.arm_publisher.publish.call_args_list), list(range(16)))
        receiver.close(); publisher.destroy_node()
        self.assert_closed()

    def test_shutdown_during_active_traffic_keeps_socket_lifecycle_in_owner(self):
        for _ in range(12):
            publisher, receiver = self.publisher(), self.receiver()
            stop = threading.Event()
            def produce():
                while not stop.is_set():
                    receiver.arm_callback(self.arm_message())
                    sleep(.001)
            producer = threading.Thread(target=produce)
            producer.start()
            try:
                self.wait_for(lambda:publisher.arm_publisher.publish.call_count >= 5)
                publisher.destroy_node()  # Sender still active while receive/poll shuts down.
                receiver.close()  # Callbacks still active while sender shuts down.
            finally:
                stop.set(); producer.join()
            self.assertFalse(publisher.thread.is_alive())
            self.assertFalse(receiver.thread.is_alive())
        self.assert_closed()


if __name__ == '__main__':
    unittest.main()
