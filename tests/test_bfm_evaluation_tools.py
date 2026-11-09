"""Diagnostic-tool tests with fake ROS/subprocess I/O, not Isaac validation."""
import importlib.util
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools/evaluation'))
sys.path.insert(0, str(ROOT/'exo2isaacsim/src/isaac_eff_control'))
from common import COMMANDS, JOINT_STATE, CLOCK, Lifecycle, recording, required_topics, schedule, validate_command
from replay import ReplayClock, load
from record import Capture, Rollout, keyboard


def status(phase, boundary='one'):
    return json.dumps(dict(schema_version=1, boundary_id=boundary, phase=phase, episode={'episode_id':boundary}))


def bag(path):
    """Metadata/file fixture only; no claim of real CDR or MCAP serialization."""
    (path/'bag').mkdir(exist_ok=True)
    (path/'bag/bag_0.mcap').write_bytes(b'\x89MCAP0\r\nfixture\x89MCAP0\r\n')
    topics = [dict(topic_metadata=dict(name=n, type=JOINT_STATE if n in COMMANDS else CLOCK,
                                      serialization_format='cdr'), message_count=2)
              for n in (*COMMANDS, '/clock')]
    info = dict(storage_identifier='mcap', relative_file_paths=['bag_0.mcap'], topics_with_message_count=topics)
    (path/'bag/metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=info)))


def command(topic=COMMANDS[0]):
    if topic == COMMANDS[0]:
        from isaac_eff_control.bfm_contract import ARM_JOINTS
        names = sorted(ARM_JOINTS)
    else:
        side = 'left' if topic == COMMANDS[1] else 'right'
        names = [side+'_'+n for n in ('thumb_MCP_FE','thumb_IP','thumb_CMC_FE','thumb_CMC_AA')]
        names += [f'{side}_{f}_{j}' for f in ('index','middle','ring','pinky') for j in ('MCP_FE','PIP','DIP')]
    return SimpleNamespace(name=names, position=[0.]*len(names), velocity=[], effort=[])


class EvaluationTests(unittest.TestCase):
    def test_recording_detection_and_missing_malformed_data(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            for sidecar in ('episode.json','capture.json'):
                (path/sidecar).write_text('{}'); bag(path)
                root, _, topics = recording(path)
                self.assertEqual(root, path); required_topics(topics)
            (path/'bag/metadata.yaml').write_text('not: [yaml')
            with self.assertRaisesRegex(ValueError, 'Invalid recording'):recording(path)
            bag(path); (path/'bag/bag_0.mcap').unlink()
            with self.assertRaises(ValueError):recording(path)
            bag(path); (path/'capture.json').write_text('[]')
            with self.assertRaises(ValueError):recording(path)

    def test_topic_types_counts_and_commands_fail_closed(self):
        topics = {n:(JOINT_STATE,1) for n in COMMANDS}; topics['/clock'] = (CLOCK,2)
        required_topics(topics)
        for bad in ({}, {**topics, COMMANDS[0]:('wrong/Type',2)}, {**topics, COMMANDS[1]:(JOINT_STATE,0)}):
            with self.assertRaises(ValueError):required_topics(bad)
        for topic in COMMANDS:
            validate_command(topic, command(topic))
            for fault in ('missing','duplicate','dimension','nan','inf','degrees','velocity'):
                msg = command(topic)
                if fault == 'missing':msg.name[0] = 'unrecognized'
                if fault == 'duplicate':msg.name[0] = msg.name[1]
                if fault == 'dimension':msg.position.pop()
                if fault in ('nan','inf'):msg.position[0] = float(fault)
                if fault == 'degrees':msg.position[0] = 90.
                if fault == 'velocity':msg.velocity = [1.]
                with self.subTest(topic=topic,fault=fault), self.assertRaises(ValueError):validate_command(topic,msg)

    def test_receive_time_join_order_and_simulation_pacing(self):
        events = schedule([(15,'a',1),(15,'b',2),(35,'c',3),(5,'d',4)],
                          [(10,1_000_000_000),(20,1_100_000_000),(30,1_200_000_000)])
        self.assertEqual(events, [(0,'d',4),(0,'a',1),(0,'b',2),(200_000_000,'c',3)])
        for wall_speed in (0.1,10):  # Wall speed is deliberately not a scheduler input.
            cursor = ReplayClock(events)
            self.assertEqual(len(cursor.advance(8_000_000_000,True)),3)
            self.assertEqual(cursor.advance(8_000_000_000,True),[])
            self.assertEqual(cursor.advance(8_000_000_000,False),[])
            self.assertEqual(cursor.advance(8_100_000_000,True),[])
            self.assertEqual(cursor.advance(8_200_000_000,True),[events[-1]])
        with self.assertRaisesRegex(ValueError,'rollback'):schedule([],[(1,10),(2,5)])
        with self.assertRaises(ValueError):schedule([],[(1,10),(2,10)])
        cursor = ReplayClock(events); cursor.advance(100,True)
        with self.assertRaisesRegex(ValueError,'reset'):cursor.advance(0,True)
        cursor = ReplayClock(events); cursor.advance(0,True)
        with self.assertRaisesRegex(ValueError,'deadline'):cursor.advance(1_000_000_000,True)

    def test_lifecycle_ignores_recorder_events_and_pause_is_not_stop(self):
        life = Lifecycle()
        self.assertFalse(life.receive(json.dumps(dict(event_type='recording_result', phase='stopped'))))
        self.assertIsNone(life.phase)
        capture = Mock(process=None)
        control = Rollout(capture,life)
        control.episode(status('stopped')); control.key(' '); capture.start.assert_not_called()
        control.episode(status('playing')); control.time(0); control.time(10)
        control.key(' '); capture.start.assert_not_called()  # Requires a fresh physics sample.
        control.time(20); capture.start.assert_called_once()
        capture.process = object(); capture.stop.reset_mock()
        control.episode(status('paused')); control.time(20)
        capture.stop.assert_not_called()
        control.episode(status('playing')); control.time(30)
        control.episode(status('stopped','two'))
        capture.stop.assert_called_once_with('simulation_stop')
        capture.stop.reset_mock(); control.episode(status('playing','two')); control.time(40); control.time(0)
        capture.stop.assert_called_once_with('clock_reset')

    def test_keyboard_toggle_quit_and_stale_start_cancellation(self):
        capture = Mock(process=None)
        control = Rollout(capture,Lifecycle())
        control.episode(status('playing')); control.time(0); control.time(10)
        control.key(' '); control.episode(status('stopped')); control.time(20)
        capture.start.assert_not_called()
        control.episode(status('playing','next')); control.time(30); control.key(' '); control.time(40)
        capture.start.assert_called_once()
        capture.process = object(); control.key(' ')
        capture.stop.assert_called_with('keyboard')
        self.assertFalse(control.key('q')); capture.stop.assert_called_with('keyboard_quit')

    def test_capture_flush_metadata_and_replay_input_roundtrip_with_fake_rosbag(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture = Capture(tmp,lambda:dict(simulation_ns=12),{'checkpoint':None})
            child = Mock(pid=123, returncode=None); child.poll.side_effect = lambda:child.returncode
            def finish(timeout):
                bag(capture.path); child.returncode = 0
            child.wait.side_effect = finish
            with patch('isaac_eff_control.recording_provenance.recording_code_provenance',return_value={'status':'test_stub'}), patch('record.subprocess.Popen',return_value=child) as spawn, patch('record.os.killpg') as kill:
                capture.start(); capture.stop('keyboard')
            self.assertIn('--all',spawn.call_args.args[0]); self.assertIn('--include-hidden-topics',spawn.call_args.args[0])
            self.assertIn('--exclude-regex',spawn.call_args.args[0])
            self.assertIn('^/bfm/recorder_control$',spawn.call_args.args[0])
            kill.assert_called_once_with(123,signal.SIGINT)
            data = json.loads((capture.path/'capture.json').read_text())
            self.assertEqual(data['status'],'complete'); self.assertTrue(data['replayable'])
            self.assertEqual(data['stop_reason'],'keyboard'); self.assertIsNotNone(data['end_wall_utc'])
            _, _, topics = recording(capture.path); required_topics(topics)
            stream = [('/clock',SimpleNamespace(clock=SimpleNamespace(sec=0,nanosec=0)),0)]
            stream += [(n,command(n),1) for n in COMMANDS]
            stream += [('/clock',SimpleNamespace(clock=SimpleNamespace(sec=0,nanosec=100_000_000)),2)]
            reader = Mock(); reader.get_all_topics_and_types.return_value = [SimpleNamespace(name=n,type=t[0]) for n,t in topics.items()]
            reader.has_next.side_effect = [True]*len(stream)+[False]
            reader.read_next.side_effect = stream
            rosbag = SimpleNamespace(SequentialReader=lambda:reader, StorageOptions=Mock(), ConverterOptions=Mock(), StorageFilter=Mock())
            modules = {'rosbag2_py':rosbag, 'rclpy.serialization':SimpleNamespace(deserialize_message=lambda data,typ:data),
                       'rosidl_runtime_py.utilities':SimpleNamespace(get_message=lambda name:object)}
            with patch.dict(sys.modules,modules):
                _, _, events = load(capture.path)
            self.assertEqual([n for _,n,_ in events],list(COMMANDS))
            self.assertIsNone(capture.process)

    def test_escalated_shutdown_reports_potentially_incomplete_capture(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture = Capture(tmp,lambda:{},{})
            child = Mock(pid=123,returncode=None); child.poll.side_effect=lambda:child.returncode
            def wait(timeout):
                if timeout == 20:raise subprocess.TimeoutExpired('rosbag',timeout)
                bag(capture.path); child.returncode = -signal.SIGTERM
            child.wait.side_effect = wait
            with patch('isaac_eff_control.recording_provenance.recording_code_provenance',return_value={'status':'test_stub'}), patch('record.subprocess.Popen',return_value=child), patch('record.os.killpg') as kill:
                capture.start()
                with self.assertRaisesRegex(RuntimeError,'incomplete'):capture.stop('signal')
            self.assertEqual([c.args[1] for c in kill.call_args_list],[signal.SIGINT,signal.SIGTERM])
            data=json.loads((capture.path/'capture.json').read_text())
            self.assertEqual(data['status'],'failed'); self.assertTrue(data['shutdown_escalated'])
            self.assertIsNone(capture.process)

    def test_terminal_restored_on_keyboard_interrupt(self):
        stream=Mock();stream.fileno.return_value=0
        with patch('record.termios.tcgetattr',return_value=['original']), patch('record.tty.setcbreak'), patch('record.termios.tcsetattr') as restore:
            with self.assertRaises(KeyboardInterrupt):
                with keyboard(stream):raise KeyboardInterrupt
            restore.assert_called_once()
        stream.isatty.return_value=False
        with self.assertRaises(ValueError):
            with keyboard(stream):pass

    def test_launchers_reuse_environment_and_start_no_competing_controller(self):
        for script in ('replay_bfm_bag.sh','record_bfm_rollout.sh'):
            source=(ROOT/script).read_text()
            self.assertIn('tools/collection/environment.bash',source)
            self.assertNotIn('ros2 run',source)
            self.assertTrue((ROOT/script).stat().st_mode & 0o111)

    def test_replay_runtime_pause_stop_conflicts_and_clock_reset(self):
        import replay
        for fault in ('stop','reset','conflict','complete','ack_timeout'):
            with self.subTest(fault=fault):
                callbacks, outputs = {}, {}
                node = Mock()
                node.get_name.return_value='test_replay'; node.get_namespace.return_value='/'
                node.count_publishers.return_value=1
                node.get_publishers_info_by_topic.side_effect=lambda topic:[SimpleNamespace(
                    node_name='competitor' if fault == 'conflict' else 'test_replay',
                    node_namespace='/', topic_type=JOINT_STATE)]
                def publisher(typ,topic,qos):
                    outputs[topic]=Mock();outputs[topic].get_subscription_count.return_value=1
                    outputs[topic].wait_for_all_acked.return_value = fault != 'ack_timeout'
                    return outputs[topic]
                node.create_publisher.side_effect=publisher
                node.create_subscription.side_effect=lambda typ,topic,cb,qos:callbacks.update({topic:cb})
                def event(phase):return lambda:callbacks['/bfm/episode'](SimpleNamespace(data=status(phase)))
                def clock(ns):return lambda:callbacks['/clock'](SimpleNamespace(clock=SimpleNamespace(sec=0,nanosec=ns)))
                steps=iter([event('playing'), clock(0), clock(10), event('paused'), clock(10),
                            event('playing'), clock(100_000_010),
                            clock(0) if fault=='reset' else event('stopped')])
                ros=SimpleNamespace(init=Mock(),create_node=lambda name:node,ok=lambda:True,
                                    spin_once=lambda *a,**k:next(steps)(),shutdown=Mock())
                qos=SimpleNamespace(QoSProfile=lambda **k:SimpleNamespace(**k),
                    ReliabilityPolicy=SimpleNamespace(RELIABLE=1,BEST_EFFORT=2),
                    DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1))
                modules={'rclpy':ros,'rclpy.qos':qos,'rclpy.duration':SimpleNamespace(Duration=lambda **k:k),'sensor_msgs.msg':SimpleNamespace(JointState=object),
                         'rosgraph_msgs.msg':SimpleNamespace(Clock=object),'std_msgs.msg':SimpleNamespace(String=object)}
                deadlines = (0,100_000_000) if fault in ('complete','ack_timeout') else (0,100_000_000,300_000_000)
                events=[(ns,COMMANDS[0],object()) for ns in deadlines]
                with patch.dict(sys.modules,modules), patch.object(replay,'load',return_value=(Path('/bag'),{},events)), \
                     patch.object(sys,'argv',['replay','/bag']), patch.object(replay.time,'monotonic',side_effect=[0]+[3]*20):
                    if fault in ('stop','complete'):replay.main()
                    else:
                        with self.assertRaisesRegex(ValueError,{'reset':'reset','conflict':'Conflicting','ack_timeout':'acknowledgement'}[fault]):replay.main()
                self.assertEqual(outputs[COMMANDS[0]].publish.call_count,0 if fault=='conflict' else 2)
                node.destroy_node.assert_called_once();ros.shutdown.assert_called_once()

    def test_incomplete_and_lfs_payloads_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp);(path/'episode.json').write_text('{}');bag(path)
            part=path/'bag/bag_0.mcap'
            for content in (b'version https://git-lfs.github.com/spec/v1\n', b'\x89MCAP0\r\nincomplete'):
                part.write_bytes(content)
                with self.assertRaises(ValueError):recording(path)

    def test_start_failure_leaves_explicit_failed_metadata_and_no_process(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture=Capture(tmp,lambda:{},{})
            with patch('isaac_eff_control.recording_provenance.recording_code_provenance',return_value={'status':'test_stub'}), \
                    patch('record.subprocess.Popen',side_effect=OSError('ros2 unavailable')):
                with self.assertRaises(OSError):capture.start()
            self.assertIsNone(capture.process)
            data=json.loads((capture.path/'capture.json').read_text())
            self.assertEqual(data['status'],'failed');self.assertEqual(data['stop_reason'],'start_error')
