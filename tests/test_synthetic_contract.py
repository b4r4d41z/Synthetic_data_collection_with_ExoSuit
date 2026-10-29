"""Focused metadata, simulation cadence and graph contract checks; no Kit claim."""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import test_recording_pipeline as harness

ROOT = harness.ROOT
PACKAGE = ROOT/'exo2isaacsim/src/isaac_eff_control'
sys.path.insert(0, str(PACKAGE))
from isaac_eff_control.recording_provenance import REPOSITORY, recording_code_provenance
from isaac_eff_control.sim_policy_clock import SimPolicyClock


class ProvenanceTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory(); self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.source = self.root/'exo2isaacsim/src/isaac_eff_control/isaac_eff_control/sim_recorder.py'
        self.source.parent.mkdir(parents=True); self.source.write_text('# actual recording code\n')
        self.git('init', '-q')
        self.git('config', 'user.email', 'fixture@example.invalid')
        self.git('config', 'user.name', 'Fixture')
        self.git('remote', 'add', 'origin', REPOSITORY+'.git')
        self.git('add', '.'); self.git('commit', '-qm', 'fixture')

    def git(self, *args):
        return subprocess.check_output(['git', '-C', str(self.root), *args]).decode().strip()

    def test_actual_sha_clean_dirty_staged_binary_untracked_and_determinism(self):
        clean = recording_code_provenance(self.source)
        self.assertEqual(clean['commit_sha'], self.git('rev-parse', 'HEAD'))
        self.assertEqual(clean['repository'], REPOSITORY)
        self.assertIs(clean['dirty'], False); self.assertIsNone(clean['dirty_sha256'])
        self.source.write_text('# changed code\n')
        dirty = recording_code_provenance(self.source)
        self.assertIs(dirty['dirty'], True)
        self.assertEqual(dirty, recording_code_provenance(self.source))
        self.git('add', '.')
        self.assertEqual(dirty, recording_code_provenance(self.source))
        binary = self.root/'new.bin'; binary.write_bytes(b'\0\xff\x10')
        with_new = recording_code_provenance(self.source)
        self.assertNotEqual(dirty['dirty_sha256'], with_new['dirty_sha256'])
        binary.write_bytes(b'\0\xff\x11')
        self.assertNotEqual(with_new['dirty_sha256'], recording_code_provenance(self.source)['dirty_sha256'])
        binary.unlink(); self.source.write_text('# actual recording code\n'); self.git('add', '.')
        self.assertEqual(recording_code_provenance(self.source), clean)

    def test_unknown_git_and_stale_installed_copy_do_not_invent_a_commit(self):
        copied = self.root/'install/isaac_eff_control/sim_recorder.py'
        copied.parent.mkdir(parents=True); copied.write_text('# old installation\n')
        result = recording_code_provenance(copied)
        self.assertEqual(result['status'], 'unavailable'); self.assertIsNone(result['commit_sha'])
        self.assertIn('differs', result['unavailable_reason'])
        shutil_git = self.root/'.git'
        shutil_git.rename(self.root/'git-hidden')
        result = recording_code_provenance(self.source)
        self.assertIsNone(result['commit_sha']); self.assertIsNone(result['dirty'])
        self.assertEqual(result['executing_module_sha256'], hashlib.sha256(self.source.read_bytes()).hexdigest())

    def test_running_source_uses_this_checkout_sha(self):
        source = PACKAGE/'isaac_eff_control/sim_recorder.py'
        result = recording_code_provenance(source)
        expected = subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD']).decode().strip()
        self.assertEqual(result['commit_sha'], expected)
        self.assertEqual(result['status'], 'available')


class ClockTests(unittest.TestCase):
    def test_integer_period_independent_of_wall_speed_and_pause(self):
        for wall_speed in (.03, .3, 1., 4.):
            clock = SimPolicyClock(); decisions = []
            for sim_ns in range(0, 10_000_000_001, 10_000_000):
                unused_wall_seconds = sim_ns / 1e9 / wall_speed
                if clock.advance(sim_ns): decisions.append(sim_ns)
                # Arbitrarily much wall time passes at the same paused stamp.
                self.assertFalse(clock.advance(sim_ns))
            self.assertEqual(decisions, list(range(100_000_000, 10_000_000_001, 100_000_000)))

    def test_physics_quantization_reset_and_missing_clock_samples_no_burst(self):
        clock = SimPolicyClock(); times = []
        for step in range(601):
            ns = round(step/60 * 1e9)
            if clock.advance(ns): times.append(ns)
        self.assertEqual(len(times), 100)
        for a,b in zip(times, times[1:]):
            self.assertLessEqual(abs((b-a)/1e9-.1), 1/60+1e-9)
        self.assertFalse(clock.advance(0))
        self.assertFalse(clock.advance(99_999_999))
        self.assertTrue(clock.advance(100_000_000))
        self.assertTrue(clock.advance(5_000_000_000))
        self.assertFalse(clock.advance(5_000_000_000))
        self.assertFalse(clock.advance(5_000_000_001))
        self.assertTrue(clock.advance(5_100_000_000))


class RecorderContractTests(unittest.TestCase):
    setUp = harness.RecordingPipelineTests.setUp

    def start(self):
        self.overrides['bfm_collection_mode'] = True
        node = self.recorder.SimRecorder()
        node.episode_callback(SimpleNamespace(data=json.dumps(dict(phase='playing', episode=dict(
            episode_id='test', object_prim_path='/World/Task/Fruits/lemon1')))))
        node.recording_callback(harness.Bool(True))
        return node

    def test_results_are_explicit_without_automatic_terminal_or_stopping(self):
        from test_episode_outcome import state
        for reason in ('user_stop', 'interrupted'):
            node = self.start()
            node.state_callback(state())
            self.assertIsNotNone(node._process)  # An inside position never stops recording.
            self.assertTrue(node.stop_recording(reason))
            saved = json.loads((node._episode_path/'episode.json').read_text())
            self.assertIs(saved['success'], True)
            self.assertIs(saved['terminal'], False)
            self.assertEqual(saved['end_reason'], reason)
            events = [json.loads(c.args[0].data) for c in node._episode_pub.publish.call_args_list]
            self.assertEqual([e['event_type'] for e in events],
                             ['recording_started','recording_end_requested','recording_result'])
            self.assertFalse(events[-2]['result_final']); self.assertTrue(events[-1]['result_final'])
            for key in ('terminal','end_reason','success','recording_code','clock_domains'):
                self.assertEqual(events[-1][key], saved[key])
            self.assertEqual(saved['time']['synchronization_source'], 'rosbag_record_timestamp')
            self.assertIn('system receive time', saved['clock_domains']['mcap_log_time'])

    def test_missing_state_and_error_never_become_terminal_failure(self):
        node = self.start()
        self.process.poll.return_value = 1
        node.check_process()
        saved = json.loads((node._episode_path/'episode.json').read_text())
        self.assertIsNone(saved['success']); self.assertFalse(saved['terminal'])
        self.assertEqual(saved['end_reason'], 'error')

    def test_event_context_failure_cannot_block_finalization(self):
        node = self.start()
        node._episode_pub.publish.side_effect = RuntimeError('ROS context closed')
        self.assertTrue(node.stop_recording('interrupted'))
        self.process.wait.assert_called_once()
        saved = json.loads((node._episode_path/'episode.json').read_text())
        self.assertEqual(saved['end_reason'], 'interrupted')
        self.assertIsNone(node._pending_outcome)
        self.assertTrue(node.logger.warning.called)

    def test_gateway_only_clock_callbacks_schedule_with_sim_stamp_and_same_targets(self):
        with patch.dict('os.environ', BFM_COLLECTION_MODE='1'):
            module = harness.load_module('contract_clock_gateway',
                'exo2isaacsim/src/isaac_eff_control/isaac_eff_control/bfm_action_gateway.py')
        with patch.dict('os.environ', BFM_COLLECTION_MODE='1'):
            gateway = module.BfmActionGateway()
        self.assertEqual(gateway.timers, [])
        arm = harness.Message(); arm.name = sorted(module.ARM_JOINTS); arm.position = [0.]*14
        hand = harness.Message(); hand.left_hand_position = hand.right_hand_position = [50.]*6
        gateway.arm_callback(arm); gateway.hand_callback(hand)
        for ns in (0, 50_000_000, 100_000_000, 100_000_000, 200_000_000):
            gateway.subscriptions['/clock'].callback(SimpleNamespace(clock=SimpleNamespace(sec=0,nanosec=ns)))
        self.assertEqual(gateway.action_pub.publish.call_count, 2)
        action = gateway.action_pub.publish.call_args.args[0]
        self.assertEqual(action.header.stamp.nanosec, 200_000_000)
        self.assertEqual(action.left_hand_target, [50.]*6)
        self.assertEqual(sum(len(getattr(action,k)) for k in
            ('left_arm_target','left_hand_target','right_arm_target','right_hand_target')),26)


@unittest.skipUnless(importlib.util.find_spec('pxr'), 'OpenUSD unavailable')
class GraphContractTests(unittest.TestCase):
    def test_only_frames_and_clock_trigger_change_in_session_layer(self):
        from pxr import Usd, UsdGeom
        from isaac_eff_control.collection_graph import CAMERA_FRAMES, configure_collection_graph, GRAPH_ROOT
        stage = Usd.Stage.Open(str(ROOT/'models/stages/stage_sorting_v1.usd'))
        original = stage.GetRootLayer().ExportToString()
        before = {str(a.GetPath()):(a.Get(),tuple(a.GetConnections()))
                  for p in stage.Traverse() for a in p.GetAttributes()}
        configure_collection_graph(stage)
        self.assertEqual(stage.GetRootLayer().ExportToString(), original)
        cameras = json.loads((PACKAGE/'isaac_eff_control/camera_metadata.json').read_text())['cameras']
        self.assertEqual(len(set(CAMERA_FRAMES.values())),3)
        changed = set()
        for p in stage.Traverse():
            for a in p.GetAttributes():
                path = str(a.GetPath())
                if path in before and before[path] != (a.Get(),tuple(a.GetConnections())): changed.add(path)
            if p.GetAttribute('node:type').Get() == 'isaacsim.ros2.bridge.ROS2CameraHelper':
                topic = p.GetAttribute('inputs:topicName').Get()
                name = next(n for n,c in cameras.items() if c['topic']==topic)
                self.assertEqual(p.GetAttribute('inputs:frameId').Get(), CAMERA_FRAMES[name])
                self.assertEqual(cameras[name]['frame'], CAMERA_FRAMES[name])
        self.assertEqual(len(changed),4)  # Three frame strings and one exec connection.
        self.assertTrue(all(p.endswith('.inputs:frameId') or p.endswith('/ros2_publish_clock.inputs:execIn') for p in changed))
        step = stage.GetPrimAtPath(GRAPH_ROOT+'/Data_collection_ActionGraph/bfm_clock_physics_step')
        self.assertEqual(step.GetAttribute('node:type').Get(), 'isaacsim.core.nodes.OnPhysicsStep')
        clock = stage.GetPrimAtPath(GRAPH_ROOT+'/Data_collection_ActionGraph/ros2_publish_clock')
        self.assertEqual(clock.GetAttribute('inputs:execIn').GetConnections(), [step.GetPath().AppendProperty('outputs:step')])
