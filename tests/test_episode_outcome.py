"""OBB math and actual recorder lifecycle with ROS/process stubs; no Isaac claim."""
import json
import math
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import test_recording_pipeline as harness


def state(position=(0., 0., 0.), orientation=(0., 0., 0., 1.)):
    return SimpleNamespace(header=SimpleNamespace(frame_id='world'),
        object_position=list(position), object_velocity=[0.]*3,
        goal_center=[0.]*3, goal_half_extents=[.2, .05, .05],
        goal_orientation=list(orientation), goal_linear_velocity=[0.]*3,
        goal_angular_velocity=[0.]*3)


class EpisodeOutcomeTests(unittest.TestCase):
    setUp = harness.RecordingPipelineTests.setUp

    def start(self, identity='one'):
        self.overrides['bfm_collection_mode'] = True
        node = self.recorder.SimRecorder()
        node.episode_callback(SimpleNamespace(data=json.dumps(dict(phase='playing', episode=dict(
            episode_id=identity, object_prim_path='/World/Task/Fruits/pomegranate')))))
        node.recording_callback(harness.Bool(True))
        self.assertIsNotNone(node._process)
        return node

    def receive(self, node, sample, time=10.):
        with patch.object(self.recorder, 'monotonic', return_value=time):
            node.state_callback(sample)

    def stop(self, node, time=10.1):
        with patch.object(self.recorder, 'monotonic', return_value=time):
            result = node.stop_recording()
        self.assertTrue(result)
        return json.loads((node._episode_path/'episode.json').read_text())

    def test_axis_aligned_inside_outside_and_boundary(self):
        from isaac_eff_control.episode_outcome import final_position_outcome
        for position, expected in (((0., 0., 0.), True), ((.3, 0., 0.), False),
                                   ((.2, .05, .05), True)):
            with self.subTest(position=position):
                self.assertIs(final_position_outcome(state(position))[0], expected)

    def test_rotated_obb_inverse_normalization_and_quaternion_sign(self):
        from isaac_eff_control.episode_outcome import final_position_outcome
        q = (0., 0., math.sqrt(2), math.sqrt(2))  # Non-unit 90 degrees about Z.
        for orientation in (q, tuple(-v for v in q)):
            self.assertTrue(final_position_outcome(state((0., .1, 0.), orientation))[0])
            self.assertFalse(final_position_outcome(state((.1, 0., 0.), orientation))[0])
        sample = state((1., 2.1, 3.), q)
        sample.goal_center = [1., 2., 3.]
        self.assertTrue(final_position_outcome(sample)[0])

    def test_normal_stop_saves_boolean_and_preserves_metadata_and_mcap_bytes(self):
        for position, expected in (((0., 0., 0.), True), ((.3, 0., 0.), False)):
            with self.subTest(success=expected):
                node = self.start()
                path = node._episode_path/'episode.json'
                before = json.loads(path.read_text())
                before['custom_metadata'] = {'keep': [1, 'unchanged']}
                path.write_text(json.dumps(before))
                node._bag_path.mkdir()
                bag = node._bag_path/'example.mcap'
                bag.write_bytes(b'unchanged recorded message bytes')
                self.receive(node, state(position))
                saved = self.stop(node)
                self.assertFalse(saved['terminal'])
                self.assertEqual(saved['end_reason'], 'user_stop')
                saved.pop('terminal'); saved.pop('end_reason')
                self.assertIs(saved.pop('success'), expected)
                audit = saved.pop('success_evaluation')
                self.assertEqual(audit['method'], 'final_object_position_in_goal_obb')
                self.assertFalse(audit['review_required'])
                self.assertEqual(saved, {k:v for k,v in before.items() if k not in ('success', 'terminal', 'end_reason')})
                self.assertEqual(bag.read_bytes(), b'unchanged recorded message bytes')
                self.assertFalse(list(path.parent.glob('.episode-outcome-*')))

    def test_missing_stale_invalid_and_mismatched_states_remain_unknown(self):
        cases = ['missing', 'stale', 'quaternion', 'nan', 'inf', 'velocity', 'extent', 'frame', 'identity']
        for kind in cases:
            with self.subTest(kind=kind):
                node = self.start()
                sample = state()
                if kind == 'quaternion': sample.goal_orientation = [0.]*4
                if kind == 'nan': sample.object_position[0] = float('nan')
                if kind == 'inf': sample.goal_center[0] = float('inf')
                if kind == 'velocity': sample.object_velocity[0] = float('nan')
                if kind == 'extent': sample.goal_half_extents[1] = 0.
                if kind == 'frame': sample.header.frame_id = 'wrong'
                if kind != 'missing': self.receive(node, sample)
                if kind == 'identity': node._episode_status['episode']['episode_id'] = 'other'
                saved = self.stop(node, time=11. if kind == 'stale' else 10.1)
                self.assertIsNone(saved['success'])
                self.assertTrue(saved['success_evaluation']['review_required'])
                self.assertTrue(saved['success_evaluation']['reason'])
                self.assertTrue(node.logger.warning.called)

    def test_invalid_new_packet_cannot_reuse_previous_valid_state(self):
        node = self.start()
        self.receive(node, state())
        sample = state(); sample.goal_orientation[3] = float('inf')
        self.receive(node, sample, time=10.05)
        self.assertIsNone(self.stop(node)['success'])

    def test_shutdown_evaluates_before_wait_and_sidecar_precedes_stop_ack(self):
        import isaac_eff_control.episode_outcome as helper
        node = self.start()
        self.receive(node, state())
        order = []
        replace = helper.os.replace
        def finalize(**kwargs):
            self.assertIsNone(json.loads((node._episode_path/'episode.json').read_text())['success'])
            order.append('bag_finalized')
            return 0
        def atomic_replace(source, target):
            self.assertEqual(Path(source).parent, Path(target).parent)
            order.append('sidecar_replaced')
            return replace(source, target)
        self.process.wait.side_effect = finalize
        node._ack_pub.publish.side_effect = lambda _: order.append('ack')
        with patch.object(self.recorder, 'monotonic', return_value=10.1), \
                patch.object(helper.os, 'replace', side_effect=atomic_replace):
            node.episode_callback(SimpleNamespace(data=json.dumps(dict(
                phase='stopped', boundary_id='stop', episode=None))))
        self.assertEqual(order, ['bag_finalized', 'sidecar_replaced', 'ack'])
        self.killpg.assert_called_with(12345, signal.SIGINT)
        self.assertIs(json.loads((node._episode_path/'episode.json').read_text())['success'], True)
        # Node shutdown uses exactly the same stop/evaluation path.
        self.process.wait.side_effect = None
        other = self.start('shutdown')
        self.receive(other, state((.3, 0., 0.)))
        with patch.object(self.recorder, 'monotonic', return_value=10.1):
            other.destroy_node()
        self.assertIs(json.loads((other._episode_path/'episode.json').read_text())['success'], False)

    def test_atomic_failure_preserves_original_and_blocks_ack_until_retry(self):
        import isaac_eff_control.episode_outcome as helper
        node = self.start()
        self.receive(node, state())
        path = node._episode_path/'episode.json'; original = path.read_bytes()
        with patch.object(helper.os, 'replace', side_effect=OSError('disk failure')), \
                patch.object(self.recorder, 'monotonic', return_value=10.1):
            node.episode_callback(SimpleNamespace(data=json.dumps(dict(
                phase='stopped', boundary_id='stop', episode=None))))
        node._ack_pub.publish.assert_not_called()
        self.assertIsNone(node._process)
        self.assertEqual(path.read_bytes(), original)
        self.assertFalse(list(path.parent.glob('.episode-outcome-*')))
        node.start_recording()
        self.assertEqual(self.popen.call_count, 1)
        # Retry writes the captured pre-wait result, never a new episode's state.
        saved = self.stop(node, time=100.)
        self.assertIs(saved['success'], True)

    def test_sidecar_identity_mismatch_and_abnormal_exit_require_review(self):
        node = self.start()
        self.receive(node, state())
        path = node._episode_path/'episode.json'; data = json.loads(path.read_text())
        data['bfm']['episode']['object_prim_path'] = '/World/Task/Fruits/lemon1'
        path.write_text(json.dumps(data))
        self.assertIsNone(self.stop(node)['success'])
        node = self.start('unexpected')
        self.receive(node, state())
        self.process.poll.return_value = 1
        node.check_process()
        saved = json.loads((node._episode_path/'episode.json').read_text())
        self.assertIsNone(saved['success'])
        self.assertIn('unexpectedly', saved['success_evaluation']['reason'])

    def test_freshness_is_captured_before_escalation_and_forced_stop_is_unknown(self):
        node = self.start()
        self.receive(node, state())
        self.process.wait.side_effect = [subprocess.TimeoutExpired('bag', 15), 0]
        saved = self.stop(node)
        self.assertIsNone(saved['success'])
        self.assertIn('SIGTERM', saved['success_evaluation']['reason'])
        self.assertEqual(self.killpg.call_args_list[-1].args[1], signal.SIGTERM)

    def test_new_recording_cannot_reuse_previous_state_and_schemas_do_not_change(self):
        node = self.start()
        self.receive(node, state())
        self.stop(node)
        node.recording_callback(harness.Bool(False))
        node.recording_callback(harness.Bool(True))
        self.assertIsNone(self.stop(node)['success'])
        for filename in ('BfmAction.msg', 'BfmState.msg'):
            text = (harness.ROOT/'exo2isaacsim/src/dynamic_biped/msg'/filename).read_text()
            fields = [line.split()[-1] for line in text.splitlines() if line and not line.startswith('#')]
            self.assertNotIn('success', fields)
