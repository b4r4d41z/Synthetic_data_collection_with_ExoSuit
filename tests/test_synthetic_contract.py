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
            node.success_callback(SimpleNamespace(data=json.dumps(dict(
                episode_id='test', object_prim_path='/World/Task/Fruits/lemon1', success=True))))
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
    def test_camera_configuration_and_clock_stay_in_session_layer(self):
        from pxr import Usd, UsdGeom
        from isaac_eff_control.collection_graph import CAMERA_FRAMES, configure_collection_graph, GRAPH_ROOT
        stage = Usd.Stage.Open(str(ROOT/'models/stages/stage_sorting_v1.usd'))
        original = stage.GetRootLayer().ExportToString()
        before = {str(a.GetPath()):(a.Get(),tuple(a.GetConnections()))
                  for p in stage.Traverse() for a in p.GetAttributes()}
        import isaac_eff_control.collection_graph as graph
        with patch.object(graph, 'configure_physics_clock') as clock_setup, sensor_authoring(graph):
            configure_collection_graph(stage)
        clock_setup.assert_called_once_with(stage)
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
        self.assertEqual(len(changed),3)
        self.assertTrue(all(p.endswith('.inputs:frameId') for p in changed))

        # No existing image settings, optics, geometry or connections changed.
        # The only new camera attribute is the requested simulation tick rate.
        added = {str(a.GetPath()) for p in stage.Traverse() for a in p.GetAttributes()
                 if str(a.GetPath()) not in before and a.HasAuthoredValueOpinion()}
        self.assertEqual(len(added), 3)
        self.assertTrue(all(p.endswith('.omni:sensor:tickRate') for p in added))
        graph.validate_camera_publishers(stage)
        for camera, _, _ in graph._camera_bindings(stage).values():
            self.assertEqual(camera.GetAttribute('omni:sensor:tickRate').Get(), 15.)
            self.assertIn('OmniSensorAPI', camera.GetMetadata('apiSchemas').GetAppliedItems())
        # A separate opening of the asset has no transient rate or schema.
        fresh = Usd.Stage.Open(stage.GetRootLayer())
        for camera, _, _ in graph._camera_bindings(fresh).values():
            self.assertFalse(camera.GetAttribute('omni:sensor:tickRate').HasAuthoredValueOpinion())


def sensor_authoring(graph):
    """Host OpenUSD lacks NVIDIA's schema. Stub only schema/rate authoring.

    Real stage composition, mappings and validation still run. In Isaac, use
    the real schema application instead. This is not a renderer/runtime test.
    """
    from contextlib import nullcontext
    from pxr import Sdf, Usd
    if Usd.SchemaRegistry().FindAppliedAPIPrimDefinition('OmniSensorAPI'):
        return nullcontext()
    def author(camera):
        camera.AddAppliedSchema('OmniSensorAPI')
        camera.CreateAttribute('omni:sensor:tickRate', Sdf.ValueTypeNames.Float, custom=False).Set(graph.CAMERA_RATE_HZ)
    return patch.object(graph, '_set_camera_rate', side_effect=author)


@unittest.skipUnless(importlib.util.find_spec('pxr'), 'OpenUSD unavailable')
class CameraRuntimeTests(unittest.TestCase):
    def setUp(self):
        from pxr import Usd
        import isaac_eff_control.collection_graph as graph
        self.graph = graph
        self.stage = Usd.Stage.Open(str(ROOT/'models/stages/stage_sorting_v1.usd'))
        self.real_set_camera_rate = graph._set_camera_rate
        self.authoring = sensor_authoring(graph)
        self.authoring.__enter__()
        self.addCleanup(self.authoring.__exit__, None, None, None)

    def test_three_expected_bindings_and_idempotent_session_configuration(self):
        from pxr import Usd
        root = self.stage.GetRootLayer().ExportToString()
        disk = {layer.realPath: hashlib.sha256(Path(layer.realPath).read_bytes()).hexdigest()
                for layer in self.stage.GetUsedLayers() if layer.realPath}
        self.graph.configure_camera_publishers(self.stage)
        session = self.stage.GetSessionLayer().ExportToString()
        self.graph.configure_camera_publishers(self.stage)
        self.assertEqual(self.stage.GetSessionLayer().ExportToString(), session)
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), root)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.stage.GetRootLayer())
        bindings = self.graph.validate_camera_publishers(self.stage)
        self.assertEqual(set(bindings), {'head', 'left_wrist', 'right_wrist'})
        for name, (camera, creator, helper) in bindings.items():
            self.assertEqual(helper.GetAttribute('inputs:topicName').Get(), f'/sensors/camera/{name}/image_raw')
            self.assertEqual(creator.GetRelationship('inputs:cameraPrim').GetTargets(), [camera.GetPath()])
            self.assertEqual(camera.GetAttribute('omni:sensor:tickRate').Get(), 15.)
        for path, digest in disk.items():
            self.assertEqual(hashlib.sha256(Path(path).read_bytes()).hexdigest(), digest)

    def test_invalid_helpers_camera_and_render_links_fail_before_any_configuration(self):
        from pxr import Sdf, Usd
        for fault in ('missing_helper', 'duplicate', 'wrong_topic', 'missing_camera',
                      'camera_type', 'wrong_camera', 'missing_camera_rel', 'missing_product_link',
                      'wrong_product_node', 'wrong_exec', 'missing_trigger', 'missing_playback', 'resolution',
                      'disabled', 'not_rgb', 'frame_skip'):
            with self.subTest(fault=fault):
                stage = Usd.Stage.Open(self.stage.GetRootLayer())
                bindings = self.graph._camera_bindings(stage)
                camera, creator, helper = bindings['left_wrist']
                with Usd.EditContext(stage, stage.GetSessionLayer()):
                    if fault == 'missing_helper': helper.SetActive(False)
                    if fault == 'duplicate':
                        extra = stage.DefinePrim('/Duplicate', 'OmniGraphNode')
                        extra.CreateAttribute('node:type', Sdf.ValueTypeNames.Token).Set('isaacsim.ros2.bridge.ROS2CameraHelper')
                        extra.CreateAttribute('inputs:topicName', Sdf.ValueTypeNames.String).Set(helper.GetAttribute('inputs:topicName').Get())
                    if fault == 'wrong_topic': helper.GetAttribute('inputs:topicName').Set('/wrong')
                    if fault == 'missing_camera': camera.SetActive(False)
                    if fault == 'camera_type': camera.SetTypeName('Xform')
                    if fault == 'wrong_camera': creator.GetRelationship('inputs:cameraPrim').SetTargets([bindings['head'][0].GetPath()])
                    if fault == 'missing_camera_rel': creator.GetRelationship('inputs:cameraPrim').SetTargets([])
                    if fault == 'missing_product_link': helper.GetAttribute('inputs:renderProductPath').SetConnections([])
                    if fault == 'wrong_product_node': creator.GetAttribute('node:type').Set('Invalid')
                    if fault == 'wrong_exec': helper.GetAttribute('inputs:execIn').SetConnections([])
                    if fault == 'missing_trigger': creator.GetAttribute('inputs:execIn').SetConnections([])
                    if fault == 'missing_playback':
                        step = stage.GetPrimAtPath(creator.GetAttribute('inputs:execIn').GetConnections()[0].GetPrimPath())
                        step.GetAttribute('inputs:execIn').SetConnections([])
                    if fault == 'resolution': creator.GetAttribute('inputs:width').Set(320)
                    if fault == 'disabled': helper.GetAttribute('inputs:enabled').Set(False)
                    if fault == 'not_rgb': helper.GetAttribute('inputs:type').Set('depth')
                    if fault == 'frame_skip': helper.GetAttribute('inputs:frameSkipCount').Set(2)
                before = stage.GetSessionLayer().ExportToString()
                with self.assertRaisesRegex(ValueError, r'\[BFM CAMERA\]'):
                    self.graph.configure_camera_publishers(stage)
                self.assertEqual(stage.GetSessionLayer().ExportToString(), before)

    def test_existing_product_must_point_to_correct_camera_and_resolution(self):
        from pxr import Gf, Usd, UsdRender
        camera, creator, _ = self.graph._camera_bindings(self.stage)['head']
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            product = UsdRender.Product.Define(self.stage, '/Render/TestHead')
            product.CreateCameraRel().SetTargets([camera.GetPath()])
            product.CreateResolutionAttr().Set(Gf.Vec2i(640, 480))
            creator.GetRelationship('inputs:renderProductPrim').SetTargets([product.GetPath()])
        self.graph.configure_camera_publishers(self.stage)
        for fault in ('wrong_camera', 'missing_product', 'wrong_resolution'):
            with self.subTest(fault=fault), Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                product.GetCameraRel().SetTargets([camera.GetPath()])
                product.GetResolutionAttr().Set(Gf.Vec2i(640, 480))
                creator.GetRelationship('inputs:renderProductPrim').SetTargets([product.GetPath()])
                if fault == 'wrong_camera': product.GetCameraRel().SetTargets(['/NoCamera'])
                if fault == 'missing_product': creator.GetRelationship('inputs:renderProductPrim').SetTargets(['/NoProduct'])
                if fault == 'wrong_resolution': product.GetResolutionAttr().Set(Gf.Vec2i(320, 240))
                with self.assertRaisesRegex(ValueError, 'invalid render product'):
                    self.graph.validate_camera_publishers(self.stage)

    def test_inconsistent_or_dynamic_rate_is_rejected(self):
        from pxr import Usd
        self.graph.configure_camera_publishers(self.stage)
        camera = self.graph._camera_bindings(self.stage)['right_wrist'][0]
        rate = camera.GetAttribute('omni:sensor:tickRate')
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            rate.Set(10.)
            with self.assertRaisesRegex(ValueError, 'right_wrist.*tickRate'):
                self.graph.validate_camera_publishers(self.stage)
            rate.Set(15.)
            rate.Set(10., 1.)
            rate.Set(15., 2.)
            with self.assertRaisesRegex(ValueError, 'right_wrist.*tickRate'):
                self.graph.validate_camera_publishers(self.stage)

    def test_real_schema_authoring_uses_supported_api_and_fails_if_unavailable(self):
        from pxr import Sdf, Usd
        from unittest.mock import Mock
        # Exercise the real authoring function with a fake prim/registry only;
        # no claim that NVIDIA's plugin exists in host OpenUSD.
        camera = Mock()
        real = self.real_set_camera_rate
        with patch.object(Usd, 'SchemaRegistry') as registry:
            real(camera)
            camera.ApplyAPI.assert_called_once_with('OmniSensorAPI')
            camera.CreateAttribute.assert_called_once_with('omni:sensor:tickRate', Sdf.ValueTypeNames.Float, custom=False)
            camera.CreateAttribute.return_value.Set.assert_called_once_with(15.)
            registry.return_value.FindAppliedAPIPrimDefinition.return_value = None
            with self.assertRaisesRegex(RuntimeError, 'OmniSensorAPI unavailable'):
                real(camera)
