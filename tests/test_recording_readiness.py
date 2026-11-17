"""Recording barrier integration with ROS/process stubs; PTY output is real.

DDS endpoint/service fixtures exercise checks, not live rosbag writer admission.
"""
import io
import json
import os
from pathlib import Path
import pty
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import test_recording_pipeline as h
sys.path.insert(0, str(h.ROOT/'exo2isaacsim/src/isaac_eff_control'))
from isaac_eff_control.recording_readiness import RecordingGate, StartupSamples, LEASE_SEC
from isaac_eff_control.terminal_status import TerminalStatus, token


def message(event):
    return NS(data=json.dumps(event))


class GateTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.
        self.gate = RecordingGate('gateway',lambda:self.now)
        self.request = dict(version=1,phase='starting',recorder_id='recorder',session_id='bag',
                            challenge='fresh',boundary_id='boundary',episode_id='episode')

    def receive(self,event):
        return self.gate.receive(event,'boundary','episode',True,True)

    def ready(self,reply):
        self.receive(dict(reply,phase='ready',heartbeat=1))

    def test_closed_until_current_session_challenge_answered(self):
        self.assertFalse(self.gate.allowed)
        self.receive(dict(self.request,phase='ready',heartbeat=1))
        self.assertFalse(self.gate.allowed)
        reply=self.receive(self.request)
        self.assertFalse(self.gate.allowed)
        self.ready(reply)
        self.assertTrue(self.gate.allowed)

    def test_old_ready_cannot_enable_new_recording_or_gateway(self):
        old=self.receive(self.request);self.ready(old)
        self.receive(dict(self.request,session_id='new-bag'))
        self.ready(old);self.assertFalse(self.gate.allowed)
        self.gate=RecordingGate('restarted-gateway',lambda:self.now)
        self.receive(self.request);self.ready(old)
        self.assertFalse(self.gate.allowed)

    def test_closed_then_replayed_ready_cannot_reopen(self):
        old=self.receive(self.request);self.ready(old)
        self.receive(dict(self.request,phase='closed'))
        self.ready(old);self.assertFalse(self.gate.allowed)
        new=self.receive(self.request)
        self.assertNotEqual(new['gateway_nonce'],old['gateway_nonce'])
        self.ready(old);self.assertFalse(self.gate.allowed)

    def test_recorder_crash_expires_lease_duplicate_heartbeat_cannot_renew(self):
        reply=self.receive(self.request);self.ready(reply)
        self.now=LEASE_SEC+.01
        self.ready(reply);self.assertFalse(self.gate.allowed)
        self.receive(dict(reply,phase='ready',heartbeat=2))
        self.assertTrue(self.gate.allowed)

    def test_wrong_scope_invalid_version_and_paused_are_rejected(self):
        for change in ({'episode_id':'old'},{'boundary_id':'old'},{'recorder_id':''},{'version':True}):
            self.assertIsNone(self.receive(dict(self.request,**change)))
        self.assertIsNone(self.gate.receive(self.request,'boundary','episode',False,True))
        self.assertFalse(self.gate.allowed)


class RecorderReadinessTests(unittest.TestCase):
    setUp = h.RecordingPipelineTests.setUp

    def start(self,ready=False):
        self.overrides['bfm_collection_mode']=True
        node=self.recorder.SimRecorder()
        node.episode_callback(message(dict(phase='playing',boundary_id='boundary',episode=dict(
            episode_id='episode',object_prim_path='/World/Task/Fruits/lemon1'))))
        node.recording_callback(h.Bool(True))
        if ready:h.complete_startup(node)
        return node

    def statuses(self,node):
        return [c.args[0] for c in node.logger.info.call_args_list if c.args[0].startswith('[REC]')]

    def assert_no_saved(self,node):
        self.assertFalse(any('SAVED' in text for text in self.statuses(node)))

    def test_idle_starting_is_not_recording_and_own_cli_node_is_unique(self):
        node=self.start()
        self.assertEqual(node._monitor.clock.clock_type,'steady_wall')
        self.assertEqual(self.statuses(node),['[REC] WAITING','[REC] STARTING'])
        command=self.popen.call_args.args[0]
        self.assertEqual(command[7:10],['--node-name',node._bag_node_name,'--disable-keyboard-controls'])
        self.assertEqual(command[10:],node.topics)
        self.assertFalse(node._ready_complete)
        node.check_process();self.assertTrue(node._matched)
        self.assert_no_saved(node)

    def test_topic_exists_or_monitor_samples_without_own_bag_subscription_are_insufficient(self):
        node=self.start()
        node.get_subscriptions_info_by_topic=lambda _: [NS(node_name='other-bag',topic_type='fixture',
                                                          node_namespace='/',qos_profile=None)]
        node.check_process()
        self.assertFalse(node._matched)
        self.assertFalse(node._ready_complete)
        self.assertIn('rosbag subscription/QoS /clock',node._bag_subscriptions()[0])

    def test_incompatible_qos_and_type_do_not_pass(self):
        node=self.start()
        with patch('rclpy.qos.qos_check_compatible',return_value=(2,'incompatible')):
            self.assertEqual(len(node._bag_subscriptions()[0]),len(node.topics))
        node.get_publishers_info_by_topic=lambda _: [NS(topic_type='wrong',qos_profile=None)]
        self.assertEqual(len(node._bag_subscriptions()[0]),len(node.topics))

    def test_complete_data_service_and_handshake_enable_only_current_session(self):
        node=self.start(ready=True)
        self.assertEqual(self.statuses(node)[-2:],['[REC] DATA STREAMS OK','[REC] RECORDING'])
        proof=json.loads((node._episode_path/'episode.json').read_text())['recording_readiness']
        self.assertFalse(proof['writer_admission_verified'])
        self.assertEqual(proof['recorder_node'],node._bag_node_name)
        before=self.statuses(node)
        for _ in range(5):node.check_process()
        self.assertEqual(self.statuses(node),before)

    def test_service_not_available_or_paused_blocks_readiness(self):
        for paused in (None,True):
            node=self.start();node.check_process()
            client=node._bag_client
            if paused is None:
                node._bag_future=None;client.service_is_ready.return_value=False
            else:client.call_async.return_value.result.return_value.paused=True
            with self.assertRaises(AssertionError):h.complete_startup(node)
            self.assertFalse(node._ready_complete)
            node.recording_callback(h.Bool(False))

    def test_missing_clock_camera_state_and_each_command_blocks_startup(self):
        node=self.start(ready=True)
        required=list(node._stream_kinds)
        for topic in required:
            with self.subTest(topic=topic):
                node._ready_complete=False
                sample=node._readiness.samples.pop(topic)
                node.check_process()
                self.assertFalse(node._ready_complete)
                self.assertTrue(any(topic in text for text in node._readiness.missing()))
                node._readiness.samples[topic]=sample
        node.check_process();self.assertTrue(node._ready_complete)

    def test_missing_gateway_or_invalid_buffers_are_not_ready(self):
        node=self.start(ready=True)
        for ack in (None,dict(node._gate_ack,inputs_valid=False)):
            node._ready_complete=False;node._gate_ack=ack
            node.check_process();self.assertFalse(node._ready_complete)

    def test_timeout_reports_missing_conditions_and_no_automatic_retry(self):
        node=self.start()
        node.get_subscriptions_info_by_topic=lambda _:[]
        with patch.object(self.recorder,'monotonic',return_value=node._ready_started+31):
            node.check_process()
        self.assertEqual(node._rec_status,'ERROR');self.assertIsNone(node._process)
        self.assertIn('/clock',str(node.logger.error.call_args_list))
        self.assert_no_saved(node)
        for _ in range(3):node.recording_callback(h.Bool(True));node.check_process()
        self.popen.assert_called_once()

    def test_stop_during_startup_and_rapid_toggles_revoke_each_session(self):
        node=self.start()
        old=dict(node._ready_request)
        node.recording_callback(h.Bool(False))
        self.assertEqual(node._rec_status,'ERROR');self.assert_no_saved(node)
        self.assertEqual(json.loads(node._ready_pub.publish.call_args.args[0].data)['phase'],'closed')
        node.recording_callback(h.Bool(True))
        self.assertNotEqual(old['session_id'],node._ready_request['session_id'])
        node.gate_callback(message(dict(old,phase='gated',gateway_id='old',gateway_nonce='old',inputs_valid=True)))
        self.assertIsNone(node._gate_ack)
        h.complete_startup(node)
        node.recording_callback(h.Bool(False))
        self.assertEqual(node._rec_status,'WAITING')
        self.assertEqual(sum('SAVED' in t for t in self.statuses(node)),1)

    def test_successful_finalize_requires_bag_and_sidecar_then_prints_actual_folder_once(self):
        node=self.start(ready=True)
        node.recording_callback(h.Bool(False))
        self.assertEqual(self.statuses(node)[-2:],[f'[REC] SAVED {node._episode_path.name}','[REC] WAITING'])
        node.stop_recording();node.stop_recording()
        self.assertEqual(sum('SAVED' in t for t in self.statuses(node)),1)

    def test_bad_exit_missing_mcap_footer_or_bag_metadata_never_saved(self):
        for failure in ('exit','footer','metadata','empty'):
            node=self.start(ready=True)
            h.finalized_bag(node._bag_path)
            self.process.wait.side_effect=None
            self.process.wait.return_value=1 if failure=='exit' else 0
            if failure=='footer':(node._bag_path/'bag_0.mcap').write_bytes(b'\x89MCAP0\r\nnot finalized')
            if failure=='metadata':(node._bag_path/'metadata.yaml').unlink()
            if failure=='empty':(node._bag_path/'bag_0.mcap').unlink()
            node.recording_callback(h.Bool(False))
            self.assertEqual(node._rec_status,'ERROR');self.assert_no_saved(node)

    def test_failed_episode_persistence_blocks_saved_and_stop_ack(self):
        node=self.start(ready=True)
        with patch.object(self.recorder,'atomic_outcome_update',side_effect=OSError('disk full')):
            node.episode_callback(message(dict(phase='stopped',boundary_id='stop',episode=None)))
        node._ack_pub.publish.assert_not_called();self.assert_no_saved(node)
        self.assertIsNotNone(node._pending_outcome)

    def test_unexpected_exit_and_shutdown_during_startup_never_saved(self):
        node=self.start()
        self.process.poll.return_value=139;node.check_process()
        self.assertIsNone(node._ready_request);self.assertEqual(node._rec_status,'ERROR')
        self.assert_no_saved(node)
        self.process.poll.return_value=None
        node=self.start();node.destroy_node()
        self.assertIsNone(node._process);self.assert_no_saved(node)

    def test_shutdown_service_context_error_cannot_skip_bag_cleanup(self):
        node=self.start();node.check_process()
        node.destroy_client=lambda _: (_ for _ in ()).throw(RuntimeError('ROS context closed'))
        node.destroy_node()
        self.process.wait.assert_called_once()
        self.assertIsNone(node._process);self.assert_no_saved(node)

    def test_pause_stop_and_episode_reset_close_ready_gate_and_finalize(self):
        for phase in ('paused','stopped','prepared','closed'):
            node=self.start(ready=True)
            node.episode_callback(message(dict(phase=phase,boundary_id='stop',episode=None)))
            self.assertIsNone(node._process);self.assertIsNone(node._ready_request)
            if phase=='stopped':node._ack_pub.publish.assert_called_once()

    def test_gateway_restart_requires_new_nonce_and_post_discovery_data(self):
        node=self.start(ready=True);old=dict(node._gate_ack)
        node.gate_callback(message(dict(phase='hello',gateway_id='new-gateway')))
        self.assertFalse(node._ready_complete);self.assertEqual(node._rec_status,'STARTING')
        node.gate_callback(message(old));self.assertIsNone(node._gate_ack)
        self.assertFalse(node._readiness.samples)

    def test_readiness_sidecar_failure_does_not_announce_ready(self):
        node=self.start()
        with patch.object(node,'_persist_readiness',side_effect=OSError('disk full')):
            with self.assertRaises(AssertionError):h.complete_startup(node)
        self.assertFalse(node._ready_complete)
        self.assertNotIn('[REC] DATA STREAMS OK',self.statuses(node))

    def test_clock_rollback_retires_recording_without_new_actions(self):
        node=self.start(ready=True)
        node._observe_startup('/clock',NS(clock=NS(sec=0,nanosec=0)))
        node.check_process()
        self.assertEqual(node._rec_status,'ERROR');self.assertIsNone(node._process)
        self.assertIsNone(node._ready_request)


class GatewayIntegrationTests(unittest.TestCase):
    setUp=h.RecordingPipelineTests.setUp
    arm_source=h.RecordingPipelineTests.arm_source
    hand_source=h.RecordingPipelineTests.hand_source

    def test_no_early_action_no_backlog_and_direct_arm_and_hand_remain_live(self):
        self.overrides['bfm_collection_mode']=True
        module=h.load_module('readiness_gateway','exo2isaacsim/src/isaac_eff_control/isaac_eff_control/bfm_action_gateway.py')
        g=module.BfmActionGateway();g.playing=True
        g.arm_callback(self.arm_source());g.hand_callback(self.hand_source())
        arm=h.load_module('readiness_arm','exo2isaacsim/src/isaac_eff_control/isaac_eff_control/arm_control_node.py').ArmControlNode()
        hand=h.load_module('readiness_hand','exo2isaacsim/src/isaac_eff_control/isaac_eff_control/eff_control_node.py').IsaacEffControlNode()
        for i in range(61):
            arm.arm_callback(self.arm_source());hand.hand_callback(self.hand_source());hand.timer_callback()
            ns=i*10**9//60;g.clock_callback(NS(clock=NS(sec=ns//10**9,nanosec=ns%10**9)))
        g.action_pub.publish.assert_not_called();g.provenance_pub.publish.assert_not_called()
        self.assertEqual(arm.pub.publish.call_count,61)
        self.assertEqual(hand.pub_left.publish.call_count,61)
        h.grant_recording(g)
        for ns in (1_000_000_000,1_000_000_000,1_066_666_666,1_066_666_666):
            g.clock_callback(NS(clock=NS(sec=ns//10**9,nanosec=ns%10**9)))
        self.assertEqual(g.action_pub.publish.call_count,1)
        self.assertEqual(g.action_pub.publish.call_args.args[0].action_id,1)

    def test_real_recorder_gateway_callbacks_complete_nonce_protocol(self):
        self.overrides['bfm_collection_mode']=True
        g=h.load_module('readiness_pair','exo2isaacsim/src/isaac_eff_control/isaac_eff_control/bfm_action_gateway.py').BfmActionGateway()
        status=message(dict(phase='playing',boundary_id='b',episode=dict(
            episode_id='e',object_prim_path='/World/Task/Fruits/lemon1')))
        g.episode_callback(status);g.arm_callback(self.arm_source());g.hand_callback(self.hand_source())
        node=self.recorder.SimRecorder();node.episode_callback(status)
        node._ready_pub.publish.side_effect=g.recorder_ready_callback
        g.gate_pub.publish.side_effect=node.gate_callback
        node.recording_callback(h.Bool(True))
        self.assertIsNotNone(node._gate_ack)
        self.assertFalse(g.recording_gate.allowed)
        h.complete_startup(node,fixture_ack=False)  # Actual request/response callbacks; fixture observation data.
        self.assertTrue(g.recording_gate.allowed)
        g.policy_tick();g.action_pub.publish.assert_called_once()
        node.recording_callback(h.Bool(False));self.assertFalse(g.recording_gate.allowed)


class SampleTests(unittest.TestCase):
    def test_clock_camera_epoch_and_source_native_timestamps_are_not_mixed(self):
        kinds={'clock':'clock','camera':'camera','raw':'command'}
        s=StartupSamples(kinds,{'camera':dict(frame='head',resolution=[640,480])},2,lambda:0)
        for ns in (1_000_000_000,1_100_000_000):
            stamp=NS(sec=ns//10**9,nanosec=ns%10**9)
            s.observe('clock',NS(clock=stamp))
            s.observe('camera',NS(header=NS(stamp=stamp,frame_id='head'),width=640,height=480,encoding='rgb8',data=b'rgb'))
        s.observe('raw',NS(name=['joint'],position=[0.],header=NS(stamp=NS(sec=1790000000,nanosec=0))))
        self.assertEqual(s.missing(),[])
        for sec in (1790000000,):
            s.observe('camera',NS(header=NS(stamp=NS(sec=sec,nanosec=0),frame_id='head'),width=640,height=480,encoding='rgb8',data=b'rgb'))
        self.assertIn('epoch',str(s.missing()))

    def test_stale_invalid_or_nonadvancing_samples_do_not_pass(self):
        now=[0.];s=StartupSamples({'clock':'clock'}, {},2,lambda:now[0])
        for _ in range(3):s.observe('clock',NS(clock=NS(sec=1,nanosec=0)))
        self.assertIn('not advancing',str(s.missing()))
        s.observe('clock',NS(clock=NS(sec=1,nanosec=1)));self.assertEqual(s.missing(),[])
        now[0]=3;self.assertIn('fresh',str(s.missing()))
        s.observe('clock',NS(clock=NS(sec=-1,nanosec=0)))
        self.assertIn('invalid',str(s.missing()))


class TerminalTests(unittest.TestCase):
    def test_real_pty_color_tokens_and_no_color(self):
        master,slave=pty.openpty()
        try:
            with os.fdopen(os.dup(slave),'w') as stream,patch.dict(os.environ,{},clear=True):
                for state,code in (('WAITING',90),('STARTING',34),('ACTIVE',32),('WARNING',33),('ERROR',31)):
                    self.assertEqual(token(state,stream),f'\x1b[{code}m{state}\x1b[0m')
                stream.write(TerminalStatus(stream).render('[INFO] [REC] STARTING')+'\n');stream.flush()
                self.assertIn(b'\x1b[34mSTARTING',os.read(master,1024))
                with patch.dict(os.environ,NO_COLOR=''):
                    self.assertEqual(token('ACTIVE',stream),'ACTIVE')
        finally:os.close(master);os.close(slave)

    def test_redirected_output_deduplicates_routine_preserves_warning_and_traceback(self):
        renderer=TerminalStatus(io.StringIO())
        self.assertEqual(renderer.render('[INFO] [REC] STARTING'),'[REC] STARTING')
        self.assertIsNone(renderer.render('[INFO] [REC] STARTING'))
        self.assertIsNone(renderer.render('[INFO] routine details'))
        for line in ('[WARN] input lost','[ERROR] disk full','Traceback (most recent call last):'):
            self.assertNotIn('\x1b',renderer.render(line))
            self.assertIsNotNone(renderer.render(line))
        self.assertEqual(renderer.render('[INFO] [REC] SAVED record_actual'),'[REC] SAVED record_actual')


if __name__=='__main__':unittest.main()
