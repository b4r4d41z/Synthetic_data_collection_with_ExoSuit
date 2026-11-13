"""Actual Gateway/recorder contract and MCAP/CDR roundtrip; ROS nodes are stubbed."""
from copy import deepcopy
from io import BytesIO
import importlib.util
import json
import math
import sys
from types import SimpleNamespace as NS
import unittest

import test_recording_pipeline as harness
sys.path.insert(0, str(harness.ROOT/'exo2isaacsim/src/isaac_eff_control'))
from isaac_eff_control.bfm_contract import collection_metadata, state_defaults
from isaac_eff_control.source_provenance import (
    InputProvenanceIndex, PROVENANCE_TOPIC, canonical_json, trace_sha256,
)


class SourceProvenanceTests(unittest.TestCase):
    setUp = harness.RecordingPipelineTests.setUp
    make_gateway = harness.RecordingPipelineTests.make_gateway
    arm_source = harness.RecordingPipelineTests.arm_source
    hand_source = harness.RecordingPipelineTests.hand_source

    def gateway(self):
        _, gateway = self.make_gateway()
        gateway.arm_callback(self.arm_source()); gateway.hand_callback(self.hand_source())
        return gateway

    def sample(self, gateway, ns=100_000_000):
        gateway.policy_tick(NS(sec=ns//10**9, nanosec=ns%10**9))
        return (gateway.action_pub.publish.call_args.args[0],
                gateway.provenance_pub.publish.call_args.args[0].data)

    def test_exact_normal_inputs_headers_names_and_independent_identities(self):
        gateway = self.gateway(); action, text = self.sample(gateway)
        index = InputProvenanceIndex(gateway.arm_processing); index.add_trace(text)
        record = index.verify(action)
        self.assertEqual(record['producer_session_id'], gateway.producer_session_id)
        self.assertEqual((record['arm']['input_sequence'],record['hand']['input_sequence']), (1,1))
        self.assertEqual(record['arm']['joint_names'],self.arm_source().name)
        self.assertEqual(record['arm']['position_rad'],self.arm_source().position)
        self.assertEqual(record['arm']['header']['stamp_sec'],1234)
        self.assertEqual(record['hand']['header']['stamp_sec'],1792639761)
        self.assertEqual(record['hand']['left_hand_position'],self.hand_source().left_hand_position)
        self.assertEqual(action.source_provenance_version,1)
        self.assertEqual(action.source_provenance_sha256,trace_sha256(text))
        self.assertNotIn('/robot_arm_joint_command',gateway.publishers)
        self.assertNotIn('/bfm/command/hand_target',gateway.publishers)

    def test_new_input_after_snapshot_cannot_change_action_or_evidence(self):
        gateway = self.gateway(); newer = self.arm_source()
        step = math.tau/4096  # AS5600 encoder count, not an acceptance tolerance.
        for i in (0,3,8,12):newer.position[i] += step
        gateway.provenance_pub.publish.side_effect = lambda msg:gateway.arm_callback(newer)
        action, text = self.sample(gateway)
        index = InputProvenanceIndex(gateway.arm_processing);index.add_trace(text)
        self.assertEqual(index.verify(action)['arm']['position_rad'],self.arm_source().position)
        self.assertEqual(gateway.arm.position,newer.position)
        tampered = deepcopy(action);tampered.left_arm_target[0] += step
        with self.assertRaisesRegex(ValueError,'exact selected'):index.verify(tampered)

    def test_identical_values_and_timestamps_still_get_distinct_source_ids(self):
        gateway = self.gateway(); a,first = self.sample(gateway)
        gateway.arm_callback(self.arm_source());gateway.hand_callback(self.hand_source())
        b,second = self.sample(gateway,200_000_000)
        index = InputProvenanceIndex(gateway.arm_processing)
        for text in (second,first):index.add_trace(text)  # Receipt order is irrelevant.
        x,y = index.verify(a),index.verify(b)
        self.assertEqual(x['arm']['header'],y['arm']['header'])
        self.assertEqual(x['arm']['position_rad'],y['arm']['position_rad'])
        self.assertEqual((x['arm']['input_sequence'],y['arm']['input_sequence']), (1,2))

    def test_independent_updates_and_unchanged_snapshots(self):
        gateway = self.gateway(); actions=[];index=InputProvenanceIndex(gateway.arm_processing)
        for kind in (None,'arm','hand',None):
            if kind=='arm':gateway.arm_callback(self.arm_source())
            if kind=='hand':gateway.hand_callback(self.hand_source([7]*6))
            action,text=self.sample(gateway,(len(actions)+1)*100_000_000)
            index.add_trace(text);actions.append(action)
        self.assertEqual([(index.verify(a)['arm']['input_sequence'],index.verify(a)['hand']['input_sequence'])
                          for a in actions], [(1,1),(2,1),(2,2),(2,2)])
        self.assertEqual([a.action_id for a in actions],[1,2,3,4])

    def test_late_recording_has_self_contained_old_input_evidence(self):
        gateway = self.gateway()
        for _ in range(20):gateway.arm_callback(self.arm_source())
        action,text=self.sample(gateway)
        index=InputProvenanceIndex(gateway.arm_processing);index.add_trace(text)
        self.assertEqual(index.verify(action)['arm']['input_sequence'],21)

    def test_stop_play_and_rollback_clear_buffers_not_lifetime_ids(self):
        gateway=self.gateway();first,_=self.sample(gateway)
        for phase in ('stopped','prepared','playing'):
            gateway.episode_callback(NS(data=json.dumps(dict(phase=phase,boundary_id='next'))))
        self.assertIsNone(gateway.arm);self.assertIsNone(gateway.hand)
        gateway.clock_callback(NS(clock=NS(sec=0,nanosec=0)))
        gateway.arm_callback(self.arm_source());gateway.hand_callback(self.hand_source())
        gateway.clock_callback(NS(clock=NS(sec=0,nanosec=66_666_666)))
        second=gateway.action_pub.publish.call_args.args[0]
        gateway.clock_callback(NS(clock=NS(sec=0,nanosec=0)))
        self.assertIsNone(gateway.arm);self.assertIsNone(gateway.hand)
        gateway.arm_callback(self.arm_source());gateway.hand_callback(self.hand_source())
        third,text=self.sample(gateway,200_000_000)
        self.assertEqual([first.action_id,second.action_id,third.action_id],[1,2,3])
        self.assertEqual(third.producer_session_id,first.producer_session_id)
        self.assertEqual(json.loads(text)['arm']['input_sequence'],3)

    def test_fifteen_sim_hz_and_basic_validation_are_unchanged(self):
        gateway=self.gateway()
        for i in range(61):
            ns=i*10**9//60;gateway.clock_callback(NS(clock=NS(sec=ns//10**9,nanosec=ns%10**9)))
        self.assertEqual(gateway.action_pub.publish.call_count,15)
        self.assertEqual(gateway.provenance_pub.publish.call_count,15)
        self.assertEqual(gateway.timers,[])
        invalid=self.arm_source();invalid.position[0]=math.nan;gateway.arm_callback(invalid)
        self.sample(gateway);self.assertEqual(gateway.action_pub.publish.call_count,15)
        self.assertEqual(gateway.input_sequences['arm'],1)

    def test_exact_saturation_is_verifiable_without_mutating_requested_input(self):
        gateway=self.gateway();source=self.arm_source();source.position=[100.]*14
        gateway.arm_callback(source);action,text=self.sample(gateway)
        index=InputProvenanceIndex(gateway.arm_processing);index.add_trace(text);record=index.verify(action)
        self.assertEqual(record['arm']['position_rad'],[100.]*14)
        self.assertEqual(source.position,[100.]*14)
        self.assertEqual(action.left_arm_target,[gateway.arm_limits[n][1] for n in action.left_arm_joint_names])

    def test_missing_duplicate_conflicting_and_corrupt_evidence_fail_explicitly(self):
        gateway=self.gateway();action,text=self.sample(gateway)
        index=InputProvenanceIndex(gateway.arm_processing)
        with self.assertRaisesRegex(ValueError,'Missing'):index.verify(action)
        index.add_trace(text);index.add_trace(text);self.assertEqual(len(index.traces),1)
        conflict=json.loads(text);conflict['decision_stamp']['nanosec']+=1
        with self.assertRaisesRegex(ValueError,'Conflicting Action'):index.add_trace(canonical_json(conflict))
        conflict=json.loads(text);conflict['action_id']=2;conflict['arm']['position_rad'][0]+=math.tau/4096
        with self.assertRaisesRegex(ValueError,'Conflicting source'):index.add_trace(canonical_json(conflict))
        for mutate in (lambda r:r['arm'].__setitem__('bounds_source_sha256','0'*64),
                       lambda r:r['arm'].__setitem__('joint_names',['bad']*14),
                       lambda r:r['hand'].__setitem__('input_sequence',0),
                       lambda r:r.__setitem__('version',2)):
            bad=json.loads(text);mutate(bad)
            with self.assertRaises(ValueError):InputProvenanceIndex(gateway.arm_processing).add_trace(canonical_json(bad))
        with self.assertRaisesRegex(ValueError,'Duplicate JSON'):
            index.add_trace(text.replace('"action_id":1','"action_id":1,"action_id":1',1))
        changed=deepcopy(action);changed.source_provenance_sha256='0'*64
        with self.assertRaisesRegex(ValueError,'digest mismatch'):index.verify(changed)

    def test_policy_or_old_messages_do_not_claim_operator_provenance(self):
        from isaac_eff_control.bfm_policy_bridge import validate_action
        gateway=self.gateway();action,text=self.sample(gateway)
        action.source_provenance_version=0;action.source_provenance_sha256=''
        validate_action(action)  # Same control validation; no invented operator IDs.
        index=InputProvenanceIndex(gateway.arm_processing);index.add_trace(text)
        with self.assertRaisesRegex(ValueError,'No operator provenance'):index.verify(action)

    def test_recorder_and_metadata_require_the_trace_for_operator_collection(self):
        self.overrides['bfm_collection_mode']=True
        node=self.recorder.SimRecorder()
        self.assertIn(PROVENANCE_TOPIC,node.topics)
        metadata=collection_metadata(True,state_defaults())['operator_input_provenance']
        self.assertEqual((metadata['version'],metadata['topic']),(1,PROVENANCE_TOPIC))
        self.overrides['topics']=['/bfm/action']
        with self.assertRaisesRegex(ValueError,'operator-input evidence'):self.recorder.SimRecorder()

    @unittest.skipUnless(importlib.util.find_spec('mcap_ros2'), 'Optional MCAP ROS2 codec unavailable')
    def test_real_mcap_cdr_schema_roundtrip_with_newer_input_recorded_first(self):
        from mcap.writer import CompressionType
        from mcap.reader import make_reader
        from mcap_ros2.writer import Writer
        from mcap_ros2.decoder import DecoderFactory
        gateway=self.gateway();a,ta=self.sample(gateway)
        newer=self.arm_source()
        for i in (0,3,8,12):newer.position[i]+=math.tau/4096
        gateway.arm_callback(newer);b,tb=self.sample(gateway,200_000_000)
        header='\n'+'='*80+'\nMSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n'
        header+='\n'+'='*80+'\nMSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n'
        definition=(harness.ROOT/'exo2isaacsim/src/dynamic_biped/msg/BfmAction.msg').read_text()
        buffer=BytesIO()
        with Writer(buffer,compression=CompressionType.NONE) as writer:
            actions=writer.register_msgdef('dynamic_biped/BfmAction',definition+header)
            strings=writer.register_msgdef('std_msgs/String','string data\n')
            joints=writer.register_msgdef('sensor_msgs/JointState',
                'std_msgs/Header header\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n'+header)
            # Two subscribers can observe B before Action(A); trace may arrive last.
            writer.write_message('/bfm/source/arm_joint_target_rad',joints,newer,log_time=10)
            writer.write_message('/bfm/action',actions,a,log_time=20)
            writer.write_message('/bfm/action',actions,b,log_time=30)
            writer.write_message(PROVENANCE_TOPIC,strings,NS(data=tb),log_time=40)
            writer.write_message(PROVENANCE_TOPIC,strings,NS(data=ta),log_time=50)
        buffer.seek(0)
        decoded=list(make_reader(buffer,decoder_factories=[DecoderFactory()]).iter_decoded_messages())
        index=InputProvenanceIndex(gateway.arm_processing)
        for _,channel,_,message in decoded:
            if channel.topic==PROVENANCE_TOPIC:index.add_trace(message.data)
        recorded=[message for _,channel,_,message in decoded if channel.topic=='/bfm/action']
        self.assertEqual(index.verify(recorded[0])['arm']['position_rad'],self.arm_source().position)
        self.assertEqual(index.verify(recorded[1])['arm']['position_rad'],newer.position)
        self.assertNotEqual(index.verify(recorded[0])['arm']['position_rad'],newer.position)
        self.assertEqual(index.verify(recorded[0])['arm']['input_sequence'],1)
        self.assertEqual(index.verify(recorded[1])['arm']['input_sequence'],2)
        self.assertEqual(sum(x!=y for x,y in zip(
            recorded[0].left_arm_target+recorded[0].right_arm_target,
            recorded[1].left_arm_target+recorded[1].right_arm_target)),4)
        self.assertEqual(recorded[0].left_hand_target,recorded[1].left_hand_target)
        missing=InputProvenanceIndex(gateway.arm_processing);missing.add_trace(tb)
        with self.assertRaisesRegex(ValueError,'Missing'):missing.verify(recorded[0])


if __name__=='__main__':unittest.main()
