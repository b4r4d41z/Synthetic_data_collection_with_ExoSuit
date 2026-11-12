"""Decoupled V4 host checks; fake ROS is not live DDS/Isaac validation."""
from pathlib import Path
from types import SimpleNamespace as NS
import json
import math
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'exo2isaacsim/src/isaac_eff_control'))
from isaac_eff_control.sim_policy_clock import SimPolicyClock
from isaac_eff_control.bfm_contract import (LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS,
    collection_metadata, state_defaults, arm_target_processing)
from isaac_eff_control.bfm_policy_bridge import validate_action
import test_recording_pipeline as harness


def action(aid=1, stamp=0):
    names = [*LEFT_ARM_JOINTS,*RIGHT_ARM_JOINTS]
    bounds = arm_target_processing()['joint_limits']
    q = [sum(bounds[n])/2 for n in names]
    return NS(producer_session_id='test-session', action_id=aid,
              header=NS(stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9)),
              left_arm_joint_names=list(LEFT_ARM_JOINTS),right_arm_joint_names=list(RIGHT_ARM_JOINTS),
              left_arm_target=q[:7],right_arm_target=q[7:],
              left_hand_target=[10.5]*6,right_hand_target=[20.5]*6)


class ClockTests(unittest.TestCase):
    def test_exact_fifteen_hz_rational_grid_at_300_physics_hz(self):
        c=SimPolicyClock()
        ticks=[i for i in range(3001) if c.advance(i*10**9//300)]
        self.assertEqual(ticks,list(range(20,3001,20)))

    def test_frozen_reset_and_missed_samples_never_burst(self):
        c=SimPolicyClock()
        self.assertFalse(c.advance(0));self.assertTrue(c.advance(10**9))
        for _ in range(10):self.assertFalse(c.advance(10**9))
        self.assertFalse(c.advance(0));self.assertTrue(c.advance(66_666_666))
        self.assertFalse(c.advance(66_666_666))

    def test_contract_explicitly_distinguishes_previous_executor_based_v4(self):
        m=collection_metadata(True,state_defaults())
        self.assertEqual((m['contract_version'],m['policy_hz'],m['action_dim']),(4,15,26))
        self.assertEqual(m['control_architecture'],'decoupled_teleoperation_v1')
        self.assertFalse(m['control']['execution_acknowledgements'])
        self.assertEqual((m['control']['hand_hz'],m['control']['physics_hz']),(50,300))
        self.assertNotIn('execution',m)
        package=ROOT/'exo2isaacsim/src/isaac_eff_control'
        for name in ('bfm_action_executor.py','bfm_executor_config.json'):
            self.assertFalse((package/'isaac_eff_control'/name).exists())
        self.assertIn('CreateTimeStepsPerSecondAttr().Set(300)',
                      (package/'isaac_eff_control/collection_graph.py').read_text())
        self.assertIn('Deprecated',(ROOT/'exo2isaacsim/src/dynamic_biped/msg/BfmExecutionStatus.msg').read_text())


class DirectControlTests(unittest.TestCase):
    setUp = harness.RecordingPipelineTests.setUp
    arm_source = harness.RecordingPipelineTests.arm_source
    hand_source = harness.RecordingPipelineTests.hand_source
    make_gateway = harness.RecordingPipelineTests.make_gateway

    def test_direct_commands_are_full_rate_without_clock_or_gateway(self):
        with patch.dict('os.environ',BFM_COLLECTION_MODE='1'):
            m=harness.load_module('decoupled_arm','exo2isaacsim/src/isaac_eff_control/isaac_eff_control/arm_control_node.py')
        arm=m.ArmControlNode();request=self.arm_source();request.position=[90.]*14
        self.assertEqual(set(arm.subscriptions),{m.INPUT_TOPIC})
        self.assertEqual(arm.source_pub.qos.reliability,'best_effort')
        for _ in range(20):arm.arm_callback(request)
        self.assertEqual(arm.pub.publish.call_count,20)
        self.assertEqual(arm.source_pub.publish.call_count,20)
        raw=arm.source_pub.publish.call_args.args[0];bounded=arm.pub.publish.call_args.args[0]
        self.assertEqual(raw.position,[math.pi/2]*14)
        limits=arm_target_processing()['joint_limits']
        self.assertEqual(dict(zip(bounded.name,bounded.position)),
            {n:min(max(v,limits[n][0]),limits[n][1]) for n,v in zip(raw.name,raw.position)})
        self.assertEqual(request.position,[90.]*14)
        for positions,names in [([float('nan')]*14,request.name),([0.]*13,request.name),
                                 ([0.]*14,['unknown']*14),([0.]*14,[request.name[0]]*14)]:
            request.position,request.name=positions,names;arm.arm_callback(request)
        self.assertEqual(arm.pub.publish.call_count,20)


    def test_passive_samples_latest_targets_and_never_gate_direct_control(self):
        _,g=self.make_gateway();arm=self.arm_source();hand=self.hand_source()
        # No monotonic watchdog: valid last targets may repeat, and their source streams remain recorded.
        for i in range(20):arm.position=[-.01*i]*14;g.arm_callback(arm);g.hand_callback(hand)
        self.assertEqual(set(g.publishers),{'/bfm/action'});g.action_pub.publish.assert_not_called()
        for i in range(301):
            ns=i*10**9//300;g.clock_callback(NS(clock=NS(sec=ns//10**9,nanosec=ns%10**9)))
        actions=[c.args[0] for c in g.action_pub.publish.call_args_list]
        self.assertEqual(len(actions),15);self.assertEqual([a.action_id for a in actions],list(range(1,16)))
        self.assertEqual(len({a.producer_session_id for a in actions}),1)
        self.assertEqual(actions[-1].right_arm_target,[-.19]*7)
        for phase in ['stopped','prepared','playing']:
            g.episode_callback(NS(data=json.dumps({'boundary_id':'next','phase':phase})))
        g.arm_callback(arm);g.hand_callback(hand)
        g.clock_callback(NS(clock=NS(sec=0,nanosec=0)))
        g.clock_callback(NS(clock=NS(sec=0,nanosec=66_666_666)))
        self.assertEqual(g.action_pub.publish.call_args.args[0].action_id,16)
        g.clock_callback(NS(clock=NS(sec=0,nanosec=0)))  # Actual rollback, no lifecycle event.
        self.assertIsNone(g.arm);self.assertIsNone(g.hand)
        g.arm_callback(arm);g.hand_callback(hand)
        g.clock_callback(NS(clock=NS(sec=0,nanosec=66_666_666)))
        self.assertEqual(g.action_pub.publish.call_args.args[0].action_id,17)



    def test_finger_wall_clock_50_hz_preserves_mapping_and_no_simulation_dependency(self):
        with patch.dict('os.environ',BFM_COLLECTION_MODE='1'):
            m=harness.load_module('decoupled_hands','exo2isaacsim/src/isaac_eff_control/isaac_eff_control/eff_control_node.py')
        node=m.IsaacEffControlNode()
        self.assertEqual(node.timer.period,.02);self.assertEqual(node.timer.clock.clock_type,'steady_wall')
        self.assertNotIn('/clock',node.subscriptions);self.assertNotIn('/bfm/episode',node.subscriptions)
        request=self.hand_source();node.hand_callback(request)
        for _ in range(50):node.timer_callback()
        self.assertEqual(node.pub_right.publish.call_count,50)
        self.assertEqual(node.current_right_values,[0,100,42,44,61,60])
        self.assertEqual(node.logical_pub.publish.call_count,1)
        request.right_hand_position=[float('nan')]*6;node.hand_callback(request)
        self.assertEqual(node.target_right_values,[0,100,42,44,61,60])
        self.assertEqual(node.logical_pub.publish.call_count,1)


    def test_policy_only_bridge_routes_directly_without_ack_or_clock(self):
        from isaac_eff_control import bfm_policy_bridge as bridge
        def exercise(node):
            self.assertEqual(set(node.subscriptions),{'/bfm/action'})
            self.assertEqual(set(node.publishers),{'/robot_arm_joint_command','/bfm/command/hand_target'})
            self.assertEqual(node.timers,[])
            for i in range(1,4):node.action(action(i,10**12))  # No local-clock/future deadline.
            self.assertEqual(node.arm.publish.call_count,3)
            self.assertEqual(node.hand.publish.call_args.args[0].left_hand_position,[10]*6)
            node.action(action(3));self.assertEqual(node.arm.publish.call_count,3)
            invalid=action(4);invalid.left_arm_target[0]=float('inf');node.action(invalid)
            self.assertEqual(node.arm.publish.call_count,3)
            node.count_publishers=lambda topic:2
            with self.assertRaisesRegex(RuntimeError,'Competing'):node.action(action(5))
        self.modules['rclpy'].spin.side_effect=exercise
        bridge.main()


    def test_bridge_diagnostics_are_per_stream_and_do_not_gate_messages(self):
        bridge=self.ros2.Ros2Publisher()
        bridge.input_counts={'arm':5,'hand':7402,'recording':1}
        bridge.input_times={'arm':0.,'hand':100.}
        with patch.object(self.ros2,'monotonic',return_value=100.):bridge.report_input()
        self.assertFalse(bridge.input_health['arm']);self.assertTrue(bridge.input_health['hand'])
        self.assertIn('/kuavo_arm_traj',bridge.logger.warning.call_args.args[0])
        self.assertEqual(bridge.input_counts['arm'],5)


class PolicyValidationTests(unittest.TestCase):
    def test_basic_validation_and_no_additional_trajectory_limits(self):
        message=action();names,q=validate_action(message)
        self.assertEqual(len(names),14);self.assertEqual(q,message.left_arm_target+message.right_arm_target)
        for mutate in [lambda m:m.left_arm_joint_names.reverse(),lambda m:m.left_hand_target.append(0),
                       lambda m:setattr(m,'action_id',0),lambda m:setattr(m,'producer_session_id',''),
                       lambda m:m.right_hand_target.__setitem__(0,101),
                       lambda m:m.right_arm_target.__setitem__(0,100)]:
            message=action();mutate(message)
            with self.assertRaises(ValueError):validate_action(message)

if __name__=='__main__':unittest.main()
