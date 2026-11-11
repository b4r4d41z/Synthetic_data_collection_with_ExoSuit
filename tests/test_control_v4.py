"""Host-side v4 scheduling/trajectory/identity tests, without claiming DDS."""
from pathlib import Path
from types import SimpleNamespace as NS
import math
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'exo2isaacsim/src/isaac_eff_control'))
from isaac_eff_control.sim_policy_clock import SimPolicyClock
from isaac_eff_control.bfm_action_executor import ActionExecutor, NAMES
from isaac_eff_control.bfm_contract import collection_metadata, state_defaults


def action(aid=1, ns=0, session='producer'):
    e = ActionExecutor()
    q = [(e.bounds[n][0]+e.bounds[n][1])/2 for n in NAMES]
    return NS(producer_session_id=session, action_id=aid,
              header=NS(stamp=NS(sec=ns//10**9, nanosec=ns%10**9)),
              left_arm_joint_names=list(NAMES[:7]), right_arm_joint_names=list(NAMES[7:]),
              left_arm_target=q[:7], right_arm_target=q[7:],
              left_hand_target=[10.]*6, right_hand_target=[20.]*6)


class V4Tests(unittest.TestCase):
    def ready(self):
        e = ActionExecutor()
        a = action()
        q = a.left_arm_target+a.right_arm_target
        self.assertTrue(e.measure(list(reversed(NAMES)), list(reversed(q)), 0, 0))
        return e, a

    def test_exact_rational_grid(self):
        for hz, stride in ((15,20),(100,3),(50,6)):
            c=SimPolicyClock(hz)
            ticks=[i for i in range(3001) if c.advance(i*10**9//300)]
            self.assertEqual(ticks, list(range(stride,3001,stride)))

    def test_frozen_reset_missed(self):
        c=SimPolicyClock()
        self.assertFalse(c.advance(0))
        self.assertTrue(c.advance(10**9))
        for _ in range(10): self.assertFalse(c.advance(10**9))
        self.assertFalse(c.advance(0))
        self.assertTrue(c.advance(66_666_666))
        self.assertFalse(c.advance(66_666_666))

    def test_acceptance_identity_watermark(self):
        e,a=self.ready()
        self.assertEqual(e.accept(a,0,0),(True,'accepted_not_attained'))
        self.assertFalse(e.accept(a,0,0)[0])
        e.reset()
        e.measure(list(NAMES),a.left_arm_target+a.right_arm_target,0,0)
        self.assertFalse(e.accept(a,0,0)[0])
        a.action_id=2
        self.assertTrue(e.accept(a,0,0)[0])
        b=action(3,1,'other')
        self.assertFalse(e.accept(b,1,0)[0])

    def test_rejections(self):
        cases=[lambda a:setattr(a,'action_id',0),lambda a:setattr(a,'producer_session_id',''),
               lambda a:a.left_arm_joint_names.reverse(),lambda a:a.left_arm_target.append(0),
               lambda a:a.left_arm_target.__setitem__(0,float('nan')),
               lambda a:a.right_hand_target.__setitem__(0,101),
               lambda a:a.left_arm_target.__setitem__(0,100)]
        for change in cases:
            e,a=self.ready();change(a)
            self.assertFalse(e.accept(a,0,0)[0])
            self.assertIsNone(e.target)
        e,a=self.ready()
        self.assertFalse(e.accept(a,1_000_000_000,0)[0])
        a.header.stamp.nanosec=1
        self.assertFalse(e.accept(a,0,0)[0])

    def test_trajectory_limits_and_retarget(self):
        e,a=self.ready();self.assertTrue(e.accept(a,0,0)[0])
        for i in range(14): e.target[i]=e.bounds[NAMES[i]][1]
        oldq=e.q[:];oldv=e.v[:]
        for k in range(1,101):
            now=k*10_000_000
            e.measure(list(NAMES),e.q,now,k*.01)
            if k==40: e.target=[e.bounds[n][0] for n in NAMES]
            q=e.step(now,k*.01)
            for i,n in enumerate(NAMES):
                self.assertLessEqual(abs(e.v[i]),1+1e-12)
                self.assertLessEqual(abs(e.v[i]-oldv[i]),.02+1e-12)
                self.assertLessEqual(abs(q[i]-oldq[i]),.01+1e-12)
                self.assertTrue(e.bounds[n][0]<=q[i]<=e.bounds[n][1])
            oldq=q[:];oldv=e.v[:]

    def test_invalid_and_stale_measurement_requires_new_action(self):
        e,a=self.ready();e.accept(a,0,0)
        self.assertIsNone(e.step(200_000_000,0))
        self.assertIsNone(e.q)
        self.assertFalse(e.measure(list(NAMES),[float('inf')]*14,0,0))
        self.assertFalse(e.measure(list(NAMES),[100.]*14,0,0))
        self.assertIsNone(e.step(0,0))

    def test_contract_and_routes(self):
        m=collection_metadata(True,state_defaults())
        self.assertEqual((m['contract_version'],m['policy_hz']),(4,15))
        c=m['execution']['configuration']
        self.assertEqual((c['arm_control_hz'],c['hand_control_hz'],c['physics_hz']),(100,50,300))
        p=ROOT/'exo2isaacsim/src/isaac_eff_control/isaac_eff_control'
        g=(p/'bfm_action_gateway.py').read_text()
        self.assertNotIn('self.arm_pub',g); self.assertNotIn('self.hand_pub',g)
        self.assertIn('source_timeout',g)
        policy=(ROOT/'start_bfm_policy_control.sh').read_text()
        self.assertIn('bfm_action_executor_node',policy)
        self.assertNotIn('bfm_action_gateway_node',policy)
        self.assertIn('bfm_action_executor_node',(ROOT/'start_isaac_control.sh').read_text())
        self.assertIn('"/bfm/execution_status"',(p/'sim_recorder.py').read_text())
        self.assertIn('SimPolicyClock(50)',(p/'eff_control_node.py').read_text())
        self.assertIn('CreateTimeStepsPerSecondAttr().Set(300)',(p/'collection_graph.py').read_text())

if __name__=='__main__':unittest.main()

class GatewayV4Tests(unittest.TestCase):
    def setUp(self):
        import test_recording_pipeline as h
        self.h = h
        h.RecordingPipelineTests.setUp(self)

    def test_buffered_source_cannot_bypass_and_staleness(self):
        from unittest.mock import patch
        h=self.h
        module,g=h.RecordingPipelineTests.make_gateway(self)
        arm=h.RecordingPipelineTests.arm_source(self)
        hand=h.RecordingPipelineTests.hand_source(self)
        with patch.object(module.time,'monotonic',return_value=0):
            for i in range(20):g.arm_callback(arm);g.hand_callback(hand)
            self.assertEqual(set(g.publishers),{'/bfm/action'})
            g.action_pub.publish.assert_not_called()
            for i in range(301):
                ns=i*10**9//300
                g.clock_callback(NS(clock=NS(sec=ns//10**9,nanosec=ns%10**9)))
            self.assertEqual(g.action_pub.publish.call_count,15)
            actions=[c.args[0] for c in g.action_pub.publish.call_args_list]
            self.assertEqual([a.action_id for a in actions],list(range(1,16)))
            self.assertEqual(len({a.producer_session_id for a in actions}),1)
        with patch.object(module.time,'monotonic',return_value=1):
            g.policy_tick()
            self.assertEqual(g.action_pub.publish.call_count,15)

    def test_finger_clock_retains_mapping_at_50_sim_hz(self):
        from unittest.mock import patch
        with patch.dict('os.environ',BFM_COLLECTION_MODE='1'):
            m=self.h.load_module('v4_fingers','exo2isaacsim/src/isaac_eff_control/isaac_eff_control/eff_control_node.py')
        h=m.IsaacEffControlNode();h.sim_playing=True
        h.hand_callback(self.h.RecordingPipelineTests.hand_source(self))
        for i in range(301):
            ns=i*10**9//300
            h.clock_callback(NS(clock=NS(sec=ns//10**9,nanosec=ns%10**9)))
        self.assertEqual(h.pub_right.publish.call_count,50)
        self.assertEqual(h.current_right_values,[0,100,42,44,61,60])
        for _ in range(10):h.clock_callback(NS(clock=NS(sec=1,nanosec=0)))
        self.assertEqual(h.pub_right.publish.call_count,50)
