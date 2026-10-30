"""Four focused runtime contracts; geometry uses USD, not a claimed PhysX run."""
import itertools
import json
from pathlib import Path
import random
import sys
import unittest
from types import SimpleNamespace
import test_recording_pipeline as harness
sys.path.insert(0,str(harness.ROOT/'exo2isaacsim/src/isaac_eff_control'))
from isaac_eff_control.fruit_success import SettledContainment, LiveFruitSuccess


def corners(x=0., radius=.03):
    return list(itertools.product((x-radius,x+radius),(-radius,radius),(.012,.012+radius*2)))


class SuccessTests(unittest.TestCase):
    def detector(self):
        return SettledContainment([(-.16,-.11),(.16,-.11),(.16,.11),(-.16,.11)],.011,.118)

    def run_dwell(self, points, speeds=(0.,0.,0.,0.)):
        d=self.detector()
        for i in range(32): result,audit=d.update(i*.01,points,speeds)
        return result,audit

    def test_outside_protruding_moving_and_contained(self):
        for p,s,expected in [(corners(.3),(0,0,0,0),False),
                (corners(.14),(0,0,0,0),False), (corners(),(.2,0,0,0),False),
                (corners(),(0,.3,0,0),False), (corners(),(0,0,0,.3),False),
                (corners(),(0,0,0,0),True)]:
            self.assertIs(self.run_dwell(p,s)[0],expected)

    def test_sizes_gaps_resets_and_duplicates(self):
        self.assertTrue(self.run_dwell(corners(.10,.02))[0])
        self.assertFalse(self.run_dwell(corners(.10,.08))[0])
        d=self.detector()
        for _ in range(100): self.assertFalse(d.update(0.,corners(),[0]*4)[0])
        self.assertFalse(d.update(.4,corners(),[0]*4)[0])  # Missing history.
        self.assertFalse(d.update(0.,corners(),[0]*4)[0])  # Reset.

    def test_actual_four_assets_and_rotated_container(self):
        from pxr import Usd, Gf
        stage=Usd.Stage.Open(str(harness.ROOT/'models/stages/stage_sorting_v1.usd'))
        sizes=[]
        for name in ('avocado','lemon1','orange2'):
            config=dict(object_prim_path='/World/Task/Fruits/'+name,container_prim_path='/World/container_b04')
            d=LiveFruitSuccess(stage,config,'test')
            sizes.append(tuple(max(p[i] for p in d.corners)-min(p[i] for p in d.corners) for i in range(3)))
            self.assertGreater(len(d.detector.polygon),3)
            # Actual fruit geometry far outside an arbitrarily rotated container.
            class View:
                def __init__(self,pose): self.pose=pose
                def get_transforms(self): return [self.pose]
                def get_velocities(self): return [[0.]*6]
            sampler=SimpleNamespace(views={'object_prim_path':View([20,20,20,0,0,0,1]),
                'container_prim_path':View([0,0,0,0,0,.7071067811865475,.7071067811865475])})
            for i in range(40): result=d.measure(sampler,i*.01)
            self.assertFalse(result['success'])
            # Find a lying orientation, fully contained in the measured prism.
            # Fake only rigid-body measurements; bounds/floor remain real USD.
            for axis in ((1,0,0),(0,1,0),(0,0,1)):
                rotation=Gf.Rotation(Gf.Vec3d(*axis),90)
                rotated=[rotation.TransformDir(Gf.Vec3d(*p)) for p in d.corners]
                lo=[min(p[i] for p in rotated) for i in range(3)]
                hi=[max(p[i] for p in rotated) for i in range(3)]
                offset=Gf.Vec3d(-.08494214117424082-(lo[0]+hi[0])/2,
                               -(lo[1]+hi[1])/2,d.detector.floor+.001-lo[2])
                local=Gf.Matrix4d(1); local.SetRotate(rotation); local.SetTranslateOnly(offset)
                container=Gf.Matrix4d(1); container.SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1),90))
                world=local*container; q=world.ExtractRotationQuat()
                sampler.views['object_prim_path'].pose=[*world.ExtractTranslation(),*q.GetImaginary(),q.GetReal()]
                for i in range(40): result=d.measure(sampler,i*.01)
                if result['success']: break
            self.assertTrue(result['success'],name)
        self.assertEqual(len(set(sizes)),3)

    def test_near_half_spawn(self):
        config=json.loads((harness.ROOT/'exo2isaacsim/src/isaac_eff_control/isaac_eff_control/sorting_scene.json').read_text())
        r=config['randomization']['object_spawn_region']
        self.assertAlmostEqual(r['x_max']-r['x_min'],(1.1-.95)/2)
        robot=.622416973
        self.assertLess(abs(r['x_min']-robot),abs(r['x_max']-robot))
        import test_bfm_collection_extension as scene
        active=config['randomization']['object_assets']
        self.assertEqual({Path(a).stem for a in active},{'avocado','lemon1','orange2'})
        selected=set()
        for _ in range(1000):
            asset,x,y,yaw=scene.runtime.sample_candidate(config['randomization'],random)
            selected.add(Path(asset).stem)
            self.assertTrue(.95<=x<=1.025 and 3.99<=y<=4.12)
        self.assertEqual(selected,{'avocado','lemon1','orange2'})


class ControlTests(unittest.TestCase):
    setUp=harness.RecordingPipelineTests.setUp

    def test_high_rate_controls_between_integer_sim_samples(self):
        _,g=harness.RecordingPipelineTests.make_gateway(self)
        arm=harness.RecordingPipelineTests.arm_source(self)
        hand=harness.RecordingPipelineTests.hand_source(self)
        def clock(ns): g.clock_callback(SimpleNamespace(clock=SimpleNamespace(sec=0,nanosec=ns)))
        clock(0)
        for i in range(10):
            arm.position=[i/100]*14; g.arm_callback(arm); g.hand_callback(hand)
        self.assertEqual(g.arm_pub.publish.call_count,10)
        self.assertEqual(g.hand_pub.publish.call_count,10)
        g.action_pub.publish.assert_not_called()
        clock(100_000_000)
        first=g.action_pub.publish.call_args.args[0]
        for i in range(10): g.arm_callback(arm)
        clock(200_000_000)
        second=g.action_pub.publish.call_args.args[0]
        self.assertEqual(second.header.stamp.nanosec-first.header.stamp.nanosec,100_000_000)
        commanded=dict(zip(g.arm_pub.publish.call_args.args[0].name,g.arm_pub.publish.call_args.args[0].position))
        self.assertEqual(second.left_arm_target,[commanded[n] for n in second.left_arm_joint_names])
        self.assertEqual(sum(len(getattr(second,k)) for k in ('left_arm_target','left_hand_target','right_arm_target','right_hand_target')),26)
