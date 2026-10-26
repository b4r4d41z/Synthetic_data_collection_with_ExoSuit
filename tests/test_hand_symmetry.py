"""Control and composed USD symmetry; does not certify measured PhysX closure."""
import importlib.util
import math
import os
import sys
import unittest
from unittest.mock import patch

from test_recording_pipeline import ROOT, Message, load_module, ros_stubs


class HandSymmetryTests(unittest.TestCase):
    def setUp(self):
        paths = patch.object(sys, 'path', [str(ROOT/'exo2isaacsim/src/isaac_eff_control'), *sys.path])
        paths.start(); self.addCleanup(paths.stop)
        modules = patch.dict(sys.modules, ros_stubs({}))
        modules.start(); self.addCleanup(modules.stop)
        with patch.dict(os.environ, {'BFM_COLLECTION_MODE': '0'}):
            self.module = load_module('test_symmetric_eff',
                'exo2isaacsim/src/isaac_eff_control/isaac_eff_control/eff_control_node.py')
        self.node = self.module.IsaacEffControlNode()

    def targets(self, side, value):
        names, positions = [], []
        self.node.append_hand_targets(names, positions, [value]*6,
                                      getattr(self.node, side+'_mapping'))
        return dict(zip(names, positions))

    def test_all_channels_mirror_at_representative_commands(self):
        for value in (0, 25, 50, 75, 100):
            left, right = self.targets('left', value), self.targets('right', value)
            self.assertEqual(len(left), len(right))
            for name, angle in left.items():
                self.assertAlmostEqual(angle, -right[name.replace('left_', 'right_', 1)])
            self.assertAlmostEqual(left['left_thumb_CMC_AA'], math.radians(-20))
            self.assertAlmostEqual(right['right_thumb_CMC_AA'], math.radians(20))
            self.assertAlmostEqual(left['left_thumb_MCP_FE'], math.radians(50)*value/100)
            self.assertAlmostEqual(left['left_thumb_IP'], math.radians(65)*value/100)

    def test_smoothing_and_actual_published_commands_remain_symmetric(self):
        # Follow the same history on both sides; keep the intentional thumb gate.
        for value in (0, 25, 50, 75, 100, 0):
            message = Message()
            message.left_hand_position = message.right_hand_position = [value]*6
            self.node.hand_callback(message)
            for _ in range(100):
                self.node.timer_callback()
                self.assertEqual(self.node.current_left_values, self.node.current_right_values)
                left = self.node.pub_left.publish.call_args.args[0]
                right = self.node.pub_right.publish.call_args.args[0]
                self.assertEqual([n.replace('left_', 'right_', 1) for n in left.name], right.name)
                for a, b in zip(left.position, right.position):
                    self.assertAlmostEqual(a, -b)
            if value in (0, 100):
                self.assertEqual(self.node.current_left_values, [value]*6)

    @unittest.skipUnless(importlib.util.find_spec('pxr'), 'Requires OpenUSD')
    def test_composed_joint_limits_drives_frames_and_graph_are_mirrored(self):
        from pxr import Gf, Usd, UsdPhysics
        root = '/World/s40_sharpa_two_cameras'
        stage = Usd.Stage.OpenMasked(str(ROOT/'models/stages/stage_sorting_v1.usd'),
                                    Usd.StagePopulationMask([root]))
        stage.SetEditTarget(stage.GetSessionLayer())
        stage.GetPrimAtPath(root).GetVariantSet('Physics').SetVariantSelection('physx')
        joints = {p.GetName(): p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)}
        mirror = Gf.Matrix4d(1); mirror[1, 1] = -1
        for name, left in joints.items():
            if not name.startswith('left_'):
                continue
            right = joints[name.replace('left_', 'right_', 1)]
            def attr(prim, key):
                return prim.GetAttribute(key).Get()
            self.assertEqual(attr(left, 'physics:axis'), attr(right, 'physics:axis'))
            self.assertAlmostEqual(attr(left, 'physics:lowerLimit'), -attr(right, 'physics:upperLimit'))
            self.assertAlmostEqual(attr(left, 'physics:upperLimit'), -attr(right, 'physics:lowerLimit'))
            for key in ('stiffness', 'damping', 'maxForce', 'targetPosition', 'targetVelocity', 'type'):
                self.assertEqual(attr(left, 'drive:angular:physics:'+key),
                                 attr(right, 'drive:angular:physics:'+key))
            for value in (0, 25, 50, 75, 100):
                matrices = []
                for side, prim in (('left', left), ('right', right)):
                    target = math.degrees(self.targets(side, value).get(prim.GetName(), 0.))
                    # PhysX applies limits: both sides must clamp symmetrically.
                    target = max(attr(prim, 'physics:lowerLimit'), min(attr(prim, 'physics:upperLimit'), target))
                    frames = []
                    for i in (0, 1):
                        frames.append(Gf.Matrix4d(1).SetRotate(Gf.Quatd(attr(prim, f'physics:localRot{i}'))))
                        frames[-1].SetTranslateOnly(Gf.Vec3d(attr(prim, f'physics:localPos{i}')))
                    rotation = Gf.Matrix4d(1).SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), target))
                    matrices.append(frames[1].GetInverse()*rotation*frames[0])
                self.assertTrue(Gf.IsClose(mirror*matrices[0]*mirror, matrices[1], 1e-5), (name, value))
        graph = root+'/Control_ActionGraph/'
        for side, subscriber, controller in (
                ('left', 'ros2_subscribe_joint_state_01', 'articulation_controller_01'),
                ('right', 'ros2_subscribe_joint_state', 'articulation_controller_02')):
            sub = stage.GetPrimAtPath(graph+subscriber)
            ctl = stage.GetPrimAtPath(graph+controller)
            self.assertEqual(sub.GetAttribute('inputs:topicName').Get(), side+'_hand_joint_command')
            for field in ('jointNames', 'positionCommand'):
                self.assertEqual([str(p) for p in ctl.GetAttribute('inputs:'+field).GetConnections()],
                                 [graph+subscriber+'.outputs:'+field])
            self.assertEqual([str(p) for p in ctl.GetRelationship('inputs:targetPrim').GetTargets()],
                             [root+'/fix_base_joint'])
