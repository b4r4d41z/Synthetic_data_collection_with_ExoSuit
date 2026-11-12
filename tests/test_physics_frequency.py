"""Real OpenUSD session tests; PhysxSchema is stubbed, not live Isaac physics."""
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from pxr import Sdf, Usd, UsdPhysics

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'exo2isaacsim/src/isaac_eff_control'))
from isaac_eff_control import bfm_contract as contract
from isaac_eff_control.collection_graph import configure_physics_timestep
from isaac_eff_control.sim_policy_clock import SimPolicyClock


class PhysicsFrequencyTests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        self.scene = UsdPhysics.Scene.Define(self.stage, '/World/PhysicsScene').GetPrim()
        schema = ModuleType('pxr.PhysxSchema')
        class SceneAPI:
            def __init__(self, prim):
                self.prim = prim

            @staticmethod
            def Apply(prim):
                return SceneAPI(prim)

            def CreateTimeStepsPerSecondAttr(self):
                return self.prim.CreateAttribute('physxScene:timeStepsPerSecond', Sdf.ValueTypeNames.Int)

            def GetTimeStepsPerSecondAttr(self):
                return self.prim.GetAttribute('physxScene:timeStepsPerSecond')
        schema.PhysxSceneAPI = SceneAPI
        self.schema = schema
        self.modules = patch.dict(sys.modules, {'pxr.PhysxSchema': schema})
        self.modules.start(); self.addCleanup(self.modules.stop)

    def test_constant_drives_effective_session_attribute_and_metadata(self):
        root_before = self.stage.GetRootLayer().ExportToString()
        for hz in (60., 120., 300., 100.):
            with self.subTest(hz=hz), patch.object(contract, 'PHYSICS_HZ', hz):
                configure_physics_timestep(self.stage)
                attr = self.scene.GetAttribute('physxScene:timeStepsPerSecond')
                self.assertEqual(attr.Get(), hz)
                self.assertEqual(attr.GetPropertyStack()[0].layer, self.stage.GetSessionLayer())
                metadata = contract.collection_metadata(True, contract.state_defaults())
                self.assertEqual(metadata['control']['physics_hz'], hz)
                self.assertEqual(metadata['policy_hz'], 15)
                self.assertEqual(metadata['control']['hand_hz'], 50)
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), root_before)
        self.assertEqual(len([p for p in self.stage.Traverse() if p.IsA(UsdPhysics.Scene)]), 1)

    def test_invalid_configuration_is_rejected_before_authoring(self):
        for invalid in (0, -1, math.nan, math.inf, -math.inf, True, None, '300',
                        59.5, 300+0j, 2**31, 10**1000, object()):
            with self.subTest(value=repr(invalid)), patch.object(contract, 'PHYSICS_HZ', invalid):
                with self.assertRaisesRegex(ValueError, 'PHYSICS_HZ'):
                    configure_physics_timestep(self.stage)
                with self.assertRaisesRegex(ValueError, 'PHYSICS_HZ'):
                    contract.collection_metadata(True, contract.state_defaults())
        self.assertFalse(self.scene.GetAttribute('physxScene:timeStepsPerSecond'))

    def test_scene_rejection_and_wrong_readback_fail_clearly(self):
        for result in (False, RuntimeError('injected USD rejection')):
            def set_value(value):
                if isinstance(result, Exception):
                    raise result
                return result
            api = SimpleNamespace(CreateTimeStepsPerSecondAttr=lambda:SimpleNamespace(Set=set_value))
            with self.subTest(result=result), patch.object(self.schema.PhysxSceneAPI, 'Apply', return_value=api):
                with self.assertRaisesRegex(RuntimeError, 'could not be applied'):
                    configure_physics_timestep(self.stage)
        api = SimpleNamespace(
            CreateTimeStepsPerSecondAttr=lambda:SimpleNamespace(Set=lambda value:True),
            GetTimeStepsPerSecondAttr=lambda:SimpleNamespace(Get=lambda:42))
        with patch.object(self.schema.PhysxSceneAPI, 'Apply', return_value=api):
            with self.assertRaisesRegex(RuntimeError, 'requested=.*effective=42'):
                configure_physics_timestep(self.stage)

    def test_requires_one_existing_scene(self):
        self.stage.RemovePrim('/World/PhysicsScene')
        with self.assertRaisesRegex(RuntimeError, 'one existing'):
            configure_physics_timestep(self.stage)
        UsdPhysics.Scene.Define(self.stage, '/One')
        UsdPhysics.Scene.Define(self.stage, '/Two')
        with self.assertRaisesRegex(RuntimeError, 'one existing'):
            configure_physics_timestep(self.stage)

    def test_rational_sampler_independent_of_physics_rate(self):
        for hz in (60, 120, 300, 100, 240):
            with self.subTest(hz=hz):
                clock = SimPolicyClock()
                samples = [i*10**9//hz for i in range(hz*10+1)]
                ticks = [t for t in samples if clock.advance(t)]
                self.assertEqual(len(ticks), 150)
                for index, tick in enumerate(ticks, 1):
                    # First available physics sample at/after the rational deadline.
                    self.assertGreaterEqual((tick+1)*15, index*10**9)
                    self.assertLess((tick-1)*15, index*10**9 + 15*10**9/hz)
                for _ in range(10):
                    self.assertFalse(clock.advance(samples[-1]))  # Frozen/PAUSE.
                self.assertFalse(clock.advance(0))  # Reset, no decision.
                self.assertTrue(clock.advance(10**9))  # Missed deadlines: one tick.
                self.assertFalse(clock.advance(10**9))
                self.assertFalse(clock.advance(10**9+1))


if __name__ == '__main__':
    unittest.main()
