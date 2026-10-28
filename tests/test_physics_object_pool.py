"""Real USD and NumPy/Warp CPU arrays around fake PhysX actors, not contact tests."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT/'exo2isaacsim/src/isaac_eff_control/isaac_eff_control/physics_object_pool.py'
spec = importlib.util.spec_from_file_location('pool_test_module', PATH)
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)


class LiveBody:
    """Separate actor state and getter arrays; setters never touch the USD stage."""
    count = 1
    def __init__(self, frontend='numpy'):
        self.frontend = frontend
        self.pose = np.array([[99.,98.,97.,0.,0.,0.,1.]], dtype=np.float32)
        self.velocity = np.full((1,6), 7., dtype=np.float32)
        self.mass = np.array([[.1]], dtype=np.float32)
        self.inertia = np.eye(3,dtype=np.float32).reshape(1,9)*.001
        self.writes = []
        self.ignore_pose = False
        self.fail_velocity = False
    def tensor(self, array):
        if self.frontend == 'warp':
            import warp as wp
            return wp.array(array, dtype=wp.float32, device='cpu')
        return array.copy()
    def get_transforms(self): return self.tensor(self.pose)
    def get_velocities(self): return self.tensor(self.velocity)
    def get_masses(self): return self.tensor(self.mass)
    def get_inertias(self): return self.tensor(self.inertia)
    def get_coms(self): return self.tensor(np.array([[0.,0.,0.,0.,0.,0.,1.]],dtype=np.float32))
    def unpack(self, data, indices, size):
        if self.frontend == 'warp':
            import warp as wp
            assert isinstance(data,wp.array) and data.dtype == wp.float32
            assert indices.dtype == wp.int32 and str(data.device) == 'cpu'
            data, indices = data.numpy(),indices.numpy()
        else:
            assert data.dtype == np.float32 and data.flags.c_contiguous
            assert indices.dtype == np.int32
        assert list(indices) == [0] and data.shape == (1,size)
        return data.copy()
    def set_transforms(self, data, indices):
        value=self.unpack(data,indices,7); self.writes.append('pose')
        if not self.ignore_pose: self.pose = value
    def set_velocities(self, data, indices):
        value=self.unpack(data,indices,6); self.writes.append('velocity')
        if self.fail_velocity:return False
        self.velocity = value


def fixture(frontend='numpy', scale=1.):
    stage=Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage,scale); UsdGeom.SetStageUpAxis(stage,'Z')
    UsdGeom.Xform.Define(stage,'/World')
    stage.GetRootLayer().subLayerPaths.append(str(ROOT/'models/stages/fruit_pool.usda'))
    prim=UsdGeom.Xform.Define(stage,'/World/container_b04').GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    cube=UsdGeom.Cube.Define(stage,'/World/container_b04/Collision')
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    selected='/World/Task/Fruits/lemon1'
    parked={f'/World/Task/Fruits/{name}':[100.+i*2,100.,1.93]
            for i,name in enumerate(('avocado','pomegranate','orange2'))}
    q=[0.,0.,.6,.8]
    targets={selected:[1.,4.,.8,*q], '/World/container_b04':[1.,4.4,.75,0.,0.,0.,1.],
             **{path:[*pos,0.,0.,0.,1.] for path,pos in parked.items()}}
    episode=dict(episode_id='test', object_prim_path=selected,position=[1.,4.,.8],
        orientation_xyzw=q,container_prim_path='/World/container_b04',container_position=[1.,4.4,.75],
        parking_positions=parked,body_poses=targets)
    bodies={path:LiveBody(frontend) for path in targets}
    view=SimpleNamespace(create_rigid_body_view=Mock(side_effect=lambda path:bodies[path]))
    return stage,episode,bodies,view


class PhysicsObjectPoolTests(unittest.TestCase):
    def test_applies_five_live_actors_verifies_and_never_changes_usd(self):
        stage,episode,bodies,view=fixture()
        before=stage.ExportToString()
        pool=module.PhysicsObjectPool(stage,episode,log=Mock())
        pool.bind(view); pool.apply_episode(); pool.verify_episode()
        for path,body in bodies.items():
            np.testing.assert_allclose(body.pose[0],episode['body_poses'][path],atol=1e-6)
            np.testing.assert_equal(body.velocity,0)
            self.assertEqual(body.writes,['pose','velocity'])
        pool.apply_episode(); pool.bind(view)
        self.assertEqual(view.create_rigid_body_view.call_count,5)
        self.assertEqual(stage.ExportToString(),before)
        self.assertTrue(pool.parking_settled())
        pool.release(); self.assertFalse(pool.bodies); self.assertIsNone(pool.simulation_view)
        pool.bind(view); pool.apply_episode(); pool.verify_episode()
        self.assertEqual(view.create_rigid_body_view.call_count,10)
        self.assertEqual(stage.ExportToString(),before)

    def test_stage_length_units_xyzw_and_quaternion_sign(self):
        stage,ep,bodies,view=fixture(scale=.01)
        pool=module.PhysicsObjectPool(stage,ep,log=Mock())
        pool.bind(view); pool.apply_episode()
        body=bodies[ep['object_prim_path']]
        np.testing.assert_allclose(body.pose[0,:3],[100,400,80])
        np.testing.assert_allclose(body.pose[0,3:],[0,0,.6,.8])
        body.pose[0,3:] *= -1
        pool.verify_episode()
        for path in ep['parking_positions']: bodies[path].velocity[0,0]=.4 # .004 m/s
        self.assertTrue(pool.parking_settled())

    def test_exact_paths_counts_mass_and_inertia_are_preflighted_before_any_write(self):
        for failure in ('missing','duplicate','mass','inertia','nan_pose','nan_velocity'):
            stage,ep,bodies,view=fixture();body=list(bodies.values())[-1]
            if failure=='missing':body.count=0
            if failure=='duplicate':body.count=2
            if failure=='mass':body.mass[:]=0
            if failure=='inertia':body.inertia[0,0]=-1
            if failure=='nan_pose':body.pose[0,0]=np.nan
            if failure=='nan_velocity':body.velocity[0,0]=np.nan
            pool=module.PhysicsObjectPool(stage,ep,log=Mock())
            with self.subTest(failure=failure),self.assertRaises(ValueError):pool.bind(view)
            self.assertTrue(all(not body.writes for body in bodies.values()))
            self.assertFalse(pool.bodies)

    def test_usd_dynamic_and_collision_checks_do_not_replace_live_resolution(self):
        for failure in ('disabled','kinematic','gravity','nested','triangle_mesh','no_collision','articulation'):
            stage,ep,bodies,view=fixture()
            prim=stage.GetPrimAtPath(ep['object_prim_path']);api=UsdPhysics.RigidBodyAPI(prim)
            if failure=='articulation':UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath('/World'))
            if failure=='disabled':api.CreateRigidBodyEnabledAttr().Set(False)
            if failure=='kinematic':api.CreateKinematicEnabledAttr().Set(True)
            if failure=='gravity':prim.CreateAttribute('physxRigidBody:disableGravity',Sdf.ValueTypeNames.Bool).Set(True)
            if failure=='nested':UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage,str(prim.GetPath())+'/Nested').GetPrim())
            for child in Usd.PrimRange(prim):
                if child.HasAPI(UsdPhysics.CollisionAPI):
                    if failure=='triangle_mesh':UsdPhysics.MeshCollisionAPI(child).CreateApproximationAttr().Set('none')
                    if failure=='no_collision':UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr().Set(False)
            with self.subTest(failure=failure),self.assertRaises(ValueError):
                module.PhysicsObjectPool(stage,ep,log=Mock()).bind(view)
            self.assertTrue(all(not body.writes for body in bodies.values()))

    def test_wrong_readback_or_setter_failure_prevents_verification(self):
        for failure in ('pose','velocity','orientation','setter'):
            stage,ep,bodies,view=fixture();pool=module.PhysicsObjectPool(stage,ep,log=Mock())
            body=bodies[ep['object_prim_path']];pool.bind(view)
            if failure=='pose':body.ignore_pose=True
            if failure=='setter':body.fail_velocity=True
            with self.subTest(failure=failure),self.assertRaises((ValueError,RuntimeError)):
                pool.apply_episode()
                if failure=='velocity':body.velocity[0,3]=.1
                if failure=='orientation':body.pose[0,3:]=[0,0,0,1]
                pool.verify_episode()
            self.assertFalse(pool.verified)

    def test_identity_disagreement_rejected_before_binding(self):
        stage,ep,bodies,view=fixture();ep['object_prim_path']='/World/Task/Fruits/avocado'
        with self.assertRaises(ValueError):module.PhysicsObjectPool(stage,ep)
        view.create_rigid_body_view.assert_not_called()

    @unittest.skipUnless(importlib.util.find_spec('warp'),'Optional Warp CPU backend')
    def test_real_warp_arrays_and_indices_on_the_getters_device(self):
        stage,ep,bodies,view=fixture(frontend='warp')
        pool=module.PhysicsObjectPool(stage,ep,log=Mock())
        pool.bind(view);pool.apply_episode();pool.verify_episode()
        self.assertTrue(pool.parking_settled())


class FruitContactCompositionTests(unittest.TestCase):
    def test_only_preloaded_fruit_colliders_receive_meter_scale_contact_profile(self):
        from pxr import UsdShade
        import hashlib
        import json
        import math
        asset = ROOT / 'models/stages/fruit_pool.usda'
        stage = Usd.Stage.Open(str(asset))
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.)
        expected = {f'/World/Task/Fruits/{n}': count for n, count in
                    (('avocado', 1), ('lemon1', 1), ('pomegranate', 1), ('orange2', 2))}
        bodies = {str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)}
        self.assertEqual(bodies, set(expected))
        material = stage.GetPrimAtPath('/World/Task/Fruits/FruitContact')
        api = UsdPhysics.MaterialAPI(material)
        self.assertAlmostEqual(api.GetStaticFrictionAttr().Get(), .6)
        self.assertAlmostEqual(api.GetDynamicFrictionAttr().Get(), .5)
        self.assertEqual(api.GetRestitutionAttr().Get(), 0.)
        for body, count in expected.items():
            shapes = [p for p in Usd.PrimRange(stage.GetPrimAtPath(body))
                      if p.HasAPI(UsdPhysics.CollisionAPI)]
            self.assertEqual(len(shapes), count)
            for shape in shapes:
                self.assertTrue(UsdPhysics.CollisionAPI(shape).GetCollisionEnabledAttr().Get())
                contact = shape.GetAttribute('physxCollision:contactOffset').Get()
                rest = shape.GetAttribute('physxCollision:restOffset').Get()
                self.assertTrue(math.isfinite(contact) and math.isfinite(rest))
                self.assertAlmostEqual(contact * UsdGeom.GetStageMetersPerUnit(stage), .002)
                self.assertEqual(rest, 0.)
                self.assertGreater(contact, rest)
                self.assertEqual(UsdPhysics.MeshCollisionAPI(shape).GetApproximationAttr().Get(),
                                 'convexDecomposition')
                bound, _ = UsdShade.MaterialBindingAPI(shape).ComputeBoundMaterial('physics')
                self.assertEqual(bound.GetPath(), material.GetPath())
        for p in stage.Traverse():
            if str(p.GetPath()).startswith('/World/Task/Parking'):
                self.assertFalse(UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial('physics')[0])
        config = json.loads((ROOT / 'exo2isaacsim/src/isaac_eff_control/isaac_eff_control/sorting_scene.json').read_text())
        self.assertEqual(config['provenance']['fruit_pool_sha256'], hashlib.sha256(asset.read_bytes()).hexdigest())

    def test_authored_contact_opinions_are_an_exact_whitelist_not_a_runtime_mutator(self):
        layer = Sdf.Layer.FindOrOpen(str(ROOT / 'models/stages/fruit_pool.usda'))
        targets = []
        def inspect(path):
            attribute = layer.GetAttributeAtPath(path)
            relation = layer.GetRelationshipAtPath(path)
            if attribute and str(attribute.name).startswith('physxCollision:'):
                targets.append(str(attribute.path.GetPrimPath()))
            if relation and relation.name == 'material:binding:physics':
                targets.append(str(relation.path.GetPrimPath()))
        layer.Traverse('/', inspect)
        expected = {
            '/World/Task/Fruits/avocado/avocado01_inst/Avocado_01',
            '/World/Task/Fruits/lemon1/lemon_01_inst/Lemon_01',
            '/World/Task/Fruits/pomegranate/pomegranate01_inst/pomegranate_01',
            '/World/Task/Fruits/orange2/orange_02_inst/Orange_02/OrangeSkin',
            '/World/Task/Fruits/orange2/orange_02_inst/Orange_02/OrangeStem',
        }
        self.assertEqual(set(targets), expected)
        self.assertEqual(len(targets), 3*len(expected))
        self.assertFalse(any('*' in p or '?' in p or 'robot' in p or 'sharpa' in p for p in targets))
        # There is no callable path accepting robot/articulation/wildcard inputs.
        # Existing pool preflight separately rejects articulation ancestors.
        stage = Usd.Stage.Open(layer)
        self.assertFalse(any(p.HasAPI(UsdPhysics.ArticulationRootAPI) or p.IsA(UsdPhysics.Scene)
                             for p in stage.Traverse()))
        # Static leaf bindings cannot spread to an injected articulation or an
        # unexpected body, even if it is placed under the fruit namespace.
        from pxr import UsdShade
        stage.SetEditTarget(stage.GetSessionLayer())
        for path in ('/World/Robot', '/World/Task/Fruits/unexpected'):
            body = UsdGeom.Xform.Define(stage, path).GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(body)
            UsdPhysics.ArticulationRootAPI.Apply(body)
            shape = UsdGeom.Cube.Define(stage, path + '/Collider').GetPrim()
            UsdPhysics.CollisionAPI.Apply(shape)
            self.assertFalse(UsdShade.MaterialBindingAPI(shape).ComputeBoundMaterial('physics')[0])
            attribute = shape.GetAttribute('physxCollision:contactOffset')
            self.assertFalse(attribute and attribute.HasAuthoredValueOpinion())
