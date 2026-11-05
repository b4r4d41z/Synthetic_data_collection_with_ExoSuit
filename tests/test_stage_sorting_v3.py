"""V3 composition and the existing planner/live-reset path; no Isaac contacts."""
from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace, ModuleType
import os
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
import test_bfm_collection_extension as collection
from test_physics_object_pool import LiveBody, module as pool_module

ROOT = collection.ROOT
runtime = collection.runtime
CONFIG = ROOT/'exo2isaacsim/src/isaac_eff_control/isaac_eff_control/sorting_scene_v3.json'
TABLE = [[(0,3,.75),(2,3,.75),(2,5,.75)], [(0,3,.75),(2,5,.75),(0,5,.75)]]
sys.path.insert(0, str(CONFIG.parent.parent))


class SortingV3Tests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.Open(str(ROOT/'models/stages/stage_sorting_v3.usda'))
        self.data = runtime.load_configuration(CONFIG, ROOT)
        self.data['randomization']['seed'] = 7
        self.support = patch.object(runtime, 'tabletop_triangles', return_value=TABLE)
        self.support.start(); self.addCleanup(self.support.stop)
        self.planner = runtime.SceneRandomizer(self.stage, self.data, ROOT)

    def test_five_preloaded_bodies_geometry_mass_and_textures(self):
        bodies = list(self.planner.pool.values())
        self.assertEqual(set(bodies), {f'/World/Task/Things/{name}' for name in
                                      ('apple_02','lunchbag','ketchup_bottle','avocado','orange2')})
        self.assertEqual(len({tuple(p) for p in self.planner.parking.values()}), 5)
        active = [str(p.GetPath()) for p in self.stage.Traverse()
                  if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith('/World/Task/')]
        self.assertEqual(set(active), set(bodies))
        self.assertFalse(self.stage.GetPrimAtPath('/World/Task/Fruits').IsActive())
        self.assertFalse(self.stage.GetPrimAtPath('/World/Task/Parking').IsActive())
        self.assertEqual(len([p for p in self.stage.Traverse() if p.IsA(UsdPhysics.Scene)]), 1)
        for asset, path in self.planner.pool.items():
            body = self.stage.GetPrimAtPath(path)
            source = Usd.Stage.Open(str(ROOT/asset)); original = source.GetDefaultPrim()
            old_body = next(p for p in source.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertEqual(UsdPhysics.MassAPI(body).GetMassAttr().Get(), UsdPhysics.MassAPI(old_body).GetMassAttr().Get())
            self.assertEqual([p for p in Usd.PrimRange(body) if p.HasAPI(UsdPhysics.RigidBodyAPI)], [body])
            pool_module.PhysicsObjectPool._validate_composition(SimpleNamespace(stage=self.stage), path)
            np.testing.assert_allclose(UsdGeom.XformCache().GetLocalToWorldTransform(body).ExtractTranslation(),
                                       self.planner.parking[asset], atol=1e-8)
            bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default','render','proxy']).ComputeUntransformedBound(body).ComputeAlignedRange()
            np.testing.assert_allclose(bounds.GetMin(), self.planner.assets[asset][1])
            np.testing.assert_allclose(bounds.GetMax(), self.planner.assets[asset][2])
            # COM promotion includes the captured body's transform, not just its mass.
            old_mass = UsdPhysics.MassAPI(old_body)
            if old_body != original:
                transform = UsdGeom.XformCache().GetLocalToWorldTransform(old_body)
                expected = transform.Transform(Gf.Vec3d(old_mass.GetCenterOfMassAttr().Get()))
                np.testing.assert_allclose(UsdPhysics.MassAPI(body).GetCenterOfMassAttr().Get(), expected, atol=1e-8)
                rotation = np.asarray(Gf.Matrix3d(transform.ExtractRotationQuat()))
                axes = np.asarray(Gf.Matrix3d(Gf.Quatd(old_mass.GetPrincipalAxesAttr().Get())))
                new_axes = np.asarray(Gf.Matrix3d(Gf.Quatd(UsdPhysics.MassAPI(body).GetPrincipalAxesAttr().Get())))
                inertia = np.diag(old_mass.GetDiagonalInertiaAttr().Get())
                np.testing.assert_allclose(new_axes.T@inertia@new_axes,
                                           rotation.T@axes.T@inertia@axes@rotation, atol=1e-12)
            textures = [a.Get() for p in Usd.PrimRange(body) for a in p.GetAttributes()
                        if str(a.GetTypeName()) == 'asset' and str(a.Get()).endswith('.png@')]
            self.assertTrue(textures, asset)
            for texture in textures:
                self.assertTrue(Path(texture.resolvedPath).is_file(), str(texture))
            for shape in runtime.collision_shapes(body):
                self.assertEqual(shape.GetAttribute('physxCollision:contactOffset').Get(), np.float32(.002))
                material, _ = UsdShade.MaterialBindingAPI(shape).ComputeBoundMaterial('physics')
                self.assertEqual(str(material.GetPath()), '/World/Task/Things/ThingContact')

    def test_both_layouts_all_objects_and_independent_random_positions(self):
        observed, positions, boxes = set(), set(), set()
        for seed in range(8):
            self.planner.rng.seed(seed)
            for _ in range(30):
                episode = self.planner.randomize()
                observed.add((episode['layout'], episode['asset']))
                positions.add(tuple(episode['position'])); boxes.add(tuple(episode['container_position']))
                _, lo, hi = self.planner.assets[episode['asset_path']]
                rotation = Gf.Rotation(Gf.Vec3d(0,0,1), math.degrees(episode['yaw']))
                corners = [rotation.TransformDir(Gf.Vec3d(x,y,z))
                           for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])]
                low = [episode['position'][i]+min(c[i] for c in corners) for i in range(3)]
                high = [episode['position'][i]+max(c[i] for c in corners) for i in range(3)]
                boxlo, boxhi = [[v+episode['container_offset'][i] for i,v in enumerate(b)]
                                for b in self.planner.container_bounds]
                for bounds, side in (((low,high), 'right' if episode['layout']=='A' else 'left'),
                                     ((boxlo,boxhi), 'left' if episode['layout']=='A' else 'right')):
                    region = episode['placement_regions'][side]
                    for i,axis in enumerate(('x','y')):
                        self.assertTrue(region[axis+'_min'] <= (bounds[0][i]+bounds[1][i])/2 <= region[axis+'_max'])
                self.assertAlmostEqual(low[2], .755)
                self.assertAlmostEqual(boxlo[2], .75, places=5)
                self.assertFalse(runtime.overlaps(low,high,boxlo,boxhi,.03))
                self.assertEqual(len(episode['body_poses']), 6)
                self.assertEqual(episode['object_prim_path'], '/World/Task/Things/'+Path(episode['asset']).stem)
                self.assertEqual(set(episode['parking_positions']),
                                 set(self.planner.pool.values()) - {episode['object_prim_path']})
        self.assertEqual(observed, {(layout, name+'.usd') for layout in ('A','B')
                                    for name in ('apple_02','lunchbag','ketchup_bottle','avocado','orange2')})
        self.assertGreater(len(positions), 200); self.assertGreater(len(boxes), 200)

    def test_seed_reproducibility_and_v2_region_definitions(self):
        other = runtime.SceneRandomizer(self.stage, deepcopy(self.data), ROOT)
        for _ in range(10):
            a, b = self.planner.randomize(), other.randomize()
            for field in ('asset','layout','position','yaw','container_position'):
                self.assertEqual(a[field], b[field])
        regions = a['placement_regions']; old = self.data['randomization']
        self.assertEqual(regions['right'], old['object_spawn_region'])
        for i,axis in enumerate(('x','y')):
            centre = sum(bound[i] for bound in self.planner.container_bounds)/2
            self.assertEqual(regions['left'][axis+'_min'], centre+old['container_offset_region'][axis+'_min'])
            self.assertEqual(regions['left'][axis+'_max'], centre+old['container_offset_region'][axis+'_max'])

    def test_unsupported_and_environment_obstacle_placements_fail_without_changing_plan(self):
        previous = self.planner.randomize()
        before = self.stage.GetRootLayer().ExportToString()
        with patch.object(runtime, 'tabletop_triangles', return_value=[]):
            with self.assertRaisesRegex(ValueError, 'No supported'):
                self.planner.randomize()
        centre = [sum(b[i] for b in self.planner.container_bounds)/2 for i in (0,1)]
        asset = self.data['randomization']['object_assets'][0]
        choose = lambda values: 'A' if values == ('A','B') else values[0]
        with patch.object(self.planner.rng, 'choice', side_effect=choose), \
                patch.object(runtime, 'sample_candidate', return_value=(asset,*centre,0.)):
            with self.assertRaisesRegex(ValueError, 'No supported, obstacle-free'):
                self.planner.randomize()  # Centre is supported, but inside the randomized box.
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            obstacle = UsdGeom.Cube.Define(self.stage, '/World/Task/TestObstacle')
            obstacle.AddTranslateOp().Set((1,4.25,.8)); obstacle.AddScaleOp().Set((1,1,.3))
            obstacle.CreateSizeAttr(1); UsdPhysics.CollisionAPI.Apply(obstacle.GetPrim())
        with self.assertRaisesRegex(ValueError, 'No supported'):
            self.planner.randomize()
        self.assertEqual(self.planner.last_episode, previous)
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), before)

    def test_v3_configuration_rejects_missing_duplicate_or_legacy_pool_assets(self):
        import tempfile
        for fault in ('missing','duplicate','lemon','pomegranate','wrong_path',
                      'selection_missing','selection_duplicate','selection_lemon','selection_pomegranate'):
            data = deepcopy(self.data)
            pool = data['randomization']['pool_assets']
            if fault == 'missing':pool.pop()
            if fault == 'duplicate':pool[-1] = pool[0]
            if fault in ('lemon','pomegranate'):
                pool[-1] = 'models/addons/objects/fruits_veggies/'+('lemon1' if fault == 'lemon' else fault)+'.usd'
            if fault == 'wrong_path':data['randomization']['pool_path'] = runtime.POOL_PATH
            selection = data['randomization']['object_assets']
            if fault == 'selection_missing':selection.pop()
            if fault == 'selection_duplicate':selection[-1] = selection[0]
            if fault in ('selection_lemon','selection_pomegranate'):
                selection[-1] = 'models/addons/objects/fruits_veggies/'+('lemon1' if fault == 'selection_lemon' else 'pomegranate')+'.usd'
            with self.subTest(fault=fault), tempfile.NamedTemporaryFile(mode='w', suffix='.json') as f:
                json.dump(data,f);f.flush()
                with self.assertRaisesRegex(ValueError, 'V3 requires'):
                    runtime.load_configuration(f.name, ROOT)

    def test_same_lifecycle_and_live_reset_six_actors_without_usd_edits(self):
        life = runtime.CollectionLifecycle(self.planner, Mock(return_value=Mock(phase='playing')), lambda:True)
        life.prepare(); episode = deepcopy(self.planner.last_episode)
        before = self.stage.ExportToString()
        life.play(); life.pause(); life.play()
        self.assertEqual(self.planner.last_episode, episode)
        actors = {path:LiveBody() for path in episode['body_poses']}
        view = SimpleNamespace(create_rigid_body_view=lambda path:actors[path])
        log = Mock()
        pool = pool_module.PhysicsObjectPool(self.stage, episode, log=log)
        pool.bind(view); pool.apply_episode(); pool.verify_episode()
        self.assertEqual(len(pool.bodies), 6)
        log.assert_any_call('[BFM PHYSX] things pool resolved: 5/5; container resolved: 1')
        self.assertTrue(pool.parking_settled())
        for path, actor in actors.items():
            np.testing.assert_allclose(actor.pose[0], episode['body_poses'][path], atol=1e-5)
            np.testing.assert_equal(actor.velocity, 0.)
        self.assertEqual(self.stage.ExportToString(), before)
        life.stop(); life.prepare()
        self.assertNotEqual(self.planner.last_episode['episode_id'], episode['episode_id'])

    def test_v2_composition_unchanged_outside_replaced_pool(self):
        v2 = Usd.Stage.Open(str(ROOT/'models/stages/stage_sorting_v2.usda'))
        for p in v2.Traverse():
            if str(p.GetPath()).startswith(('/World/Task/Fruits','/World/Task/Parking')):continue
            other = self.stage.GetPrimAtPath(p.GetPath())
            self.assertTrue(other, str(p.GetPath()))
            self.assertEqual(p.GetTypeName(), other.GetTypeName())
            for attr in p.GetAttributes():
                self.assertEqual(attr.Get(), other.GetAttribute(attr.GetName()).Get(), str(attr.GetPath()))
                self.assertEqual(attr.GetConnections(), other.GetAttribute(attr.GetName()).GetConnections())

    def test_extension_attaches_v3_and_uses_its_config_without_new_lifecycle(self):
        names = ('omni','omni.ext','omni.kit','omni.kit.app','omni.timeline','omni.usd','carb')
        modules = {name:ModuleType(name) for name in names}
        for name in names:
            parent,_,child = name.rpartition('.')
            if parent in modules:setattr(modules[parent],child,modules[name])
        modules['omni.ext'].IExt = object
        modules['carb'].log_info = Mock(); modules['carb'].log_error = Mock()
        timeline = Mock(); timeline.is_stopped.return_value = True
        modules['omni.timeline'].get_timeline_interface = lambda:timeline
        modules['omni.kit.app'].get_app = Mock()
        modules['omni.usd'].get_context = lambda:Mock(get_stage=lambda:self.stage)
        package = ModuleType('bfm_collection'); package.__path__=[str(collection.MODULE.parent)]
        modules['bfm_collection']=package; modules['bfm_collection.scene_randomizer']=runtime
        channel = ModuleType('isaac_eff_control.bfm_episode');channel.EpisodeChannel=Mock()
        channel.EpisodeChannel.return_value.quiescent=True
        modules['isaac_eff_control.bfm_episode']=channel
        import importlib.util
        clean={k:v for k,v in os.environ.items() if not k.startswith('BFM_')}
        with patch.dict(sys.modules,modules), patch.dict(os.environ,clean,clear=True):
            spec=importlib.util.spec_from_file_location('bfm_collection.extension',collection.MODULE.with_name('extension.py'))
            adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)
            ext=adapter.CollectionExtension()
            with patch.object(ext,'_bootstrap_ros_python_paths'), patch('isaac_eff_control.collection_graph.configure_collection_graph'):
                ext.on_startup('bfm_collection')
                self.assertIsNotNone(ext.lifecycle, modules['carb'].log_error.call_args)
                self.assertEqual(Path(ext.config_path), CONFIG)
                self.assertEqual(ext.parameters['object_prim_path'], '/World/Task/Things/apple_02')
                episode=ext.lifecycle.randomizer.last_episode
                self.assertEqual(episode['scenario'], 'things_v3')
                self.assertEqual(episode['scene_configuration']['scene'], 'models/stages/stage_sorting_v3.usda')
                ext._detach()
