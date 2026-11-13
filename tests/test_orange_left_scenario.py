"""Orange-left composition/planning with real USD; fake support/PhysX/ROS.

No physical contacts, remote tabletop geometry or live DDS are claimed here.
"""
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
import test_stage_sorting_v3 as v3
import test_v1_manipulation_scenes as existing
import test_recording_pipeline as recording
from test_physics_object_pool import LiveBody, module as physics
from isaac_eff_control.scenarios import DEFAULT, scenario, scenario_for_stage
from isaac_eff_control.bfm_contract import scene_configuration, state_defaults

ROOT = v3.ROOT
NAME = 'v1_manipulation_orange_left'
DIRECTORY = ROOT/'models/stages'/NAME
CONFIG = DIRECTORY/'sorting_scene.json'
OBJECT = '/World/Task/Things/orange2'
CONTAINER = '/World/container_b04'
ASSET = 'models/addons/objects/fruits_veggies/orange2.usd'
runtime = v3.runtime


class OrangeLeftTests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.Open(str(DIRECTORY/'stage_manipulation.usda'))
        self.data = runtime.load_configuration(CONFIG, ROOT)
        self.data['randomization']['seed'] = 7
        support = patch.object(runtime, 'tabletop_triangles', return_value=v3.TABLE)
        support.start(); self.addCleanup(support.stop)
        self.planner = runtime.SceneRandomizer(self.stage, self.data, ROOT)

    def test_single_body_composition_matches_verified_orange_and_stable_scene(self):
        self.assertEqual({p.name for p in DIRECTORY.iterdir()},
                         {'sorting_scene.json','stage_manipulation.usda','things_pool.usda'})
        self.assertEqual([str(p.GetPath()) for p in self.stage.Traverse()
                          if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith('/World/Task/')],
                         [OBJECT])
        self.assertEqual(sum(p.IsA(UsdPhysics.Scene) for p in self.stage.Traverse()), 1)
        self.assertFalse(self.stage.GetPrimAtPath('/World/Task/Fruits').IsActive())
        self.assertFalse(self.stage.GetPrimAtPath('/World/Task/Parking').IsActive())
        for name in ('lemon1','avocado','pomegranate','lunchbag','ketchup_bottle','apple_02'):
            self.assertFalse(self.stage.GetPrimAtPath('/World/Task/Things/'+name))
        before = Usd.Stage.Open(str(existing.AMBI/'stage_manipulation.usda'))
        for path in ('/World/s40_sharpa_two_cameras', CONTAINER,
                     '/World/SM_SecretaryDeskA', OBJECT, '/World/Task/ThingsParking/orange2'):
            existing.compare_prims(self, before, self.stage, path)
        for field in ('metersPerUnit','upAxis','timeCodesPerSecond','startTimeCode','endTimeCode'):
            self.assertEqual(self.stage.GetMetadata(field), before.GetMetadata(field))
        self.assertEqual(self.stage.GetRootLayer().subLayerPaths,
                         ['things_pool.usda','../v1_manipulation_one_side_right/stage_sorting_v1.usd'])
        for path in (OBJECT, CONTAINER):
            physics.PhysicsObjectPool._validate_composition(SimpleNamespace(stage=self.stage), path)
        self.assertEqual(set(self.planner.pool.values()), {OBJECT})
        self.assertEqual([p.GetName() for p in self.stage.GetPrimAtPath('/World/Task/ThingsParking').GetChildren()], ['orange2'])
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath('/World/Task/ThingsParking')):
            self.assertFalse(prim.HasAPI(UsdPhysics.RigidBodyAPI))

    def test_fixed_layout_randomizes_both_existing_regions_with_original_safety(self):
        positions, containers, yaws = set(), set(), set()
        for seed in range(3):
            self.planner.rng.seed(seed)
            for _ in range(20):
                episode = self.planner.randomize()
                self.assertEqual((episode['asset_path'],episode['asset'],episode['object_prim_path'],
                                  episode['scenario'],episode['layout']),
                                 (ASSET,'orange2.usd',OBJECT,'orange_left_v1','B'))
                self.assertEqual(set(episode['body_poses']), {OBJECT,CONTAINER})
                self.assertEqual(episode['parking_positions'], {})
                self.assertEqual(episode['container_prim_path'], CONTAINER)
                _, low, high = self.planner.assets[ASSET]
                rotation = Gf.Rotation(Gf.Vec3d(0,0,1), np.degrees(episode['yaw']))
                corners = [rotation.TransformDir(Gf.Vec3d(x,y,z))
                           for x in (low[0],high[0]) for y in (low[1],high[1]) for z in (low[2],high[2])]
                lo = [episode['position'][i]+min(p[i] for p in corners) for i in range(3)]
                hi = [episode['position'][i]+max(p[i] for p in corners) for i in range(3)]
                boxlo, boxhi = [[value+episode['container_offset'][i] for i,value in enumerate(bound)]
                                for bound in self.planner.container_bounds]
                for region, centre in ((episode['placement_regions']['left'],[(a+b)/2 for a,b in zip(lo,hi)]),
                                       (episode['placement_regions']['right'],[(a+b)/2 for a,b in zip(boxlo,boxhi)])):
                    for i,axis in enumerate(('x','y')):
                        self.assertGreaterEqual(centre[i],region[axis+'_min']-1e-9)
                        self.assertLessEqual(centre[i],region[axis+'_max']+1e-9)
                self.assertEqual(episode['placement_regions']['right'],self.data['randomization']['object_spawn_region'])
                for i,axis in enumerate(('x','y')):
                    template_centre = sum(b[i] for b in self.planner.container_bounds)/2
                    for side in ('min','max'):
                        self.assertEqual(episode['placement_regions']['left'][axis+'_'+side],
                                         template_centre+self.data['randomization']['container_offset_region'][axis+'_'+side])
                self.assertAlmostEqual(lo[2], .75+self.data['randomization']['clearance_m'])
                self.assertAlmostEqual(boxlo[2], .75, places=5)
                self.assertFalse(runtime.overlaps(lo,hi,boxlo,boxhi,self.data['randomization']['obstacle_margin_m']))
                self.assertTrue(self.data['randomization']['yaw_min'] <= episode['yaw'] <= self.data['randomization']['yaw_max'])
                positions.add(tuple(episode['position'])); containers.add(tuple(episode['container_position'])); yaws.add(episode['yaw'])
        self.assertEqual((len(positions),len(containers),len(yaws)), (60,60,60))

    def test_seed_reproducibility_and_read_only_planning(self):
        other = runtime.SceneRandomizer(self.stage,deepcopy(self.data),ROOT)
        before = self.stage.ExportToString()
        for _ in range(15):
            a,b = self.planner.randomize(),other.randomize()
            for key in ('asset_path','layout','position','yaw','container_position','body_poses'):
                self.assertEqual(a[key],b[key])
        self.assertEqual(self.stage.ExportToString(), before)

    def test_support_overlap_and_obstacle_rejections_preserve_previous_plan(self):
        previous = self.planner.randomize()
        before = self.stage.ExportToString()
        self.planner.config['max_attempts'] = 3
        with patch.object(runtime,'tabletop_triangles',return_value=[]):
            with self.assertRaisesRegex(ValueError,'No supported'):
                self.planner.randomize()
        # Even a supplied candidate inside the randomized container is rejected.
        def candidate(config,rng):
            region = self.planner.config['object_spawn_region']
            return ASSET,(region['x_min']+region['x_max'])/2,(region['y_min']+region['y_max'])/2,0.
        with patch.object(runtime,'sample_candidate',side_effect=candidate):
            with self.assertRaisesRegex(ValueError,'No supported, obstacle-free'):
                self.planner.randomize()
        self.assertEqual(self.planner.last_episode,previous)
        self.assertEqual(self.stage.ExportToString(),before)
        with Usd.EditContext(self.stage,self.stage.GetSessionLayer()):
            obstacle = UsdGeom.Cube.Define(self.stage,'/World/TestObstacle')
            obstacle.AddTranslateOp().Set((1,4.25,.8)); obstacle.AddScaleOp().Set((2,2,.3))
            obstacle.CreateSizeAttr(1); UsdPhysics.CollisionAPI.Apply(obstacle.GetPrim())
        with self.assertRaisesRegex(ValueError,'No supported'):
            self.planner.randomize()
        self.assertEqual(self.planner.last_episode,previous)

    def test_strict_configuration_rejects_wrong_objects_roles_paths_and_layouts(self):
        for fault in ('layout_A','layout_missing','layout_null','extra','duplicate','selection',
                      'pool','asset_path','object_path','container','pool_path','parking_path','scenario'):
            data = deepcopy(self.data); config=data['randomization']
            if fault=='layout_A': config['fixed_layout']='A'
            if fault=='layout_missing': config.pop('fixed_layout')
            if fault=='layout_null': config['fixed_layout']=None
            if fault=='extra':
                config['object_assets'].append('models/addons/objects/fruits_veggies/lemon1.usd')
                config['pool_assets']=list(config['object_assets'])
            if fault=='duplicate': config['pool_assets'].append(ASSET)
            if fault=='selection': config['object_assets']=[]
            if fault=='pool': config['pool_assets']=[]
            if fault=='asset_path': config['object_assets']=config['pool_assets']=['other/orange2.usd']
            if fault=='object_path': data['parameters']['object_prim_path']='/World/Task/Things/lemon1'
            if fault=='container': data['parameters']['container_prim_path']='/Wrong'
            if fault=='pool_path': config['pool_path']='/World/Task/Fruits'
            if fault=='parking_path': config['parking_path']='/World/Task/Parking'
            if fault=='scenario': config['scenario']='unknown'
            with self.subTest(fault=fault), tempfile.NamedTemporaryFile(mode='w',suffix='.json') as file:
                json.dump(data,file);file.flush()
                with self.assertRaises(ValueError):runtime.load_configuration(file.name,ROOT)

    def test_exact_two_live_actors_reset_without_usd_edits(self):
        episode = self.planner.randomize(); before=self.stage.ExportToString()
        bodies={path:LiveBody() for path in episode['body_poses']}
        view=SimpleNamespace(create_rigid_body_view=Mock(side_effect=lambda path:bodies[path]))
        log=Mock();pool=physics.PhysicsObjectPool(self.stage,episode,log)
        pool.bind(view)
        self.assertEqual(view.create_rigid_body_view.call_count,2)
        self.assertTrue(all(not body.writes for body in bodies.values()))
        pool.apply_episode();pool.verify_episode();self.assertTrue(pool.parking_settled())
        self.assertIn('things pool resolved: 1/1',log.call_args_list[0].args[0])
        for path,body in bodies.items():
            np.testing.assert_allclose(body.pose[0],episode['body_poses'][path],atol=1e-6)
            np.testing.assert_equal(body.velocity,0.)
            self.assertEqual(body.writes,['pose','velocity'])
        pool.apply_episode();pool.bind(view)
        self.assertEqual(view.create_rigid_body_view.call_count,2)
        pool.release();self.assertFalse(pool.bodies)
        self.assertEqual(self.stage.ExportToString(),before)

    def test_invalid_actor_plans_preflight_and_readback_do_not_weaken_reset(self):
        episode=self.planner.randomize()
        for fault in ('extra','wrong_object','mass','inertia','missing','readback'):
            plan=deepcopy(episode)
            bodies={path:LiveBody() for path in plan['body_poses']}
            view=SimpleNamespace(create_rigid_body_view=lambda path:bodies[path])
            if fault=='extra':plan['body_poses']['/Wrong']=[0,0,0,0,0,0,1]
            if fault=='wrong_object':plan['object_prim_path']='/Wrong'
            if fault=='mass':bodies[CONTAINER].mass[:]=0
            if fault=='inertia':bodies[CONTAINER].inertia[0,0]=-1
            if fault=='missing':bodies[CONTAINER].count=0
            if fault=='readback':bodies[OBJECT].ignore_pose=True
            with self.subTest(fault=fault),self.assertRaises(ValueError):
                pool=physics.PhysicsObjectPool(self.stage,plan,Mock())
                pool.bind(view);pool.apply_episode();pool.verify_episode()
            if fault!='readback':self.assertTrue(all(not body.writes for body in bodies.values()))

    def test_stop_plan_play_pause_resume_uses_existing_lifecycle(self):
        stopped=SimpleNamespace(value=True)
        handle=SimpleNamespace(phase='waiting_for_physics',stop=Mock())
        start=Mock(return_value=handle)
        lifecycle=runtime.CollectionLifecycle(self.planner,start,lambda:stopped.value)
        before=self.stage.ExportToString()
        first=lifecycle.prepare();self.assertEqual(lifecycle.state,'prepared')
        start.assert_not_called();self.assertEqual(self.stage.ExportToString(),before)
        stopped.value=False;lifecycle.play();start.assert_called_once()
        self.assertEqual(lifecycle.state,'waiting_for_physics')
        handle.phase='playing';lifecycle.sync();lifecycle.pause();lifecycle.play()
        self.assertIs(self.planner.last_episode,first);start.assert_called_once()
        stopped.value=True;lifecycle.stop();handle.stop.assert_called_once()
        second=lifecycle.prepare()
        self.assertNotEqual(first['episode_id'],second['episode_id'])
        self.assertNotEqual(first['position'],second['position'])
        self.assertNotEqual(first['container_position'],second['container_position'])
        self.assertEqual(second['layout'],'B');self.assertIsNone(lifecycle.prepare())
        lifecycle.close()

    def test_existing_success_uses_the_selected_orange_and_original_container(self):
        from isaac_eff_control.fruit_success import LiveFruitSuccess
        episode=self.planner.randomize()
        detector=LiveFruitSuccess(self.stage,self.data['parameters'],episode['episode_id'])
        class View:
            def __init__(self,position):self.position=position
            def get_transforms(self):return [[*self.position,0,0,0,1]]
            def get_velocities(self):return [[0.]*6]
        result=detector.measure(SimpleNamespace(views={'object_prim_path':View([100,100,2]),
                                                     'container_prim_path':View([1,4,.75])}),1.)
        self.assertEqual(result['object_prim_path'],OBJECT)
        self.assertEqual(result['episode_id'],episode['episode_id'])
        self.assertFalse(result['success']);self.assertGreater(len(detector.vertices),3)

    def test_provenance_registry_and_package_installation(self):
        entry=scenario(NAME,ROOT)
        self.assertEqual(scenario_for_stage(entry['stage'],ROOT),entry)
        self.assertEqual(DEFAULT,'sorting_experimental')
        for key,name in (('fruit_pool_sha256','things_pool.usda'),('scene_sha256','stage_manipulation.usda')):
            self.assertEqual(self.data['provenance'][key],hashlib.sha256((DIRECTORY/name).read_bytes()).hexdigest())
        with patch('setuptools.setup') as setup:
            runpy.run_path(str(ROOT/'exo2isaacsim/src/isaac_eff_control/setup.py'))
        files=dict(setup.call_args.kwargs['data_files'])
        self.assertEqual(files['share/isaac_eff_control/scenarios/'+NAME],
                         ['../../../models/stages/'+NAME+'/sorting_scene.json'])
        import isaac_eff_control.scenarios as registry
        with tempfile.TemporaryDirectory() as temporary:
            share=Path(temporary)/'share'
            installed=share/'scenarios'/NAME/'sorting_scene.json'
            for name in registry.SCENARIOS:
                source=registry.scenario(name,ROOT)['config']
                destination=share/'scenarios'/name/source.name
                destination.parent.mkdir(parents=True);destination.write_bytes(source.read_bytes())
            packages=SimpleNamespace(get_package_share_directory=lambda _:str(share))
            with patch.object(registry,'__file__',str(Path(temporary)/'install/scenarios.py')), \
                    patch.dict(sys.modules,{'ament_index_python.packages':packages}):
                selected=registry.scenario(NAME)
                self.assertIsNone(selected['stage'])
                self.assertEqual(selected['config'],installed)
                self.assertEqual(hashlib.sha256(installed.read_bytes()).hexdigest(),
                                 scene_configuration(CONFIG)['sha256'])
                self.assertEqual(scene_configuration(installed)['scenario'],NAME)

    def test_extension_attaches_new_scene_and_prepares_while_stopped(self):
        names=('omni','omni.ext','omni.kit','omni.kit.app','omni.timeline','omni.usd','carb')
        modules={name:ModuleType(name) for name in names}
        for name in names:
            parent,_,child=name.rpartition('.')
            if parent in modules:setattr(modules[parent],child,modules[name])
        modules['omni.ext'].IExt=object
        modules['carb'].log_info=Mock();modules['carb'].log_error=Mock()
        timeline=Mock();timeline.is_stopped.return_value=True
        modules['omni.timeline'].get_timeline_interface=lambda:timeline
        modules['omni.kit.app'].get_app=Mock()
        modules['omni.usd'].get_context=lambda:Mock(get_stage=lambda:self.stage)
        package=ModuleType('bfm_collection');package.__path__=[str(v3.collection.MODULE.parent)]
        modules['bfm_collection']=package;modules['bfm_collection.scene_randomizer']=runtime
        channel=ModuleType('isaac_eff_control.bfm_episode');channel.EpisodeChannel=Mock()
        channel.EpisodeChannel.return_value.quiescent=True
        modules['isaac_eff_control.bfm_episode']=channel
        clean={k:v for k,v in os.environ.items() if not k.startswith('BFM_')}
        with patch.dict(sys.modules,modules),patch.dict(os.environ,clean,clear=True):
            spec=importlib.util.spec_from_file_location('bfm_collection.extension',v3.collection.MODULE.with_name('extension.py'))
            adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)
            ext=adapter.CollectionExtension()
            with patch.object(ext,'_bootstrap_ros_python_paths'),patch('isaac_eff_control.collection_graph.configure_collection_graph'):
                ext.on_startup('bfm_collection')
                try:
                    self.assertIsNotNone(ext.lifecycle,modules['carb'].log_error.call_args)
                    self.assertEqual(ext.lifecycle.state,'prepared')
                    self.assertEqual(Path(ext.config_path),CONFIG)
                    self.assertEqual(ext.parameters['object_prim_path'],OBJECT)
                    episode=ext.lifecycle.randomizer.last_episode
                    self.assertEqual((episode['scenario'],episode['layout']),('orange_left_v1','B'))
                    self.assertEqual(episode['scene_configuration']['scenario'],NAME)
                    timeline.play.assert_not_called()
                finally:ext._detach()

    def test_historical_seeded_plans_match_the_pushed_baseline_planner(self):
        path='exts/bfm_collection/bfm_collection/scene_randomizer.py'
        source=subprocess.check_output(['git','show','HEAD:'+path],cwd=ROOT)
        baseline=ModuleType('baseline_randomizer');exec(compile(source,path,'exec'),baseline.__dict__)
        with patch.object(baseline,'tabletop_triangles',return_value=v3.TABLE):
            for name in ('sorting_experimental','v1_manipulation_ambidextrous','v1_manipulation_one_side_right'):
                entry=scenario(name,ROOT);stage=Usd.Stage.Open(str(entry['stage']))
                data=runtime.load_configuration(entry['config'],ROOT);data['randomization']['seed']=42
                a=runtime.SceneRandomizer(stage,deepcopy(data),ROOT)
                b=baseline.SceneRandomizer(stage,deepcopy(data),ROOT)
                for _ in range(8):
                    x,y=a.randomize(),b.randomize()
                    for key in ('asset_path','position','yaw','container_position','body_poses'):
                        self.assertEqual(x[key],y[key],(name,key))
                    self.assertEqual(x.get('layout'),y.get('layout'))


class OrangeRecorderTests(unittest.TestCase):
    setUp=recording.RecordingPipelineTests.setUp

    def test_authoritative_episode_metadata_without_shared_terminal_environment(self):
        self.overrides['bfm_collection_mode']=True
        node=self.recorder.SimRecorder()
        stage=Usd.Stage.Open(str(DIRECTORY/'stage_manipulation.usda'))
        data=runtime.load_configuration(CONFIG,ROOT)
        with patch.object(runtime,'tabletop_triangles',return_value=v3.TABLE):
            episode=runtime.SceneRandomizer(stage,data,ROOT).randomize()
        episode.update(scene_configuration=scene_configuration(CONFIG),state_parameters=state_defaults(CONFIG))
        node._episode_status=dict(phase='playing',episode=episode)
        metadata=node.episode_metadata();bfm=metadata['bfm']
        self.assertEqual((bfm['contract_version'],bfm['control_architecture'],bfm['policy_hz'],bfm['action_dim']),
                         (4,'decoupled_teleoperation_v1',15,26))
        self.assertEqual(bfm['control']['hand_hz'],50)
        self.assertEqual(bfm['episode'],episode)
        self.assertEqual(bfm['episode']['scenario'],'orange_left_v1')
        self.assertEqual(bfm['episode']['layout'],'B')
        self.assertEqual(bfm['task_state']['object_prim_path'],OBJECT)
        self.assertEqual(bfm['task_state']['container_prim_path'],CONTAINER)
        self.assertEqual(bfm['task_state']['scene_configuration']['scenario'],NAME)
        self.assertEqual(bfm['task_state']['scene_configuration']['scene'],str((DIRECTORY/'stage_manipulation.usda').relative_to(ROOT)))
        self.assertEqual(metadata['recording']['topics'],self.recorder.DEFAULT_TOPICS)


if __name__=='__main__':unittest.main()
