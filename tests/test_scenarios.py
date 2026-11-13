"""Scene/config identity and launcher tests; no live Isaac or DDS claims."""
from copy import deepcopy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
import numpy  # Load the native module outside temporary ROS sys.modules patches.
from unittest.mock import patch

import test_collection_launch as launches
import test_recording_pipeline as recording
ROOT = recording.ROOT
sys.path.insert(0, str(ROOT/'exo2isaacsim/src/isaac_eff_control'))
from isaac_eff_control.scenarios import DEFAULT, SCENARIOS, scenario, scenario_for_stage
from isaac_eff_control.bfm_contract import scene_configuration, state_defaults

ROOT = recording.ROOT


class SelectorTests(unittest.TestCase):
    setUp = launches.LaunchTests.setUp
    executable = launches.LaunchTests.executable

    def launch(self, argument=None, environment=None):
        isaac = self.executable('isaac-sim.sh', '#!/bin/bash\nprintf "ENV:%s:%s\\n" "$SCENARIO" "$BFM_SCENE_CONFIG"\nprintf "%s\\n" "$@"\n')
        script=self.root/'launch_bfm_isaac.sh'
        script.write_text(script.read_text().replace('/isaac-sim/isaac-sim.sh',str(isaac)))
        env={k:v for k,v in self.env.items() if k != 'SCENARIO'}
        if environment is not None:env['SCENARIO']=environment
        return subprocess.run(['bash',str(script),*([argument] if argument is not None else [])],
                              env=env,text=True,capture_output=True)

    def test_environment_argument_and_default_select_the_same_fixed_registry(self):
        for name in SCENARIOS:
            for argument,environment in ((name,None),(None,name),(name,name)):
                result=self.launch(argument,environment)
                self.assertEqual(result.returncode,0,result.stderr)
                self.assertIn('[BFM] Scenario: '+name,result.stdout)
                expected=scenario(name,self.root)
                self.assertIn(str(expected['stage']),result.stdout)
                self.assertIn(str(expected['config']),result.stdout)
                self.assertIn(f"ENV:{name}:{expected['config']}",result.stdout)
                self.assertNotIn('--play',result.stdout)
        self.assertIn('[BFM] Scenario: '+DEFAULT,self.launch().stdout)

    def test_unknown_paths_empty_names_and_conflicting_selectors_fail(self):
        for arg,env in (('unknown',None),('../sorting_v1',None),('',None),
                        ('v1_manipulation_one_side_right','sorting_experimental')):
            result=self.launch(arg,env)
            self.assertNotEqual(result.returncode,0)
            self.assertNotIn('Starting Isaac Sim',result.stdout)
        self.assertIn('conflicts',self.launch('v1_manipulation_one_side_right','sorting_experimental').stderr)

    def test_missing_or_mismatched_configuration_fails_before_isaac(self):
        config=scenario('v1_manipulation_one_side_right',self.root)['config']
        content=json.loads(config.read_text());content['scene']='wrong_scene.usd'
        config.write_text(json.dumps(content))
        self.assertIn('mismatch',self.launch('v1_manipulation_one_side_right').stderr)
        config.unlink()
        self.assertIn('Missing scenario configuration',self.launch('v1_manipulation_one_side_right').stderr)


class RegistryTests(unittest.TestCase):
    def test_exact_stage_config_mapping_and_no_filename_guess(self):
        for name in SCENARIOS:
            entry=scenario(name,ROOT)
            self.assertEqual(entry['config'].parent,entry['stage'].parent)
            self.assertEqual(scenario_for_stage(entry['stage'],ROOT),entry)
            self.assertIsNone(scenario_for_stage(ROOT/'unregistered'/entry['stage'].name,ROOT))
            data=json.loads(entry['config'].read_text())
            self.assertEqual(data['scene'],entry['scene'])
        stable=json.loads(scenario('v1_manipulation_one_side_right',ROOT)['config'].read_text())['randomization']
        self.assertEqual({Path(p).name for p in stable['object_assets']},{'avocado.usd','lemon1.usd','orange2.usd'})
        self.assertEqual(stable['object_spawn_region'],dict(x_min=.95,x_max=1.025,y_min=3.99,y_max=4.12))
        experimental=json.loads(scenario('sorting_experimental',ROOT)['config'].read_text())['randomization']
        self.assertEqual(experimental['scenario'],'things_v3')
        self.assertEqual(len(experimental['object_assets']),5)

    def test_installed_configuration_fallback(self):
        import isaac_eff_control.scenarios as registry
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for name,(_,filename) in SCENARIOS.items():
                target=root/'share/scenarios'/name/filename
                target.parent.mkdir(parents=True);shutil.copy(scenario(name,ROOT)['config'],target)
            packages=SimpleNamespace(get_package_share_directory=lambda _:str(root/'share'))
            with patch.object(registry,'__file__',str(root/'install/scenarios.py')), \
                    patch.dict(sys.modules,{'ament_index_python.packages':packages}):
                entry=registry.scenario('sorting_experimental')
                self.assertIsNone(entry['stage'])
                self.assertEqual(entry['config'],root/'share/scenarios/sorting_experimental/sorting_scene_v3.json')


class RecorderScenarioTests(unittest.TestCase):
    setUp=recording.RecordingPipelineTests.setUp

    def test_active_episode_controls_metadata_without_shared_shell_environment(self):
        self.overrides['bfm_collection_mode']=True
        with patch.dict(os.environ,{'SCENARIO':'v1_manipulation_one_side_right'}):
            node=self.recorder.SimRecorder()
        for name in SCENARIOS:
            entry=scenario(name,ROOT)
            state=state_defaults(entry['config'])
            episode=dict(episode_id='test-'+name,object_prim_path=state['object_prim_path'],
                         container_prim_path=state['container_prim_path'],state_parameters=state,
                         scene_configuration=scene_configuration(entry['config']))
            # Sender and receiver may use different source/install file paths.
            episode['scene_configuration']['file']='/another/process/config.json'
            node._episode_status=dict(phase='playing',episode=episode)
            metadata=node.episode_metadata()['bfm']['task_state']
            self.assertEqual(metadata['scene_configuration']['scenario'],name)
            self.assertEqual(metadata['scene_configuration']['scene'],entry['scene'])
            self.assertEqual(metadata['object_prim_path'],state['object_prim_path'])
            bad=deepcopy(episode);bad['scene_configuration']['sha256']='outdated-install'
            node._episode_status=dict(phase='playing',episode=bad)
            with self.assertRaisesRegex(ValueError,'rebuild'):node.episode_metadata()
            bad=deepcopy(episode);bad['state_parameters']['container_prim_path']='/Wrong'
            node._episode_status=dict(phase='playing',episode=bad)
            with self.assertRaisesRegex(ValueError,'roles'):node.episode_metadata()
            bad=deepcopy(episode);bad.pop('state_parameters')
            node._episode_status=dict(phase='playing',episode=bad)
            with self.assertRaisesRegex(ValueError,'runtime task parameters'):node.episode_metadata()

class CompositionTests(unittest.TestCase):
    def test_local_sublayers_references_and_robot_topology(self):
        from pxr import Sdf, Usd, UsdPhysics, UsdUtils
        for name in SCENARIOS:
            entry=scenario(name,ROOT)
            stage=Usd.Stage.Open(str(entry['stage']))
            self.assertTrue(stage)
            for layer in stage.GetUsedLayers():
                if not layer.realPath:continue
                for paths in UsdUtils.ExtractExternalReferences(layer.realPath):
                    for asset in paths:
                        if asset.startswith(('http:','https:','omniverse:')):continue
                        if asset == 'OmniPBR.mdl':continue  # Isaac's built-in MDL search path, not a moved file.
                        self.assertTrue(Path(Sdf.ComputeAssetPathRelativeToLayer(layer,asset)).is_file(),asset)
            self.assertEqual(sum(p.IsA(UsdPhysics.Scene) for p in stage.Traverse()),1)
            body=stage.GetPrimAtPath('/World/s40_sharpa_two_cameras')
            # The stable layer intentionally keeps its original unsaved variant;
            # select PhysX only in the test session for topology comparison.
            if name!='sorting_experimental':
                self.assertEqual(body.GetVariantSet('Physics').GetVariantSelection(),'')
                with Usd.EditContext(stage,stage.GetSessionLayer()):
                    body.GetVariantSet('Physics').SetVariantSelection('physx')
            prims=list(Usd.PrimRange(body,Usd.TraverseInstanceProxies()))
            self.assertEqual(sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in prims),81)
            self.assertEqual(sum(p.IsA(UsdPhysics.Joint) for p in prims),81)
            self.assertEqual(sum(p.HasAPI(UsdPhysics.CollisionAPI) and
                                 UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() for p in prims),127)
            self.assertEqual(stage.GetRootLayer().subLayerPaths,
                             (['fruit_pool.usda'] if name=='v1_manipulation_one_side_right' else
                              ['things_pool.usda','../v1_manipulation_one_side_right/stage_sorting_v1.usd']
                              if name in ('v1_manipulation_ambidextrous','v1_manipulation_orange_left')
                              else ['things_pool.usda','stage_sorting_v2.usda']))
