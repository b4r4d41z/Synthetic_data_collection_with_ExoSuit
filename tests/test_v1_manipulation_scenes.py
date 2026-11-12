"""Composed scene migration checks; mocked support/reset is not live PhysX."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import unittest
from unittest.mock import patch

from pxr import Sdf, Usd
import test_stage_sorting_v3 as v3

ROOT = v3.ROOT
BASE = 'c7408e7c276b9f515f1f912064eaf211ff570e8b'
STABLE = ROOT/'models/stages/v1_manipulation_one_side_right'
AMBI = ROOT/'models/stages/v1_manipulation_ambidextrous'
EXPERIMENTAL = ROOT/'models/stages/sorting_experimental'


def original(path):
    return subprocess.check_output(['git', 'show', BASE+':'+path], cwd=ROOT)


def compare_prims(test, before, after, root='/World'):
    a = {str(p.GetPath()): p for p in Usd.PrimRange(
        before.GetPrimAtPath(root), Usd.TraverseInstanceProxies())}
    b = {str(p.GetPath()): p for p in Usd.PrimRange(
        after.GetPrimAtPath(root), Usd.TraverseInstanceProxies())}
    test.assertEqual(set(a), set(b))
    for path, p in a.items():
        q = b[path]
        test.assertEqual(p.GetTypeName(), q.GetTypeName(), path)
        test.assertEqual(p.GetAppliedSchemas(), q.GetAppliedSchemas(), path)
        test.assertEqual([x.GetName() for x in p.GetAttributes()],
                         [x.GetName() for x in q.GetAttributes()], path)
        for attr in p.GetAttributes():
            other = q.GetAttribute(attr.GetName())
            test.assertEqual(attr.Get(), other.Get(), str(attr.GetPath()))
            test.assertEqual(attr.GetConnections(), other.GetConnections())
        test.assertEqual([(r.GetName(), r.GetTargets()) for r in p.GetRelationships()],
                         [(r.GetName(), r.GetTargets()) for r in q.GetRelationships()])


class PreservationTests(unittest.TestCase):
    def test_exactly_three_directories_without_compatibility_aliases(self):
        stages = ROOT/'models/stages'
        self.assertEqual({p.name for p in stages.iterdir() if p.is_dir()},
                         {'sorting_experimental', STABLE.name, AMBI.name})
        self.assertFalse((stages/'sorting_v1').exists())
        self.assertFalse((stages/'sorting_v1').is_symlink())
        self.assertFalse(any(p.is_symlink() for p in stages.rglob('*')))
        from isaac_eff_control.scenarios import SCENARIOS, scenario
        self.assertEqual(set(SCENARIOS), {p.name for p in stages.iterdir()})
        with self.assertRaises(ValueError):
            scenario('sorting_v1', ROOT)

    def test_stable_bytes_configuration_and_mechanical_behavior_preserved(self):
        for name in ('stage_sorting_v1.usd', 'fruit_pool.usda'):
            self.assertEqual((STABLE/name).read_bytes(),
                             original('models/stages/sorting_v1/'+name))
        before = json.loads(original('models/stages/sorting_v1/sorting_scene.json'))
        after = json.loads((STABLE/'sorting_scene.json').read_text())
        self.assertEqual(before['randomization'], after['randomization'])
        self.assertEqual(before['parameters'], after['parameters'])
        self.assertEqual(after['provenance']['scene_sha256'],
                         hashlib.sha256((STABLE/'stage_sorting_v1.usd').read_bytes()).hexdigest())
        stable = Usd.Stage.Open(str(STABLE/'stage_sorting_v1.usd'))
        ambi = Usd.Stage.Open(str(AMBI/'stage_manipulation.usda'))
        for stage in (stable, ambi):
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                stage.GetPrimAtPath('/World/s40_sharpa_two_cameras').GetVariantSet('Physics').SetVariantSelection('physx')
        compare_prims(self, stable, ambi, '/World/s40_sharpa_two_cameras')

    def test_experimental_edits_are_only_reference_paths_and_pool_checksum(self):
        for name in ('stage_sorting_v2.usda', 'things_pool.usda'):
            expected = original('models/stages/sorting_experimental/'+name).decode().replace(
                '../sorting_v1/', '../v1_manipulation_one_side_right/')
            self.assertEqual((EXPERIMENTAL/name).read_text(), expected)
        self.assertEqual((EXPERIMENTAL/'stage_sorting_v3.usda').read_bytes(),
                         original('models/stages/sorting_experimental/stage_sorting_v3.usda'))
        before = json.loads(original('models/stages/sorting_experimental/sorting_scene_v3.json'))
        after = json.loads((EXPERIMENTAL/'sorting_scene_v3.json').read_text())
        before['provenance']['fruit_pool_sha256'] = hashlib.sha256(
            (EXPERIMENTAL/'things_pool.usda').read_bytes()).hexdigest()
        self.assertEqual(after, before)
        for p in (ROOT/'models/stages').rglob('*.usda'):
            self.assertNotIn('../sorting_v1/', p.read_text(), str(p))

    def test_composed_experimental_scene_matches_baseline(self):
        # Anchor baseline source references explicitly in anonymous test layers.
        # Only relocate the removed stable directory; never alter a scene on disk.
        layers = []
        for name in ('things_pool.usda', 'stage_sorting_v2.usda', 'stage_sorting_v3.usda'):
            text = original('models/stages/sorting_experimental/'+name).decode()
            def anchor(match):
                asset = match.group(1).replace('../sorting_v1/', '../v1_manipulation_one_side_right/')
                return '@'+str((EXPERIMENTAL/asset).resolve())+'@'
            layer = Sdf.Layer.CreateAnonymous(name)
            layer.ImportFromString(re.sub(r'@([^@]+)@', anchor, text))
            layers.append(layer)
        layers[2].subLayerPaths = [layers[0].identifier, layers[1].identifier]
        baseline = Usd.Stage.Open(layers[2])
        current = Usd.Stage.Open(str(EXPERIMENTAL/'stage_sorting_v3.usda'))
        compare_prims(self, baseline, current)

    def test_independent_four_object_pool_preserves_other_randomization_settings(self):
        a = json.loads((AMBI/'sorting_scene.json').read_text())
        b = json.loads((EXPERIMENTAL/'sorting_scene_v3.json').read_text())
        expected = {'lemon1','orange2','lunchbag','ketchup_bottle'}
        self.assertEqual({Path(x).stem for x in a['randomization']['object_assets']}, expected)
        self.assertEqual(a['randomization']['object_assets'], a['randomization']['pool_assets'])
        for key in b['randomization']:
            if key not in ('object_assets','pool_assets','collection_objects_note','scenario'):
                self.assertEqual(a['randomization'][key], b['randomization'][key], key)
        self.assertEqual(a['parameters']['object_prim_path'],'/World/Task/Things/lemon1')
        for key in b['parameters']:
            if key != 'object_prim_path': self.assertEqual(a['parameters'][key], b['parameters'][key])
        self.assertEqual(a['provenance']['fruit_pool_asset'], 'models/stages/'+AMBI.name+'/things_pool.usda')
        for key, file in [('fruit_pool_sha256','things_pool.usda'),('scene_sha256','stage_manipulation.usda')]:
            self.assertEqual(a['provenance'][key], hashlib.sha256((AMBI/file).read_bytes()).hexdigest())
        self.assertNotIn('sorting_experimental', (AMBI/'things_pool.usda').read_text())
        self.assertIn('@things_pool.usda@', (AMBI/'stage_manipulation.usda').read_text())
        self.assertGreater(Usd.Stage.Open(str(AMBI/'stage_manipulation.usda')).GetEndTimeCode(), 0)


class AmbidextrousTests(unittest.TestCase):
    object_names = ('lemon1','orange2','lunchbag','ketchup_bottle')
    def setUp(self):
        self.stage = Usd.Stage.Open(str(AMBI/'stage_manipulation.usda'))
        self.data = v3.runtime.load_configuration(AMBI/'sorting_scene.json', ROOT)
        self.data['randomization']['seed'] = 7
        support = patch.object(v3.runtime, 'tabletop_triangles', return_value=v3.TABLE)
        support.start(); self.addCleanup(support.stop)
        self.planner = v3.runtime.SceneRandomizer(self.stage, self.data, ROOT)

    # Run the existing composition, placement, rejection and reset assertions on
    # the new scene instead of duplicating or changing the production algorithm.
    test_pool = v3.SortingV3Tests.test_five_preloaded_bodies_geometry_mass_and_textures
    test_layouts = v3.SortingV3Tests.test_both_layouts_all_objects_and_independent_random_positions
    test_reproducibility = v3.SortingV3Tests.test_seed_reproducibility_and_v2_region_definitions
    test_invalid_placements = v3.SortingV3Tests.test_unsupported_and_environment_obstacle_placements_fail_without_changing_plan
    test_reset = v3.SortingV3Tests.test_same_lifecycle_and_live_reset_six_actors_without_usd_edits

    def test_local_pool_rejects_extra_missing_and_wrong_object_identities(self):
        from copy import deepcopy
        import tempfile
        for fault in ('missing','duplicate','apple','avocado'):
            data = deepcopy(self.data)
            for key in ('object_assets','pool_assets'):
                assets = data['randomization'][key]
                if fault == 'missing': assets.pop()
                elif fault == 'duplicate': assets[-1] = assets[0]
                elif fault == 'apple': assets[-1] = 'models/addons/objects/objaverse/apple_02.usd'
                else: assets[-1] = 'models/addons/objects/fruits_veggies/avocado.usd'
            with self.subTest(fault=fault), tempfile.NamedTemporaryFile(mode='w',suffix='.json') as f:
                json.dump(data,f);f.flush()
                with self.assertRaisesRegex(ValueError,'Ambidextrous V4 requires'):
                    v3.runtime.load_configuration(f.name, ROOT)

    def test_success_binds_each_selected_local_object(self):
        from isaac_eff_control.fruit_success import LiveFruitSuccess
        from types import SimpleNamespace
        class View:
            def __init__(self, position): self.position = position
            def get_transforms(self): return [[*self.position,0,0,0,1]]
            def get_velocities(self): return [[0.]*6]
        for asset,path in self.planner.pool.items():
            detector = LiveFruitSuccess(self.stage,dict(object_prim_path=path,
                container_prim_path='/World/container_b04'),'episode-'+Path(asset).stem)
            result = detector.measure(SimpleNamespace(views={
                'object_prim_path': View([100,100,2]),
                'container_prim_path': View([1,4,.75])}), 1.)
            self.assertEqual(result['object_prim_path'],path)
            self.assertEqual(result['episode_id'],'episode-'+Path(asset).stem)
            self.assertFalse(result['success'])
            self.assertGreater(len(detector.vertices),3)

    def test_seeded_plans_match_shared_bidirectional_randomizer(self):
        stage = Usd.Stage.Open(str(EXPERIMENTAL/'stage_sorting_v3.usda'))
        data = v3.runtime.load_configuration(AMBI/'sorting_scene.json', ROOT)
        data['randomization']['seed'] = 7
        stage = Usd.Stage.Open(str(AMBI/'stage_manipulation.usda'))
        other = v3.runtime.SceneRandomizer(stage, data, ROOT)
        for _ in range(30):
            a, b = self.planner.randomize(), other.randomize()
            for key in ('asset', 'layout', 'position', 'yaw', 'container_position', 'body_poses'):
                self.assertEqual(a[key], b[key], key)
