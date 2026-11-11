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

    def test_same_experimental_pool_and_randomization_configuration(self):
        a = json.loads((AMBI/'sorting_scene.json').read_text())
        b = json.loads((EXPERIMENTAL/'sorting_scene_v3.json').read_text())
        self.assertEqual(a['randomization'], b['randomization'])
        self.assertEqual(a['parameters'], b['parameters'])
        self.assertEqual(a['provenance']['fruit_pool_asset'], b['provenance']['fruit_pool_asset'])
        self.assertEqual(a['provenance']['fruit_pool_sha256'], b['provenance']['fruit_pool_sha256'])
        self.assertEqual(a['provenance']['scene_sha256'],
                         hashlib.sha256((AMBI/'stage_manipulation.usda').read_bytes()).hexdigest())


class AmbidextrousTests(unittest.TestCase):
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

    def test_seeded_plans_match_existing_experimental_randomizer(self):
        stage = Usd.Stage.Open(str(EXPERIMENTAL/'stage_sorting_v3.usda'))
        data = v3.runtime.load_configuration(EXPERIMENTAL/'sorting_scene_v3.json', ROOT)
        data['randomization']['seed'] = 7
        other = v3.runtime.SceneRandomizer(stage, data, ROOT)
        for _ in range(30):
            a, b = self.planner.randomize(), other.randomize()
            for key in ('asset', 'layout', 'position', 'yaw', 'container_position', 'body_poses'):
                self.assertEqual(a[key], b[key], key)
