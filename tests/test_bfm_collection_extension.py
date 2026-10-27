"""Real USD composition, fake timeline/ROS lifecycle; no live PhysX claim."""

from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path
import random
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/sorting_scene.json"
MODULE = ROOT / "exts/bfm_collection/bfm_collection/scene_randomizer.py"
spec = importlib.util.spec_from_file_location("bfm_scene_randomizer_test", MODULE)
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.data = runtime.load_configuration(CONFIG, ROOT)

    def test_exact_whitelist_semantic_path_and_separate_state_parameters(self):
        self.assertEqual([Path(p).name for p in self.data["randomization"]["object_assets"]],
                         ["avocado.usd", "lemon1.usd", "pomegranate.usd", "orange2.usd"])
        self.assertEqual(self.data["parameters"]["object_prim_path"], runtime.OBJECT_PATH)
        self.assertNotIn("randomization", self.data["parameters"])
        self.assertEqual(self.data["parameters"]["container_prim_path"], "/World/container_b04")
        self.assertNotIn("orange1", CONFIG.read_text())

    def test_seeded_sampling_is_repeatable_bounded_and_not_reseeded_each_episode(self):
        c = self.data["randomization"]
        a, b = random.Random(42), random.Random(42)
        samples = [runtime.sample_candidate(c, a) for _ in range(30)]
        self.assertEqual(samples, [runtime.sample_candidate(c, b) for _ in range(30)])
        self.assertGreater(len(set(samples)), 1)
        self.assertEqual(set(s[0] for s in samples), set(c["object_assets"]))
        r = c["object_spawn_region"]
        for _, x, y, yaw in samples:
            self.assertTrue(r["x_min"] <= x <= r["x_max"])
            self.assertTrue(r["y_min"] <= y <= r["y_max"])
            self.assertTrue(c["yaw_min"] <= yaw <= c["yaw_max"])

    def test_bad_config_fails_before_stage_edits(self):
        import tempfile
        for key, value in (("object_assets", ["missing.usd"]*4),
                           ("seed", True), ("clearance_m", -.1),
                           ("container_offset_region", dict(x_min=1, x_max=0, y_min=0, y_max=1)),
                           ("object_spawn_region", dict(x_min=1, x_max=0, y_min=0, y_max=1)),
                           ("object_spawn_region", dict(x_min=0, x_max=float('nan'), y_min=0, y_max=1))):
            data = deepcopy(self.data)
            data["randomization"][key] = value
            with tempfile.TemporaryDirectory() as tmp:
                p = Path(tmp)/"scene.json"
                p.write_text(json.dumps(data))
                with self.subTest(key=key), self.assertRaises(ValueError):
                    runtime.load_configuration(p, ROOT)


class LifecycleTests(unittest.TestCase):
    def test_prepare_play_pause_stop_and_duplicates(self):
        r, start = Mock(), Mock()
        stopped = Mock(return_value=True)
        life = runtime.CollectionLifecycle(r, start, stopped)
        self.assertTrue(life.pending_prepare)
        life.prepare(); life.prepare()
        self.assertEqual(life.state, "prepared")
        r.randomize.assert_called_once()
        start.assert_not_called()
        stopped.return_value = False
        life.play(); life.play(); life.pause(); life.play()
        self.assertEqual(life.state, "playing")
        r.randomize.assert_called_once()
        start.assert_called_once()
        life.stop(); life.stop()
        start.return_value.stop.assert_called_once()
        self.assertTrue(life.pending_prepare)
        r.randomize.assert_called_once()  # STOP does not compose anything.
        with self.assertRaisesRegex(RuntimeError, "stopped timeline"):
            life.prepare()
        with self.assertRaisesRegex(RuntimeError, "prepared before Play"):
            life.play()
        stopped.return_value = True
        life.prepare()
        life.stop()  # Late duplicate STOP must not consume the prepared episode.
        life.prepare()
        self.assertEqual(r.randomize.call_count, 2)
        self.assertEqual(life.state, "prepared")
        life.play()
        self.assertEqual(start.call_count, 2)
        life.close()
        r.close.assert_called_once()
        self.assertFalse(life.pending_prepare)

    def test_failed_composition_never_starts_or_reuses_publisher(self):
        r, start = Mock(), Mock()
        r.randomize.side_effect = ValueError("no valid body")
        life = runtime.CollectionLifecycle(r, start, lambda: True)
        with self.assertRaises(ValueError): life.prepare()
        self.assertEqual(life.state, "failed")
        self.assertFalse(life.pending_prepare)
        start.assert_not_called()
        with self.assertRaises(RuntimeError): life.play()
        life.stop()
        r.randomize.side_effect = None
        life.prepare(); life.play()
        start.assert_called_once()

@unittest.skipUnless(importlib.util.find_spec("pxr"), "Requires OpenUSD; not live PhysX")
class CompositionTests(unittest.TestCase):
    def setUp(self):
        from pxr import Usd, UsdGeom, UsdPhysics
        self.Usd, self.UsdGeom, self.UsdPhysics = Usd, UsdGeom, UsdPhysics
        self.data = runtime.load_configuration(CONFIG, ROOT)
        self.data["randomization"]["seed"] = 9
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.)
        UsdGeom.SetStageUpAxis(self.stage, "Z")
        UsdGeom.Xform.Define(self.stage, "/World")
        for path in (self.data["parameters"]["container_prim_path"], self.data["provenance"]["robot_root"]):
            root = UsdGeom.Xform.Define(self.stage, path)
            UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
            root.AddTranslateOp().Set((1.09, 4.44, .8) if path.endswith("container_b04")
                                      else (4, 4, 0))
            cube = UsdGeom.Cube.Define(self.stage, path+"/Collider")
            cube.CreateSizeAttr(.1)
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        # Synthetic table fixture: source table is an inaccessible external USD.
        mesh = UsdGeom.Mesh.Define(self.stage, self.data["randomization"]["table_prim_path"]+"/Top")
        mesh.CreatePointsAttr([(0,3,.75),(2,3,.75),(2,5,.75),(0,5,.75)])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0,1,2,3])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        self.stage.GetRootLayer().subLayerPaths.append(str(ROOT / "models/stages/fruit_pool.usda"))
        self.container_before = self.stage.GetRootLayer().ExportToString()
        self.randomizer = runtime.SceneRandomizer(self.stage, self.data, ROOT)
        self.addCleanup(self.randomizer.close)

    def test_episode_exists_before_play_and_play_pause_never_mutate_usd(self):
        stopped = Mock(return_value=True)
        start = Mock()
        life = runtime.CollectionLifecycle(self.randomizer, start, stopped)
        life.prepare()
        self.assertTrue(self.stage.GetPrimAtPath(runtime.OBJECT_PATH))
        before = self.randomizer.layer.ExportToString()
        episode = deepcopy(self.randomizer.last_episode)
        stopped.return_value = False
        with patch.object(self.randomizer, "randomize", side_effect=AssertionError("PLAY composition")):
            life.play(); life.play(); life.pause(); life.play(); life.stop()
            self.assertEqual(self.randomizer.layer.ExportToString(), before)
            self.assertEqual(self.randomizer.last_episode, episode)
        stopped.return_value = True
        life.prepare()
        self.assertNotEqual(self.randomizer.last_episode, episode)
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), self.container_before)
        life.close()

    def test_inspected_default_prims_bounds_colliders_and_dependencies(self):
        from pxr import UsdUtils
        expected = ["/avocado01", "/lemon_01", "/pomegranate01", "/orange_02"]
        lowest = [.0001744633773341775, .0011238913284614682,
                  -.00021009051124565303, -.00005986162432236597]
        for asset, default, z in zip(self.data["randomization"]["object_assets"], expected, lowest):
            path = ROOT/asset
            prim, lo, hi = runtime.asset_geometry(path)
            self.assertEqual(prim, default)
            self.assertAlmostEqual(lo[2], z)
            layers, assets, missing = UsdUtils.ComputeAllDependencies(str(path))
            self.assertFalse(missing)
            self.assertTrue(all(Path(p).is_file() for p in assets))
            self.assertTrue(all(Path(p).resolve().is_relative_to(ROOT) for p in assets))

    def test_repeated_selection_retains_all_four_bodies_and_only_changes_properties(self):
        from pxr import Sdf
        paths = self.data["randomization"]["object_assets"]
        for asset in paths + paths:
            with patch.object(runtime, "sample_candidate", return_value=(asset, 1., 4.05, .4)):
                episode = self.randomizer.randomize()
            prim = self.stage.GetPrimAtPath(self.randomizer.current_object_path)
            self.assertTrue(prim.HasAPI(self.UsdPhysics.RigidBodyAPI))
            bodies = [p for p in self.Usd.PrimRange(prim) if p.HasAPI(self.UsdPhysics.RigidBodyAPI)]
            self.assertEqual([str(p.GetPath()) for p in bodies], [self.randomizer.current_object_path])
            self.assertFalse(self.stage.GetPrimAtPath("/World/orange1"))
            ops = self.UsdGeom.Xformable(prim).GetOrderedXformOps()
            self.assertEqual(len(ops), 1)
            bound = self.UsdGeom.BBoxCache(self.Usd.TimeCode.Default(), ["default", "render", "proxy"]).ComputeWorldBound(prim).ComputeAlignedRange()
            self.assertAlmostEqual(bound.GetMin()[2], .755)
            self.assertEqual(episode["asset"], Path(asset).name)
            self.assertEqual(self.stage.GetRootLayer().ExportToString(), self.container_before)
        self.randomizer.close()
        self.assertTrue(self.stage.GetPrimAtPath(runtime.OBJECT_PATH))
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), self.container_before)

    def test_pool_topology_and_parking_stay_stable_across_every_selection(self):
        def topology():
            return [(str(p.GetPath()), tuple(p.GetAppliedSchemas()),
                     str(p.GetMetadata("references")), p.IsActive())
                    for p in self.stage.Traverse()]
        before = topology()
        for asset in self.randomizer.pool:
            with patch.object(runtime, "sample_candidate", return_value=(asset, 1., 4.05, .4)):
                episode = self.randomizer.randomize()
            self.assertEqual(topology(), before)
            self.assertEqual(self.randomizer.current_object_path, runtime.fruit_path(asset))
            self.assertEqual(episode["object_prim_path"], self.randomizer.current_object_path)
            self.assertEqual(len(episode["parking_positions"]), 3)
            for path, expected in episode["parking_positions"].items():
                prim = self.stage.GetPrimAtPath(path)
                pos = self.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                    self.Usd.TimeCode.Default()).ExtractTranslation()
                self.assertEqual(list(pos), expected)
            for path in self.randomizer.pool.values():
                body = self.UsdPhysics.RigidBodyAPI(self.stage.GetPrimAtPath(path))
                self.assertEqual(list(body.GetVelocityAttr().Get()), [0,0,0])
                self.assertEqual(list(body.GetAngularVelocityAttr().Get()), [0,0,0])
        self.randomizer.close()
        self.assertEqual(topology(), before)

    def test_stronger_writeback_pose_fails_without_changing_episode_or_topology(self):
        episode = self.randomizer.randomize()
        prim = self.stage.GetPrimAtPath(runtime.OBJECT_PATH)
        from pxr import Gf
        with self.Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            prim.GetAttribute("xformOp:transform:bfmEpisode").Set(
                Gf.Matrix4d(1).SetTranslate((900,900,900)))
        with self.assertRaisesRegex(ValueError, "reset pose is overridden"):
            self.randomizer.randomize()
        self.assertEqual(self.randomizer.last_episode, episode)
        for path in self.randomizer.pool.values():
            self.assertTrue(self.stage.GetPrimAtPath(path).IsActive())

    def test_missing_table_or_unsupported_xy_fails_instead_of_inventing_z(self):
        self.stage.GetPrimAtPath(self.data["randomization"]["table_prim_path"]).SetActive(False)
        with self.assertRaisesRegex(ValueError, "Table payload"):
            self.randomizer.randomize()
        self.assertIsNone(self.randomizer.last_episode)
        self.assertTrue(self.stage.GetPrimAtPath(runtime.OBJECT_PATH))

    def test_region_outside_surface_and_container_overlap_are_rejected(self):
        asset = self.data["randomization"]["object_assets"][0]
        for xy in ((30., 40.), (4., 4.)):
            with patch.object(runtime, "sample_candidate", return_value=(asset, *xy, 0.)):
                with self.assertRaisesRegex(ValueError, "No supported"):
                    self.randomizer.randomize()
        # A true overlap on supported tabletop, not just lack of surface.
        container = self.stage.GetPrimAtPath(self.data["parameters"]["container_prim_path"])
        self.randomizer.container_transform.SetTranslateOnly((1,4.05,.8))
        self.randomizer.container_bounds = ([.95,4.,.75], [1.05,4.1,.85])
        with patch.object(runtime, "sample_candidate", return_value=(asset, 1., 4.05, 0.)):
            with self.assertRaisesRegex(ValueError, "No supported"):
                self.randomizer.randomize()

    def test_runtime_rejects_visual_only_or_nested_rigid_default(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"bad.usda"
            stage = self.Usd.Stage.CreateNew(str(path))
            root = self.UsdGeom.Xform.Define(stage,"/Wrapper").GetPrim()
            stage.SetDefaultPrim(root)
            body = self.UsdGeom.Xform.Define(stage,"/Wrapper/Body").GetPrim()
            self.UsdPhysics.RigidBodyAPI.Apply(body)
            stage.GetRootLayer().Save()
            with self.assertRaisesRegex(ValueError,"dynamic default-prim"):
                runtime.asset_geometry(path)

    def test_container_sequence_is_seeded_supported_and_does_not_accumulate(self):
        goal_before = deepcopy(self.data["parameters"])
        episodes = [self.randomizer.randomize() for _ in range(5)]
        self.assertEqual(len({tuple(e["container_position"]) for e in episodes}), 5)
        for episode in episodes:
            x, y, z = episode["container_position"]
            self.assertTrue(1.03 <= x <= 1.15)
            self.assertTrue(4.38 <= y <= 4.50)
            self.assertEqual(z, .8)
        self.randomizer.close()
        again = runtime.SceneRandomizer(self.stage, self.data, ROOT)
        self.addCleanup(again.close)
        replay = [again.randomize() for _ in range(5)]
        self.assertEqual([{k:v for k,v in e.items() if k != "episode_id"} for e in episodes],
                         [{k:v for k,v in e.items() if k != "episode_id"} for e in replay])
        self.assertEqual(self.data["parameters"], goal_before)
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), self.container_before)

    def test_six_cm_offsets_are_reachable_when_supported(self):
        self.assertEqual(self.data["randomization"]["container_offset_region"],
                         dict(x_min=-.06, x_max=.06, y_min=-.06, y_max=.06))
        asset = self.data["randomization"]["object_assets"][0]
        for dx, dy in ((-.0599, -.0599), (-.0599, .0599),
                       (.0599, -.0599), (.0599, .0599)):
            with patch.object(self.randomizer.rng, "uniform", side_effect=[dx, dy]), \
                    patch.object(runtime, "sample_candidate", return_value=(asset, 1., 4.05, 0.)):
                episode = self.randomizer.randomize()
            self.assertEqual(episode["container_offset"], [dx, dy, 0.])
            self.assertAlmostEqual(episode["container_position"][0], 1.09 + dx)
            self.assertAlmostEqual(episode["container_position"][1], 4.44 + dy)

    def test_existing_episode_op_is_reused_without_add_or_accumulation(self):
        container = self.stage.GetPrimAtPath(runtime.CONTAINER_PATH)
        xform = self.UsdGeom.Xformable(container)
        # Mimic an op surviving in another layer after our runtime layer clears.
        existing = xform.AddTransformOp(opSuffix="bfmEpisode")
        existing.Set(self.randomizer.container_transform)
        xform.SetXformOpOrder([existing])
        source = self.stage.GetRootLayer().ExportToString()
        with patch.object(self.UsdGeom.Xformable, "AddTransformOp",
                          side_effect=AssertionError("Existing op must not be added again")):
            for _ in range(10):
                episode = self.randomizer.randomize()
                ops = xform.GetOrderedXformOps()
                self.assertEqual([op.GetOpName() for op in ops], ["xformOp:transform:bfmEpisode"])
                position = xform.ComputeLocalToWorldTransform(self.Usd.TimeCode.Default()).ExtractTranslation()
                self.assertEqual(list(position), episode["container_position"])
                self.assertAlmostEqual(position[0], 1.09 + episode["container_offset"][0])
                self.assertEqual(self.stage.GetRootLayer().ExportToString(), source)

    def test_container_outside_support_or_overlapping_robot_rejected(self):
        with patch.dict(self.data["randomization"]["container_offset_region"],
                        x_min=10., x_max=11.):
            with self.assertRaisesRegex(ValueError, "robot-free container"):
                self.randomizer.randomize()
        robot = self.stage.GetPrimAtPath(self.data["provenance"]["robot_root"])
        self.UsdGeom.Xformable(robot).GetOrderedXformOps()[0].Set((1.09, 4.44, .8))
        with self.assertRaisesRegex(ValueError, "robot-free container"):
            self.randomizer.randomize()
        self.assertTrue(self.stage.GetPrimAtPath(runtime.OBJECT_PATH))

    def test_fruit_avoids_new_container_and_failure_restores_runtime_layer(self):
        episode = self.randomizer.randomize()
        before = self.randomizer.layer.ExportToString()
        asset = self.data["randomization"]["object_assets"][0]
        with patch.object(runtime, "sample_candidate", return_value=(asset, 1.09, 4.44, 0.)):
            with self.assertRaisesRegex(ValueError, "obstacle-free fruit"):
                self.randomizer.randomize()
        self.assertEqual(self.randomizer.layer.ExportToString(), before)
        self.assertEqual(self.randomizer.last_episode, episode)

    def test_real_container_reference_retains_body_rotation_goal_and_support(self):
        from pxr import Gf
        self.randomizer.close()
        self.stage.RemovePrim(runtime.CONTAINER_PATH)
        xform = self.UsdGeom.Xform.Define(self.stage, runtime.CONTAINER_PATH)
        xform.GetPrim().GetReferences().AddReference(str(
            ROOT / "models/addons/objects/vomp/container_b04/container_b04.usd"))
        xform.ClearXformOpOrder()
        xform.AddTransformOp(opSuffix="testTemplate").Set(
            Gf.Matrix4d(1).SetTranslate((1.0898085832595825, 4.441446304321289, .7499995231628418)))
        source = self.stage.GetRootLayer().ExportToString()
        r = runtime.SceneRandomizer(self.stage, self.data, ROOT)
        self.addCleanup(r.close)
        for _ in range(5):
            episode = r.randomize()
            prim = self.stage.GetPrimAtPath(runtime.CONTAINER_PATH)
            self.assertTrue(prim.HasAPI(self.UsdPhysics.RigidBodyAPI))
            matrix = self.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(self.Usd.TimeCode.Default())
            self.assertEqual(list(matrix.ExtractTranslation()), episode["container_position"])
            self.assertEqual(matrix.ExtractRotationQuat(), Gf.Quatd(1))
            cache = self.UsdGeom.BBoxCache(self.Usd.TimeCode.Default(), ["default", "render", "proxy"])
            a = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            b = cache.ComputeWorldBound(self.stage.GetPrimAtPath(r.current_object_path)).ComputeAlignedRange()
            self.assertAlmostEqual(a.GetMin()[2], .75, places=5)
            self.assertFalse(runtime.overlaps(a.GetMin(), a.GetMax(), b.GetMin(), b.GetMax(), .03))
            self.assertEqual(self.stage.GetRootLayer().ExportToString(), source)


class AdapterTests(unittest.TestCase):
    def test_kit_registers_early_pop_and_cleans_up_on_shutdown(self):
        names = ("omni", "omni.ext", "omni.kit", "omni.kit.app", "omni.timeline", "omni.usd", "carb")
        modules = {name: ModuleType(name) for name in names}
        for name in names:
            if "." in name:
                parent, child = name.rsplit(".",1)
                setattr(modules[parent], child, modules[name])
        modules["omni.ext"].IExt = object
        modules["omni.timeline"].TimelineEventType = SimpleNamespace(PLAY=1,PAUSE=2,STOP=3)
        modules["omni.usd"].StageEventType = SimpleNamespace(OPENED=1,CLOSING=2,CLOSED=3)
        app, timeline, usd = Mock(), Mock(), Mock()
        usd.get_stage.return_value = None
        modules["omni.kit.app"].get_app = lambda: app
        modules["omni.timeline"].get_timeline_interface = lambda: timeline
        modules["omni.usd"].get_context = lambda: usd
        modules["carb"].log_info = Mock(); modules["carb"].log_error = Mock()
        package = ModuleType("bfm_collection")
        package.__path__ = [str(MODULE.parent)]
        modules["bfm_collection"] = package
        modules["bfm_collection.scene_randomizer"] = runtime
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location("bfm_collection.extension", MODULE.with_name("extension.py"))
            adapter = importlib.util.module_from_spec(spec); spec.loader.exec_module(adapter)
            extension = adapter.CollectionExtension()
            with patch.object(extension, "_bootstrap_ros_python_paths"):
                extension.on_startup("bfm_collection")
            pop = timeline.get_timeline_event_stream.return_value.create_subscription_to_pop
            timeline.get_timeline_event_stream.return_value.create_subscription_to_push.assert_not_called()
            self.assertLess(pop.call_args.kwargs["order"], 0)
            # Exercise real attachment/lifecycle with fake Kit and ROS services.
            stage = Mock()
            stage.GetRootLayer.return_value.realPath = str(ROOT / "models/stages/stage_sorting_v1.usd")
            contract = ModuleType("isaac_eff_control.bfm_contract")
            channel_module = ModuleType("isaac_eff_control.bfm_episode")
            channel_module.EpisodeChannel = Mock()
            channel_module.EpisodeChannel.return_value.quiescent = True
            sys.modules["isaac_eff_control.bfm_episode"] = channel_module
            self.addCleanup(sys.modules.pop, "isaac_eff_control.bfm_episode", None)
            contract.scene_config_default = lambda: str(CONFIG)
            contract.state_defaults = lambda _: runtime.load_configuration(CONFIG, ROOT)["parameters"]
            r = Mock()
            def compose():
                self.assertTrue(timeline.is_stopped())
                return dict(asset="lemon1.usd", position=[1,4,.8], yaw=0,
                            container_position=[1,4.4,.75], container_offset=[0,0,0])
            r.randomize.side_effect = compose
            with patch.dict(sys.modules, {"isaac_eff_control.bfm_contract": contract}), \
                    patch.object(adapter, "SceneRandomizer", return_value=r), \
                    patch.object(extension, "_start_publisher", return_value=Mock()) as start:
                usd.get_stage.return_value = stage
                timeline.is_stopped.return_value = True
                extension._update(None)
                self.assertEqual(extension.lifecycle.state, "prepared")
                r.randomize.assert_called_once()
                start.assert_not_called()
                extension._update(None)  # No repeat preparation on idle updates.
                timeline.is_stopped.return_value = False
                callback = pop.call_args.args[0]
                callback(SimpleNamespace(type=1)); callback(SimpleNamespace(type=1))
                callback(SimpleNamespace(type=2)); callback(SimpleNamespace(type=1))
                self.assertEqual(r.randomize.call_count, 1)
                start.assert_called_once()
                callback(SimpleNamespace(type=3)); callback(SimpleNamespace(type=3))
                start.return_value.stop.assert_called_once()
                self.assertEqual(r.randomize.call_count, 1)
                extension._update(None)  # STOP event received but timeline still unwinding.
                self.assertEqual(r.randomize.call_count, 1)
                timeline.is_stopped.return_value = True
                extension.episode_channel.quiescent = False
                extension._update(None)
                self.assertEqual(r.randomize.call_count, 1)  # Bag still finalizing.
                extension.episode_channel.quiescent = True
                extension._update(None)
                self.assertEqual(r.randomize.call_count, 2)
                self.assertEqual(extension.lifecycle.state, "prepared")
                callback(SimpleNamespace(type=3)); extension._update(None)
                self.assertEqual(r.randomize.call_count, 2)
                extension._detach()
                r.close.assert_called_once()
            life = Mock()
            extension.episode_channel = Mock()
            extension.lifecycle = life
            extension.stage = "stage"
            usd.get_stage.return_value = "stage"
            life.play.return_value = None
            callback = pop.call_args.args[0]
            for event, method in ((1,life.play),(2,life.pause),(3,life.stop)):
                callback(SimpleNamespace(type=event)); method.assert_called_once()
            life.play.side_effect = ValueError("bad asset")
            callback(SimpleNamespace(type=1))
            timeline.stop.assert_called_once()
            stage_callback = usd.get_stage_event_stream.return_value.create_subscription_to_pop.call_args.args[0]
            stage_callback(SimpleNamespace(type=2))  # CLOSING still returns the old context stage.
            life.close.assert_called_once()
            extension._update(None)
            self.assertIsNone(extension.lifecycle)  # It must not reattach while closing.
            with patch.object(extension, "_attach") as attach:
                stage_callback(SimpleNamespace(type=1))
                attach.assert_called_once()
                self.assertFalse(extension.closing)
            extension.on_shutdown()
            life.close.assert_called_once()
            self.assertEqual(adapter.get_status(), {"attached":False})

    def test_state_publisher_uses_authoritative_selected_body_and_descriptor(self):
        # Reuse the adapter import with its minimal Kit module surface.
        names = ("omni", "omni.ext", "omni.kit", "omni.kit.app", "omni.timeline", "omni.usd", "carb")
        modules = {name: ModuleType(name) for name in names}
        for name in names:
            if "." in name:
                parent, child = name.rsplit(".",1)
                setattr(modules[parent], child, modules[name])
        modules["omni.ext"].IExt = object
        task = ModuleType("isaac_eff_control.bfm_task_state")
        task.start_task_state_publisher = Mock()
        package = ModuleType("bfm_collection"); package.__path__ = [str(MODULE.parent)]
        modules.update({"bfm_collection": package, "bfm_collection.scene_randomizer": runtime,
                        "isaac_eff_control.bfm_task_state": task})
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location("bfm_collection.extension", MODULE.with_name("extension.py"))
            adapter = importlib.util.module_from_spec(spec); spec.loader.exec_module(adapter)
            ext = adapter.CollectionExtension()
            ext.parameters = {"object_prim_path": runtime.OBJECT_PATH}
            ext.config_path = str(CONFIG)
            episode = dict(object_prim_path=runtime.POOL_PATH+"/lemon1", episode_id="two")
            ext.lifecycle = SimpleNamespace(randomizer=SimpleNamespace(
                current_object_path=episode["object_prim_path"], last_episode=episode))
            ext._start_publisher()
            kwargs = task.start_task_state_publisher.call_args.kwargs
            self.assertEqual(kwargs["object_prim_path"], episode["object_prim_path"])
            self.assertIs(kwargs["episode"], episode)
            self.assertEqual(ext.parameters["object_prim_path"], runtime.OBJECT_PATH)

    def test_extension_manifest_declares_module_and_required_runtime_dependencies(self):
        import tomllib
        data = tomllib.loads((MODULE.parent.parent/"config/extension.toml").read_text())
        self.assertEqual(data["python"]["module"], [{"name":"bfm_collection"}])
        self.assertTrue({"omni.timeline", "omni.usd", "omni.physx",
                         "omni.physics.tensors", "isaacsim.ros2.bridge"}
                        <= set(data["dependencies"]))


if __name__ == "__main__":
    unittest.main()
