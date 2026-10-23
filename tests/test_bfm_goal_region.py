"""Moving-OBB contract/config and live-read mathematics; no Isaac runtime claim."""

import hashlib
import importlib
import importlib.util
import json
import math
import os
from pathlib import Path
from array import array
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from test_recording_pipeline import ROOT, load_module, ros_stubs


class GoalTestBase(unittest.TestCase):
    def setUp(self):
        clean = {key: value for key, value in os.environ.items() if not key.startswith("BFM_")}
        env = patch.dict(os.environ, clean, clear=True)
        env.start()
        self.addCleanup(env.stop)
        paths = patch.object(sys, "path", [str(ROOT / "exo2isaacsim/src/isaac_eff_control"), *sys.path])
        paths.start()
        self.addCleanup(paths.stop)
        self.overrides = {"bfm_collection_mode": True}
        modules = patch.dict(sys.modules, ros_stubs(self.overrides))
        modules.start()
        self.addCleanup(modules.stop)
        self.contract = importlib.import_module("isaac_eff_control.bfm_contract")
        self.module = load_module("test_moving_goal",
            "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/bfm_task_state.py")


class GoalConfigTests(GoalTestBase):
    def test_message_has_exact_moving_obb_fields_and_xyzw_convention(self):
        source = (ROOT / "exo2isaacsim/src/dynamic_biped/msg/BfmState.msg").read_text()
        fields = [line.split() for line in source.splitlines() if line and not line.startswith("#")]
        self.assertEqual(fields, [["std_msgs/Header", "header"],
            ["float64[3]", "right_eef_position"], ["float64[3]", "object_position"],
            ["float64[3]", "object_velocity"], ["float64[3]", "goal_center"],
            ["float64[3]", "goal_half_extents"], ["float64[4]", "goal_orientation"],
            ["float64[3]", "goal_linear_velocity"], ["float64[3]", "goal_angular_velocity"]])
        self.assertNotIn("goal_position", source)
        self.assertIn("[x, y, z, w]", source)
        self.assertIn("MCAP", source)
        self.assertIn("rad/s", source)

    def test_scene_defaults_and_environment_overrides_are_explicit(self):
        config = self.contract.state_defaults()
        self.assertEqual(config["object_prim_path"], "/World/orange1")
        self.assertEqual(config["container_prim_path"], "/World/container_b04")
        self.assertTrue(config["eef_prim_path"].endswith("/right_hand_C_MC"))
        self.assertEqual(config["reference_prim_path"], "/")
        self.assertEqual(config["state_frame"], "world")
        with patch.dict(os.environ, {"BFM_CONTAINER_PRIM_PATH": "/Other/Container",
                    "BFM_GOAL_LOCAL_ORIENTATION": "[0, 0, 0, 2]"}):
            overridden = self.contract.state_defaults()
        self.assertEqual(overridden["container_prim_path"], "/Other/Container")
        self.assertEqual(overridden["goal_local_orientation"], [0., 0., 0., 1.])
        self.assertNotIn("goal_prim_path", config)

    def test_bad_config_and_legacy_goal_are_rejected_clearly(self):
        config = self.contract.state_defaults()
        for key, value in (("goal_half_extents", [0, 1, 1]),
                           ("goal_local_orientation", [0, 0, 0, 0]),
                           ("goal_local_center", [1, 2]),
                           ("goal_local_center", [float("nan"), 0, 0])):
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.contract.validate_state_parameters({**config, key: value})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text('{"schema_version":1,"parameters":{}}')
            with self.assertRaises(ValueError):
                self.contract.state_defaults(str(path))
        with patch.dict(os.environ, {"BFM_GOAL_PRIM_PATH": "/Old/Goal"}), self.assertRaisesRegex(
                ValueError, "BFM_CONTAINER_PRIM_PATH"):
            self.contract.state_defaults()
        with self.assertRaisesRegex(ValueError, "replaced"):
            self.module.start_task_state_publisher(goal_prim_path="/Old/Goal")

    def test_custom_scene_file_then_ros_parameters_override_packaged_defaults(self):
        data = json.loads(Path(self.contract.scene_config_default()).read_text())
        data["parameters"]["goal_half_extents"] = [.3, .2, .1]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"scene.json"
            path.write_text(json.dumps(data))
            with patch.dict(os.environ, {"BFM_SCENE_CONFIG": str(path)}):
                self.assertEqual(self.contract.state_defaults()["goal_half_extents"], [.3, .2, .1])
            self.overrides.update(scene_config_path=str(path),
                goal_half_extents=array('d', [.2, .1, .05]), goal_local_orientation=[0., 0., 0., 2.])
            factory = Mock()
            node = self.module.BfmTaskStatePublisher("stage", sampler_factory=factory)
            self.assertEqual(node.config["goal_half_extents"], [.2, .1, .05])
            self.assertEqual(node.config["goal_local_orientation"], [0., 0., 0., 1.])
            self.assertEqual(factory.call_args.args[1], node.config)

    def test_ros_overrides_metadata_and_existing_timestamp_policy_are_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            self.overrides.update(output_root=str(Path(directory)/"records"),
                container_prim_path="/Custom/Container", goal_half_extents=[.2, .1, .05])
            recorder = load_module("test_goal_recorder",
                "exo2isaacsim/src/isaac_eff_control/isaac_eff_control/sim_recorder.py").SimRecorder()
            metadata = recorder.episode_metadata()
        state = metadata["bfm"]["task_state"]
        self.assertEqual(state["goal_representation"], "oriented_bounding_box")
        self.assertEqual(state["container_prim_path"], "/Custom/Container")
        self.assertEqual(state["goal_half_extents"], [.2, .1, .05])
        self.assertEqual(state["linear_velocity_unit"], "m/s")
        self.assertEqual(state["angular_velocity_unit"], "rad/s")
        self.assertTrue(state["quaternion_convention"].startswith("xyzw"))
        self.assertFalse(state["runtime_verified"])
        self.assertTrue(state["configured"])
        self.assertEqual(metadata["bfm"]["action_dim"], 13)
        self.assertEqual(metadata["bfm"]["policy_hz"], 10)
        self.assertEqual(metadata["time"], {"synchronization_source": "rosbag_record_timestamp",
                                            "header_stamp": "source_native"})
        self.assertIn("/bfm/state", metadata["recording"]["topics"])
        self.assertEqual(set(metadata["cameras"]), {"head", "left_wrist", "right_wrist"})


class FakeBody:
    def __init__(self, pose=None, velocity=None, com=None, count=1):
        self.count = count
        self.pose = pose or [0., 0., 0., 0., 0., 0., 1.]
        self.velocity = velocity or [0.] * 6
        self.com = com or [0., 0., 0., 0., 0., 0., 1.]

    def get_transforms(self):
        return [self.pose]

    def get_velocities(self):
        return [self.velocity]

    def get_coms(self):
        return [self.com]


@unittest.skipUnless(importlib.util.find_spec("pxr"), "Requires OpenUSD; PhysX is mocked")
class MovingGoalTests(GoalTestBase):
    def setUp(self):
        super().setUp()
        from pxr import Usd, UsdGeom, UsdPhysics
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.)
        UsdGeom.Xform.Define(self.stage, "/World")
        for name in ("Eef", "Object", "Container"):
            prim = UsdGeom.Xform.Define(self.stage, "/World/"+name).GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(prim)
        self.config = {**self.contract.state_defaults(),
            "eef_prim_path": "/World/Eef", "object_prim_path": "/World/Object",
            "container_prim_path": "/World/Container",
            "goal_local_center": [.5, 0., 0.], "goal_half_extents": [.2, .1, .05],
            "goal_local_orientation": [0., 0., 0., 1.]}
        self.bodies = {"/World/"+name: FakeBody() for name in ("Eef", "Object", "Container")}
        self.view = SimpleNamespace(create_rigid_body_view=lambda path: self.bodies[path])

    def sampler(self):
        return self.module.IsaacTaskSampler(self.stage, self.config, self.view)

    def assertVector(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=9)

    def test_translation_uses_live_pose_not_authored_usd_and_keeps_local_dimensions(self):
        from pxr import UsdGeom
        UsdGeom.Xformable(self.stage.GetPrimAtPath("/World/Container")).AddTranslateOp().Set((999,999,999))
        sampler = self.sampler()
        first = sampler.read()
        self.bodies["/World/Container"].pose[:3] = [1., 2., 3.]
        second = sampler.read()
        self.assertVector(first["goal_center"], [.5, 0., 0.])
        self.assertVector(second["goal_center"], [1.5, 2., 3.])
        self.assertEqual(first["goal_half_extents"], second["goal_half_extents"])

    def test_rotation_composes_xyzw_quaternions_and_rotates_goal_offset(self):
        h = math.sqrt(.5)
        self.bodies["/World/Container"].pose = [1., 2., 3., 0., 0., 3*h, 3*h]
        self.config["goal_local_orientation"] = [2*h, 0., 0., 2*h]
        result = self.sampler().read()
        self.assertVector(result["goal_center"], [1., 2.5, 3.])
        self.assertVector(result["goal_orientation"], [.5, .5, .5, .5])
        self.assertAlmostEqual(sum(v*v for v in result["goal_orientation"]), 1.)
        self.assertVector(result["goal_half_extents"], [.2, .1, .05])

    def test_point_velocity_includes_rotation_and_nonzero_physics_com(self):
        h = math.sqrt(.5)
        body = self.bodies["/World/Container"]
        body.pose = [1., 2., 3., 0., 0., h, h]
        body.com = [0., .25, 0., 0., 0., 0., 1.]
        # Origin velocity is [1,2,3]; rotated COM lever is [-.25,0,0].
        body.velocity = [1., 1.5, 3., 0., 0., 2.]
        result = self.sampler().read()
        self.assertVector(result["goal_linear_velocity"], [0., 2., 3.])
        self.assertVector(result["goal_angular_velocity"], [0., 0., 2.])

    def test_reference_transforms_all_vectors_and_orientation_with_correct_si_units(self):
        from pxr import UsdGeom
        UsdGeom.SetStageMetersPerUnit(self.stage, .01)
        ref = UsdGeom.Xform.Define(self.stage, "/World/Reference")
        ref.AddTranslateOp().Set((10., 0., 0.))
        ref.AddRotateZOp().Set(90.)
        self.config.update(reference_prim_path="/World/Reference", state_frame="fixture",
                           goal_local_orientation=[math.sqrt(.5), 0., 0., math.sqrt(.5)])
        h = math.sqrt(.5)
        self.bodies["/World/Eef"].pose[:3] = [10., 100., 0.]
        self.bodies["/World/Object"].pose[:3] = [10., 200., 0.]
        self.bodies["/World/Object"].velocity[:3] = [0., 100., 0.]
        container = self.bodies["/World/Container"]
        container.pose = [10., 100., 0., 0., 0., h, h]
        container.velocity = [200., 0., 0., 0., 0., 2.]
        result = self.sampler().read()
        for key, expected in {
            "right_eef_position": [1., 0., 0.], "object_position": [2., 0., 0.],
            "object_velocity": [1., 0., 0.], "goal_center": [1.5, 0., 0.],
            "goal_linear_velocity": [0., -1., 0.], "goal_angular_velocity": [0., 0., 2.],
            "goal_orientation": [h, 0., 0., h]}.items():
            self.assertVector(result[key], expected)

    def test_object_uses_live_center_of_mass_position_and_velocity(self):
        h = math.sqrt(.5)
        body = self.bodies["/World/Object"]
        body.pose = [4., 0., 0., 0., 0., h, h]
        body.com = [.1, 0., 0., 0., 0., 0., 1.]
        body.velocity = [-.1, 0., 0., 0., 0., 1.]
        result = self.sampler().read()
        self.assertVector(result["object_position"], [4., .1, 0.])
        self.assertVector(result["object_velocity"], [-.1, 0., 0.])

    def test_bad_roles_and_nonunique_runtime_bodies_fail_instead_of_publishing_zeros(self):
        for key, value in (("container_prim_path", ""), ("object_prim_path", "/World/Missing"),
                           ("eef_prim_path", "/World"), ("reference_prim_path", "/World/Container")):
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.module.IsaacTaskSampler(self.stage, {**self.config, key: value}, self.view)
        for path in self.bodies:
            for count in (0, 2):
                self.bodies[path].count = count
                with self.assertRaisesRegex(ValueError, "one body"):
                    self.sampler().read()
            self.bodies[path].count = 1

    def test_zero_and_nonfinite_live_quaternions_are_rejected(self):
        for quaternion in ([0., 0., 0., 0.], [0., 0., float("nan"), 1.]):
            self.bodies["/World/Container"].pose[3:] = quaternion
            with self.assertRaises(ValueError):
                self.sampler().read()

    def test_stop_play_recreates_views_and_reads_new_container_motion(self):
        factory = Mock(side_effect=lambda stage, config: self.sampler())
        publisher = self.module.BfmTaskStatePublisher(self.stage, sampler_factory=factory)
        publisher.publish_at(.02)
        publisher.reset()
        self.bodies["/World/Container"].pose[0] = 2.
        publisher.publish_at(.01)
        self.assertEqual(factory.call_count, 2)
        message = publisher.publisher.publish.call_args.args[0]
        self.assertVector(message.goal_center, [2.5, 0., 0.])
        self.assertFalse(hasattr(message, "goal_position"))

    def test_packaged_scene_paths_and_interior_measurements_match_composed_assets(self):
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        data = json.loads(Path(self.contract.scene_config_default()).read_text())
        params, provenance = data["parameters"], data["provenance"]
        mask = Usd.StagePopulationMask()
        for path in (params["object_prim_path"], params["container_prim_path"], provenance["robot_root"]):
            mask.Add(path)
        stage = Usd.Stage.OpenMasked(str(ROOT/data["scene"]), mask)
        # Only the anonymous session is changed for inspection; no USD is saved.
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            stage.GetPrimAtPath(provenance["robot_root"]).GetVariantSets().GetVariantSet(
                "Physics").SetVariantSelection(provenance["required_robot_physics_variant"])
        for key in ("eef_prim_path", "object_prim_path", "container_prim_path"):
            prim = stage.GetPrimAtPath(params[key])
            self.assertTrue(prim and prim.IsActive() and prim.HasAPI(UsdPhysics.RigidBodyAPI), key)
        self.module.IsaacTaskSampler(stage, params)  # Validation only, no live views claimed.
        for file_key, hash_key in (("container_asset", "container_asset_sha256"),
                                   ("orange_asset", "orange_asset_sha256")):
            self.assertEqual(hashlib.sha256((ROOT/provenance[file_key]).read_bytes()).hexdigest(),
                             provenance[hash_key])
        self.assertEqual(hashlib.sha256((ROOT/data["scene"]).read_bytes()).hexdigest(),
                         provenance["scene_sha256"])
        cache = UsdGeom.XformCache()
        mesh = UsdGeom.Mesh(stage.GetPrimAtPath(provenance["container_mesh"]))
        container = stage.GetPrimAtPath(params["container_prim_path"])
        matrix, reset = cache.ComputeRelativeTransform(mesh.GetPrim(), container)
        self.assertFalse(reset)
        self.assertVector(matrix.ExtractTranslation(), provenance["container_mesh_to_body_translation_m"])
        rotation = matrix.ExtractRotationQuat().GetNormalized()
        self.assertAlmostEqual(abs(rotation.GetReal()), 1.)
        self.assertVector(rotation.GetImaginary(), [0., 0., 0.])
        lo, hi = provenance["conservative_interior_mesh_min_m"], provenance["conservative_interior_mesh_max_m"]
        clearance = provenance["clearance_radius_m"] + provenance["safety_margin_m"]
        lower = (provenance["interior_floor_mesh_z_m"] +
                 provenance["orange_skin_minimum_support_radius_m"] -
                 provenance["floor_contact_tolerance_m"])
        upper = hi[2] - clearance
        self.assertVector(matrix.Transform(Gf.Vec3d(0., 0., (lower+upper)/2)), params["goal_local_center"])
        self.assertVector([(hi[0]-lo[0])/2-clearance, (hi[1]-lo[1])/2-clearance,
                           (upper-lower)/2], params["goal_half_extents"])
        self.assertGreater(lo[2], provenance["interior_floor_mesh_z_m"])
        self.assertLess(hi[2], provenance["rim_mesh_z_m"])
        # AABB rejection of every triangle is conservative: none can cross this
        # inscribed volume. This checks the actual cavity, not exterior bounds.
        points = mesh.GetPointsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        offset = 0
        for count in mesh.GetFaceVertexCountsAttr().Get():
            face = indices[offset:offset+count]
            offset += count
            for i in range(1,count-1):
                triangle = [points[j] for j in (face[0],face[i],face[i+1])]
                overlaps = all(max(v[a] for v in triangle) >= lo[a] and
                               min(v[a] for v in triangle) <= hi[a] for a in range(3))
                self.assertFalse(overlaps)
        skin = UsdGeom.Mesh(stage.GetPrimAtPath(provenance["orange_skin_mesh"]))
        points = skin.GetPointsAttr().Get()
        center = Gf.Vec3d(*[(min(p[a] for p in points)+max(p[a] for p in points))/2 for a in range(3)])
        skin_to_body, _ = cache.ComputeRelativeTransform(skin.GetPrim(), stage.GetPrimAtPath(params["object_prim_path"]))
        for axis in [(1.,0.,0.), (0.,1.,0.), (0.,0.,1.)]:
            self.assertAlmostEqual(skin_to_body.TransformDir(Gf.Vec3d(*axis)).GetLength(), 1.)
        self.assertLessEqual(max((Gf.Vec3d(p)-center).GetLength() for p in points),
                             provenance["clearance_radius_m"])
        resting = provenance["interior_floor_mesh_z_m"] + center[2] - min(p[2] for p in points)
        self.assertLessEqual(lower, resting)  # A settled orange must still be a valid goal.
        self.assertLessEqual(resting, upper)
        if importlib.util.find_spec("scipy"):
            # Optional calibration check; scipy is not a runtime dependency.
            import numpy as np
            from scipy.spatial import ConvexHull
            hull = ConvexHull(np.asarray(points)-np.asarray(center))
            self.assertAlmostEqual(float((-hull.equations[:,3]).min()),
                                   provenance["orange_skin_minimum_support_radius_m"])


if __name__ == "__main__":
    unittest.main()
