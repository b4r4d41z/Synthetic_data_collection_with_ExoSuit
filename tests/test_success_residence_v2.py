"""Placement geometry and episode state machine; no live Isaac/replay claim."""
import json
import math
from types import SimpleNamespace
import unittest

import numpy as np
from test_four_fixes import corners, harness
from isaac_eff_control.fruit_success import DWELL_NS, LiveFruitSuccess, SettledContainment


class ResidenceTests(unittest.TestCase):
    def detector(self):
        return SettledContainment([(-.16,-.11),(.16,-.11),(.16,.11),(-.16,.11)], .011, .118)

    def sample(self, detector, seconds, points=None, speeds=(0., 0., 0., 0.)):
        return detector.update(seconds, corners() if points is None else points, speeds)

    def test_inside_qualifies_on_first_sample_after_duration(self):
        d = self.detector()
        for t in (0., .07, .19, .299):
            self.assertFalse(self.sample(d, t)[0])
        success, audit = self.sample(d, .3)
        self.assertTrue(success)
        self.assertTrue(audit['stable'])
        self.assertEqual(audit['first_success_sim_sec'], .3)
        self.assertEqual(audit['stable_duration_sim_sec'], .3)

    def test_outside_above_below_and_protruding_never_qualify(self):
        for points in (corners(.5), corners(.14),
                       np.asarray(corners()) + [0, 0, .2],
                       np.asarray(corners()) - [0, 0, .1]):
            with self.subTest(points=points):
                d = self.detector()
                for t in (0., .11, .33, 1.):
                    success, audit = self.sample(d, t, points)
                    self.assertFalse(success)
                    self.assertFalse(audit['contained'])
                    self.assertEqual(audit['stable_duration_sim_sec'], 0.)

    def test_brief_pass_and_separate_visits_do_not_add_up(self):
        d = self.detector()
        for base in (0., 1., 2.):
            for t, points in ((base, corners()), (base+.2, corners()),
                              (base+.21, corners(.4))):
                self.assertFalse(self.sample(d, t, points)[0])
        self.assertEqual(self.sample(d, 3.)[1]['stable_duration_sim_sec'], 0.)

    def test_frozen_time_cannot_qualify_or_increase_dwell(self):
        d = self.detector()
        self.sample(d, 0.)
        for _ in range(100):
            success, audit = self.sample(d, .1)
            self.assertFalse(success)
            self.assertEqual(audit['stable_duration_sim_sec'], .1)
        self.assertTrue(self.sample(d, .3)[0])

    def test_variable_intervals_and_skipped_callbacks_keep_residence(self):
        for times in ((0., .06, .12, .18, .24, .3),
                      (0., .017, .151, .151, .237, .31), (0., .4)):
            with self.subTest(times=times):
                d = self.detector()
                for t in times:
                    success, audit = self.sample(d, t)
                    self.assertEqual(success, t >= .3)
                self.assertEqual(audit['stable_duration_sim_sec'], times[-1])

    def test_equivalent_physics_and_update_rates(self):
        # These are test cadences, not production settings or a sample recording.
        for rate in (7, 10, 20, 30, 60, 120, 240):
            with self.subTest(rate=rate):
                d = self.detector()
                for i in range(math.ceil(.3*rate)+1):
                    t = i/rate
                    success, _ = self.sample(d, t)
                    self.assertEqual(success, round(t*1e9) >= DWELL_NS)
                self.assertTrue(success)

    def test_time_origin_does_not_change_dwell(self):
        for origin in (0., 10., 10000.):
            d = self.detector()
            self.sample(d, origin)
            self.assertFalse(self.sample(d, origin+.299)[0])
            self.assertTrue(self.sample(d, origin+.3)[0])

    def test_rollback_clears_pending_and_latched_state(self):
        for qualified in (False, True):
            d = self.detector()
            self.sample(d, 10.)
            self.sample(d, 10.3 if qualified else 10.2)
            success, audit = self.sample(d, .1)
            self.assertFalse(success)
            self.assertEqual(audit['stable_duration_sim_sec'], 0.)
            self.assertIsNone(audit['first_success_sim_sec'])
            self.assertTrue(self.sample(d, .4)[0])

    def test_latch_survives_later_exit_but_new_episode_clears_everything(self):
        d = self.detector()
        self.sample(d, 0.)
        self.sample(d, .3)
        success, audit = self.sample(d, .4, corners(.4))
        self.assertTrue(success)
        self.assertFalse(audit['contained'])
        self.assertFalse(audit['stable'])
        self.assertEqual(audit['first_success_sim_sec'], .3)
        d.reset()
        self.assertIsNone(d.since_ns)
        self.assertIsNone(d.last_ns)
        self.assertIsNone(d.first_success_ns)
        self.assertFalse(self.sample(d, .5)[0])
        self.assertFalse(self.sample(self.detector(), .5)[0])

    def test_invalid_samples_break_qualification_without_inventing_success(self):
        invalid = [(float('nan'), corners(), [0]*4),
                   (float('inf'), corners(), [0]*4), (-1., corners(), [0]*4),
                   (.2, [], [0]*4), (.2, [[1, 2]]*8, [0]*4),
                   (.2, np.full((8, 3), float('nan')), [0]*4),
                   (.2, np.full((8, 3), float('inf')), [0]*4),
                   (.2, corners(), [float('inf')]*4), (.2, corners(), [0]*3)]
        for args in invalid:
            with self.subTest(args=args):
                d = self.detector()
                self.sample(d, 0.)
                self.sample(d, .19)
                with self.assertRaises(ValueError):
                    d.update(*args)
                self.assertIsNone(d.since_ns)
                self.assertIsNone(d.invalid_sample('unavailable')['success'])
                self.assertFalse(self.sample(d, .4)[0])
                self.assertTrue(self.sample(d, .7)[0])

    def test_invalid_after_success_retains_evidence_not_current_validity(self):
        d = self.detector()
        self.sample(d, 0.)
        self.sample(d, .3)
        invalid = d.invalid_sample('pose missing')
        self.assertTrue(invalid['success'])
        self.assertFalse(invalid['measurement_valid'])
        self.assertIsNone(invalid['contained'])
        json.dumps(invalid, allow_nan=False)
        # Keep rollback detection even across an invalid observation.
        self.assertFalse(self.sample(d, .1)[0])

    def test_finite_velocity_is_telemetry_only(self):
        d = self.detector()
        for t in (0., .15, .3):
            success, audit = self.sample(d, t, speeds=(1., 10., 2., 20.))
        self.assertTrue(success)
        self.assertFalse(audit['velocity_gate'])


class MeshTests(unittest.TestCase):
    def test_real_mesh_avoids_empty_box_corners_and_uses_container_frame(self):
        from pxr import Gf, Usd
        stage = Usd.Stage.Open(str(harness.ROOT/'models/stages/stage_sorting_v1.usd'))
        config = dict(object_prim_path='/World/Task/Fruits/lemon1',
                      container_prim_path='/World/container_b04')
        live = LiveFruitSuccess(stage, config, 'geometry-test')
        rotation = Gf.Rotation(Gf.Vec3d(0, 1, 0), 45)
        rotated = np.asarray([rotation.TransformDir(Gf.Vec3d(*p)) for p in live.vertices])
        center = np.mean(live.detector.polygon, axis=0)
        offset = Gf.Vec3d(center[0]-(rotated[:, 0].min()+rotated[:, 0].max())/2,
                         center[1]-(rotated[:, 1].min()+rotated[:, 1].max())/2,
                         live.detector.floor+.001-rotated[:, 2].min())
        local = Gf.Matrix4d(1); local.SetRotate(rotation); local.SetTranslateOnly(offset)
        box = [local.Transform(Gf.Vec3d(*p)) for p in live.corners]
        self.assertLess(min(p[2] for p in box), live.detector.floor-.003)
        container = Gf.Matrix4d(1)
        container.SetRotate(Gf.Rotation(Gf.Vec3d(1, 2, 3), 33))
        container.SetTranslateOnly(Gf.Vec3d(2, -3, 1))
        def view(matrix):
            q = matrix.ExtractRotationQuat()
            pose = [*matrix.ExtractTranslation(), *q.GetImaginary(), q.GetReal()]
            return SimpleNamespace(get_transforms=lambda: [pose], get_velocities=lambda: [[0.]*6])
        sampler = SimpleNamespace(views={'object_prim_path': view(local*container),
                                         'container_prim_path': view(container)})
        for t in (0., .13, .27, .35):
            result = live.measure(sampler, t)
        self.assertTrue(result['success'])
        self.assertTrue(result['contained'])
        self.assertGreater(result['collision_vertices_tested'], 8)
        # Exactly the same fruit in another receptacle/world location cannot
        # satisfy the intended container-relative predicate in a fresh episode.
        live.detector.reset()
        other = Gf.Matrix4d(container)
        other.SetTranslateOnly(Gf.Vec3d(5, -3, 1))
        sampler.views['object_prim_path'] = view(local*other)
        for t in (0., .4):
            result = live.measure(sampler, t)
        self.assertFalse(result['success'])
