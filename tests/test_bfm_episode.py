"""Reset safety protocol with fake ROS and tensor views; no PhysX runtime claim."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

PATH = Path(__file__).resolve().parents[1] / 'exo2isaacsim/src/isaac_eff_control/isaac_eff_control/bfm_episode.py'
spec = importlib.util.spec_from_file_location('episode_protocol_test', PATH)
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)


class EpisodeProtocolTests(unittest.TestCase):
    def channel(self):
        channel = module.EpisodeChannel.__new__(module.EpisodeChannel)
        channel.message_type = lambda: SimpleNamespace(data='')
        channel.node = Mock(); channel.publisher = Mock(); channel.control = Mock()
        channel.control.get_subscription_count.return_value = 1
        channel.acks = set(); channel.status = {}
        channel.begin_stop()
        return channel

    def test_reset_barrier_requires_current_boundary_and_deduplicates_acknowledgements(self):
        channel = self.channel()
        channel.control.get_subscription_count.return_value = 2
        channel.begin_stop()
        def ack(boundary, recorder):
            channel._ack(SimpleNamespace(data=json.dumps(dict(boundary_id=boundary, recorder_id=recorder))))
        current = channel.status['boundary_id']
        ack('stale', 'one'); self.assertFalse(channel.quiescent)
        ack(current, 'one'); ack(current, 'one'); self.assertFalse(channel.quiescent)
        ack(current, 'two'); self.assertTrue(channel.quiescent)
        channel.begin_stop(); self.assertFalse(channel.quiescent)
        ack(current, 'one'); self.assertFalse(channel.quiescent)

    def test_no_recorder_does_not_require_a_fake_ack(self):
        channel = self.channel()
        channel.control.get_subscription_count.return_value = 0
        channel.begin_stop(); self.assertTrue(channel.quiescent)

    def gate(self):
        episode = dict(object_prim_path='/Selected', position=[1.,2.,3.],
            orientation_xyzw=[0.,0.,0.,1.], parking_positions={'/Parked':[20.,0.,-5.]},
            container_prim_path='/World/container_b04', container_position=[2.,3.,.75])
        targets = {'/Selected':episode['position'], '/Parked':[20.,0.,-5.],
                   '/World/container_b04':episode['container_position']}
        bodies = {}
        for path, xyz in targets.items():
            body = Mock(count=1)
            body.get_transforms.return_value = [[*xyz,0.,0.,0.,1.]]
            body.get_velocities.return_value = [[0.]*6]
            bodies[path] = body
        view = SimpleNamespace(create_rigid_body_view=lambda path: bodies[path])
        return module.EpisodePhysicsGate(episode), bodies, view

    def test_live_gate_waits_for_parked_linear_and_angular_motion(self):
        gate, bodies, view = self.gate()
        bodies['/Parked'].get_velocities.return_value = [[0.,0.,0.,0.,.1,0.]]
        self.assertFalse(gate.ready(view))
        bodies['/Parked'].get_velocities.return_value = [[0.]*6]
        bodies['/Parked'].get_transforms.return_value = [[20.08,0.,-4.98,0.,0.,0.,1.]]
        self.assertTrue(gate.ready(view))
        bodies['/Parked'].get_transforms.return_value = [[200.,0.,-5.,0.,0.,0.,1.]]
        with self.assertRaisesRegex(ValueError, 'escaped'):
            gate.ready(view)

    def test_live_gate_rejects_stale_missing_duplicate_and_nonfinite_measurements(self):
        for failure in ('stale', 'missing', 'duplicate', 'nan', 'orientation'):
            gate, bodies, view = self.gate()
            body = bodies['/Selected']
            if failure in ('missing','duplicate'):
                body.count = 0 if failure == 'missing' else 2
            elif failure == 'nan':
                body.get_velocities.return_value = [[float('nan')]*6]
            elif failure == 'orientation':
                body.get_transforms.return_value = [[1.,2.,3.,0.,0.,1.,0.]]
            else:
                body.get_transforms.return_value = [[9.,9.,9.,0.,0.,0.,1.]]
            with self.subTest(failure=failure), self.assertRaises(ValueError):
                gate.ready(view)

    def test_warp_host_copy_precedes_row_indexing(self):
        class Warp:
            def __getitem__(self, _):
                raise AssertionError('backend indexing')
            def numpy(self):
                return [[1.,2.,3.,0.,0.,0.]]
        self.assertEqual(module.EpisodePhysicsGate.row(Warp(),6), [1.,2.,3.,0.,0.,0.])
