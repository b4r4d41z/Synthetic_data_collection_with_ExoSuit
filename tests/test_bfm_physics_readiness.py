"""Exercise the real Kit adapter with fake events; not a live PhysX test."""
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

from test_bfm_goal_region import GoalTestBase
from test_physics_object_pool import fixture


class PhysicsReadinessTests(GoalTestBase):
    def setUp(self):
        super().setUp()
        names = ('omni', 'omni.kit', 'omni.kit.app',
                 'omni.timeline', 'omni.usd', 'rclpy.context', 'rclpy.parameter',
                 'isaacsim', 'isaacsim.core', 'isaacsim.core.simulation_manager')
        self.modules = {name: ModuleType(name) for name in names}
        for name in names:
            parent, _, child = name.rpartition('.')
            if parent in self.modules:
                setattr(self.modules[parent], child, self.modules[name])
        self.app, self.timeline, self.manager = Mock(), Mock(), Mock()
        self.manager.get_physics_simulation_view.return_value = None
        self.modules['isaacsim.core.simulation_manager'].SimulationManager = self.manager
        self.timeline.is_playing.return_value = True
        self.timeline.get_current_time.return_value = .01
        self.modules['omni.kit.app'].get_app = lambda: self.app
        self.modules['omni.timeline'].get_timeline_interface = lambda: self.timeline
        self.modules['omni.timeline'].TimelineEventType = SimpleNamespace(STOP=2, PAUSE=3)
        self.modules['omni.usd'].get_context = lambda: SimpleNamespace(get_stage=lambda: 'stage')
        self.modules['rclpy.context'].Context = Mock()
        self.modules['rclpy.parameter'].Parameter = Mock()
        sys.modules['rclpy.executors'].SingleThreadedExecutor = Mock()
        patcher = patch.dict(sys.modules, self.modules)
        patcher.start(); self.addCleanup(patcher.stop)
        success_patch = patch('isaac_eff_control.fruit_success.LiveFruitSuccess')
        success_patch.start().return_value.measure.return_value = {'success': False}
        self.addCleanup(success_patch.stop)
        self.factory = Mock(side_effect=lambda *_: Mock(read=Mock(return_value={
            key: ([0., 0., 0., 1.] if key == 'goal_orientation' else
                  [1., 1., 1.] if key == 'goal_half_extents' else [0.] * size)
            for key, size in self.module.STATE_FIELD_SIZES.items()})))
        self.node = self.module.BfmTaskStatePublisher('stage', sampler_factory=self.factory)
        with patch.object(self.module, 'BfmTaskStatePublisher', return_value=self.node):
            self.handle = self.module.start_task_state_publisher(bfm_collection_mode=True)
        self.addCleanup(self.handle.stop)
        self.update = self.app.get_post_update_event_stream.return_value.create_subscription_to_pop.call_args.args[0]
        self.event = self.timeline.get_timeline_event_stream.return_value.create_subscription_to_pop.call_args.args[0]

    def ready(self):
        view = Mock(is_valid=True)
        self.manager.get_physics_simulation_view.return_value = view
        return view

    def test_waiting_for_physics_never_reads_or_disables_node(self):
        sampler = self.node.sampler
        for _ in range(30):
            self.update(None)
        sampler.read.assert_not_called()
        self.node.publisher.publish.assert_not_called()
        self.assertTrue(self.node.enabled)
        view = self.ready()
        self.update(None)
        sampler.read.assert_called_once()
        self.node.publisher.publish.assert_called_once()
        self.assertTrue(self.handle.reported_ready)
        self.assertIs(sampler.simulation_view, view)
        self.update(None)
        sampler.read.assert_called_once()  # Same simulation time, no duplicate publish.

    def test_pause_preserves_views_stop_waits_again_and_recreates_them(self):
        self.ready(); self.update(None)
        first = self.node.sampler
        self.timeline.is_playing.return_value = False
        self.event(SimpleNamespace(type=3)); self.update(None)
        self.assertIs(self.node.sampler, first)
        self.timeline.is_playing.return_value = True
        self.timeline.get_current_time.return_value = .02
        self.update(None)
        self.assertIs(self.node.sampler, first)
        self.event(SimpleNamespace(type=2))
        self.manager.get_physics_simulation_view.return_value = None
        self.assertIsNone(self.node.sampler)
        self.timeline.get_current_time.return_value = .01
        self.update(None)
        self.assertEqual(self.factory.call_count, 1)
        self.ready(); self.update(None)
        self.assertEqual(self.factory.call_count, 2)
        self.assertIsNot(self.node.sampler, first)

    def test_real_read_error_fails_closed_instead_of_fake_state(self):
        self.node.sampler.read.side_effect = ValueError('PhysX did not resolve one body')
        self.ready(); self.update(None)
        self.assertFalse(self.node.enabled)
        self.node.publisher.publish.assert_not_called()
        self.assertFalse(self.handle.reported_ready)
        self.node.get_logger().error.assert_called_once()

    def test_stop_releases_all_subscriptions_and_ignores_late_callbacks(self):
        self.handle.stop(); self.handle.stop()
        self.assertIsNone(self.node.sampler)
        for name in ('subscription', 'timeline_subscription', 'simulation_view'):
            self.assertIsNone(getattr(self.handle, name))
        self.ready(); self.update(None)
        self.node.publisher.publish.assert_not_called()

    def test_invalid_view_waits_without_disabling_or_publishing(self):
        for invalid in (Mock(is_valid=False), Mock(is_valid=Mock(return_value=False))):
            self.manager.get_physics_simulation_view.return_value = invalid
            self.update(None)
            self.assertTrue(self.node.enabled)
            self.node.sampler.read.assert_not_called()
        self.ready(); self.update(None)
        self.node.publisher.publish.assert_called_once()

    def test_manager_replacement_drops_old_rigid_body_views(self):
        self.ready(); self.update(None)
        first = self.node.sampler
        replacement = self.ready()
        self.timeline.get_current_time.return_value = .02
        self.update(None)
        self.assertIsNot(self.node.sampler, first)
        self.assertIs(self.node.sampler.simulation_view, replacement)
        self.assertEqual(self.factory.call_count, 2)

    def test_readiness_loss_waits_and_recovers_with_fresh_sampler(self):
        self.ready(); self.update(None)
        first = self.node.sampler
        self.manager.get_physics_simulation_view.return_value = None
        self.update(None)
        self.assertTrue(self.node.enabled)
        self.assertIsNone(self.node.sampler)
        self.ready(); self.update(None)
        self.assertIsNot(self.node.sampler, first)

    def install_live_episode(self):
        self.handle.stop()
        stage, ep, bodies, view = fixture()
        view.is_valid = True
        self.overrides['object_prim_path'] = ep['object_prim_path']
        self.node = self.module.BfmTaskStatePublisher(stage, sampler_factory=self.factory)
        self.modules['omni.usd'].get_context = lambda: SimpleNamespace(get_stage=lambda: stage)
        with patch.object(self.module, 'BfmTaskStatePublisher', return_value=self.node):
            self.handle = self.module.start_task_state_publisher(episode=ep, bfm_collection_mode=True)
        self.addCleanup(self.handle.stop)
        self.update = self.app.get_post_update_event_stream.return_value.create_subscription_to_pop.call_args.args[0]
        self.event = self.timeline.get_timeline_event_stream.return_value.create_subscription_to_pop.call_args.args[0]
        return bodies,view

    def test_live_reset_precedes_publication_and_waits_for_a_real_step_and_parking(self):
        bodies,view = self.install_live_episode()
        self.update(None)
        self.assertEqual(self.handle.phase,'waiting_for_physics')
        self.assertTrue(all(not b.writes for b in bodies.values()))
        self.manager.get_physics_simulation_view.return_value = view
        self.update(None)
        self.assertEqual(self.handle.phase,'settling')
        self.assertTrue(self.handle.object_pool.verified)
        self.node.publisher.publish.assert_not_called()
        self.node.sampler.read.assert_not_called()
        self.update(None)  # Same physics time cannot activate or apply twice.
        self.node.publisher.publish.assert_not_called()
        parked = next(path for path in bodies if path not in
                      (self.node.config['object_prim_path'],'/World/container_b04'))
        bodies[parked].velocity[0,0] = .1
        self.timeline.get_current_time.return_value = .02
        self.update(None)
        self.node.publisher.publish.assert_not_called()
        bodies[parked].velocity[:] = 0
        self.timeline.get_current_time.return_value = .03
        self.update(None)
        self.assertEqual(self.handle.phase,'playing')
        self.assertTrue(self.handle.reported_ready)
        self.node.publisher.publish.assert_called_once()
        self.assertIs(self.node.sampler.body_views,self.handle.object_pool.bodies)
        self.assertTrue(all(b.writes == ['pose','velocity'] for b in bodies.values()))

    def test_live_reset_readback_failure_never_publishes_and_fails_once(self):
        bodies,view = self.install_live_episode()
        bodies[self.node.config['object_prim_path']].ignore_pose = True
        self.manager.get_physics_simulation_view.return_value = view
        self.update(None); self.update(None)
        self.assertEqual(self.handle.phase,'failed')
        self.assertFalse(self.node.enabled)
        self.assertFalse(self.handle.reported_ready)
        self.node.publisher.publish.assert_not_called()
        self.assertFalse(self.handle.object_pool.bodies)
        self.assertIn(self.node.config['object_prim_path'],self.node.get_logger().error.call_args.args[0])

    def test_stop_releases_live_pool_next_play_rebinds_and_pause_does_not_reset(self):
        bodies,view = self.install_live_episode()
        self.manager.get_physics_simulation_view.return_value = view
        self.update(None)
        self.timeline.get_current_time.return_value = .02; self.update(None)
        pool = self.handle.object_pool
        self.timeline.is_playing.return_value = False
        self.event(SimpleNamespace(type=3)); self.update(None)
        self.assertTrue(pool.verified)
        self.timeline.is_playing.return_value = True
        self.timeline.get_current_time.return_value = .03; self.update(None)
        self.assertTrue(all(b.writes == ['pose','velocity'] for b in bodies.values()))
        self.event(SimpleNamespace(type=2))
        self.assertFalse(pool.bodies); self.assertIsNone(pool.simulation_view)
        self.assertIsNone(self.node.sampler)
        self.timeline.is_playing.return_value = False; self.update(None)
        self.assertEqual(view.create_rigid_body_view.call_count,5)
        self.timeline.is_playing.return_value = True
        self.timeline.get_current_time.return_value = .01; self.update(None)
        self.assertEqual(view.create_rigid_body_view.call_count,10)
        self.assertEqual(self.handle.phase,'settling')
        self.assertTrue(all(b.writes == ['pose','velocity','pose','velocity'] for b in bodies.values()))

    def test_manager_replacement_cannot_silently_reset_an_active_collection(self):
        bodies,view = self.install_live_episode()
        self.manager.get_physics_simulation_view.return_value = view
        self.update(None)
        self.timeline.get_current_time.return_value = .02; self.update(None)
        published = self.node.publisher.publish.call_count
        self.ready()  # Unexpected new manager view, not a user STOP/PLAY.
        self.update(None)
        self.assertFalse(self.node.enabled)
        self.assertEqual(self.handle.phase,'failed')
        self.assertEqual(self.node.publisher.publish.call_count,published)
        self.assertTrue(all(b.writes == ['pose','velocity'] for b in bodies.values()))
