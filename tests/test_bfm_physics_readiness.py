"""Exercise the real Kit adapter with fake events; not a live PhysX test."""
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

from test_bfm_goal_region import GoalTestBase


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
