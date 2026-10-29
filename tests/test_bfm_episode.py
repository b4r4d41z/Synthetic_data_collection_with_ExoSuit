"""Reset safety protocol with fake ROS and tensor views; no PhysX runtime claim."""
import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

PATH = Path(__file__).resolve().parents[1] / 'exo2isaacsim/src/isaac_eff_control/isaac_eff_control/bfm_episode.py'
sys.path.insert(0, str(PATH.parents[1]))
spec = importlib.util.spec_from_file_location('isaac_eff_control.episode_protocol_test', PATH)
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

    def test_phase_events_expose_code_clock_and_nonterminal_results(self):
        channel = self.channel()
        event = json.loads(channel.publisher.publish.call_args.args[0].data)
        self.assertIn('recording_code', event)
        self.assertIn('clock_domains', event)
        self.assertFalse(event['terminal'])
        self.assertIsNone(event['end_reason'])
        self.assertIsNone(event['success'])
        self.assertEqual(channel.control.publish.call_args.args[0].data,
                         channel.publisher.publish.call_args.args[0].data)

    def test_no_recorder_does_not_require_a_fake_ack(self):
        channel = self.channel()
        channel.control.get_subscription_count.return_value = 0
        channel.begin_stop(); self.assertTrue(channel.quiescent)
