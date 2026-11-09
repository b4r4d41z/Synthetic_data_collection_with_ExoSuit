"""Direct controller-command replay, paced only by live Isaac simulation time."""
import argparse
from pathlib import Path
import signal
import time
from uuid import uuid4

from common import (COMMANDS, JOINT_STATE, CLOCK, Lifecycle, clock_ns, recording,
                    required_topics, schedule, subscriptions, validate_command)


def load(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    root, metadata, topics = recording(path)
    if (root/'capture.json').exists() and metadata.get('status') != 'complete':
        raise ValueError('Rollout capture is not finalized successfully; inspect capture.json/rosbag2.log')
    required_topics(topics)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(root/'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    actual = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for name in (*COMMANDS, '/clock'):
        if actual.get(name) != topics[name][0]:
            raise ValueError(f'Bag schema disagrees with metadata: {name}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[*COMMANDS, '/clock']))
    types = {n:get_message(actual[n]) for n in (*COMMANDS, '/clock')}
    commands, clocks, seen = [], [], set()
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg = deserialize_message(data, types[topic])
        if topic == '/clock':
            clocks.append((stamp, clock_ns(msg)))
        else:
            validate_command(topic, msg)
            commands.append((stamp, topic, msg)); seen.add(topic)
    if seen != set(COMMANDS):
        raise ValueError('MCAP payload lacks required controller commands')
    events = schedule(commands, clocks)
    return root, metadata, events


class ReplayClock:
    def __init__(self, events):
        self.events, self.index = events, 0
        self.origin = self.previous = None

    def advance(self, now, playing):
        if self.previous is not None and now < self.previous:
            raise ValueError('Live simulation clock reset; replay terminated')
        duplicate = now == self.previous
        self.previous = now
        if not playing or duplicate:
            return []
        if self.origin is None:
            self.origin = now
        elapsed = now-self.origin
        # Abort a stale backlog instead of dumping old targets after a jump.
        if self.index < len(self.events) and elapsed-self.events[self.index][0] > 500_000_000:
            raise ValueError('Replay missed its command deadline by >0.5 simulation seconds')
        ready = []
        while self.index < len(self.events) and self.events[self.index][0] <= elapsed:
            ready.append(self.events[self.index]); self.index += 1
        return ready


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording')
    parser.add_argument('--validate-only', action='store_true')
    args = parser.parse_args()
    root, _, events = load(args.recording)
    duration = events[-1][0]/1e9
    print(f'[REPLAY] Recording: {root}\n[REPLAY] Duration: {duration:.3f} simulation seconds')
    print('[REPLAY] Controller topics: '+', '.join(COMMANDS))
    print('[REPLAY] Timing: preceding recorded /clock by MCAP receive time; headers/values unchanged')
    if args.validate_only:
        print(f'[REPLAY] Validated {len(events)} commands; no publishers created')
        return
    import rclpy
    from sensor_msgs.msg import JointState
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init()
    node = rclpy.create_node('bfm_direct_replay_'+uuid4().hex[:8])
    publishers = {n:node.create_publisher(JointState, n,
                    QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)) for n in COMMANDS}
    life, cursor = Lifecycle(), ReplayClock(events)
    state = dict(done=False, armed=False, boundary=None, reason='Completed', progress=-1, clock_seen=None)
    discovery_until = time.monotonic()+2.0  # DDS discovery only, never replay pacing.

    def check_endpoints():
        if node.count_publishers('/clock') != 1:
            raise ValueError('Require exactly one live /clock publisher')
        for topic in COMMANDS:
            for endpoint in node.get_publishers_info_by_topic(topic):
                if endpoint.node_name != node.get_name() or endpoint.node_namespace != node.get_namespace():
                    raise ValueError(f'Conflicting publisher on {topic}: {endpoint.node_name}')
                if endpoint.topic_type != JOINT_STATE:
                    raise ValueError(f'Unexpected live command type: {topic}')
        return all(p.get_subscription_count() > 0 for p in publishers.values())

    def episode(msg):
        if not life.receive(msg.data):
            return
        if cursor.origin is not None and (life.phase not in ('playing','paused')
                                          or life.boundary != state['boundary']):
            state.update(done=True, reason='Simulation stopped / episode boundary changed')

    def clock(msg):
        now = clock_ns(msg)
        if state['done']:
            return
        if time.monotonic() < discovery_until:
            return
        if not check_endpoints():
            return
        if life.phase not in ('playing','paused'):
            return  # A retained stopped event cannot start or prematurely end replay.
        if not state['armed']:
            previous = state['clock_seen']
            state['clock_seen'] = now
            if life.phase != 'playing' or previous is None or now <= previous:
                return
            state['armed'] = True
            state['boundary'] = life.boundary
            print('[REPLAY] PLAY detected; replaying...', flush=True)
        for _, topic, command in cursor.advance(now, life.phase == 'playing'):
            publishers[topic].publish(command)
        elapsed = 0. if cursor.origin is None else (now-cursor.origin)/1e9
        progress = min(100, int(100*elapsed/duration)) if duration else 100
        if progress//10 != state['progress']//10:
            print(f'[REPLAY] {progress}% ({elapsed:.3f}/{duration:.3f} sim s; {cursor.index}/{len(events)} commands)', flush=True)
            state['progress'] = progress
        if cursor.index == len(events):
            state['done'] = True

    handles = subscriptions(node, clock, episode)
    old = signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    print('[REPLAY] Waiting for simulation, lifecycle and controller subscribers...', flush=True)
    try:
        while rclpy.ok() and not state['done']:
            rclpy.spin_once(node, timeout_sec=.1)
        if state['reason'] == 'Completed' and cursor.index == len(events):
            from rclpy.duration import Duration
            for topic, publisher in publishers.items():
                if not publisher.wait_for_all_acked(Duration(seconds=2)):
                    raise ValueError(f'DDS acknowledgement timeout on {topic}; delivery unconfirmed')
        print('[REPLAY] '+state['reason'])
    finally:
        signal.signal(signal.SIGTERM, old)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('[REPLAY] Interrupted; no further commands published')
    except Exception as error:
        raise SystemExit(f'[REPLAY] ERROR: {error}')
