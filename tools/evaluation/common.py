"""Diagnostic-only helpers. No controller, task or stage modifications."""
from bisect import bisect_right
import json
import math
from pathlib import Path

COMMANDS = ('/robot_arm_joint_command', '/left_hand_joint_command', '/right_hand_joint_command')
JOINT_STATE = 'sensor_msgs/msg/JointState'
CLOCK = 'rosgraph_msgs/msg/Clock'
PHASES = {'playing', 'paused', 'stopped', 'prepared', 'waiting', 'failed', 'closed'}


def recording(path):
    """Accept a demonstration episode or rollout capture; never a loose MCAP."""
    import yaml
    root = Path(path).expanduser().resolve()
    sidecar = root / ('capture.json' if (root/'capture.json').exists() else 'episode.json')
    metadata = root/'bag/metadata.yaml'
    try:
        identity = json.loads(sidecar.read_text())
        info = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
        if not isinstance(identity, dict) or info['storage_identifier'] != 'mcap':
            raise ValueError('Expected a JSON object and MCAP storage')
        files = info['relative_file_paths']
        if not files:
            raise ValueError('Empty bag file list')
        for name in files:
            part = (root/'bag'/name).resolve()
            if part.parent != root/'bag' or part.suffix != '.mcap' or not part.is_file():
                raise ValueError(f'Missing or unsafe MCAP part: {name}')
            with part.open('rb') as stream:
                first = stream.read(8)
                if first == b'version ':
                    raise ValueError(f'Git LFS pointer, not downloaded MCAP data: {name}')
                stream.seek(-8, 2)
                if first != b'\x89MCAP0\r\n' or stream.read(8) != first:
                    raise ValueError(f'Missing MCAP magic/footer: {name}')
        topics = {t['topic_metadata']['name']: (t['topic_metadata']['type'], t['message_count'])
                  for t in info['topics_with_message_count']}
        if len(topics) != len(info['topics_with_message_count']) or any(
                not isinstance(n, str) or not n.startswith('/') or not isinstance(t, str)
                or type(c) is not int or c < 0 for n,(t,c) in topics.items()):
            raise ValueError('Invalid or duplicate topic metadata')
        if any(t['topic_metadata']['serialization_format'] != 'cdr' for t in info['topics_with_message_count']):
            raise ValueError('Only ROS CDR bags are supported')
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        raise ValueError(f'Invalid recording {root}: {error}') from error
    return root, identity, topics


def required_topics(topics):
    for name, kind in [*( (n, JOINT_STATE) for n in COMMANDS), ('/clock', CLOCK)]:
        if name not in topics or topics[name][0] != kind or topics[name][1] <= 0:
            raise ValueError(f'Require nonempty {name} ({kind}); no raw-command fallback')


def validate_command(topic, message):
    from isaac_eff_control.bfm_contract import ARM_JOINTS, arm_target_processing
    if topic == COMMANDS[0]:
        expected = ARM_JOINTS
    elif topic in COMMANDS[1:]:
        side = 'left' if topic == COMMANDS[1] else 'right'
        expected = {side+'_'+n for n in ('thumb_MCP_FE', 'thumb_IP', 'thumb_CMC_FE', 'thumb_CMC_AA')}
        expected |= {f'{side}_{finger}_{joint}' for finger in ('index','middle','ring','pinky')
                     for joint in ('MCP_FE','PIP','DIP')}
    else:
        raise ValueError(f'Unsupported command topic: {topic}')
    names = list(message.name)
    if len(names) != len(set(names)) or set(names) != expected:
        raise ValueError(f'{topic}: missing, duplicate or unexpected joints')
    if len(message.position) != len(names):
        raise ValueError(f'{topic}: wrong position dimension')
    for field in ('position', 'velocity', 'effort'):
        values = list(getattr(message, field))
        if len(values) not in (0, len(names)) or not all(math.isfinite(v) for v in values):
            raise ValueError(f'{topic}: invalid {field}')
    limits = arm_target_processing()['joint_limits'] if topic == COMMANDS[0] else {}
    for name, value in zip(names, message.position):
        low, high = limits.get(name, (-2*math.pi, 2*math.pi))
        if not low-1e-6 <= value <= high+1e-6:
            raise ValueError(f'{topic}: {name} outside supported radian range; values are never clamped')


def clock_ns(msg):
    sec, nano = msg.clock.sec, msg.clock.nanosec
    if sec < 0 or not 0 <= nano < 1_000_000_000:
        raise ValueError('Invalid /clock')
    return sec*1_000_000_000+nano


def schedule(commands, clocks):
    """Causal receive-time join, quantized to recorded physics clock samples.

    Headers are deliberately ignored: controller headers may be wall/source time.
    Boundary commands use the nearest endpoint clock (no wall-time extrapolation).
    Input order breaks ties and preserves inter-topic receive order.
    """
    if any(b[0] < a[0] or b[1] < a[1] for a,b in zip(clocks, clocks[1:])):
        raise ValueError('Recorded clock rollback/unordered receive time; replay one physics epoch only')
    if len(clocks) < 2 or clocks[-1][1] <= clocks[0][1]:
        raise ValueError('Recording needs advancing /clock samples')
    logs = [c[0] for c in clocks]
    ordered = sorted(commands, key=lambda c: c[0])
    return [(clocks[max(0, bisect_right(logs, stamp)-1)][1]-clocks[0][1], topic, msg)
            for stamp, topic, msg in ordered]


class Lifecycle:
    """Only simulator status envelopes, never recorder result/event envelopes."""
    def __init__(self):
        self.phase = None
        self.boundary = None
        self.episode = None

    def receive(self, text):
        data = json.loads(text)
        if not isinstance(data, dict) or 'event_type' in data:
            return False
        if (data.get('schema_version') != 1 or data.get('phase') not in PHASES
                or not isinstance(data.get('boundary_id'), str) or not data['boundary_id']):
            return False
        self.phase, self.boundary = data['phase'], data['boundary_id']
        self.episode = data.get('episode')
        return True


def subscriptions(node, clock_callback, episode_callback):
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    return [node.create_subscription(Clock, '/clock', clock_callback,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
            node.create_subscription(String, '/bfm/episode', episode_callback,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))]
