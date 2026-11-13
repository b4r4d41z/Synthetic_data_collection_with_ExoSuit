"""Versioned Gateway-input evidence; pure Python, no ROS or simulation dependency.

Digests bind recorded bytes/values, not an authenticated upstream ROS1 identity.
"""
from copy import deepcopy
import hashlib
import json
import math
import struct

from .bfm_contract import ARM_SOURCE_TOPIC, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS

PROVENANCE_VERSION = 1
PROVENANCE_TOPIC = '/bfm/source/input_provenance'
PROTOCOL = 'synthetic.gateway_input'
NAMES = (*LEFT_ARM_JOINTS, *RIGHT_ARM_JOINTS)


def canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False)


def trace_sha256(text):
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def _header(header):
    return dict(stamp_sec=header.stamp.sec, stamp_nanosec=header.stamp.nanosec,
                frame_id=header.frame_id)


def encode_trace(action, arm, hand, sequences, processing, hand_topic):
    """One self-contained selected-input bundle per Action, including reused inputs."""
    record = dict(protocol=PROTOCOL, version=PROVENANCE_VERSION,
        producer_session_id=action.producer_session_id, action_id=action.action_id,
        decision_stamp=dict(sec=action.header.stamp.sec, nanosec=action.header.stamp.nanosec),
        arm=dict(kind='arm', input_sequence=sequences['arm'], topic=ARM_SOURCE_TOPIC,
                 header=_header(arm.header), joint_names=list(arm.name),
                 position_rad=[float(v) for v in arm.position],
                 processing='per_joint_saturation_rad',
                 bounds_source_sha256=processing['bounds_source_sha256']),
        hand=dict(kind='hand', input_sequence=sequences['hand'], topic=hand_topic,
                  header=_header(hand.header),
                  left_hand_position=[float(v) for v in hand.left_hand_position],
                  right_hand_position=[float(v) for v in hand.right_hand_position]))
    text = canonical_json(record)
    return text, trace_sha256(text)


def _fields(value, expected):
    if not isinstance(value, dict) or set(value) != set(expected.split()):
        raise ValueError('Invalid provenance fields')


def _uint64(value):
    if type(value) is not int or not 0 < value < 2**64:
        raise ValueError('Invalid provenance sequence/action ID')


def _sha(value):
    if not isinstance(value, str) or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError('Invalid provenance SHA-256')


def _values(values, size):
    try:
        result = list(values)
        if len(result) != size or any(type(v) not in (int, float) or not math.isfinite(v) for v in result):
            raise ValueError('Invalid provenance numeric values')
        return [float(v) for v in result]
    except (TypeError, OverflowError) as error:
        raise ValueError('Invalid provenance numeric values') from error


def _no_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate JSON provenance key')
        result[key] = value
    return result


class InputProvenanceIndex:
    """Offline verifier: index the whole bag first, then verify Actions in any order.

    Supply the recording's bfm.arm_target_processing, not today's model bounds.
    Identical duplicate traces are idempotent; conflicting identities fail closed.
    """
    def __init__(self, processing):
        self.processing = deepcopy(processing)
        if (not isinstance(processing, dict) or processing.get('clamping_enabled') is not True
                or processing.get('transformation') != 'per_joint_saturation'
                or processing.get('unit') != 'rad'):
            raise ValueError('Unsupported recorded arm processing')
        _sha(processing.get('bounds_source_sha256'))
        self.limits = self.processing.get('joint_limits')
        if not isinstance(self.limits, dict) or set(self.limits) != set(NAMES):
            raise ValueError('Incomplete recorded arm bounds')
        for limits in self.limits.values():
            lo, hi = _values(limits, 2)
            if lo >= hi:
                raise ValueError('Invalid recorded arm bounds')
        self.traces, self.inputs = {}, {}

    def add_trace(self, text):
        if not isinstance(text, str):
            raise ValueError('Expected String.data provenance text')
        data = json.loads(text, object_pairs_hook=_no_duplicate_keys)
        _fields(data, 'protocol version producer_session_id action_id decision_stamp arm hand')
        if data['protocol'] != PROTOCOL or type(data['version']) is not int or data['version'] != PROVENANCE_VERSION:
            raise ValueError('Unsupported provenance protocol/version')
        session = data['producer_session_id']
        if not isinstance(session, str) or not session.strip():
            raise ValueError('Invalid provenance session')
        _uint64(data['action_id'])
        stamp = data['decision_stamp']
        _fields(stamp, 'sec nanosec')
        if (type(stamp['sec']) is not int or not 0 <= stamp['sec'] < 2**31
                or type(stamp['nanosec']) is not int or not 0 <= stamp['nanosec'] < 10**9):
            raise ValueError('Invalid decision stamp')
        updates = {}
        for kind in ('arm', 'hand'):
            sample = data[kind]
            fields = ('joint_names position_rad processing bounds_source_sha256' if kind == 'arm'
                      else 'left_hand_position right_hand_position')
            _fields(sample, 'kind input_sequence topic header ' + fields)
            _uint64(sample['input_sequence'])
            if (sample['kind'] != kind or not isinstance(sample['topic'], str)
                    or not sample['topic'].startswith('/') or any(c.isspace() for c in sample['topic'])):
                raise ValueError('Invalid input kind/topic')
            header = sample['header']
            _fields(header, 'stamp_sec stamp_nanosec frame_id')
            # Preserve native source header fields; do not interpret them as IDs
            # or equate their clock with the decision/recorder clock.
            if (type(header['stamp_sec']) is not int or not -2**31 <= header['stamp_sec'] < 2**31
                    or type(header['stamp_nanosec']) is not int or not 0 <= header['stamp_nanosec'] < 2**32
                    or not isinstance(header['frame_id'], str)):
                raise ValueError('Invalid source header representation')
            if kind == 'arm':
                names = sample['joint_names']
                if (not isinstance(names, list) or len(names) != 14 or any(type(n) is not str for n in names)
                        or set(names) != set(NAMES)):
                    raise ValueError('Invalid input joint names/order')
                _values(sample['position_rad'], 14)
                if (sample['topic'] != ARM_SOURCE_TOPIC or sample['processing'] != 'per_joint_saturation_rad'
                        or sample['bounds_source_sha256'] != self.processing['bounds_source_sha256']):
                    raise ValueError('Input arm bounds/processing disagree with recorded contract')
            else:
                values = _values(sample['left_hand_position'], 6) + _values(sample['right_hand_position'], 6)
                if any(not 0 <= v <= 100 for v in values):
                    raise ValueError('Invalid logical hand input')
            identity = (session, kind, sample['input_sequence'])
            encoded = canonical_json(sample)
            if identity in self.inputs and self.inputs[identity] != encoded:
                raise ValueError('Conflicting source-sample identity')
            updates[identity] = encoded
        key = (session, data['action_id'])
        if key in self.traces and self.traces[key][0] != text:
            raise ValueError('Conflicting Action trace identity')
        self.inputs.update(updates)
        self.traces[key] = (text, data)
        return key

    def verify(self, action):
        version = getattr(action, 'source_provenance_version', 0)
        digest = getattr(action, 'source_provenance_sha256', '')
        if version == 0 and digest == '':
            raise ValueError('No operator provenance claimed (policy/legacy/unprovenanced)')
        if type(version) is not int or version != PROVENANCE_VERSION:
            raise ValueError('Unsupported Action provenance version')
        _sha(digest)
        _uint64(action.action_id)
        if not isinstance(action.producer_session_id, str) or not action.producer_session_id.strip():
            raise ValueError('Invalid Action provenance session')
        key = (action.producer_session_id, action.action_id)
        if key not in self.traces:
            raise ValueError('Missing Action provenance evidence')
        text, record = self.traces[key]
        if trace_sha256(text) != digest:
            raise ValueError('Action provenance digest mismatch')
        if record['decision_stamp'] != dict(sec=action.header.stamp.sec, nanosec=action.header.stamp.nanosec):
            raise ValueError('Action/trace decision stamp mismatch')
        if list(action.left_arm_joint_names) != list(LEFT_ARM_JOINTS) or list(action.right_arm_joint_names) != list(RIGHT_ARM_JOINTS):
            raise ValueError('Invalid canonical Action joint order')
        source = record['arm']
        bounded = {name: min(max(value, self.limits[name][0]), self.limits[name][1])
                   for name, value in zip(source['joint_names'], source['position_rad'])}
        expected = [*[bounded[n] for n in LEFT_ARM_JOINTS], *record['hand']['left_hand_position'],
                    *[bounded[n] for n in RIGHT_ARM_JOINTS], *record['hand']['right_hand_position']]
        actual = [*_values(action.left_arm_target, 7), *_values(action.left_hand_target, 6),
                  *_values(action.right_arm_target, 7), *_values(action.right_hand_target, 6)]
        # Exact binary64 equality, including signed zero; never encoder-count tolerances.
        if struct.pack('<26d', *actual) != struct.pack('<26d', *expected):
            raise ValueError('Action differs from its exact selected Gateway inputs')
        return deepcopy(record)
