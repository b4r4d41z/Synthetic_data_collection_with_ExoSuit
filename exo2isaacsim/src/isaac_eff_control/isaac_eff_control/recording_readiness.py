"""Recording-only startup checks and a nonce-bound publication lease; no control IO."""
import math
from time import monotonic
from uuid import uuid4

READY_TOPIC = '/bfm/recorder_ready'
GATE_TOPIC = '/bfm/recording_gate'
LEASE_SEC = 1.5


class RecordingGate:
    """A stale READY cannot answer a fresh Gateway challenge or a new session."""
    def __init__(self, producer, clock=monotonic):
        self.producer, self.clock = producer, clock
        self.close()

    def close(self):
        self.pending = self.nonce = self.received = None
        self.sequence = -1

    def receive(self, event, boundary, episode, playing, inputs_valid):
        keys = ('recorder_id', 'session_id', 'challenge', 'boundary_id', 'episode_id')
        if (type(event.get('version')) is not int or event['version'] != 1 or not playing
                or any(not isinstance(event.get(k), str) or not event[k] for k in keys)
                or event['boundary_id'] != boundary or event['episode_id'] != episode):
            return None
        identity = tuple(event[k] for k in keys)
        phase = event.get('phase')
        if phase == 'closed':
            if identity == self.pending:
                self.close()
        elif phase == 'starting':
            if identity != self.pending:
                self.close()
                self.pending = identity
                self.nonce = uuid4().hex
            return dict(event, phase='gated', gateway_id=self.producer,
                        gateway_nonce=self.nonce, inputs_valid=bool(inputs_valid))
        elif (phase == 'ready' and identity == self.pending
              and event.get('gateway_id') == self.producer and event.get('gateway_nonce') == self.nonce
              and type(event.get('heartbeat')) is int and event['heartbeat'] > self.sequence):
            self.sequence, self.received = event['heartbeat'], self.clock()
        return None

    @property
    def allowed(self):
        return self.received is not None and 0 <= self.clock()-self.received <= LEASE_SEC


def stamp_ns(stamp):
    if (type(stamp.sec) is not int or stamp.sec < 0 or type(stamp.nanosec) is not int
            or not 0 <= stamp.nanosec < 10**9):
        raise ValueError('invalid simulation stamp')
    return stamp.sec*10**9 + stamp.nanosec


class StartupSamples:
    """Independent-monitor evidence, explicitly NOT rosbag writer admission proof."""
    def __init__(self, kinds, cameras, max_age, clock=monotonic):
        self.kinds, self.cameras, self.max_age, self.clock = kinds, cameras, max_age, clock
        self.reset()

    def reset(self):
        self.samples, self.errors = {}, {}
        self.clock_first = self.clock_last = None
        self.rollback = False

    def observe(self, topic, msg):
        kind = self.kinds[topic]
        now, stamp = self.clock(), None
        try:
            if kind == 'clock':
                stamp = stamp_ns(msg.clock)
                if self.clock_last is not None and stamp < self.clock_last:
                    self.rollback = True
                if self.clock_first is None:
                    self.clock_first = stamp
                self.clock_last = stamp
            elif kind in ('camera', 'state', 'joints'):
                stamp = stamp_ns(msg.header.stamp)
                if kind == 'camera':
                    info = self.cameras[topic]
                    if (msg.header.frame_id != info['frame'] or [msg.width,msg.height] != info['resolution']
                            or msg.encoding != 'rgb8' or not len(msg.data)):
                        raise ValueError('camera frame/resolution/encoding/payload mismatch')
            elif kind == 'command':
                if not msg.name or len(msg.name) != len(msg.position):
                    raise ValueError('empty/incomplete command')
            elif kind == 'hand':
                if len(msg.left_hand_position) != 6 or len(msg.right_hand_position) != 6:
                    raise ValueError('incomplete hand input')
        except (ValueError, TypeError, AttributeError) as error:
            self.errors[topic] = str(error)
            self.samples.pop(topic, None)
            return
        previous = self.samples.get(topic)
        advanced = previous is not None and (previous[2] or (stamp is not None and stamp > previous[1]))
        self.samples[topic] = (now, stamp, advanced)
        self.errors.pop(topic, None)

    def missing(self):
        missing, now = [], self.clock()
        for topic, kind in self.kinds.items():
            sample = self.samples.get(topic)
            if topic in self.errors:
                missing.append(topic+': '+self.errors[topic])
            elif sample is None or not 0 <= now-sample[0] <= self.max_age:
                missing.append(topic+': no fresh post-discovery data')
            elif kind in ('clock','camera','state','joints') and not sample[2]:
                missing.append(topic+': simulation stamp not advancing')
            elif kind == 'camera' and (self.clock_first is None or
                    not self.clock_first <= sample[1] <= self.clock_last+1):
                missing.append(topic+': camera stamp outside observed /clock epoch window')
        return missing


def positive_seconds(value, label):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(label+' must be finite and positive')
    return value
