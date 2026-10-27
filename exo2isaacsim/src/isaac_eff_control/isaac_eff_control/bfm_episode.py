"""Episode identity/control channel; independent of the action and state contracts.

Only the recorder subscribes to CONTROL_TOPIC. Diagnostics use /bfm/episode.
STOP acknowledgements are sent only after rosbag has exited and finalized.
"""
import json
import math
from uuid import uuid4

EPISODE_TOPIC = '/bfm/episode'
CONTROL_TOPIC = '/bfm/recorder_control'
ACK_TOPIC = '/bfm/recorder_ack'


class EpisodeChannel:
    def __init__(self):
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from std_msgs.msg import String
        self.message_type = String
        self.context = Context()
        self.context.init(args=[])
        self.node = Node('bfm_episode', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.node.create_publisher(String, EPISODE_TOPIC, qos)
        self.control = self.node.create_publisher(String, CONTROL_TOPIC, qos)
        self.subscription = self.node.create_subscription(String, ACK_TOPIC, self._ack, 10)
        self.acks = set()
        self.required = 0
        self.status = {}
        self.begin_stop()

    def _ack(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data.get('recorder_id'), str):
                return
        except (ValueError, TypeError, AttributeError):
            return
        if data.get('boundary_id') == self.status.get('boundary_id'):
            self.acks.add(data['recorder_id'])

    def publish(self, phase, episode=None):
        self.status = dict(self.status, phase=phase, episode=episode)
        msg = self.message_type()
        msg.data = json.dumps(self.status, allow_nan=False)
        self.publisher.publish(msg)
        self.control.publish(msg)

    def begin_stop(self):
        self.required = self.control.get_subscription_count()
        self.acks.clear()
        self.status = {'schema_version': 1, 'boundary_id': uuid4().hex}
        self.publish('stopped')
        if self.required:
            self.node.get_logger().info(f"Waiting for {self.required} recorder STOP acknowledgements")

    def poll(self):
        self.executor.spin_once(timeout_sec=0.)

    @property
    def quiescent(self):
        return len(self.acks) >= self.required

    def close(self):
        self.publish('closed')
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        self.context.shutdown()


class EpisodePhysicsGate:
    """Verify reset reached live PhysX, then wait for parked bodies to settle.

    This borrows body views from the existing manager view. It never writes
    physics, owns a backend, or reads USD as a substitute for live measurements.
    """
    def __init__(self, episode):
        self.episode = episode
        self.view = None
        self.bodies = {}
        self.verified = False
        self.opened = False

    def release(self):
        self.bodies = {}
        self.view = None
        self.verified = False
        self.opened = False

    @staticmethod
    def row(values, size):
        if hasattr(values, 'detach'):
            values = values.detach()
        if hasattr(values, 'cpu'):
            values = values.cpu()
        if hasattr(values, 'numpy'):
            values = values.numpy()
        result = [float(v) for v in values[0]]
        if len(result) != size or not all(math.isfinite(v) for v in result):
            raise ValueError('Invalid live episode reset measurement')
        return result

    def ready(self, simulation_view):
        if simulation_view is not self.view:
            self.view = simulation_view
            self.bodies = {}
            self.verified = False
            self.opened = False
        targets = {self.episode['object_prim_path']: self.episode['position'],
                   **self.episode['parking_positions'],
                   self.episode['container_prim_path']: self.episode['container_position']}
        if not self.bodies:
            for path in targets:
                body = simulation_view.create_rigid_body_view(path)
                if body.count != 1:
                    raise ValueError(f'Live reset must resolve exactly one body: {path}')
                self.bodies[path] = body
        stationary = True
        for path, target in targets.items():
            pose = self.row(self.bodies[path].get_transforms(), 7)
            twist = self.row(self.bodies[path].get_velocities(), 6)
            if not self.verified and any(abs(a-b) > .025 for a, b in zip(pose[:3], target)):
                raise ValueError(f'Live PhysX did not receive prepared reset: {path}')
            if not self.verified and path == self.episode['object_prim_path']:
                expected = self.episode['orientation_xyzw']
                norm = math.hypot(*pose[3:])
                if norm < 1e-12 or abs(sum(a*b for a,b in zip(pose[3:], expected))/norm) < .98:
                    raise ValueError(f'Live PhysX reset orientation differs: {path}')
            if path in self.episode['parking_positions']:
                if any(abs(a-b) > .20 for a, b in zip(pose[:3], target)):
                    raise ValueError(f'Parked fruit escaped its cradle: {path}')
                stationary = stationary and max(abs(v) for v in twist) <= .005
        self.verified = True
        if self.opened and not stationary:
            raise ValueError('Parked fruit started moving during collection')
        self.opened = stationary
        return stationary
