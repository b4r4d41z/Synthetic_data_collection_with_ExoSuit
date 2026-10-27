"""Episode identity/control channel; independent of the action and state contracts.

Only the recorder subscribes to CONTROL_TOPIC. Diagnostics use /bfm/episode.
STOP acknowledgements are sent only after rosbag has exited and finalized.
"""
import json
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
