#!/usr/bin/env python3

import os
import queue
import threading

import rospy
import zmq

from dynamic_biped.msg import robotHandPosition
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


ROS1_HAND_TOPIC = os.getenv(
    "ROS1_HAND_TOPIC",
    "/control_robot_hand_position",
)

ROS1_ARM_TOPIC = os.getenv(
    "ROS1_ARM_TOPIC",
    "/kuavo_arm_traj",
)

ROS1_RECORDING_TOPIC = os.getenv(
    "ROS1_RECORDING_TOPIC",
    "/data_collection/recording",
)

ZMQ_ENDPOINT = os.getenv(
    "ZMQ_ENDPOINT",
    "tcp://127.0.0.1:5555",
)


class Ros1Receiver:
    def __init__(self):
        # Callbacks never touch ZeroMQ or block on a disconnected receiver.
        self.pending = queue.Queue(maxsize=32)
        self._stop = threading.Event()
        self._ready = threading.Event()
        self.sender_error = None
        self.thread = threading.Thread(target=self.send_loop, name='exo-zmq-sender')
        self.thread.start()
        if not self._ready.wait(5.0) or self.sender_error:
            self.close()
            raise RuntimeError(f'Bridge sender startup failed: {self.sender_error or "startup timeout"}')
        rospy.on_shutdown(self.close)

        self.hand_subscriber = rospy.Subscriber(
            ROS1_HAND_TOPIC,
            robotHandPosition,
            self.hand_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

        self.arm_subscriber = rospy.Subscriber(
            ROS1_ARM_TOPIC,
            JointState,
            self.arm_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

        self.recording_subscriber = rospy.Subscriber(
            ROS1_RECORDING_TOPIC,
            Bool,
            self.recording_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

        rospy.loginfo(
            "Reading ROS1 hand topic %s, arm topic %s, and recording topic %s; "
            "forwarding to %s",
            ROS1_HAND_TOPIC,
            ROS1_ARM_TOPIC,
            ROS1_RECORDING_TOPIC,
            ZMQ_ENDPOINT,
        )

    def send_payload(self, payload):
        if self._stop.is_set():
            return
        if self.sender_error:
            rospy.logerr_throttle(2.0, 'Bridge sender stopped: %s', self.sender_error)
            return
        try:
            self.pending.put_nowait(payload)
        except queue.Full:
            rospy.logwarn_throttle(2.0, 'Bridge send queue full (32); dropped new payload; check ROS2 receiver/connection')

    def send_loop(self):
        # A mutex cannot make PUSH a thread-safe socket. Keep creation, all sends
        # and close in this one owner, with a private context and bounded IO.
        context = socket = None
        try:
            context = zmq.Context()
            socket = context.socket(zmq.PUSH)
            socket.setsockopt(zmq.SNDHWM, 1)
            socket.setsockopt(zmq.IMMEDIATE, 1)
            socket.connect(ZMQ_ENDPOINT)
            self._ready.set()
            while not self._stop.is_set():
                try:
                    payload = self.pending.get(timeout=0.1)
                except queue.Empty:
                    continue
                while not self._stop.is_set():
                    if socket.poll(100, zmq.POLLOUT):
                        try:
                            socket.send_json(payload, flags=zmq.NOBLOCK)
                            break
                        except zmq.Again:
                            continue
        except Exception as error:
            self.sender_error = str(error)
            rospy.logerr('Bridge sender stopped: %s', error)
        finally:
            if socket is not None:
                socket.close(linger=0)
            if context is not None:
                context.term()
            self._ready.set()

    def close(self):
        self._stop.set()
        self.thread.join()  # No blocked send; socket/context are closed by owner.

    def hand_callback(self, msg):
        left = list(msg.left_hand_position)
        right = list(msg.right_hand_position)

        if len(left) != 6 or len(right) != 6:
            rospy.logwarn_throttle(
                2.0,
                "Expected six values for each hand; got left=%d right=%d",
                len(left),
                len(right),
            )
            return

        payload = {
            "kind": "hand",
            "stamp_sec": int(msg.header.stamp.secs),
            "stamp_nanosec": int(msg.header.stamp.nsecs),
            "frame_id": msg.header.frame_id,
            "left_hand_position": left,
            "right_hand_position": right,
        }

        self.send_payload(payload)

    def arm_callback(self, msg):
        payload = {
            "kind": "arm",
            "stamp_sec": int(msg.header.stamp.secs),
            "stamp_nanosec": int(msg.header.stamp.nsecs),
            "frame_id": msg.header.frame_id,
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
        }

        self.send_payload(payload)

    def recording_callback(self, msg):
        self.send_payload({
            "kind": "recording",
            "enabled": bool(msg.data),
        })


def main():
    rospy.init_node("exo_ros1_receiver")
    receiver = Ros1Receiver()
    try:
        rospy.spin()
    finally:
        receiver.close()


if __name__ == "__main__":
    main()
