#!/usr/bin/env python3

import os
import threading

import rospy
import zmq

from dynamic_biped.msg import robotHandPosition
from sensor_msgs.msg import JointState


ROS1_HAND_TOPIC = os.getenv(
    "ROS1_HAND_TOPIC",
    "/control_robot_hand_position",
)

ROS1_ARM_TOPIC = os.getenv(
    "ROS1_ARM_TOPIC",
    "/kuavo_arm_traj",
)

ZMQ_ENDPOINT = os.getenv(
    "ZMQ_ENDPOINT",
    "tcp://127.0.0.1:5555",
)


class Ros1Receiver:
    def __init__(self):
        context = zmq.Context.instance()
        self.socket = context.socket(zmq.PUSH)

        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.connect(ZMQ_ENDPOINT)

        self.send_lock = threading.Lock()

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

        rospy.loginfo(
            "Reading ROS1 hand topic %s and arm topic %s; forwarding to %s",
            ROS1_HAND_TOPIC,
            ROS1_ARM_TOPIC,
            ZMQ_ENDPOINT,
        )

    def send_payload(self, payload):
        with self.send_lock:
            self.socket.send_json(payload)

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


def main():
    rospy.init_node("exo_ros1_receiver")
    Ros1Receiver()
    rospy.spin()


if __name__ == "__main__":
    main()
