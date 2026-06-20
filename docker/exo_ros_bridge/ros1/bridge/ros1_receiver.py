#!/usr/bin/env python3

import os
import rospy
import zmq

from dynamic_biped.msg import robotHandPosition


TOPIC = os.getenv(
    "ROS1_HAND_TOPIC",
    "/control_robot_hand_position",
)

ZMQ_ENDPOINT = os.getenv(
    "ZMQ_ENDPOINT",
    "tcp://127.0.0.1:5555",
)


class Ros1Receiver:
    def __init__(self):
        context = zmq.Context.instance()
        self.socket = context.socket(zmq.PUSH)

        # Не позволяем очереди старых команд бесконечно расти.
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.connect(ZMQ_ENDPOINT)

        self.subscriber = rospy.Subscriber(
            TOPIC,
            robotHandPosition,
            self.callback,
            queue_size=1,
            tcp_nodelay=True,
        )

        rospy.loginfo(
            "Reading ROS1 topic %s and forwarding to %s",
            TOPIC,
            ZMQ_ENDPOINT,
        )

    def callback(self, msg):
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
            "stamp_sec": int(msg.header.stamp.secs),
            "stamp_nanosec": int(msg.header.stamp.nsecs),
            "frame_id": msg.header.frame_id,
            "left_hand_position": left,
            "right_hand_position": right,
        }

        self.socket.send_json(payload)


def main():
    rospy.init_node("exo_ros1_eff_receiver")
    Ros1Receiver()
    rospy.spin()


if __name__ == "__main__":
    main()
