#!/usr/bin/env python3

import os
import threading

import rclpy
import zmq

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from dynamic_biped.msg import RobotHandPosition


TOPIC = os.getenv(
    "ROS2_HAND_TOPIC",
    "/control_robot_hand_position",
)

ZMQ_ENDPOINT = os.getenv(
    "ZMQ_BIND_ENDPOINT",
    "tcp://0.0.0.0:5555",
)


class Ros2Publisher(Node):
    def __init__(self):
        super().__init__("exo_ros2_eff_publisher")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.publisher = self.create_publisher(
            RobotHandPosition,
            TOPIC,
            qos,
        )

        context = zmq.Context.instance()
        self.socket = context.socket(zmq.PULL)
        self.socket.setsockopt(zmq.RCVHWM, 1)
        self.socket.bind(ZMQ_ENDPOINT)

        self.running = True
        self.thread = threading.Thread(
            target=self.receive_loop,
            daemon=True,
        )
        self.thread.start()

        self.get_logger().info(
            f"Publishing {TOPIC}; waiting for bridge data on {ZMQ_ENDPOINT}"
        )

    def receive_loop(self):
        while self.running and rclpy.ok():
            try:
                payload = self.socket.recv_json()

                left = payload.get("left_hand_position", [])
                right = payload.get("right_hand_position", [])

                if len(left) != 6 or len(right) != 6:
                    self.get_logger().warning(
                        "Dropped command with invalid array length"
                    )
                    continue

                msg = RobotHandPosition()

                msg.header.stamp.sec = int(
                    payload.get("stamp_sec", 0)
                )
                msg.header.stamp.nanosec = int(
                    payload.get("stamp_nanosec", 0)
                )
                msg.header.frame_id = str(
                    payload.get("frame_id", "")
                )

                msg.left_hand_position = [
                    int(value) for value in left
                ]
                msg.right_hand_position = [
                    int(value) for value in right
                ]

                self.publisher.publish(msg)

            except zmq.error.ContextTerminated:
                break
            except Exception as error:
                self.get_logger().error(
                    f"Bridge processing error: {error}"
                )

    def destroy_node(self):
        self.running = False
        self.socket.close(linger=0)
        return super().destroy_node()


def main():
    rclpy.init()
    node = Ros2Publisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
