#!/usr/bin/env python3

import threading

from autoware_auto_planning_msgs.msg import Trajectory
import rclpy
from rclpy.node import Node


class RelayTrajectoryNode(Node):
    def __init__(self):
        super().__init__("relay_trajectory")
        self.subscription = self.create_subscription(
            Trajectory, "/tmp/planning/scenario_planning/trajectory", self.listener_callback, 10
        )
        self.publisher = self.create_publisher(
            Trajectory, "/planning/scenario_planning/trajectory", 10
        )
        self.running = True

    def listener_callback(self, msg):
        if self.running:
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RelayTrajectoryNode()

    def input_thread():
        nonlocal node
        while True:
            user_input = input("Enter 'y' to stop publishing: ")
            if user_input.lower() == "y":
                node.running = False
                print("Publishing stopped.")
                break

    thread = threading.Thread(target=input_thread)
    thread.start()

    rclpy.spin(node)

    thread.join()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
