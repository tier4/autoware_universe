#!/usr/bin/env python3

import pandas as pd
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuPublisher(Node):
    def __init__(self):
        super().__init__("imu_publisher")
        self.publisher_ = self.create_publisher(Imu, "/imu_raw", 10)
        self.timer = self.create_timer(0.005, self.timer_callback)

        self.data = pd.read_csv("test/data/gyro_test_data.csv")
        self.current_index = 0

    def timer_callback(self):
        if self.current_index >= len(self.data):
            self.current_index = 0

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        msg.angular_velocity.x = float(self.data.iloc[self.current_index]["x"])
        msg.angular_velocity.y = float(self.data.iloc[self.current_index]["y"])
        msg.angular_velocity.z = float(self.data.iloc[self.current_index]["z"])

        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        self.publisher_.publish(msg)
        self.current_index += 1


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()

    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
