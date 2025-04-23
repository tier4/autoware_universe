#!/usr/bin/env python3

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.node import Node


class GyroBiasPublisher(Node):
    def __init__(self):
        super().__init__("gyro_bias_publisher")
        self.publisher = self.create_publisher(Vector3Stamped, "/gyro_bias", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 20秒間隔

    def timer_callback(self):
        msg = Vector3Stamped()
        current_time = self.get_clock().now()
        msg.header.stamp = Time(sec=current_time.seconds_nanoseconds()[0] - 10)  # 現在時刻から10秒前
        msg.header.frame_id = "base_link"
        msg.vector.x = 0.0
        msg.vector.y = 0.0
        msg.vector.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("Published message with timestamp 10 seconds ago")


def main():
    rclpy.init()
    node = GyroBiasPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
