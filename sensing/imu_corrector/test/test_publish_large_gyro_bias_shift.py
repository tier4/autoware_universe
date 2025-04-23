#!/usr/bin/env python3

import time

from geometry_msgs.msg import TwistWithCovarianceStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuTwistPublisher(Node):
    def __init__(self):
        super().__init__("imu_twist_publisher")
        # パブリッシャーの設定（200Hz用に周期を0.005秒に設定）
        self.imu_publisher = self.create_publisher(
            Imu,
            "/imu_raw",
            10,
        )
        self.twist_publisher = self.create_publisher(TwistWithCovarianceStamped, "/twist", 10)
        self.timer = self.create_timer(0.005, self.timer_callback)
        self.start_time = time.time()

    def timer_callback(self):
        current_time = self.get_clock().now()
        elapsed_time = time.time() - self.start_time

        if elapsed_time > 6.0:
            velocity_x = 0.0
            angular_velocity_z = 1.0
        elif elapsed_time > 5.0:
            velocity_x = 1.0
            angular_velocity_z = 1.0
        elif elapsed_time > 3.0:
            velocity_x = 1.0
            angular_velocity_z = 0.0
        else:
            velocity_x = 0.0
            angular_velocity_z = 0.0

        # IMUメッセージの作成
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = "base_link"
        # ダミーデータをセット
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = angular_velocity_z
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81

        # TwistWithCovarianceStampedメッセージの作成
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = current_time.to_msg()
        twist_msg.header.frame_id = "base_link"
        # ダミーデータをセット
        twist_msg.twist.twist.linear.x = velocity_x
        twist_msg.twist.twist.angular.z = 0.0
        # 共分散行列を設定（必要に応じて）
        twist_msg.twist.covariance = [0.0] * 36  # 6x6の共分散行列

        # メッセージのパブリッシュ
        self.imu_publisher.publish(imu_msg)
        self.twist_publisher.publish(twist_msg)


def main():
    rclpy.init()
    node = ImuTwistPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
