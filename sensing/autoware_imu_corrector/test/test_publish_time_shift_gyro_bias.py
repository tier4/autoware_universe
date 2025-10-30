# Copyright 2025 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.node import Node


class GyroBiasPublisher(Node):
    def __init__(self, time_offset):
        super().__init__("gyro_bias_publisher")
        self.time_offset = time_offset
        self.publisher = self.create_publisher(Vector3Stamped, "/gyro_bias", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Vector3Stamped()
        current_time = self.get_clock().now()
        msg.header.stamp = Time(sec=current_time.seconds_nanoseconds()[0] - self.time_offset)
        msg.header.frame_id = "base_link"
        msg.vector.x = 0.0
        msg.vector.y = 0.0
        msg.vector.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("Published message with timestamp 10 seconds ago")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--time-offset",
        type=int,
        default=10,
        help="Time offset in seconds for the published message",
    )

    rclpy.init()
    args = parser.parse_args()

    node = GyroBiasPublisher(args.time_offset)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
