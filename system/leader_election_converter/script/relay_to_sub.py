#!/usr/bin/env python3

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_auto_planning_msgs.msg import Trajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from tier4_system_msgs.msg import OperationModeAvailability


class RemapNode(Node):
    def __init__(self):
        super().__init__("remap_node")

        self.create_subscription(
            OperationModeAvailability,
            "/system/operation_mode/availability",
            self.operation_mode_callback,
            10,
        )
        self.create_subscription(
            OperationModeState,
            "/system/operation_mode/state",
            self.operation_mode_state_callback,
            10,
        )
        self.sub_trajectory = self.create_subscription(
            Trajectory, "/planning/scenario_planning/trajectory", self.trajectory_callback, 10
        )
        self.sub_pose_with_covariance = self.create_subscription(
            PoseWithCovarianceStamped, "/localization/pose_with_covariance", self.pose_callback, 10
        )
        # self.sub_initialpose3d = self.create_subscription(PoseWithCovarianceStamped, '/initialpose3d', self.initialpose_callback, 10)

        self.pub_trajectory = self.create_publisher(
            Trajectory, "/to_sub/planning/scenario_planning/trajectory", 10
        )
        self.pub_pose_with_covariance = self.create_publisher(
            PoseWithCovarianceStamped, "/to_sub/localization/pose_with_covariance", 10
        )
        # self.pub_initialpose3d = self.create_publisher(PoseWithCovarianceStamped, '/to_sub/initialpose3d', 10)

        self.autonomous_mode = False
        self.operation_mode_autonomous_state = False
        self.get_logger().info(f"Initial autonomous mode: {self.autonomous_mode}")
        self.tmp_operation_mode_autonomous_state = False

    def operation_mode_callback(self, msg):
        if msg.autonomous != self.autonomous_mode:
            self.autonomous_mode = msg.autonomous
            self.get_logger().info(f"Autonomous mode changed: {self.autonomous_mode}")

    def operation_mode_state_callback(self, msg):
        self.tmp_operation_mode_autonomous_state = self.operation_mode_autonomous_state
        if msg.mode == 2:
            self.operation_mode_autonomous_state = True
            if self.tmp_operation_mode_autonomous_state != self.operation_mode_autonomous_state:
                self.get_logger().info(f"Operation mode changed: {self.operation_mode_autonomous_state}")
        else:
            self.operation_mode_autonomous_state = False
            if self.tmp_operation_mode_autonomous_state != self.operation_mode_autonomous_state:
                self.get_logger().info(f"Operation mode changed: {self.operation_mode_autonomous_state}")

    def trajectory_callback(self, msg):
        if self.autonomous_mode or self.operation_mode_autonomous_state == False:
            self.pub_trajectory.publish(msg)

    def pose_callback(self, msg):
        if self.autonomous_mode or self.operation_mode_autonomous_state == False:
            self.pub_pose_with_covariance.publish(msg)

    # def initialpose_callback(self, msg):
    #     if self.autonomous_mode:
    #         self.pub_initialpose3d.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RemapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
