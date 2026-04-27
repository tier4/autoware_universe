"""ROS 2 node that publishes tf and localization from CARLA ground truth.

When splatsim is active, LiDAR-based localization is unavailable.  This
node bridges the CARLA ground-truth ego pose to the localization output
topics that Autoware expects, following the same pattern as
``simple_planning_simulator``'s publish methods.

Published topics:
  * ``/tf``  (map → base_link)
  * ``/localization/kinematic_state``  (nav_msgs/Odometry)
  * ``/localization/pose_estimator/pose_with_covariance``
  * ``/localization/acceleration``  (AccelWithCovarianceStamped)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import (
    AccelWithCovarianceStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from autoware_vehicle_msgs.msg import VelocityReport
from tf2_msgs.msg import TFMessage


class PosePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("splatsim_pose_publisher")

        # ── Parameters ──
        self.declare_parameter("origin_frame_id", "map")
        self.declare_parameter("simulated_frame_id", "base_link")
        self.declare_parameter("publish_rate", 50.0)

        self._origin_frame = self.get_parameter("origin_frame_id").value
        self._child_frame = self.get_parameter("simulated_frame_id").value
        publish_rate = self.get_parameter("publish_rate").value

        # ── Cached data from subscribers ──
        self._latest_pose: PoseWithCovarianceStamped | None = None
        self._latest_velocity: VelocityReport | None = None
        self._latest_imu: Imu | None = None

        # ── Subscribers ──
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/sensing/gnss/pose_with_covariance",
            self._pose_cb,
            10,
        )
        self.create_subscription(
            VelocityReport,
            "/vehicle/status/velocity_status",
            self._velocity_cb,
            10,
        )
        self.create_subscription(
            Imu,
            "/sensing/imu/tamagawa/imu_raw",
            self._imu_cb,
            10,
        )

        # ── Publishers ──
        tf_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub_tf = self.create_publisher(TFMessage, "/tf", tf_qos)
        self._pub_odom = self.create_publisher(
            Odometry, "/localization/kinematic_state", 10
        )
        self._pub_pose = self.create_publisher(
            PoseWithCovarianceStamped,
            "/localization/pose_estimator/pose_with_covariance",
            10,
        )
        self._pub_accel = self.create_publisher(
            AccelWithCovarianceStamped, "/localization/acceleration", 10
        )

        # ── Timer ──
        period = 1.0 / publish_rate
        self._timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            "PosePublisher: %.0f Hz, %s → %s",
            publish_rate,
            self._origin_frame,
            self._child_frame,
        )

    # ── Subscriber callbacks ──

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_pose = msg

    def _velocity_cb(self, msg: VelocityReport) -> None:
        self._latest_velocity = msg

    def _imu_cb(self, msg: Imu) -> None:
        self._latest_imu = msg

    # ── Timer callback ──

    def _timer_callback(self) -> None:
        if self._latest_pose is None:
            return

        now = self.get_clock().now().to_msg()
        pose = self._latest_pose.pose.pose

        self._publish_tf(now, pose)
        self._publish_kinematic_state(now, pose)
        self._publish_pose_with_covariance(now, pose)
        self._publish_acceleration(now)

    def _publish_tf(self, stamp, pose) -> None:
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._origin_frame
        tf.child_frame_id = self._child_frame
        tf.transform.translation.x = pose.position.x
        tf.transform.translation.y = pose.position.y
        tf.transform.translation.z = pose.position.z
        tf.transform.rotation = pose.orientation

        tf_msg = TFMessage()
        tf_msg.transforms.append(tf)
        self._pub_tf.publish(tf_msg)

    def _publish_kinematic_state(self, stamp, pose) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._origin_frame
        odom.child_frame_id = self._child_frame
        odom.pose.pose = pose

        # Covariance (diagonal, small values matching simple_planning_simulator)
        cov = odom.pose.covariance
        cov[0] = 0.0001  # x
        cov[7] = 0.0001  # y
        cov[14] = 0.0001  # z
        cov[21] = 0.0001  # roll
        cov[28] = 0.0001  # pitch
        cov[35] = 0.0001  # yaw

        if self._latest_velocity is not None:
            odom.twist.twist.linear.x = float(self._latest_velocity.longitudinal_velocity)
            odom.twist.twist.linear.y = float(self._latest_velocity.lateral_velocity)
            odom.twist.twist.angular.z = float(self._latest_velocity.heading_rate)

        self._pub_odom.publish(odom)

    def _publish_pose_with_covariance(self, stamp, pose) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._origin_frame
        msg.pose.pose = pose

        cov = msg.pose.covariance
        cov[0] = 0.0001
        cov[7] = 0.0001
        cov[14] = 0.0001
        cov[21] = 0.0001
        cov[28] = 0.0001
        cov[35] = 0.0001

        self._pub_pose.publish(msg)

    def _publish_acceleration(self, stamp) -> None:
        msg = AccelWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._child_frame

        if self._latest_imu is not None:
            msg.accel.accel.linear.x = self._latest_imu.linear_acceleration.x
            msg.accel.accel.linear.y = self._latest_imu.linear_acceleration.y
            msg.accel.accel.linear.z = self._latest_imu.linear_acceleration.z

        self._pub_accel.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
