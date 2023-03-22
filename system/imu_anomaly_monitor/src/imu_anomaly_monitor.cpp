// Copyright 2021 TierIV
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "imu_anomaly_monitor/imu_anomaly_monitor.hpp"
#include <eigen3/Eigen/Core>


ImuAnomalyMonitor::ImuAnomalyMonitor() : Node("imu_anomaly_monitor")
{
  // set covariance value for twist with covariance msg
  stddev_vx_ = declare_parameter("velocity_stddev_xx", 0.2);
  stddev_wz_ = declare_parameter("angular_velocity_stddev_zz", 0.1);
  frame_id_ = declare_parameter("frame_id", "base_link");

  vehicle_report_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "~/input/velocity_status", rclcpp::QoS{100},
    std::bind(&ImuAnomalyMonitor::on_velocity_report, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "~/input/imu", rclcpp::QoS{100},
    std::bind(&ImuAnomalyMonitor::on_imu, this, std::placeholders::_1));

  imu_yaw_rate_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "imu_yaw_rate", rclcpp::QoS{10});

  vehicle_yaw_rate_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "vehicle_yaw_rate", rclcpp::QoS{10});

  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);

  imu_filter_.set_proc_dev(4.0);
  vehicle_filter_.set_proc_dev(1.0);
}

void ImuAnomalyMonitor::on_velocity_report(
  const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg)
{
  if (msg->header.frame_id != frame_id_) {
    RCLCPP_WARN(get_logger(), "frame_id is not base_link.");
    return;
  }

  auto yaw_rate =  msg->heading_rate;
  vehicle_filter_.update(yaw_rate, 0.0093, msg->header.stamp);
  yaw_rate = vehicle_filter_.get_x();

  auto yaw_rate_msg = std::make_unique<tier4_debug_msgs::msg::Float32Stamped>();
  yaw_rate_msg->stamp = this->now();
  yaw_rate_msg->data = yaw_rate;
  vehicle_yaw_rate_pub_->publish(std::move(yaw_rate_msg));
}

void ImuAnomalyMonitor::on_imu(
  const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  auto imu_frame_ = msg->header.frame_id;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf2_imu_link_to_base_link =
    transform_listener_->getLatestTransform(imu_frame_, frame_id_);
  if (!tf2_imu_link_to_base_link) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", frame_id_.c_str(),
      (imu_frame_).c_str());
    return;
  }

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.vector = msg->angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf2_imu_link_to_base_link);

  auto yaw_rate =  transformed_angular_velocity.vector.z;
  // imu_filter_.update(yaw_rate, 0.0093, msg->header.stamp);
  // yaw_rate = imu_filter_.get_x();

  auto yaw_rate_msg = std::make_unique<tier4_debug_msgs::msg::Float32Stamped>();
  yaw_rate_msg->stamp = this->now();
  yaw_rate_msg->data = yaw_rate;
  imu_yaw_rate_pub_->publish(std::move(yaw_rate_msg));
}