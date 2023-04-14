// Copyright 2023 TierIV
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

#include "imu_monitor/imu_monitor.hpp"

#include "tier4_autoware_utils/ros/msg_covariance.hpp"

namespace imu_monitor
{

ImuMonitor::ImuMonitor(const rclcpp::NodeOptions & node_options)
: Node("imu_monitor", node_options), updater_(this)
{
  yaw_rate_diff_threshold_ = declare_parameter("yaw_rate_diff_threshold", 0.07);
  frame_id_ = declare_parameter("frame_id", "base_link");

  twist_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "~/input/twist", rclcpp::QoS{100},
    std::bind(&ImuMonitor::on_twist, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "~/input/imu", rclcpp::QoS{100}, std::bind(&ImuMonitor::on_imu, this, std::placeholders::_1));

  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);

  imu_filter_.set_proc_dev(0.1);
  twist_filter_.set_proc_dev(0.1);

  // Diagnostics Updater
  updater_.setHardwareID("imu_monitor");
  updater_.add("yaw_rate_status", this, &ImuMonitor::check_yaw_rate);
  updater_.setPeriod(0.1);
}

void ImuMonitor::on_twist(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
  if (msg->header.frame_id != frame_id_) {
    RCLCPP_WARN(get_logger(), "frame_id is not base_link.");
    return;
  }

  twist_ptr_ = msg;

  using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  twist_filter_.update(
    twist_ptr_->twist.twist.angular.z, twist_ptr_->twist.covariance[COV_IDX::YAW_YAW],
    twist_ptr_->header.stamp);
  twist_yaw_rate_ = twist_filter_.get_x();
}

void ImuMonitor::on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  auto imu_frame_ = msg->header.frame_id;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf2_imu_link_to_base_link =
    transform_listener_->getLatestTransform(imu_frame_, frame_id_);
  if (!tf2_imu_link_to_base_link) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", frame_id_.c_str(), (imu_frame_).c_str());
    return;
  }
  imu_ptr_ = msg;

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.vector = imu_ptr_->angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf2_imu_link_to_base_link);

  imu_filter_.update(
    transformed_angular_velocity.vector.z, imu_ptr_->angular_velocity_covariance[8],
    imu_ptr_->header.stamp);
  imu_yaw_rate_ = imu_filter_.get_x();
}

void ImuMonitor::check_yaw_rate(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!twist_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for twist info...");
    return;
  }

  if (!imu_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for imu info...");
    return;
  }

  diagnostic_msgs::msg::DiagnosticStatus status;
  if (std::abs(imu_yaw_rate_ - twist_yaw_rate_) > yaw_rate_diff_threshold_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "The yaw_rate deviation is large";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  }
  stat.addf("yaw rate from imu", "%lf", imu_yaw_rate_);
  stat.addf("yaw rate from twist", "%lf", twist_yaw_rate_);
  stat.summary(status.level, status.message);
}

}  // namespace imu_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_monitor::ImuMonitor)
