// Copyright 2020 Tier IV, Inc.
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

#include "imu_corrector_core.hpp"

#include <algorithm>
#include <chrono>
#include <string>

using namespace std::chrono_literals;  // NOLINT

namespace imu_corrector
{
ImuCorrector::ImuCorrector(const rclcpp::NodeOptions & node_options)
: Node("imu_corrector", node_options)
{
  sensor_model_ = declare_parameter<std::string>("sensor_model", "aip_x1");

  if (sensor_model_ == "aip_x1") {
    angular_velocity_offset_x_ = declare_parameter<double>("angular_velocity_offset_x", 0.0);
    angular_velocity_offset_y_ = declare_parameter<double>("angular_velocity_offset_y", 0.0);
    angular_velocity_offset_z_ = declare_parameter<double>("angular_velocity_offset_z", 0.0);
  }
  angular_velocity_stddev_xx_ = declare_parameter<double>("angular_velocity_stddev_xx", 0.03);
  angular_velocity_stddev_yy_ = declare_parameter<double>("angular_velocity_stddev_yy", 0.03);
  angular_velocity_stddev_zz_ = declare_parameter<double>("angular_velocity_stddev_zz", 0.03);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "input", rclcpp::QoS{1}, std::bind(&ImuCorrector::callbackImu, this, std::placeholders::_1));
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("output", rclcpp::QoS{10});
  gyro_bias_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "gyro_bias", rclcpp::SensorDataQoS(),
    std::bind(&ImuCorrector::callbackGyroBias, this, std::placeholders::_1));
  is_calibrated_pub_ =
    create_publisher<tier4_calibration_msgs::msg::BoolStamped>("is_calibrated", rclcpp::QoS{1});
  timer_ = rclcpp::create_timer(
    this, get_clock(), 1000ms, std::bind(&ImuCorrector::callbackIsCalibrated, this));
}

void ImuCorrector::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg = *imu_msg_ptr;
  if (angular_velocity_offset_x_ != std::nullopt) {
    imu_msg.angular_velocity.x += angular_velocity_offset_x_.value();
  }
  if (angular_velocity_offset_y_ != std::nullopt) {
    imu_msg.angular_velocity.y += angular_velocity_offset_y_.value();
  }
  if (angular_velocity_offset_z_ != std::nullopt) {
    imu_msg.angular_velocity.z += angular_velocity_offset_z_.value();
  }
  imu_msg.angular_velocity_covariance[0 * 3 + 0] =
    angular_velocity_stddev_xx_ * angular_velocity_stddev_xx_;
  imu_msg.angular_velocity_covariance[1 * 3 + 1] =
    angular_velocity_stddev_yy_ * angular_velocity_stddev_yy_;
  imu_msg.angular_velocity_covariance[2 * 3 + 2] =
    angular_velocity_stddev_zz_ * angular_velocity_stddev_zz_;
  imu_pub_->publish(imu_msg);
}

void ImuCorrector::callbackGyroBias(
  const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr gyro_bias_msg_ptr)
{
  const double elapsed_time =
    std::abs(rclcpp::Time(gyro_bias_msg_ptr->header.stamp).seconds() - this->now().seconds());
  if (elapsed_time < 10.0) {
    angular_velocity_offset_x_ = gyro_bias_msg_ptr->vector.x;
    angular_velocity_offset_y_ = gyro_bias_msg_ptr->vector.y;
    angular_velocity_offset_z_ = gyro_bias_msg_ptr->vector.z;
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "/sensing/imu/gyro_bias timeout detected . Last data received %f[s] ago. Threshold: "
      "10.000[s].",
      elapsed_time);
  }
}

void ImuCorrector::callbackIsCalibrated()
{
  tier4_calibration_msgs::msg::BoolStamped is_calibrated_msg;
  if (
    angular_velocity_offset_x_ != std::nullopt && angular_velocity_offset_y_ != std::nullopt &&
    angular_velocity_offset_z_ != std::nullopt) {
    is_calibrated_msg.data = true;
  } else {
    is_calibrated_msg.data = false;
  }
  is_calibrated_msg.header.stamp = this->now();
  is_calibrated_msg.header.frame_id = "";
  is_calibrated_pub_->publish(is_calibrated_msg);
}
}  // namespace imu_corrector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_corrector::ImuCorrector)
