// Copyright 2023 TIER IV, Inc.
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

#include "gyro_bias_estimator.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>

using namespace std::chrono_literals;  // NOLINT

namespace imu_corrector
{
GyroBiasEstimator::GyroBiasEstimator(const rclcpp::NodeOptions & node_options)
: Node("gyro_bias_validator", node_options),
  angular_velocity_offset_x_(declare_parameter<double>("angular_velocity_offset_x")),
  angular_velocity_offset_y_(declare_parameter<double>("angular_velocity_offset_y")),
  angular_velocity_offset_z_(declare_parameter<double>("angular_velocity_offset_z")),
  gyro_bias_(std::nullopt)
{
  const double timestamp_threshold = declare_parameter<double>("timestamp_threshold");
  const size_t data_num_threshold =
    static_cast<size_t>(declare_parameter<int>("data_num_threshold"));
  const double bias_change_threshold = declare_parameter<double>("bias_change_threshold");
  const double stddev_threshold = declare_parameter<double>("stddev_threshold");
  gyro_bias_estimation_module_ = std::make_unique<GyroBiasEstimationModule>(
    timestamp_threshold, data_num_threshold, bias_change_threshold, stddev_threshold, get_logger(),
    get_clock());

  imu_sub_ = create_subscription<Imu>(
    "~/input/imu_raw", rclcpp::SensorDataQoS(),
    [this](const Imu::ConstSharedPtr msg) { callback_imu(msg); });
  twist_sub_ = create_subscription<TwistWithCovarianceStamped>(
    "~/input/twist", rclcpp::SensorDataQoS(),
    [this](const TwistWithCovarianceStamped::ConstSharedPtr msg) { callback_twist(msg); });

  timer_ =
    rclcpp::create_timer(this, get_clock(), 1000ms, std::bind(&GyroBiasEstimator::on_timer, this));

  gyro_bias_pub_ = create_publisher<Vector3Stamped>("~/output/gyro_bias", rclcpp::SensorDataQoS());
}

void GyroBiasEstimator::callback_imu(const Imu::ConstSharedPtr imu_msg_ptr)
{
  // Update gyro data
  gyro_bias_estimation_module_->update_gyro(
    rclcpp::Time(imu_msg_ptr->header.stamp).seconds(), imu_msg_ptr->angular_velocity);
}

void GyroBiasEstimator::on_timer()
{
  // Estimate gyro bias
  try {
    gyro_bias_ = gyro_bias_estimation_module_->get_bias();
  } catch (const std::runtime_error & e) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 1000, e.what());
  }
  // Publish results for debugging
  if (gyro_bias_ != std::nullopt) {
    Vector3Stamped gyro_bias_msg;
    gyro_bias_msg.header.stamp = this->now();
    gyro_bias_msg.vector = gyro_bias_.value();
    gyro_bias_pub_->publish(gyro_bias_msg);
  }
}

void GyroBiasEstimator::callback_twist(
  const TwistWithCovarianceStamped::ConstSharedPtr twist_msg_ptr)
{
  gyro_bias_estimation_module_->update_velocity(
    rclcpp::Time(twist_msg_ptr->header.stamp).seconds(), twist_msg_ptr->twist.twist.linear.x);
}

}  // namespace imu_corrector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_corrector::GyroBiasEstimator)
