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

#include "gyro_bias_estimation_module.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;  // NOLINT

namespace imu_corrector
{
GyroBiasEstimationModule::GyroBiasEstimationModule(
  const double timestamp_threshold, const size_t data_num_threshold,
  const double bias_change_threshold, const double stddev_threshold, rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock)
: timestamp_threshold_(timestamp_threshold),
  data_num_threshold_(data_num_threshold),
  bias_change_threshold_(bias_change_threshold),
  stddev_threshold_(stddev_threshold),
  current_median_(std::nullopt),
  current_stddev_(),
  is_stopped_(false),
  last_velocity_time_(0.0),
  logger_(logger),
  clock_(clock),
  gyro_buffer_(data_num_threshold),
  is_gyro_buffer_full_(false),
  is_calibratable_(false)
{
}

void GyroBiasEstimationModule::update_gyro(
  const double time, const geometry_msgs::msg::Vector3 & gyro)
{
  if (time - last_velocity_time_ < timestamp_threshold_) {
    if (is_stopped_) {
      gyro_buffer_.push_back(gyro);
    } else {
      gyro_buffer_.clear();
    }
  }
}

void GyroBiasEstimationModule::update_velocity(const double time, const double velocity)
{
  if (velocity == 0.0) {
    last_velocity_time_ = time;
    is_stopped_ = true;
  } else {
    is_stopped_ = false;
  }
}

std::optional<geometry_msgs::msg::Vector3> GyroBiasEstimationModule::get_bias()
{
  update_gyro_buffer_full_flag(gyro_buffer_);
  if (!is_gyro_buffer_full_) {
    throw std::runtime_error("Bias estimation is not yet ready because of insufficient data.");
  }
  geometry_msgs::msg::Vector3 buffer_stddev = calculate_stddev(gyro_buffer_);
  update_calibratable_flag(buffer_stddev);
  if (is_calibratable_) {
    geometry_msgs::msg::Vector3 previous_median;
    if (current_median_ == std::nullopt) {
      current_median_ = calculate_median(gyro_buffer_);
      previous_median = current_median_.value();
    } else {
      previous_median = current_median_.value();
      current_median_ = calculate_median(gyro_buffer_);
    }
    current_stddev_ = buffer_stddev;
    if (
      abs(current_median_.value().x - previous_median.x) > bias_change_threshold_ ||
      abs(current_median_.value().y - previous_median.y) > bias_change_threshold_ ||
      abs(current_median_.value().z - previous_median.z) > bias_change_threshold_) {
      RCLCPP_WARN(
        logger_,
        "Significant gyro bias change detected!\n"
        "Previous bias: [x: %f, y: %f, z: %f] rad/s\n"
        "Current bias: [x: %f, y: %f, z: %f] rad/s\n"
        "Previous standard dev: [x: %f, y: %f, z: %f] rad/s\n"
        "Current standard dev: [x: %f, y: %f, z: %f] rad/s",
        previous_median.x, previous_median.y, previous_median.z, current_median_.value().x,
        current_median_.value().y, current_median_.value().z, buffer_stddev.x, buffer_stddev.y,
        buffer_stddev.z, current_stddev_.x, current_stddev_.y, current_stddev_.z);
    }
  }
  if (current_median_.has_value()) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 10000,
      "Bias: [x: %f, y: %f, z: %f] rad/s, Stddev: [x: %f, y: %f, z: %f] "
      "rad/s",
      current_median_.value().x, current_median_.value().y, current_median_.value().z,
      buffer_stddev.x, buffer_stddev.y, buffer_stddev.z);
  }
  return current_median_;
}

void GyroBiasEstimationModule::update_gyro_buffer_full_flag(
  boost::circular_buffer<geometry_msgs::msg::Vector3> & buffer)
{
  is_gyro_buffer_full_ = buffer.full();
}

void GyroBiasEstimationModule::update_calibratable_flag(
  const geometry_msgs::msg::Vector3 & buffer_stddev)
{
  is_calibratable_ = buffer_stddev.x <= stddev_threshold_ && buffer_stddev.y <= stddev_threshold_ &&
                     buffer_stddev.z <= stddev_threshold_;
}

geometry_msgs::msg::Vector3 GyroBiasEstimationModule::calculate_stddev(
  const boost::circular_buffer<geometry_msgs::msg::Vector3> & buffer) const
{
  if (buffer.empty()) {
    throw std::runtime_error("Buffer is empty.");
  }

  // 各軸の合計値を計算
  geometry_msgs::msg::Vector3 sum;
  for (const auto & gyro : buffer) {
    sum.x += gyro.x;
    sum.y += gyro.y;
    sum.z += gyro.z;
  }

  geometry_msgs::msg::Vector3 mean;
  mean.x = sum.x / buffer.size();
  mean.y = sum.y / buffer.size();
  mean.z = sum.z / buffer.size();

  geometry_msgs::msg::Vector3 variance;
  for (const auto & gyro : buffer) {
    variance.x += (gyro.x - mean.x) * (gyro.x - mean.x);
    variance.y += (gyro.y - mean.y) * (gyro.y - mean.y);
    variance.z += (gyro.z - mean.z) * (gyro.z - mean.z);
  }
  variance.x /= (buffer.size() - 1);
  variance.y /= (buffer.size() - 1);
  variance.z /= (buffer.size() - 1);

  geometry_msgs::msg::Vector3 stddev;
  stddev.x = std::sqrt(variance.x);
  stddev.y = std::sqrt(variance.y);
  stddev.z = std::sqrt(variance.z);

  return stddev;
}

geometry_msgs::msg::Vector3 GyroBiasEstimationModule::calculate_median(
  const boost::circular_buffer<geometry_msgs::msg::Vector3> & buffer) const
{
  if (buffer.empty()) {
    throw std::runtime_error("Buffer is empty.");
  }

  std::vector<double> x_values, y_values, z_values;
  x_values.reserve(buffer.size());
  y_values.reserve(buffer.size());
  z_values.reserve(buffer.size());

  for (const auto & gyro : buffer) {
    x_values.push_back(gyro.x);
    y_values.push_back(gyro.y);
    z_values.push_back(gyro.z);
  }

  std::sort(x_values.begin(), x_values.end());
  std::sort(y_values.begin(), y_values.end());
  std::sort(z_values.begin(), z_values.end());

  geometry_msgs::msg::Vector3 median;
  size_t mid = buffer.size() / 2;
  if (buffer.size() % 2 == 0) {
    median.x = (x_values[mid - 1] + x_values[mid]) / 2.0;
    median.y = (y_values[mid - 1] + y_values[mid]) / 2.0;
    median.z = (z_values[mid - 1] + z_values[mid]) / 2.0;
  } else {
    median.x = x_values[mid];
    median.y = y_values[mid];
    median.z = z_values[mid];
  }

  return median;
}

}  // namespace imu_corrector
