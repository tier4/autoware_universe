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

#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace autoware::imu_corrector
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
  can_calibrate_(false)
{
}

void GyroBiasEstimationModule::update_gyro(
  const double time, const geometry_msgs::msg::Vector3 & gyro)
{
  if (time - last_velocity_time_ > timestamp_threshold_) {
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
  geometry_msgs::msg::Vector3 buffer_stddev;
  if (is_gyro_buffer_full_) {
    buffer_stddev = calculate_stddev(gyro_buffer_);
    update_can_calibrate_flag(buffer_stddev);
    if (can_calibrate_) {
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
  }
  if (current_median_.has_value()) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 10000,
      "Bias: [x: %f, y: %f, z: %f] rad/s, Stddev: [x: %f, y: %f, z: %f] rad/s",
      current_median_.value().x, current_median_.value().y, current_median_.value().z,
      buffer_stddev.x, buffer_stddev.y, buffer_stddev.z);
  } else {
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 10000, "Bias and stddev is not ready");
  }
  return current_median_;
}

void GyroBiasEstimationModule::update_gyro_buffer_full_flag(
  boost::circular_buffer<geometry_msgs::msg::Vector3> & buffer)
{
  is_gyro_buffer_full_ = buffer.full();
}

void GyroBiasEstimationModule::update_can_calibrate_flag(
  const geometry_msgs::msg::Vector3 & buffer_stddev)
{
  can_calibrate_ = buffer_stddev.x <= stddev_threshold_ && buffer_stddev.y <= stddev_threshold_ &&
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

/**
 * @brief perform dead reckoning based on "gyro_list" and return a relative pose (in RPY)
 */
geometry_msgs::msg::Vector3 integrate_orientation(
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list,
  const geometry_msgs::msg::Vector3 & gyro_bias)
{
  geometry_msgs::msg::Vector3 d_rpy{};
  double t_prev = rclcpp::Time(gyro_list.front().header.stamp).seconds();
  for (std::size_t i = 0; i < gyro_list.size() - 1; ++i) {
    double t_cur = rclcpp::Time(gyro_list[i + 1].header.stamp).seconds();

    d_rpy.x += (t_cur - t_prev) * (gyro_list[i].vector.x - gyro_bias.x);
    d_rpy.y += (t_cur - t_prev) * (gyro_list[i].vector.y - gyro_bias.y);
    d_rpy.z += (t_cur - t_prev) * (gyro_list[i].vector.z - gyro_bias.z);

    t_prev = t_cur;
  }
  return d_rpy;
}

/**
 * @brief calculate RPY error on dead-reckoning (calculated from "gyro_list") compared to the
 * ground-truth pose from "pose_list".
 */
geometry_msgs::msg::Vector3 calculate_error_rpy(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list,
  const geometry_msgs::msg::Vector3 & gyro_bias)
{
  const geometry_msgs::msg::Vector3 rpy_0 =
    autoware_utils::get_rpy(pose_list.front().pose.orientation);
  const geometry_msgs::msg::Vector3 rpy_1 =
    autoware_utils::get_rpy(pose_list.back().pose.orientation);
  const geometry_msgs::msg::Vector3 d_rpy = integrate_orientation(gyro_list, gyro_bias);

  geometry_msgs::msg::Vector3 error_rpy;
  error_rpy.x = autoware_utils::normalize_radian(-rpy_1.x + rpy_0.x + d_rpy.x);
  error_rpy.y = autoware_utils::normalize_radian(-rpy_1.y + rpy_0.y + d_rpy.y);
  error_rpy.z = autoware_utils::normalize_radian(-rpy_1.z + rpy_0.z + d_rpy.z);
  return error_rpy;
}

/**
 * @brief update gyroscope bias based on a given trajectory data
 */
void GyroBiasEstimationModule::update_bias(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list)
{
  const double dt_pose =
    (rclcpp::Time(pose_list.back().header.stamp) - rclcpp::Time(pose_list.front().header.stamp))
      .seconds();
  const double dt_gyro =
    (rclcpp::Time(gyro_list.back().header.stamp) - rclcpp::Time(gyro_list.front().header.stamp))
      .seconds();
  if (dt_pose == 0 || dt_gyro == 0) {
    throw std::runtime_error("dt_pose or dt_gyro is zero");
  }

  auto error_rpy = calculate_error_rpy(pose_list, gyro_list, geometry_msgs::msg::Vector3{});
  error_rpy.x *= dt_pose / dt_gyro;
  error_rpy.y *= dt_pose / dt_gyro;
  error_rpy.z *= dt_pose / dt_gyro;

  gyro_bias_pair_.first.x += dt_pose * error_rpy.x;
  gyro_bias_pair_.first.y += dt_pose * error_rpy.y;
  gyro_bias_pair_.first.z += dt_pose * error_rpy.z;
  gyro_bias_pair_.second.x += dt_pose * dt_pose;
  gyro_bias_pair_.second.y += dt_pose * dt_pose;
  gyro_bias_pair_.second.z += dt_pose * dt_pose;

  geometry_msgs::msg::Vector3 gyro_bias;
  gyro_bias.x = error_rpy.x / dt_pose;
  gyro_bias.y = error_rpy.y / dt_pose;
  gyro_bias.z = error_rpy.z / dt_pose;
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

/**
 * @brief getter function for current estimated bias
 */
geometry_msgs::msg::Vector3 GyroBiasEstimationModule::get_bias_base_link() const
{
  geometry_msgs::msg::Vector3 gyro_bias_base;
  if (
    gyro_bias_pair_.second.x == 0 || gyro_bias_pair_.second.y == 0 ||
    gyro_bias_pair_.second.z == 0) {
    throw std::runtime_error("gyro_bias_pair_.second is zero");
  }
  gyro_bias_base.x = gyro_bias_pair_.first.x / gyro_bias_pair_.second.x;
  gyro_bias_base.y = gyro_bias_pair_.first.y / gyro_bias_pair_.second.y;
  gyro_bias_base.z = gyro_bias_pair_.first.z / gyro_bias_pair_.second.z;
  return gyro_bias_base;
}

}  // namespace autoware::imu_corrector
