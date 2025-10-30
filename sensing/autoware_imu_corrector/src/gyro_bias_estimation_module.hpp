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

#ifndef GYRO_BIAS_ESTIMATION_MODULE_HPP_
#define GYRO_BIAS_ESTIMATION_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <boost/circular_buffer.hpp>

#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::imu_corrector
{
class GyroBiasEstimationModule
{
public:
  GyroBiasEstimationModule(
    const double timestamp_threshold, const size_t data_num_threshold,
    const double bias_change_threshold, const double stddev_threshold, rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock);
  void update_bias(
    const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
    const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list);
  [[nodiscard]] geometry_msgs::msg::Vector3 get_bias_base_link() const;
  std::optional<geometry_msgs::msg::Vector3> get_bias();
  void update_gyro(const double time, const geometry_msgs::msg::Vector3 & gyro);
  void update_velocity(const double time, const double velocity);

private:
  std::pair<geometry_msgs::msg::Vector3, geometry_msgs::msg::Vector3> gyro_bias_pair_;
  double timestamp_threshold_;
  size_t data_num_threshold_;
  double bias_change_threshold_;
  double stddev_threshold_;
  std::optional<geometry_msgs::msg::Vector3> current_median_;
  geometry_msgs::msg::Vector3 current_stddev_;
  bool is_stopped_;
  double last_velocity_time_;
  geometry_msgs::msg::Vector3 calculate_stddev(
    const boost::circular_buffer<geometry_msgs::msg::Vector3> & buffer) const;
  geometry_msgs::msg::Vector3 calculate_median(
    const boost::circular_buffer<geometry_msgs::msg::Vector3> & buffer) const;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

protected:
  boost::circular_buffer<geometry_msgs::msg::Vector3> gyro_buffer_;
  bool is_gyro_buffer_full_;
  bool can_calibrate_;
  virtual void update_gyro_buffer_full_flag(
    boost::circular_buffer<geometry_msgs::msg::Vector3> & buffer);
  void update_can_calibrate_flag(const geometry_msgs::msg::Vector3 & buffer_stddev);
};
}  // namespace autoware::imu_corrector

#endif  // GYRO_BIAS_ESTIMATION_MODULE_HPP_
