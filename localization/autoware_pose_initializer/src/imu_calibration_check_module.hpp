// Copyright 2024 TIER IV, Inc.
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

#ifndef IMU_CALIBRATION_CHECK_MODULE_HPP_
#define IMU_CALIBRATION_CHECK_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_calibration_msgs/msg/bool_stamped.hpp>

namespace autoware::pose_initializer
{
class ImuCalibrationCheckModule
{
public:
  explicit ImuCalibrationCheckModule(rclcpp::Node * node);
  bool isImuCalibrated() const;

private:
  using BoolStamped = tier4_calibration_msgs::msg::BoolStamped;
  rclcpp::Subscription<BoolStamped>::SharedPtr sub_imu_calibrated_;
  bool is_imu_calibrated_;

  void on_imu_calibrated(BoolStamped::ConstSharedPtr msg);
};
}  // namespace autoware::pose_initializer

#endif  // IMU_CALIBRATION_CHECK_MODULE_HPP_
