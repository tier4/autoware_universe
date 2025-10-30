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

#include "imu_calibration_check_module.hpp"

namespace autoware::pose_initializer
{
ImuCalibrationCheckModule::ImuCalibrationCheckModule(rclcpp::Node * node)
: is_imu_calibrated_(false)
{
  sub_imu_calibrated_ = node->create_subscription<BoolStamped>(
    "/sensing/imu/is_calibrated", 1,
    std::bind(&ImuCalibrationCheckModule::on_imu_calibrated, this, std::placeholders::_1));
}

bool ImuCalibrationCheckModule::isImuCalibrated() const
{
  return is_imu_calibrated_;
}

void ImuCalibrationCheckModule::on_imu_calibrated(BoolStamped::ConstSharedPtr msg)
{
  is_imu_calibrated_ = msg->data;
}
}  // namespace autoware::pose_initializer
