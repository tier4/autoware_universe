// Copyright 2025 TIER IV, Inc.
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

#include "../src/gyro_bias_estimation_module.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

namespace imu_corrector
{

class GyroBiasEstimationModuleTestWrapper : public GyroBiasEstimationModule
{
public:
  GyroBiasEstimationModuleTestWrapper(
    double velocity_threshold, double timestamp_threshold, size_t data_num_threshold,
    double bias_change_threshold, const rclcpp::Logger & logger,
    const std::shared_ptr<rclcpp::Clock> & clock)
  : GyroBiasEstimationModule(
      velocity_threshold, timestamp_threshold, data_num_threshold, bias_change_threshold, logger,
      clock)
  {
  }
  size_t get_buffer_size() const { return gyro_buffer_.size(); }
  bool get_is_calibration_possible() const { return is_calibration_possible_; }
};

class GyroBiasEstimationModuleTest : public ::testing::Test
{
public:
  double velocity_threshold = 1.0;
  double timestamp_threshold = 0.1;
  size_t data_num_threshold = 5;
  double bias_change_threshold = 0.00061;
  rclcpp::Logger mock_logger_{rclcpp::get_logger("GyroBiasEstimationModuleTest")};
  std::shared_ptr<rclcpp::Clock> mock_clock_{std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)};

  GyroBiasEstimationModuleTestWrapper module = GyroBiasEstimationModuleTestWrapper(
    velocity_threshold, timestamp_threshold, data_num_threshold, bias_change_threshold,
    mock_logger_, mock_clock_);
};

}  // namespace imu_corrector
