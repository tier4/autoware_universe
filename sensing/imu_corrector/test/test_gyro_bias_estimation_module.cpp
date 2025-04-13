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

#include "../src/gyro_bias_estimation_module.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

namespace imu_corrector
{

class GyroBiasEstimationModuleTestWrapper : public GyroBiasEstimationModule {
public:
  GyroBiasEstimationModuleTestWrapper(
    double velocity_threshold,
    double timestamp_threshold,
    size_t data_num_threshold,
    double bias_change_threshold,
    const rclcpp::Logger & logger,
    const std::shared_ptr<rclcpp::Clock> & clock)
  : GyroBiasEstimationModule(
      velocity_threshold,
      timestamp_threshold,
      data_num_threshold,
      bias_change_threshold,
      logger,
      clock)
  {}
  size_t get_buffer_size() const { return gyro_buffer_.size(); }
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
    velocity_threshold, timestamp_threshold, data_num_threshold, bias_change_threshold, mock_logger_, mock_clock_);
};

TEST_F(GyroBiasEstimationModuleTest, GetBiasEstimationWhenVehicleStopped)
{
  geometry_msgs::msg::Vector3 gyro;
  gyro.x = 0.1;
  gyro.y = 0.2;
  gyro.z = 0.3;
  for (size_t i = 0; i < data_num_threshold + 1; ++i) {
    module.update_velocity(
      i * 0.1 * timestamp_threshold, 0.0);  // velocity = 0.0 < 1.0 = velocity_threshold
    module.update_gyro(i * 0.1 * timestamp_threshold, gyro);
  }
  ASSERT_NEAR(module.get_bias().x, gyro.x, 0.0001);
  ASSERT_NEAR(module.get_bias().y, gyro.y, 0.0001);
  ASSERT_NEAR(module.get_bias().z, gyro.z, 0.0001);
}

TEST_F(GyroBiasEstimationModuleTest, GetInsufficientDataException)
{
  ASSERT_THROW(module.get_bias(), std::runtime_error);
}

TEST_F(GyroBiasEstimationModuleTest, GetInsufficientDataExceptionWhenVehicleMoving)
{
  geometry_msgs::msg::Vector3 gyro;
  gyro.x = 0.1;
  gyro.y = 0.2;
  gyro.z = 0.3;
  for (size_t i = 0; i < data_num_threshold + 1; ++i) {
    module.update_velocity(
      i * 0.1 * timestamp_threshold, 5.0);  // velocity = 5.0 > 1.0 = velocity_threshold
    module.update_gyro(i * 0.1 * timestamp_threshold, gyro);
  }
  ASSERT_THROW(module.get_bias(), std::runtime_error);
}

// FSR01-3
TEST_F(GyroBiasEstimationModuleTest, RecordsImuDataToBuffer)
{
  geometry_msgs::msg::Vector3 gyro;
  gyro.x = 0.1;
  gyro.y = 0.2;
  gyro.z = 0.3;
  module.update_velocity(0.0, 0.0);
  module.update_gyro(0.0, gyro);
  ASSERT_EQ(module.get_buffer_size(), 1);
}



// FSR01-4

// FSR01-5

// FSR01-6

// FSR01-7


}  // namespace imu_corrector
