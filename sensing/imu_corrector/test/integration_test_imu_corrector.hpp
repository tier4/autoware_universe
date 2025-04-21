// Copyright 2025 Tier IV, Inc.
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

#ifndef INTEGRATION_TEST_IMU_CORRECTOR_HPP_
#define INTEGRATION_TEST_IMU_CORRECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tier4_calibration_msgs/msg/bool_stamped.hpp>

#include <gtest/gtest.h>

#include <memory>

class TestImuCorrector : public ::testing::Test
{
protected:
  void SetUp() override;
  void TearDown() override;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyro_bias_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  sensor_msgs::msg::Imu received_imu_;
};

#endif  // INTEGRATION_TEST_IMU_CORRECTOR_HPP_
