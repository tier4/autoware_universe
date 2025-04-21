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

#include "integration_test_imu_corrector.hpp"

void TestImuCorrector::SetUp()
{
  rclcpp::init(0, nullptr);
  node_ = std::make_shared<rclcpp::Node>("test_imu_corrector_node");

  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("input", 1);
  gyro_bias_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("gyro_bias", 1);

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "output", 1, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { received_imu_ = *msg; });
}

void TestImuCorrector::TearDown() { rclcpp::shutdown(); }

TEST_F(TestImuCorrector, TestGyroBiasCorrection)
{
  // 入力IMUデータの作成
  sensor_msgs::msg::Imu input_imu;
  input_imu.header.stamp = node_->now();
  input_imu.angular_velocity.x = 1.0;
  input_imu.angular_velocity.y = 2.0;
  input_imu.angular_velocity.z = 3.0;

  // ジャイロバイアスデータの作成
  geometry_msgs::msg::Vector3Stamped gyro_bias;
  gyro_bias.header.stamp = node_->now();
  gyro_bias.vector.x = 0.1;
  gyro_bias.vector.y = 0.2;
  gyro_bias.vector.z = 0.3;

  // メッセージの配信
  imu_pub_->publish(input_imu);
  gyro_bias_pub_->publish(gyro_bias);

  // メッセージの受信を待機
  rclcpp::spin_some(node_);
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 補正値の確認
  EXPECT_NEAR(received_imu_.angular_velocity.x, 1.1, 1e-6);
  EXPECT_NEAR(received_imu_.angular_velocity.y, 2.2, 1e-6);
  EXPECT_NEAR(received_imu_.angular_velocity.z, 3.3, 1e-6);

  // 共分散の確認
  EXPECT_NEAR(received_imu_.angular_velocity_covariance[0], 0.03 * 0.03, 1e-6);
  EXPECT_NEAR(received_imu_.angular_velocity_covariance[4], 0.03 * 0.03, 1e-6);
  EXPECT_NEAR(received_imu_.angular_velocity_covariance[8], 0.03 * 0.03, 1e-6);
}
