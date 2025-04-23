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

#include "../src/imu_corrector_core.hpp"

#include <gtest/gtest.h>

class ImuCorrectorTest : public imu_corrector::ImuCorrector
{
public:
  explicit ImuCorrectorTest(std::string sensor_model)
  : imu_corrector::ImuCorrector(setupNodeOptions(sensor_model))
  {
  }

private:
  rclcpp::NodeOptions setupNodeOptions(std::string sensor_model)
  {
    rclcpp::NodeOptions options;

    // テストに必要なパラメータを設定
    std::vector<rclcpp::Parameter> params{
      rclcpp::Parameter("sensor_model", sensor_model),
      rclcpp::Parameter("angular_velocity_offset_x", 0.1),
      rclcpp::Parameter("angular_velocity_offset_y", 0.2),
      rclcpp::Parameter("angular_velocity_offset_z", 0.3),
      rclcpp::Parameter("angular_velocity_stddev_xx", 0.03),
      rclcpp::Parameter("angular_velocity_stddev_yy", 0.03),
      rclcpp::Parameter("angular_velocity_stddev_zz", 0.03)};

    options.parameter_overrides(params);
    return options;
  }
};

TEST(ImuCorrectorTest, DT_1_7_1)
{
  rclcpp::init(0, nullptr);
  sensor_msgs::msg::Imu imu;
  sensor_msgs::msg::Imu imu_corrected;
  auto node = std::make_shared<ImuCorrectorTest>("aip_x1");
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto imu_pub = test_node->create_publisher<sensor_msgs::msg::Imu>("/input", rclcpp::QoS{1});
  auto imu_sub = test_node->create_subscription<sensor_msgs::msg::Imu>(
    "/output", rclcpp::QoS{10},
    [&imu_corrected, &test_node](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      imu_corrected = *msg;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);

  imu.header.stamp = rclcpp::Clock().now();
  imu.header.frame_id = "imu_link";
  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;
  imu_pub->publish(imu);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(imu_corrected.angular_velocity.x, 0.1);
  EXPECT_EQ(imu_corrected.angular_velocity.y, 0.2);
  EXPECT_EQ(imu_corrected.angular_velocity.z, 0.3);

  rclcpp::shutdown();
}

TEST(ImuCorrectorTest, DT_1_7_2)
{
  rclcpp::init(0, nullptr);
  sensor_msgs::msg::Imu imu;
  sensor_msgs::msg::Imu imu_corrected;
  auto node = std::make_shared<ImuCorrectorTest>("aip_x1_1");
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto imu_pub = test_node->create_publisher<sensor_msgs::msg::Imu>("/input", rclcpp::QoS{1});
  auto imu_sub = test_node->create_subscription<sensor_msgs::msg::Imu>(
    "/output", rclcpp::QoS{10},
    [&imu_corrected, &test_node](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      imu_corrected = *msg;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);

  imu.header.stamp = rclcpp::Clock().now();
  imu.header.frame_id = "imu_link";
  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;
  imu_pub->publish(imu);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(imu_corrected.angular_velocity.x, 0.0);
  EXPECT_EQ(imu_corrected.angular_velocity.y, 0.0);
  EXPECT_EQ(imu_corrected.angular_velocity.z, 0.0);

  rclcpp::shutdown();
}

TEST(ImuCorrectorTest, DT_1_7_3)
{
  rclcpp::init(0, nullptr);
  sensor_msgs::msg::Imu imu;
  sensor_msgs::msg::Imu imu_corrected;
  geometry_msgs::msg::Vector3Stamped gyro_bias;
  auto node = std::make_shared<ImuCorrectorTest>("aip_x1_1");
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto imu_pub = test_node->create_publisher<sensor_msgs::msg::Imu>("/input", rclcpp::QoS{1});
  auto imu_sub = test_node->create_subscription<sensor_msgs::msg::Imu>(
    "/output", rclcpp::QoS{10},
    [&imu_corrected, &test_node](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      imu_corrected = *msg;
    });
  auto gyro_bias_pub =
    test_node->create_publisher<geometry_msgs::msg::Vector3Stamped>("/gyro_bias", rclcpp::QoS{1});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);

  imu.header.stamp = rclcpp::Clock().now();
  imu.header.frame_id = "imu_link";
  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;
  gyro_bias.header.stamp = rclcpp::Clock().now();
  gyro_bias.header.frame_id = "imu_link";
  gyro_bias.vector.x = 0.1;
  gyro_bias.vector.y = 0.2;
  gyro_bias.vector.z = 0.3;

  gyro_bias_pub->publish(gyro_bias);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  imu_pub->publish(imu);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(imu_corrected.angular_velocity.x, 0.1);
  EXPECT_EQ(imu_corrected.angular_velocity.y, 0.2);
  EXPECT_EQ(imu_corrected.angular_velocity.z, 0.3);

  rclcpp::shutdown();
}
