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

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

// Helper function to publish static TF
void publishStaticTransform(const rclcpp::Node::SharedPtr & node)
{
  auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node->now();
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "imu_link";
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(transform);
}

class ImuCorrectorTest : public autoware::imu_corrector::ImuCorrector
{
public:
  explicit ImuCorrectorTest(bool is_offline_calibration)
  : autoware::imu_corrector::ImuCorrector(setupNodeOptions(is_offline_calibration))
  {
  }

private:
  rclcpp::NodeOptions setupNodeOptions(bool is_offline_calibration)
  {
    rclcpp::NodeOptions options;

    // テストに必要なパラメータを設定
    std::vector<rclcpp::Parameter> params{
      rclcpp::Parameter("is_offline_calibration", is_offline_calibration),
      rclcpp::Parameter("angular_velocity_offset_x", -0.1),
      rclcpp::Parameter("angular_velocity_offset_y", -0.2),
      rclcpp::Parameter("angular_velocity_offset_z", -0.3),
      rclcpp::Parameter("angular_velocity_stddev_xx", 0.03),
      rclcpp::Parameter("angular_velocity_stddev_yy", 0.03),
      rclcpp::Parameter("angular_velocity_stddev_zz", 0.03)};

    options.parameter_overrides(params);
    return options;
  }
};

// DT_1_7
TEST(ImuCorrectorTest, DT_1_7_1)
{
  rclcpp::init(0, nullptr);
  sensor_msgs::msg::Imu imu;
  sensor_msgs::msg::Imu imu_corrected;
  auto node = std::make_shared<ImuCorrectorTest>(true);  // is_offline_calibration = true
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto imu_pub = test_node->create_publisher<sensor_msgs::msg::Imu>("/input", rclcpp::QoS{1});
  auto imu_sub = test_node->create_subscription<sensor_msgs::msg::Imu>(
    "/output", rclcpp::QoS{10},
    [&imu_corrected, &test_node](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      imu_corrected = *msg;
    });
  // Publish static TF
  publishStaticTransform(test_node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);

  imu.header.stamp = rclcpp::Clock().now();
  imu.header.frame_id = "imu_link";
  imu.angular_velocity.x = -0.1;
  imu.angular_velocity.y = -0.2;
  imu.angular_velocity.z = -0.3;
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

TEST(ImuCorrectorTest, DT_1_7_2)
{
  rclcpp::init(0, nullptr);
  sensor_msgs::msg::Imu imu;
  sensor_msgs::msg::Imu imu_corrected;
  auto node = std::make_shared<ImuCorrectorTest>(false);  // is_offline_calibration = false
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
  // Publish static TF
  publishStaticTransform(test_node);

  imu.header.stamp = rclcpp::Clock().now();
  imu.header.frame_id = "imu_link";
  imu.angular_velocity.x = -0.1;
  imu.angular_velocity.y = -0.2;
  imu.angular_velocity.z = -0.3;
  imu_pub->publish(imu);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(imu_corrected.angular_velocity.x, -0.1);
  EXPECT_EQ(imu_corrected.angular_velocity.y, -0.2);
  EXPECT_EQ(imu_corrected.angular_velocity.z, -0.3);

  rclcpp::shutdown();
}

TEST(ImuCorrectorTest, DT_1_7_3)
{
  rclcpp::init(0, nullptr);
  sensor_msgs::msg::Imu imu;
  sensor_msgs::msg::Imu imu_corrected;
  geometry_msgs::msg::Vector3Stamped gyro_bias;
  auto node = std::make_shared<ImuCorrectorTest>(false);  // is_offline_calibration = false
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
  // Publish static TF
  publishStaticTransform(test_node);

  imu.header.stamp = rclcpp::Clock().now();
  imu.header.frame_id = "imu_link";
  imu.angular_velocity.x = -0.1;
  imu.angular_velocity.y = -0.2;
  imu.angular_velocity.z = -0.3;
  gyro_bias.header.stamp = rclcpp::Clock().now();
  gyro_bias.header.frame_id = "imu_link";
  gyro_bias.vector.x = -0.1;
  gyro_bias.vector.y = -0.2;
  gyro_bias.vector.z = -0.3;

  gyro_bias_pub->publish(gyro_bias);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

// DT_2_1
TEST(ImuCorrectorTest, DT_2_1_1)
{
  rclcpp::init(0, nullptr);
  geometry_msgs::msg::Vector3Stamped gyro_bias;
  std::optional<bool> is_calibrated;
  auto node = std::make_shared<ImuCorrectorTest>(false);  // is_offline_calibration = false
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto is_calibrated_sub = test_node->create_subscription<tier4_calibration_msgs::msg::BoolStamped>(
    "/is_calibrated", rclcpp::QoS{10},
    [&is_calibrated](const tier4_calibration_msgs::msg::BoolStamped::ConstSharedPtr msg) {
      RCLCPP_INFO(rclcpp::get_logger("test"), "is_calibrated test: %d", msg->data);
      is_calibrated = msg->data;
    });
  auto gyro_bias_pub =
    test_node->create_publisher<geometry_msgs::msg::Vector3Stamped>("/gyro_bias", rclcpp::QoS{1});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);

  gyro_bias.header.stamp = rclcpp::Clock().now();
  gyro_bias.header.frame_id = "imu_link";
  gyro_bias.vector.x = 0.1;
  gyro_bias.vector.y = 0.2;
  gyro_bias.vector.z = 0.3;

  // 2秒間のスピンを実行
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_INFO(rclcpp::get_logger("test"), "spin completed");

  ASSERT_TRUE(is_calibrated.has_value());
  ASSERT_EQ(is_calibrated.value(), false);  // before publishing /gyro_bias

  gyro_bias_pub->publish(gyro_bias);

  // 2秒間のスピンを実行
  start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_INFO(rclcpp::get_logger("test"), "spin completed");

  EXPECT_EQ(is_calibrated, true);  // after publishing /gyro_bias
  rclcpp::shutdown();
}

TEST(ImuCorrectorTest, DT_2_1_2)
{
  rclcpp::init(0, nullptr);
  geometry_msgs::msg::Vector3Stamped gyro_bias;
  std::optional<bool> is_calibrated;
  auto node = std::make_shared<ImuCorrectorTest>(true);  // is_offline_calibration = true
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto is_calibrated_sub = test_node->create_subscription<tier4_calibration_msgs::msg::BoolStamped>(
    "/is_calibrated", rclcpp::QoS{10},
    [&is_calibrated](const tier4_calibration_msgs::msg::BoolStamped::ConstSharedPtr msg) {
      is_calibrated = msg->data;
    });
  auto gyro_bias_pub =
    test_node->create_publisher<geometry_msgs::msg::Vector3Stamped>("/gyro_bias", rclcpp::QoS{1});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);

  gyro_bias.header.stamp = rclcpp::Clock().now();
  gyro_bias.header.frame_id = "imu_link";
  gyro_bias.vector.x = 0.1;
  gyro_bias.vector.y = 0.2;
  gyro_bias.vector.z = 0.3;

  // 2秒間のスピンを実行
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(is_calibrated.has_value());
  ASSERT_EQ(is_calibrated.value(), true);  // before publishing /gyro_bias

  gyro_bias_pub->publish(gyro_bias);

  // 2秒間のスピンを実行
  start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_EQ(is_calibrated, true);  // after publishing /gyro_bias
  rclcpp::shutdown();
}
