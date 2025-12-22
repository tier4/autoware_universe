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

#include "test_utils_plotter.hpp"
#include "time_to_space_converter_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>
#include <tf2/utils.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using autoware::time_to_space_trajectory_converter::TimeToSpaceConverterNode;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

namespace autoware::time_to_space_trajectory_converter
{

class TimeToSpaceNodeTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }

  static Trajectory create_sparse_trajectory()
  {
    Trajectory traj;
    traj.header.frame_id = "map";
    traj.header.stamp = rclcpp::Clock().now();

    auto add_pt = [&](double x, double t_sec) {
      TrajectoryPoint p;
      p.pose.position.x = x;
      p.pose.position.y = 0.0;
      p.longitudinal_velocity_mps = 10.0;
      p.acceleration_mps2 = 0.0;
      p.time_from_start = rclcpp::Duration::from_seconds(t_sec);
      traj.points.push_back(p);
    };

    add_pt(0.0, 0.0);
    add_pt(50.0, 5.0);
    add_pt(100.0, 10.0);

    return traj;
  }

  static nav_msgs::msg::Odometry create_dummy_odometry()
  {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.header.stamp = rclcpp::Clock().now();
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.twist.twist.linear.x = 0.0;
    return odom;
  }

  static rclcpp::NodeOptions set_params()
  {
    const auto package_name = "autoware_time_to_space_trajectory_converter";
    const auto package_path = ament_index_cpp::get_package_share_directory(package_name);
    const auto config_file_abs_path =
      package_path + "/config/time_to_space_trajectory_converter.param.yaml";

    std::vector<const char *> arguments;
    arguments.push_back("--ros-args");
    arguments.push_back("--params-file");
    arguments.push_back(config_file_abs_path.c_str());

    rclcpp::NodeOptions options;
    options.arguments(std::vector<std::string>{arguments.begin(), arguments.end()});
    return options;
  }
};

TEST_F(TimeToSpaceNodeTest, EndToEndConversion)
{
  auto options = set_params();
  options.parameter_overrides({
    {"resampling_resolution_m", 1.0},
  });

  auto node = std::make_shared<TimeToSpaceConverterNode>(options);

  auto traj_pub = node->create_publisher<Trajectory>(
    "~/time_to_space_converter/input/trajectory", rclcpp::QoS(1));
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(
    "~/time_to_space_converter/input/odometry", rclcpp::QoS(1));

  Trajectory::SharedPtr received_msg;
  bool received = false;
  auto test_sub = node->create_subscription<Trajectory>(
    "~/time_to_space_converter/output/trajectory", rclcpp::QoS(1),
    [&](const Trajectory::SharedPtr msg) {
      received_msg = msg;
      received = true;
    });

  Trajectory input_traj = create_sparse_trajectory();
  nav_msgs::msg::Odometry input_odom = create_dummy_odometry();

  auto start_time = std::chrono::steady_clock::now();
  while (!received && (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2))) {
    if (traj_pub->get_subscription_count() > 0 && odom_pub->get_subscription_count() > 0) {
      input_traj.header.stamp = node->now();
      input_odom.header.stamp = node->now();
      traj_pub->publish(input_traj);
      odom_pub->publish(input_odom);
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(received) << "Failed to receive output trajectory from node.";
  ASSERT_TRUE(received_msg != nullptr);

  EXPECT_GT(received_msg->points.size(), 90);
  EXPECT_LT(received_msg->points.size(), 110);
  EXPECT_EQ(received_msg->header.frame_id, input_traj.header.frame_id);

  plot_conversion_result(input_traj, *received_msg, "end_to_end_conversion.png");
}

TEST_F(TimeToSpaceNodeTest, EmptyInputHandling)
{
  auto options = set_params();
  auto node = std::make_shared<TimeToSpaceConverterNode>(options);

  auto traj_pub = node->create_publisher<Trajectory>(
    "~/time_to_space_converter/input/trajectory", rclcpp::QoS(1));
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(
    "~/time_to_space_converter/input/odometry", rclcpp::QoS(1));

  bool received = false;
  auto test_sub = node->create_subscription<Trajectory>(
    "~/time_to_space_converter/output/trajectory", rclcpp::QoS(1),
    [&](const Trajectory::SharedPtr) { received = true; });

  Trajectory empty_traj;
  empty_traj.header.frame_id = "map";
  nav_msgs::msg::Odometry input_odom = create_dummy_odometry();

  auto start = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(500)) {
    if (traj_pub->get_subscription_count() > 0 && odom_pub->get_subscription_count() > 0) {
      traj_pub->publish(empty_traj);
      odom_pub->publish(input_odom);
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_FALSE(received) << "Node should not publish output for empty input.";
}

TEST_F(TimeToSpaceNodeTest, StartOrientationOverride)
{
  auto options = set_params();
  auto node = std::make_shared<TimeToSpaceConverterNode>(options);

  auto traj_pub = node->create_publisher<Trajectory>(
    "~/time_to_space_converter/input/trajectory", rclcpp::QoS(1));
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(
    "~/time_to_space_converter/input/odometry", rclcpp::QoS(1));

  Trajectory::SharedPtr received_msg;
  auto test_sub = node->create_subscription<Trajectory>(
    "~/time_to_space_converter/output/trajectory", rclcpp::QoS(1),
    [&](const Trajectory::SharedPtr msg) { received_msg = msg; });

  // 1. Scenario:
  // Trajectory goes strictly East (Yaw = 0).
  Trajectory input_traj = create_sparse_trajectory();

  // Vehicle is facing North (Yaw = 90 deg / 1.57 rad).
  nav_msgs::msg::Odometry input_odom = create_dummy_odometry();
  tf2::Quaternion q_north;
  q_north.setRPY(0, 0, M_PI_2);  // 90 deg
  input_odom.pose.pose.orientation = tf2::toMsg(q_north);

  // 2. Publish
  auto start = std::chrono::steady_clock::now();
  while (!received_msg && (std::chrono::steady_clock::now() - start) < std::chrono::seconds(1)) {
    input_traj.header.stamp = node->now();
    input_odom.header.stamp = node->now();
    traj_pub->publish(input_traj);
    odom_pub->publish(input_odom);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  ASSERT_TRUE(received_msg != nullptr);
  ASSERT_FALSE(received_msg->points.empty());

  // 3. Verification
  // Ensure first point of the output trajectory matches the Odom orientation.
  double output_yaw = tf2::getYaw(received_msg->points[0].pose.orientation);
  double odom_yaw = tf2::getYaw(input_odom.pose.pose.orientation);

  EXPECT_NEAR(output_yaw, odom_yaw, 1e-4) << "First trajectory point orientation must be "
                                             "overwritten by Ego Orientation to allow engagement.";
}
}  // namespace autoware::time_to_space_trajectory_converter
