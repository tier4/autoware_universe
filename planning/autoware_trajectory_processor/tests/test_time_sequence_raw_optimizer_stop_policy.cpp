// Copyright 2026 TIER IV, Inc.
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

#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_time_sequence_raw_optimizer.hpp"
#include "trajectory_processor_test_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_trajectory_processor/trajectory_processor_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

namespace
{
using autoware::trajectory_processor::TrajectoryProcessorContext;
using autoware::trajectory_processor::TrajectoryProcessorData;
using autoware::trajectory_processor::TrajectoryProcessorParams;
using autoware::trajectory_processor::plugin::TrajectoryPoints;
using autoware::trajectory_processor::plugin::TrajectoryTimeSequenceRawOptimizer;
using autoware::trajectory_processor::test::process_plugin;
using autoware::trajectory_processor::time_sequence_raw::opt_horizon;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_vehicle_msgs::msg::SteeringReport;
using nav_msgs::msg::Odometry;

TrajectoryPoint make_point(const double x, const double v)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = 0.0;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = static_cast<float>(v);
  point.front_wheel_angle_rad = 0.5F;
  point.heading_rate_rps = 0.1F;
  point.acceleration_mps2 = 1.0F;
  return point;
}

TrajectoryPoints make_stopped_trajectory(const size_t num_points, const double span_m)
{
  TrajectoryPoints trajectory;
  trajectory.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    const double x = span_m * static_cast<double>(i) / static_cast<double>(num_points - 1);
    trajectory.push_back(make_point(x, 0.0));
  }
  return trajectory;
}

Odometry::ConstSharedPtr make_odometry(const double x, const double y, const double vx = 0.0)
{
  auto odom = std::make_shared<Odometry>();
  odom->pose.pose.position.x = x;
  odom->pose.pose.position.y = y;
  odom->pose.pose.orientation.w = 1.0;
  odom->twist.twist.linear.x = vx;
  return odom;
}

SteeringReport::ConstSharedPtr make_steering(const double angle_rad)
{
  auto steering = std::make_shared<SteeringReport>();
  steering->steering_tire_angle = angle_rad;
  return steering;
}

LaneletRoute::ConstSharedPtr make_route_near_ego(const double goal_x, const double goal_y)
{
  auto route = std::make_shared<LaneletRoute>();
  route->goal_pose.position.x = goal_x;
  route->goal_pose.position.y = goal_y;
  route->goal_pose.orientation.w = 1.0;
  return route;
}

class TimeSequenceRawOptimizerStopPolicyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    auto node_options = rclcpp::NodeOptions{};
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    autoware::test_utils::updateNodeOptions(
      node_options, {autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<rclcpp::Node>("test_time_sequence_raw_stop_policy", node_options);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();
    context_ = std::make_shared<TrajectoryProcessorContext>(node_.get());

    params_.use_time_sequence_raw_optimizer = true;
    params_.time_sequence_raw_optimizer.publish_debug_topics = false;
    params_.time_sequence_raw_optimizer.stopped_velocity_threshold_mps = 0.08;
    params_.time_sequence_raw_optimizer.stopped_trajectory_max_length_m = 1.5;
    params_.time_sequence_raw_optimizer.goal_steer_zero_enable = true;
    params_.time_sequence_raw_optimizer.goal_steer_zero_distance_m = 5.0;
    params_.time_sequence_raw_optimizer.goal_steer_zero_requires_stopped = true;
    params_.road_border_avoidance.enable = false;

    plugin_ = std::make_unique<TrajectoryTimeSequenceRawOptimizer>();
    plugin_->initialize(
      "test_time_sequence_raw_optimizer", node_.get(), time_keeper_, context_,
      TrajectoryProcessorParams{params_});
  }

  void TearDown() override
  {
    plugin_.reset();
    context_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::shared_ptr<TrajectoryProcessorContext> context_;
  std::unique_ptr<TrajectoryTimeSequenceRawOptimizer> plugin_;
  trajectory_processor_params::Params params_;
};

TEST_F(TimeSequenceRawOptimizerStopPolicyTest, LatchesSteeringOnStoppedReference)
{
  auto trajectory = make_stopped_trajectory(opt_horizon, 0.5);
  TrajectoryProcessorData data;
  data.current_odometry = make_odometry(0.0, 0.0);
  data.current_steering = make_steering(0.35);

  ASSERT_TRUE(process_plugin(*plugin_, trajectory, data));
  for (const auto & point : trajectory) {
    EXPECT_NEAR(point.front_wheel_angle_rad, 0.35F, 1e-5F);
    EXPECT_NEAR(point.longitudinal_velocity_mps, 0.0F, 1e-5F);
    EXPECT_NEAR(point.heading_rate_rps, 0.0F, 1e-5F);
  }

  data.current_steering = make_steering(0.60);
  ASSERT_TRUE(process_plugin(*plugin_, trajectory, data));
  for (const auto & point : trajectory) {
    EXPECT_NEAR(point.front_wheel_angle_rad, 0.35F, 1e-5F);
  }
}

TEST_F(TimeSequenceRawOptimizerStopPolicyTest, ZerosSteeringNearGoalWhenStopped)
{
  auto trajectory = make_stopped_trajectory(opt_horizon, 0.5);
  TrajectoryProcessorData data;
  data.current_odometry = make_odometry(0.0, 0.0);
  data.current_steering = make_steering(0.40);
  data.route = make_route_near_ego(2.0, 0.0);

  ASSERT_TRUE(process_plugin(*plugin_, trajectory, data));
  for (const auto & point : trajectory) {
    EXPECT_NEAR(point.front_wheel_angle_rad, 0.0F, 1e-5F);
    EXPECT_NEAR(point.longitudinal_velocity_mps, 0.0F, 1e-5F);
  }
}

TEST_F(TimeSequenceRawOptimizerStopPolicyTest, ZerosSteeringNearGoalDespiteNoisyReferenceVelocity)
{
  TrajectoryPoints trajectory;
  trajectory.reserve(opt_horizon);
  trajectory.push_back(make_point(0.0, 0.10));
  for (size_t i = 1; i < opt_horizon; ++i) {
    trajectory.push_back(make_point(0.01 * static_cast<double>(i), 0.02));
  }

  TrajectoryProcessorData data;
  data.current_odometry = make_odometry(0.0, 0.0);
  data.current_steering = make_steering(0.40);
  data.route = make_route_near_ego(2.0, 0.0);

  ASSERT_TRUE(process_plugin(*plugin_, trajectory, data));
  for (const auto & point : trajectory) {
    EXPECT_NEAR(point.front_wheel_angle_rad, 0.0F, 1e-5F);
    EXPECT_NEAR(point.longitudinal_velocity_mps, 0.0F, 1e-5F);
  }
}

TEST_F(TimeSequenceRawOptimizerStopPolicyTest, StaysInGoalZeroAfterBriefEgoMotion)
{
  auto trajectory = make_stopped_trajectory(opt_horizon, 0.5);
  TrajectoryProcessorData data;
  data.current_odometry = make_odometry(0.0, 0.0, 0.0);
  data.current_steering = make_steering(0.40);
  data.route = make_route_near_ego(2.0, 0.0);

  ASSERT_TRUE(process_plugin(*plugin_, trajectory, data));
  for (const auto & point : trajectory) {
    EXPECT_NEAR(point.front_wheel_angle_rad, 0.0F, 1e-5F);
  }

  data.current_odometry = make_odometry(0.0, 0.0, 0.5);
  ASSERT_TRUE(process_plugin(*plugin_, trajectory, data));
  for (const auto & point : trajectory) {
    EXPECT_NEAR(point.front_wheel_angle_rad, 0.0F, 1e-5F);
  }
}

}  // namespace
