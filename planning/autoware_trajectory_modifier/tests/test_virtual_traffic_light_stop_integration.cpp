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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/virtual_traffic_light_stop.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace
{
using autoware::trajectory_modifier::TrajectoryModifierContext;
using autoware::trajectory_modifier::plugin::InputData;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware::trajectory_modifier::plugin::VirtualTrafficLightStop;
using autoware_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoints make_trajectory()
{
  TrajectoryPoints trajectory;
  for (size_t i = 0; i < 3; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(i);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 5.0;
    trajectory.push_back(point);
  }
  return trajectory;
}

void expect_same_trajectory(const TrajectoryPoints & actual, const TrajectoryPoints & expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (size_t i = 0; i < actual.size(); ++i) {
    EXPECT_DOUBLE_EQ(actual[i].pose.position.x, expected[i].pose.position.x);
    EXPECT_DOUBLE_EQ(actual[i].pose.position.y, expected[i].pose.position.y);
    EXPECT_FLOAT_EQ(
      actual[i].longitudinal_velocity_mps, expected[i].longitudinal_velocity_mps);
  }
}
}  // namespace

class VirtualTrafficLightStopIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions node_options;
    const auto test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    autoware::test_utils::updateNodeOptions(
      node_options, {test_utils_dir + "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<rclcpp::Node>("test_virtual_traffic_light_stop_node", node_options);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();
    context_ = std::make_shared<TrajectoryModifierContext>(node_.get());

    params_.use_virtual_traffic_light_stop = true;
    params_.virtual_traffic_light.max_delay_sec = 3.0;
    params_.virtual_traffic_light.near_line_distance = 1.0;
    params_.virtual_traffic_light.dead_line_margin = 1.0;
    params_.virtual_traffic_light.max_yaw_deviation_deg = 90.0;
    params_.virtual_traffic_light.check_timeout_after_stop_line = true;
    params_.virtual_traffic_light.hold_stop_margin_distance = 0.0;

    plugin_ = std::make_unique<VirtualTrafficLightStop>();
    plugin_->initialize(
      "test_virtual_traffic_light_stop", node_.get(), time_keeper_, context_, params_);
  }

  void TearDown() override
  {
    plugin_.reset();
    context_.reset();
    time_keeper_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::shared_ptr<TrajectoryModifierContext> context_;
  std::unique_ptr<VirtualTrafficLightStop> plugin_;
  trajectory_modifier_params::Params params_;
};

TEST_F(VirtualTrafficLightStopIntegrationTest, DisabledPluginDoesNotModify)
{
  params_.use_virtual_traffic_light_stop = false;
  plugin_->update_params(params_);

  auto trajectory = make_trajectory();
  const auto original = trajectory;
  InputData input;

  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory, input));
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();
  expect_same_trajectory(trajectory, original);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, MissingMapAndRouteAreSafe)
{
  auto trajectory = make_trajectory();
  const auto original = trajectory;
  InputData input;
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  input.current_odometry = std::make_shared<nav_msgs::msg::Odometry>(odometry);
  input.lanelet_map = nullptr;
  input.route = nullptr;

  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory, input));
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();
  expect_same_trajectory(trajectory, original);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, EmptyRouteDoesNotModify)
{
  auto trajectory = make_trajectory();
  const auto original = trajectory;
  InputData input;
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  input.current_odometry = std::make_shared<nav_msgs::msg::Odometry>(odometry);
  input.lanelet_map = std::make_shared<lanelet::LaneletMap>();
  input.route = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>();

  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory, input));
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();
  expect_same_trajectory(trajectory, original);
}
