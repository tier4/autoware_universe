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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/external_velocity_limit.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>

namespace
{
using autoware::trajectory_modifier::TrajectoryModifierContext;
using autoware::trajectory_modifier::plugin::ExternalVelocityLimit;
using autoware::trajectory_modifier::plugin::InputData;
using autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;

TEST(VelocityLimitPluginAdapter, AppliesExternalLimitThroughHotfixPluginInterface)
{
  rclcpp::init(0, nullptr);
  const auto test_utils_share =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  auto options = rclcpp::NodeOptions{};
  autoware::test_utils::updateNodeOptions(
    options, {test_utils_share + "/config/test_vehicle_info.param.yaml"});
  auto node = std::make_shared<rclcpp::Node>("test_external_velocity_limit_adapter", options);
  auto context = std::make_shared<TrajectoryModifierContext>(node.get());
  auto time_keeper = std::make_shared<autoware_utils_debug::TimeKeeper>();

  trajectory_modifier_params::Params params;
  params.use_external_velocity_limit = true;
  params.velocity_limits.max_acceleration = 1.0;
  params.velocity_limits.max_deceleration = 1.0;
  params.velocity_limits.max_jerk = 3.0;
  params.velocity_limits.fixed_profile.enable = false;

  auto plugin = std::make_shared<ExternalVelocityLimit>();
  plugin->initialize(
    "autoware::trajectory_modifier::plugin::ExternalVelocityLimit", node.get(), time_keeper,
    context, params);
  std::shared_ptr<TrajectoryModifierPluginBase> base = plugin;

  InputData input;
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->twist.twist.linear.x = 10.0;
  input.current_odometry = odometry;
  auto acceleration = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
  input.current_acceleration = acceleration;
  auto limit = std::make_shared<autoware_internal_planning_msgs::msg::VelocityLimit>();
  limit->max_velocity = 5.0F;
  input.has_external_velocity_limit_snapshot = true;
  input.external_velocity_limit = limit;
  input.candidate_count = 1U;

  TrajectoryPoints points(100);
  for (std::size_t index = 0; index < points.size(); ++index) {
    const double time = 0.1 * static_cast<double>(index + 1);
    points[index].time_from_start = rclcpp::Duration::from_seconds(time);
    points[index].pose.position.x = 10.0 * time;
    points[index].pose.orientation.w = 1.0;
    points[index].longitudinal_velocity_mps = 10.0F;
  }

  EXPECT_TRUE(base->modify_trajectory(points, input));
  EXPECT_LT(points.back().longitudinal_velocity_mps, 10.0F);

  plugin.reset();
  base.reset();
  context.reset();
  node.reset();
  rclcpp::shutdown();
}
}  // namespace
