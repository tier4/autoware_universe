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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/model_planning_factor_id.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace
{
using autoware::trajectory_modifier::TrajectoryModifierContext;
using autoware::trajectory_modifier::plugin::InputData;
using autoware::trajectory_modifier::plugin::ModelPlanningFactorID;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoint make_point(
  const double time_from_start_sec, const double x, const float velocity, const float accel)
{
  TrajectoryPoint p;
  p.time_from_start.sec = static_cast<int32_t>(time_from_start_sec);
  p.time_from_start.nanosec =
    static_cast<uint32_t>((time_from_start_sec - static_cast<int32_t>(time_from_start_sec)) * 1e9);
  p.pose.position.x = x;
  p.pose.position.y = 0.0;
  p.pose.position.z = 0.0;
  p.pose.orientation.w = 1.0;
  p.longitudinal_velocity_mps = velocity;
  p.acceleration_mps2 = accel;
  return p;
}

}  // namespace

class ModelPlanningFactorIDIntegrationTest : public ::testing::Test
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

    node_ = std::make_shared<rclcpp::Node>("test_model_planning_factor_id_node", node_options);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();

    params_.use_model_planning_factor_id = true;
    params_.model_planning_factor_id.enable_stop = true;
    params_.model_planning_factor_id.enable_slowdown = true;
    params_.model_planning_factor_id.stop_velocity_threshold = 0.1;
    params_.model_planning_factor_id.stop_keep_duration_threshold = 1.0;
    params_.model_planning_factor_id.slowdown_accel_threshold = -0.3;

    context_ = std::make_shared<TrajectoryModifierContext>(node_.get());
    plugin_ = std::make_unique<ModelPlanningFactorID>();
    plugin_->initialize(
      "test_model_planning_factor_id", node_.get(), time_keeper_, context_, params_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    plugin_.reset();
    context_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::unique_ptr<ModelPlanningFactorID> plugin_;
  trajectory_modifier_params::Params params_;
  std::shared_ptr<TrajectoryModifierContext> context_;
};

TEST_F(ModelPlanningFactorIDIntegrationTest, DoesNotMutateTrajectory)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(make_point(0.0, 0.0, 3.0f, 0.0f));
  trajectory.push_back(make_point(0.5, 1.0, 0.0f, 0.0f));
  trajectory.push_back(make_point(1.5, 2.0, 0.0f, 0.0f));
  trajectory.push_back(make_point(2.5, 3.0, 0.0f, 0.0f));
  const auto original_size = trajectory.size();
  const auto original_v0 = trajectory.front().longitudinal_velocity_mps;

  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, InputData{}));
  EXPECT_EQ(trajectory.size(), original_size);
  EXPECT_FLOAT_EQ(trajectory.front().longitudinal_velocity_mps, original_v0);
}

TEST_F(ModelPlanningFactorIDIntegrationTest, PublishesStopFactorFromInputTrajectory)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(make_point(0.0, 0.0, 3.0f, 0.0f));
  trajectory.push_back(make_point(0.5, 1.0, 2.0f, -0.2f));
  trajectory.push_back(make_point(1.0, 2.0, 1.0f, -0.2f));
  trajectory.push_back(make_point(1.5, 3.0, 0.05f, -0.1f));
  trajectory.push_back(make_point(2.0, 4.0, 0.0f, 0.0f));
  trajectory.push_back(make_point(2.5, 5.0, 0.0f, 0.0f));
  trajectory.push_back(make_point(3.0, 6.0, 0.0f, 0.0f));

  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, InputData{}));

  const auto factors = plugin_->get_planning_factors();
  ASSERT_EQ(factors.size(), 1u);
  EXPECT_EQ(factors.front().behavior, PlanningFactor::STOP);
  EXPECT_EQ(factors.front().module, "planning_model");
  ASSERT_FALSE(factors.front().control_points.empty());
  EXPECT_DOUBLE_EQ(factors.front().control_points.front().pose.position.x, 3.0);
}

TEST_F(ModelPlanningFactorIDIntegrationTest, PublishesSlowdownFactorWhenEnabled)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(make_point(0.0, 0.0, 5.0f, 0.0f));
  trajectory.push_back(make_point(0.5, 1.0, 4.5f, -0.1f));
  trajectory.push_back(make_point(1.0, 2.0, 3.5f, -0.5f));
  trajectory.push_back(make_point(1.5, 3.0, 2.5f, -0.5f));
  trajectory.push_back(make_point(2.0, 4.0, 2.0f, -0.1f));
  trajectory.push_back(make_point(2.5, 5.0, 2.0f, 0.0f));

  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, InputData{}));

  const auto factors = plugin_->get_planning_factors();
  ASSERT_EQ(factors.size(), 1u);
  EXPECT_EQ(factors.front().behavior, PlanningFactor::SLOW_DOWN);
  ASSERT_EQ(factors.front().control_points.size(), 2u);
  EXPECT_DOUBLE_EQ(factors.front().control_points.front().pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(factors.front().control_points.back().pose.position.x, 4.0);
}

TEST_F(ModelPlanningFactorIDIntegrationTest, DisabledPluginEmitsNoFactors)
{
  params_.use_model_planning_factor_id = false;
  plugin_->update_params(params_);

  TrajectoryPoints trajectory;
  trajectory.push_back(make_point(0.0, 0.0, 0.0f, 0.0f));

  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, InputData{}));
  EXPECT_TRUE(plugin_->get_planning_factors().empty());
}

TEST_F(ModelPlanningFactorIDIntegrationTest, StopDisabledSkipsStopFactor)
{
  params_.model_planning_factor_id.enable_stop = false;
  plugin_->update_params(params_);

  TrajectoryPoints trajectory;
  trajectory.push_back(make_point(0.0, 0.0, 3.0f, 0.0f));
  trajectory.push_back(make_point(1.5, 3.0, 0.0f, 0.0f));
  trajectory.push_back(make_point(2.5, 4.0, 0.0f, 0.0f));
  trajectory.push_back(make_point(3.5, 5.0, 0.0f, 0.0f));

  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, InputData{}));
  EXPECT_TRUE(plugin_->get_planning_factors().empty());
}
