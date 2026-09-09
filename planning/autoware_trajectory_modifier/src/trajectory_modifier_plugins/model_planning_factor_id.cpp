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

#include "autoware/trajectory_modifier/trajectory_modifier_utils/planning_factor_utils.hpp"

#include <memory>

namespace autoware::trajectory_modifier::plugin
{

void ModelPlanningFactorID::apply_params(const TrajectoryModifierParams & params)
{
  enabled_ = params.use_model_planning_factor_id;
  params_ = params.model_planning_factor_id;
  detection_config_.stop_velocity_threshold = params_.stop_velocity_threshold;
  detection_config_.stop_keep_duration_threshold = params_.stop_keep_duration_threshold;
  detection_config_.slowdown_accel_threshold = params_.slowdown_accel_threshold;
}

void ModelPlanningFactorID::on_initialize(const TrajectoryModifierParams & params)
{
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      get_node_ptr(), "model_planning_factor_id");
  apply_params(params);
}

void ModelPlanningFactorID::update_params(const TrajectoryModifierParams & params)
{
  apply_params(params);
}

bool ModelPlanningFactorID::is_trajectory_modification_required(
  const TrajectoryPoints & traj_points, [[maybe_unused]] const InputData & input)
{
  if (!enabled_ || traj_points.empty()) {
    return false;
  }
  const auto result = utils::detect_planning_factors(traj_points, detection_config_);
  return (params_.enable_stop && result.stop.has_value()) ||
         (params_.enable_slowdown && result.slowdown.has_value());
}

void ModelPlanningFactorID::add_detected_factors(const TrajectoryPoints & traj_points)
{
  const auto result = utils::detect_planning_factors(traj_points, detection_config_);

  if (params_.enable_stop && result.stop) {
    const auto & stop = *result.stop;
    planning_factor_interface_->add(
      traj_points, stop.ego_pose, stop.stop_pose, PlanningFactor::STOP,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{});
  }

  if (params_.enable_slowdown && result.slowdown) {
    const auto & slowdown = *result.slowdown;
    planning_factor_interface_->add(
      traj_points, slowdown.ego_pose, slowdown.start_pose, slowdown.end_pose,
      PlanningFactor::SLOW_DOWN, autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true,
      slowdown.start_velocity, slowdown.end_velocity);
  }
}

bool ModelPlanningFactorID::modify_trajectory(
  TrajectoryPoints & traj_points, [[maybe_unused]] const InputData & input)
{
  if (!enabled_ || traj_points.empty()) {
    return false;
  }
  add_detected_factors(traj_points);
  // Inspection only: never rewrite the planner trajectory.
  return false;
}

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::ModelPlanningFactorID,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
