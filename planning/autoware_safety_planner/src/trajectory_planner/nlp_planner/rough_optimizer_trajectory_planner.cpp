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

#include "rough_optimizer_trajectory_planner.hpp"

#include "../../utils/trajectory_conversion.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <utility>

namespace autoware::safety_planner
{

namespace
{

//! Copies the generated ROS parameters into the parameter struct of the rough planner
RoughPlannerParams to_rough_planner_params(const Params & params)
{
  const auto & p = params.rough_planner;
  RoughPlannerParams rough;
  rough.time_step_s = p.time_step_s;
  rough.num_points = static_cast<int>(p.num_points);
  rough.max_candidates = static_cast<int>(p.max_candidates);
  rough.dp.s_max_m = p.dp.s_max_m;
  rough.dp.s_step_m = p.dp.s_step_m;
  rough.dp.l_range_m = p.dp.l_range_m;
  rough.dp.l_step_m = p.dp.l_step_m;
  rough.dp.t_step_s = p.dp.t_step_s;
  rough.dp.horizon_s = p.dp.horizon_s;
  rough.dp.v_step_mps = p.dp.v_step_mps;
  rough.dp.lateral_slope_max = p.dp.lateral_slope_max;
  rough.dp.lateral_rate_max_mps = p.dp.lateral_rate_max_mps;
  rough.dp.weights.progress = p.dp.weights.progress;
  rough.dp.weights.lateral = p.dp.weights.lateral;
  rough.dp.weights.lateral_rate = p.dp.weights.lateral_rate;
  rough.dp.weights.velocity = p.dp.weights.velocity;
  rough.dp.weights.accel = p.dp.weights.accel;
  rough.dp.weights.accel_nominal = p.dp.weights.accel_nominal;
  return rough;
}

}  // namespace

void RoughOptimizerTrajectoryPlanner::on_initialize(
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper, const Params & params)
{
  TrajectoryPlannerInterface::on_initialize(time_keeper, params);
  rough_planner_.emplace(to_rough_planner_params(params_));
  load_trajectory_optimizer_plugin();
}

void RoughOptimizerTrajectoryPlanner::load_trajectory_optimizer_plugin()
{
  const auto logger = rclcpp::get_logger("safety_planner");
  trajectory_optimizer_loader_ = std::make_unique<TrajectoryOptimizerLoader>(
    "autoware_safety_planner", "autoware::safety_planner::TrajectoryOptimizerInterface");

  const auto & class_name = params_.trajectory_optimizer.plugin;
  try {
    trajectory_optimizer_ = trajectory_optimizer_loader_->createSharedInstance(class_name);
    trajectory_optimizer_->on_initialize(time_keeper_, params_);
    RCLCPP_INFO(
      logger, "Loaded trajectory optimizer plugin: %s (%s)",
      trajectory_optimizer_->get_name().c_str(), class_name.c_str());
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_ERROR(
      logger, "Failed to load trajectory optimizer plugin '%s': %s", class_name.c_str(), e.what());
  }
}

TrajectoryPlannerResult RoughOptimizerTrajectoryPlanner::plan(const TrajectoryPlannerInput & input)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  TrajectoryPlannerResult result;

  {
    autoware_utils_debug::ScopedTimeTrack side_st("plan_normal", *time_keeper_);
    // Each set is compiled on its own
    auto compiled = compile_constraint_list(input.context, input.normal_constraints);
    RoughPlanResult rough_plan_result;
    TrajectoryOptimizerResult optimizer_result;
    result.normal_trajectory =
      plan_one_side(input.context, compiled, normal_state_, rough_plan_result, optimizer_result);
    // Debug output covers the normal side only
    result.debug.compiled_constraints = std::move(compiled);
    result.debug.rough_plan_result = std::move(rough_plan_result);
    result.debug.trajectory_optimizer_result = std::move(optimizer_result);
  }
  {
    autoware_utils_debug::ScopedTimeTrack side_st("plan_cautious", *time_keeper_);
    const auto compiled = compile_constraint_list(input.context, input.cautious_constraints);
    RoughPlanResult rough_plan_result;
    TrajectoryOptimizerResult optimizer_result;
    result.cautious_trajectory =
      plan_one_side(input.context, compiled, cautious_state_, rough_plan_result, optimizer_result);
  }

  return result;
}

std::optional<Trajectory> RoughOptimizerTrajectoryPlanner::plan_one_side(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  SideState & state, RoughPlanResult & rough_plan_result,
  TrajectoryOptimizerResult & optimizer_result)
{
  // The rough plan settles the homotopy. It returns candidates in priority order, of which only
  // the first is consumed for now
  rough_plan_result = [&]() {
    autoware_utils_debug::ScopedTimeTrack rough_st("plan_rough_trajectories", *time_keeper_);
    return rough_planner_->plan_rough_trajectories(
      context, compiled_constraints, state.prev_planning_result);
  }();

  // Refine it into the final trajectory
  optimizer_result = optimize_trajectory(context, compiled_constraints, rough_plan_result, state);

  // update what is carried into the next cycle
  if (!rough_plan_result.plans.empty()) {
    state.prev_planning_result.plan = rough_plan_result.plans.front();
  }
  state.prev_optimized_trajectory = optimizer_result.status == TrajectoryOptimizerStatus::SUCCESS
                                      ? std::make_optional(optimizer_result.trajectory)
                                      : std::nullopt;

  if (optimizer_result.status != TrajectoryOptimizerStatus::SUCCESS) {
    return std::nullopt;
  }

  const double z = context.odometry.pose.pose.position.z;
  const double wheel_base_m = context.vehicle_info.wheel_base_m;

  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = context.odometry.header.stamp;
  trajectory.points.reserve(optimizer_result.trajectory.points.size());
  for (const auto & optimized_point : optimizer_result.trajectory.points) {
    trajectory.points.push_back(to_trajectory_point(optimized_point, z, wheel_base_m));
  }
  const double engage_velocity_mps =
    params_.engage_velocity.enable ? params_.engage_velocity.velocity_hard_mps : 0.0;
  return set_engage_speed(trajectory, engage_velocity_mps);
}

TrajectoryOptimizerResult RoughOptimizerTrajectoryPlanner::optimize_trajectory(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const RoughPlanResult & rough_plan_result, const SideState & state) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  TrajectoryOptimizerResult result;
  if (!trajectory_optimizer_) {
    result.debug.message = "trajectory optimizer plugin is not loaded";
    return result;
  }
  // The rough planner always returns at least one candidate, the stop plan being unconditionally
  // feasible, but guard against an empty result anyway
  if (rough_plan_result.plans.empty()) {
    result.debug.message = "no rough plan candidate";
    return result;
  }

  const TrajectoryOptimizerInput input{
    context, compiled_constraints, rough_plan_result.plans.front(),
    state.prev_optimized_trajectory};
  return trajectory_optimizer_->optimize(input);
}

}  // namespace autoware::safety_planner

PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::RoughOptimizerTrajectoryPlanner,
  autoware::safety_planner::TrajectoryPlannerInterface)
