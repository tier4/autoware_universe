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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/traffic_light_stop.hpp"

#include "autoware/traffic_light_compliance_checker/traffic_light_compliance_checker.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp"

#include <algorithm>
#include <memory>
#include <string>

namespace
{
autoware::traffic_light_compliance_checker::Parameters to_checker_params(
  const autoware::trajectory_modifier::plugin::TrajectoryModifierParams & params)
{
  const auto tl_stop_p = params.traffic_light_stop;
  const auto stopping_params = params.stopping_constraints;
  autoware::traffic_light_compliance_checker::Parameters p;
  p.deceleration_limit = stopping_params.nominal_deceleration;
  p.jerk_limit = stopping_params.jerk_limit;
  p.crossing_time_limit = tl_stop_p.crossing_time_limit;
  p.treat_amber_light_as_red_light = tl_stop_p.treat_amber_light_as_red;
  p.stop_overshoot_margin = tl_stop_p.overshoot_tolerance;
  p.stable_duration_threshold_red = tl_stop_p.th_stable_duration_red;
  p.stable_duration_threshold_amber = tl_stop_p.th_stable_duration_amber;
  p.amber_rejection_hysteresis_duration = tl_stop_p.th_amber_rejection_hysteresis;
  return p;
}
}  // namespace

namespace autoware::trajectory_modifier::plugin
{

void TrafficLightStop::on_initialize([[maybe_unused]] const TrajectoryModifierParams & params)
{
  const auto node_ptr = get_node_ptr();
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      node_ptr, "modifier_traffic_light_stop");

  enabled_ = params.use_traffic_light_stop;
  params_ = params.traffic_light_stop;
  stopping_params_ = params.stopping_constraints;

  checker_ =
    std::make_unique<autoware::traffic_light_compliance_checker::TrafficLightComplianceChecker>(
      to_checker_params(params), context_->vehicle_info);
}

void TrafficLightStop::update_params([[maybe_unused]] const TrajectoryModifierParams & params)
{
  enabled_ = params.use_traffic_light_stop;
  params_ = params.traffic_light_stop;
  stopping_params_ = params.stopping_constraints;
  checker_->update_parameters(to_checker_params(params));
}

bool TrafficLightStop::is_trajectory_modification_required(
  [[maybe_unused]] const TrajectoryPoints & traj_points, [[maybe_unused]] const InputData & input)
{
  if (!enabled_ || !check_inputs(input)) return false;

  if (!checker_) {
    RCLCPP_ERROR(get_node_ptr()->get_logger(), "Compliance checker is not initialized.");
    return false;
  }

  const traffic_light_compliance_checker::Inputs inputs{
    traj_points,
    input.lanelet_map,
    *input.route,
    *input.traffic_light_signals,
    get_clock()->now(),
    input.current_odometry->twist.twist.linear.x,
    input.current_acceleration->accel.accel.linear.x};

  const auto result = checker_->check(inputs);
  if (!result) return false;

  if (result->violations.empty()) return false;

  const auto nearest_it = std::min_element(result->violations.begin(), result->violations.end());
  nearest_violation_ = *nearest_it;
  return true;
}

bool TrafficLightStop::check_inputs(const InputData & input)
{
  return input.current_odometry && input.current_acceleration && input.route &&
         input.traffic_light_signals && input.lanelet_map;
}

bool TrafficLightStop::modify_trajectory(
  [[maybe_unused]] TrajectoryPoints & traj_points, [[maybe_unused]] const InputData & input)
{
  if (!is_trajectory_modification_required(traj_points, input)) return false;

  if (!nearest_violation_) return false;

  return true;
}

void TrafficLightStop::check_traffic_lights(
  [[maybe_unused]] const TrajectoryPoints & traj_points, [[maybe_unused]] const InputData & input)
{
}

bool TrafficLightStop::set_stop_point(
  [[maybe_unused]] TrajectoryPoints & traj_points, [[maybe_unused]] const InputData & input)
{
  return false;
}

bool TrafficLightStop::apply_stopping(
  [[maybe_unused]] TrajectoryPoints & traj_points,
  [[maybe_unused]] const double target_stop_point_arc_length) const
{
  return false;
}

void TrafficLightStop::publish_debug_string([[maybe_unused]] bool is_safe) const
{
}

void TrafficLightStop::publish_debug_data([[maybe_unused]] const std::string & ns) const
{
}

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::TrafficLightStop,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
