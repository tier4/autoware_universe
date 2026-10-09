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

#include "planning_factors.hpp"

#include "../utils/frenet_utils.hpp"

#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_internal_planning_msgs/msg/safety_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>

#include <algorithm>
#include <string>
#include <utility>

namespace autoware::safety_planner
{
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::SafetyFactor;
using autoware_internal_planning_msgs::msg::SafetyFactorArray;

namespace
{

std::string detail_of(const Source & source)
{
  if (source.plugin_name.empty()) {
    return source.detail;
  }
  if (source.detail.empty()) {
    return source.plugin_name;
  }
  return source.plugin_name + ": " + source.detail;
}

//! Object id and position of the predicted object named by the constraint. Empty when that object
//! is not in the current predictions.
SafetyFactorArray safety_factors_of(const Constraint & constraint, const PlannerContext & context)
{
  SafetyFactorArray safety_factors;
  safety_factors.is_safe = true;
  if (!context.predicted_objects) {
    return safety_factors;
  }

  for (const auto & object : context.predicted_objects->objects) {
    if (autoware_utils_uuid::to_hex_string(object.object_id) != constraint.source.target_id) {
      continue;
    }
    SafetyFactor factor;
    factor.type = SafetyFactor::OBJECT;
    factor.object_id = object.object_id;
    factor.is_safe = false;
    factor.points.push_back(object.kinematics.initial_pose_with_covariance.pose.position);
    safety_factors.is_safe = false;
    safety_factors.detail = constraint.source.detail;
    safety_factors.factors.push_back(std::move(factor));
    break;
  }
  return safety_factors;
}

Pose pose_on_path(const PathPointTrajectory & path, const double s)
{
  return path.compute(std::clamp(s, 0.0, path.length())).point.pose;
}

//! The same bar stop_target_s / the MPPI reference stop at: the nearest one closed within the
//! horizon, at the rear-axle arc length where the footprint front touches it
const StopBarEntry * nearest_stop_bar(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const double horizon_s, const double s_ego, double & s_stop)
{
  const double front = context.vehicle_info.max_longitudinal_offset_m;
  const StopBarEntry * nearest = nullptr;
  s_stop = context.reference_path.length();
  for (const auto & bar : compiled_constraints.stop_bars) {
    if (bar.time.t1 < 0.0 || bar.time.t0 > horizon_s) {
      continue;
    }
    if (bar.raw_index >= compiled_constraints.raw_constraints.size()) {
      continue;
    }
    const double s_axle = bar.s_stop - front;
    if (s_axle < s_stop) {
      s_stop = s_axle;
      nearest = &bar;
    }
  }
  s_stop = std::max(s_stop, s_ego);
  return nearest;
}

}  // namespace

void add_planning_factors(
  PlanningFactorInterface * const planning_factor_interface, const PlannerContext & context,
  const CompiledConstraints & compiled_constraints, const double horizon_s)
{
  if (planning_factor_interface == nullptr) {
    return;
  }

  const auto & path = context.reference_path;
  const double s_ego = compute_ego_frenet_state(context).s;

  double s_stop = 0.0;
  if (
    const auto * bar = nearest_stop_bar(context, compiled_constraints, horizon_s, s_ego, s_stop)) {
    const auto & constraint = compiled_constraints.raw_constraints[bar->raw_index];
    planning_factor_interface->add(
      s_stop - s_ego, pose_on_path(path, s_stop), PlanningFactor::STOP,
      safety_factors_of(constraint, context), true, 0.0, 0.0, detail_of(constraint.source));
  } else {
    SafetyFactorArray safety_factors;
    safety_factors.is_safe = true;
    planning_factor_interface->add(
      path.length() - s_ego, pose_on_path(path, path.length()), PlanningFactor::STOP,
      safety_factors);
  }
}

void copy_planning_factors(
  const PlanningFactorInterface * const from, PlanningFactorInterface * const to)
{
  if (from == nullptr || to == nullptr || from == to) {
    return;
  }
  for (const auto & factor : from->get_factors()) {
    const auto & points = factor.control_points;
    if (points.size() == 1) {
      to->add(
        points[0].distance, points[0].pose, factor.behavior, factor.safety_factors,
        factor.is_driving_forward, points[0].velocity, points[0].shift_length, factor.detail);
    } else if (points.size() >= 2) {
      to->add(
        points[0].distance, points[1].distance, points[0].pose, points[1].pose, factor.behavior,
        factor.safety_factors, factor.is_driving_forward, points[0].velocity, points[1].velocity,
        points[0].shift_length, points[1].shift_length, factor.detail);
    }
  }
}

}  // namespace autoware::safety_planner
