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

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_internal_planning_msgs/msg/safety_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

namespace autoware::safety_planner
{
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::SafetyFactor;
using autoware_internal_planning_msgs::msg::SafetyFactorArray;

namespace
{

std::optional<unique_identifier_msgs::msg::UUID> parse_uuid(const std::string & hex)
{
  if (hex.size() != 32) {
    return std::nullopt;
  }
  unique_identifier_msgs::msg::UUID uuid;
  try {
    for (std::size_t i = 0; i < uuid.uuid.size(); ++i) {
      uuid.uuid[i] = static_cast<std::uint8_t>(std::stoul(hex.substr(i * 2, 2), nullptr, 16));
    }
  } catch (const std::exception &) {
    return std::nullopt;
  }
  return uuid;
}

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

//! Object id and position when the constraint names a predicted object
SafetyFactorArray safety_factors_of(const Constraint & constraint, const PlannerContext & context)
{
  SafetyFactorArray safety;
  const auto uuid = parse_uuid(constraint.source.target_id);
  if (!uuid) {
    safety.is_safe = true;
    return safety;
  }

  SafetyFactor factor;
  factor.type = SafetyFactor::OBJECT;
  factor.object_id = *uuid;
  factor.is_safe = false;
  if (context.predicted_objects) {
    for (const auto & object : context.predicted_objects->objects) {
      if (autoware_utils_uuid::to_hex_string(object.object_id) != constraint.source.target_id) {
        continue;
      }
      factor.points.push_back(object.kinematics.initial_pose_with_covariance.pose.position);
      break;
    }
  }
  if (factor.points.empty()) {
    if (const auto * keep_out = std::get_if<KeepOut>(&constraint.payload)) {
      if (!keep_out->waypoints.empty()) {
        geometry_msgs::msg::Point point;
        point.x = keep_out->waypoints.front().pose.position.x();
        point.y = keep_out->waypoints.front().pose.position.y();
        factor.points.push_back(point);
      }
    }
  }

  safety.is_safe = false;
  safety.detail = constraint.source.detail;
  safety.factors.push_back(std::move(factor));
  return safety;
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
  const CompiledConstraints & compiled_constraints, const double horizon_s,
  const double goal_search_radius_m)
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
  } else if (
    autoware_utils_geometry::calc_distance2d(pose_on_path(path, path.length()), context.goal_pose) <
    goal_search_radius_m) {
    SafetyFactorArray safety;
    safety.is_safe = true;
    planning_factor_interface->add(
      path.length() - s_ego, pose_on_path(path, path.length()), PlanningFactor::STOP, safety, true,
      0.0, 0.0, "goal");
  }

  for (const auto & bound : compiled_constraints.scalar_bounds) {
    const bool global = bound.s0 == -INF && bound.s1 == INF;
    if (bound.quantity != BoundedQuantity::VELOCITY || global) {
      continue;
    }
    if (bound.raw_index >= compiled_constraints.raw_constraints.size()) {
      continue;
    }
    if (bound.s1 < s_ego || bound.s0 > path.length()) {
      continue;
    }
    const double s0 = std::max(bound.s0, s_ego);
    const double s1 = std::min(bound.s1, path.length());
    const auto & constraint = compiled_constraints.raw_constraints[bound.raw_index];
    const auto safety = safety_factors_of(constraint, context);
    const auto detail = detail_of(constraint.source);
    if (s1 <= s0) {
      planning_factor_interface->add(
        s0 - s_ego, pose_on_path(path, s0), PlanningFactor::SLOW_DOWN, safety, true, bound.max, 0.0,
        detail);
    } else {
      planning_factor_interface->add(
        s0 - s_ego, s1 - s_ego, pose_on_path(path, s0), pose_on_path(path, s1),
        PlanningFactor::SLOW_DOWN, safety, true, bound.max, bound.max, 0.0, 0.0, detail);
    }
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
