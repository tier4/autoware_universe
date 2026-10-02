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

#include "vehicle_kinematics.hpp"

#include <optional>
#include <string>
#include <utility>

namespace autoware::safety_planner
{

ConstraintGeneratorOutput VehicleKinematicsConstraintGenerator::generate_constraints(
  const PlannerContext & context)
{
  ConstraintGeneratorOutput output;

  const auto add = [&output](
                     const BoundedQuantity quantity, const double min, const double max,
                     const std::string & detail, const Hardness hardness) {
    Constraint constraint;
    constraint.hardness = hardness;
    constraint.payload = ScalarBound{quantity, min, max};
    constraint.source = Source{"vehicle_kinematics", "", detail};
    output.constraints.push_back(std::move(constraint));
  };

  // HARD constraints
  const auto & hard_params = params_.vehicle_kinematics.max;
  // NOTE(odashima): the speed bound is the one of external_velocity_limit, which owns both the
  // default and the limit given from outside
  add(
    BoundedQuantity::LON_ACCEL, hard_params.lon_accel_min_mps2, hard_params.lon_accel_max_mps2,
    "lon_accel", Hardness::HARD);
  // NOTE(odashima): left at -INF for quantities bounded in absolute value
  add(BoundedQuantity::LON_JERK, -INF, hard_params.lon_jerk_mps3, "lon_jerk", Hardness::HARD);
  add(BoundedQuantity::LAT_ACCEL, -INF, hard_params.lat_accel_mps2, "lat_accel", Hardness::HARD);
  add(
    BoundedQuantity::STEER_ANGLE, -INF, context.vehicle_info.max_steer_angle_rad, "steer_angle",
    Hardness::HARD);
  add(
    BoundedQuantity::STEER_RATE, -INF, hard_params.steer_rate_radps, "steer_rate", Hardness::HARD);

  // SOFT constraints
  const auto & soft_params = params_.vehicle_kinematics.nominal;
  add(
    BoundedQuantity::LON_ACCEL, soft_params.lon_accel_min_mps2, soft_params.lon_accel_max_mps2,
    "lon_accel", Hardness::SOFT);
  add(BoundedQuantity::LON_JERK, -INF, soft_params.lon_jerk_mps3, "lon_jerk", Hardness::SOFT);
  add(BoundedQuantity::LAT_ACCEL, -INF, soft_params.lat_accel_mps2, "lat_accel", Hardness::SOFT);

  return output;
}

}  // namespace autoware::safety_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::VehicleKinematicsConstraintGenerator,
  autoware::safety_planner::ConstraintGeneratorInterface)
