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
                     const std::string & detail) {
    Constraint constraint;
    constraint.payload = ScalarBound{quantity, min, max, std::nullopt};
    constraint.source = Source{"vehicle_kinematics", Category::SAFETY, "", detail};
    output.constraints.push_back(std::move(constraint));
  };

  const auto & p = params_.vehicle_kinematics;

  add(BoundedQuantity::VELOCITY, 0.0, p.velocity_hard_mps, "velocity");
  add(
    BoundedQuantity::LON_ACCEL, p.lon_accel_hard_min_mps2, p.lon_accel_hard_max_mps2, "lon_accel");
  // NOTE(odashima): left at -INF for quantities bounded in absolute value
  add(BoundedQuantity::LON_JERK, -INF, p.lon_jerk_hard_mps3, "lon_jerk");
  add(BoundedQuantity::LAT_ACCEL, -INF, p.lat_accel_hard_mps2, "lat_accel");
  add(BoundedQuantity::STEER_ANGLE, -INF, context.vehicle_info.max_steer_angle_rad, "steer_angle");
  add(BoundedQuantity::STEER_RATE, -INF, p.steer_rate_hard_radps, "steer_rate");

  // TODO(odashima): add soft constraints for comfort

  return output;
}

}  // namespace autoware::safety_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::VehicleKinematicsConstraintGenerator,
  autoware::safety_planner::ConstraintGeneratorInterface)
