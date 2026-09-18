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

#include "external_velocity_limit.hpp"

#include "../utils/frenet_utils.hpp"

#include <boost/geometry/algorithms/correct.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

Polygon2d make_band_polygon(const std::vector<Pose2d> & centerline, const double half_width_m)
{
  Polygon2d polygon;
  auto & ring = polygon.outer();
  ring.reserve(2 * centerline.size() + 1);
  for (const auto & pose : centerline) {
    const double nx = -std::sin(pose.yaw);  // left normal
    const double ny = std::cos(pose.yaw);
    ring.emplace_back(pose.position.x() + half_width_m * nx, pose.position.y() + half_width_m * ny);
  }
  for (auto it = centerline.rbegin(); it != centerline.rend(); ++it) {
    const double nx = -std::sin(it->yaw);
    const double ny = std::cos(it->yaw);
    ring.emplace_back(it->position.x() - half_width_m * nx, it->position.y() - half_width_m * ny);
  }
  boost::geometry::correct(polygon);
  return polygon;
}

}  // namespace

ConstraintGeneratorOutput ExternalVelocityLimitConstraintGenerator::generate_constraints(
  const PlannerContext & context)
{
  ConstraintGeneratorOutput output;

  const auto & p = params_.external_velocity_limit;
  const double v_limit = context.external_velocity_limit_mps.value_or(p.default_velocity_mps);
  const double v_ego = std::max(0.0, context.odometry.twist.twist.linear.x);

  // latch velocity limit
  if (satisfied_limit_mps_ != v_limit) {
    satisfied_limit_mps_.reset();
  }
  if (v_ego <= v_limit) {
    satisfied_limit_mps_ = v_limit;
  }

  {
    Constraint constraint;
    constraint.payload = ScalarBound{
      BoundedQuantity::VELOCITY, 0.0, satisfied_limit_mps_ ? v_limit : std::max(v_limit, v_ego)};
    constraint.source = Source{get_name(), "", "velocity"};
    output.constraints.push_back(std::move(constraint));
  }

  if (satisfied_limit_mps_) {
    return output;  // a limit the ego meets takes effect at once, and needs no braking distance
  }

  const auto & path = context.reference_path;
  const double length = path.length();
  const double braking_distance_m =
    (v_ego * v_ego - v_limit * v_limit) / (2.0 * p.decel_mps2) + p.margin_m;
  const double s_begin = compute_ego_frenet_state(context).s + braking_distance_m;
  if (s_begin >= length) {
    return output;  // the limit falls beyond the reference path of this cycle
  }

  const auto num_division =
    static_cast<std::size_t>(std::ceil((length - s_begin) / p.sample_interval_m));
  std::vector<Pose2d> centerline;
  centerline.reserve(num_division + 1);
  for (std::size_t i = 0; i <= num_division; ++i) {
    const double s = std::min(s_begin + static_cast<double>(i) * p.sample_interval_m, length);
    const auto position = path.compute(s).point.pose.position;
    centerline.push_back(Pose2d{Point2d{position.x, position.y}, path.azimuth(s)});
  }

  auto region = make_band_polygon(centerline, p.half_width_m);
  if (region.outer().size() < 3) {
    return output;
  }

  Constraint constraint;
  constraint.payload = SpeedLimitZone{std::move(region), v_limit};
  constraint.source = Source{get_name(), "", "speed_limit"};
  output.constraints.push_back(std::move(constraint));

  return output;
}

}  // namespace autoware::safety_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::ExternalVelocityLimitConstraintGenerator,
  autoware::safety_planner::ConstraintGeneratorInterface)
