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

#include "frenet_utils.hpp"

#include <autoware/trajectory/utils/closest.hpp>

#include <algorithm>
#include <cmath>

namespace autoware::safety_planner
{

double lateral_offset_at(const PathPointTrajectory & path, const double s, const Point2d & q)
{
  const auto ref_position = path.compute(s).point.pose.position;
  const double ref_yaw = path.azimuth(s);
  const double dx = q.x() - ref_position.x;
  const double dy = q.y() - ref_position.y;
  return -std::sin(ref_yaw) * dx + std::cos(ref_yaw) * dy;
}

EgoFrenetState compute_ego_frenet_state(const PlannerContext & context)
{
  const auto & path = context.reference_path;
  const auto & position = context.odometry.pose.pose.position;
  EgoFrenetState state;

  // TODO(odashima): check trajectory class closest logic
  state.s = experimental::trajectory::closest(path, position);
  // closest() approximates the path between its bases by a chord, which puts the foot off by tens
  // of centimeters in both s and l on a curve (about 20 cm at a base spacing of 4 m and R = 10 m).
  // s is corrected with Newton's method (f' = -(1 - k*l)) until the tangential residual
  // f(s) = (q - p(s)).t(s) vanishes. Adding the residual directly instead is a fixed point
  // iteration with contraction |k*l|, which does not converge outside a tight curve (k*l -> 1)
  for (int i = 0; i < 10; ++i) {
    const auto ref_position = path.compute(state.s).point.pose.position;
    const double ref_yaw = path.azimuth(state.s);
    const double dx = position.x - ref_position.x;
    const double dy = position.y - ref_position.y;
    const double residual = std::cos(ref_yaw) * dx + std::sin(ref_yaw) * dy;
    const double l = -std::sin(ref_yaw) * dx + std::cos(ref_yaw) * dy;
    // Near the center of curvature (1 - k*l = 0) the foot is undetermined, so limit the step
    const double denom = std::max(1.0 - path.curvature(state.s) * l, 0.2);
    const double ds = residual / denom;
    state.s = std::clamp(state.s + ds, 0.0, path.length());
    if (std::abs(ds) < 1e-3) {
      break;
    }
  }
  state.l = lateral_offset_at(path, state.s, Point2d{position.x, position.y});
  return state;
}

Pose2d to_world_pose(const PathPointTrajectory & path, const double s, const double l)
{
  const auto ref_position = path.compute(s).point.pose.position;
  const double ref_yaw = path.azimuth(s);
  Pose2d pose;
  // Left normal of the centerline, the positive direction of l
  pose.position =
    Point2d{ref_position.x - std::sin(ref_yaw) * l, ref_position.y + std::cos(ref_yaw) * l};
  pose.yaw = ref_yaw;
  return pose;
}

SlBox footprint_sl_box(const VehicleInfo & vehicle_info, const double s, const double l)
{
  SlBox box;
  box.s_min = s + vehicle_info.min_longitudinal_offset_m;  // rear
  box.s_max = s + vehicle_info.max_longitudinal_offset_m;  // front
  box.l_min = l + vehicle_info.min_lateral_offset_m;       // right
  box.l_max = l + vehicle_info.max_lateral_offset_m;       // left
  return box;
}

SlBox footprint_sl_box(const VehicleInfo & vehicle_info, const SlBox & reference_box)
{
  SlBox box;
  box.s_min = reference_box.s_min + vehicle_info.min_longitudinal_offset_m;
  box.s_max = reference_box.s_max + vehicle_info.max_longitudinal_offset_m;
  box.l_min = reference_box.l_min + vehicle_info.min_lateral_offset_m;
  box.l_max = reference_box.l_max + vehicle_info.max_lateral_offset_m;
  return box;
}

}  // namespace autoware::safety_planner
