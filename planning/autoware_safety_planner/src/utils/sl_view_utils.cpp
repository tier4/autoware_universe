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

#include "sl_view_utils.hpp"

#include <autoware/trajectory/utils/closest.hpp>

#include <algorithm>
#include <cmath>
#include <variant>
#include <vector>

namespace autoware::safety_planner
{

KinematicLimits collect_kinematic_limits(const CompiledConstraints & compiled_constraints)
{
  KinematicLimits limits;
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    const bool is_global = bound.s0 == -INF && bound.s1 == INF;
    if (!is_global) {
      continue;  // a bound limited to an interval (a map speed limit, ...) is read per s
    }
    // Every global bound of the IR is a hard limit; the nominal values keep their defaults. In
    // particular the hard LAT_ACCEL is not fed into a_lat_nom, which would loosen the corner
    // deceleration
    switch (bound.quantity) {
      case BoundedQuantity::VELOCITY:
        limits.v_hard = std::min(limits.v_hard, bound.max);
        break;
      case BoundedQuantity::LON_ACCEL:
        limits.a_hard_min = std::max(limits.a_hard_min, bound.min);
        limits.a_hard_max = std::min(limits.a_hard_max, bound.max);
        break;
      default:
        // LAT_ACCEL / LON_JERK / CURVATURE / STEER_* are the NLP's job, not the views'
        break;
    }
  }
  return limits;
}

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

double interpolate_boundary_l(const std::vector<SlPoint> & polyline, const double s)
{
  if (s <= polyline.front().s) {
    return polyline.front().l;
  }
  if (s >= polyline.back().s) {
    return polyline.back().l;
  }
  for (std::size_t seg = 0; seg + 1 < polyline.size(); ++seg) {
    const auto & p0 = polyline[seg];
    const auto & p1 = polyline[seg + 1];
    if (s <= p1.s) {
      const double ratio = (p1.s > p0.s) ? (s - p0.s) / (p1.s - p0.s) : 0.0;
      return p0.l * (1.0 - ratio) + p1.l * ratio;
    }
  }
  return polyline.back().l;
}

bool lateral_bound_extreme_l(
  const LateralBoundEntry & bound, const double s_lo_in, const double s_hi_in, double & extreme_l)
{
  const auto & polyline = bound.polyline;
  if (polyline.size() < 2) {
    return false;
  }
  if (s_hi_in < polyline.front().s || s_lo_in > polyline.back().s) {
    return false;  // the constraint does not apply where the s ranges do not overlap
  }

  // Extremum of the boundary l over the overlap [s_lo, s_hi]. LEFT forbids everything left of the
  // boundary (larger l), so the tightest value is min(l_b); RIGHT is the other way round
  const double s_lo = std::max(s_lo_in, polyline.front().s);
  const double s_hi = std::min(s_hi_in, polyline.back().s);
  double value = interpolate_boundary_l(polyline, s_lo);
  const auto update = [&](const double l_b) {
    value = (bound.forbidden_side == Side::LEFT) ? std::min(value, l_b) : std::max(value, l_b);
  };
  update(interpolate_boundary_l(polyline, s_hi));
  for (const auto & vertex : polyline) {
    if (vertex.s > s_lo && vertex.s < s_hi) {
      update(vertex.l);
    }
  }
  extreme_l = value;
  return true;
}

bool violates_lateral_bound(const LateralBoundEntry & bound, const SlBox & box)
{
  double extreme_l = 0.0;
  if (!lateral_bound_extreme_l(bound, box.s_min, box.s_max, extreme_l)) {
    return false;
  }
  if (bound.forbidden_side == Side::LEFT) {
    return box.l_max > extreme_l;
  }
  return box.l_min < extreme_l;
}

bool violates_occupancy(
  const OccupancyEntry & occupancy, const SlBox & box, const double t0, const double t1)
{
  for (const auto & slab : occupancy.slabs) {
    const bool time_overlaps = slab.t1 >= t0 && slab.t0 <= t1;
    if (!time_overlaps) {
      continue;
    }
    const bool box_overlaps =
      slab.s1 >= box.s_min && slab.s0 <= box.s_max && slab.l1 >= box.l_min && slab.l0 <= box.l_max;
    if (box_overlaps) {
      return true;
    }
  }
  return false;
}

bool violates_stop_bar(
  const StopBarEntry & stop_bar, const SlBox & box, const double t0, const double t1)
{
  const bool time_overlaps = stop_bar.time.t1 >= t0 && stop_bar.time.t0 <= t1;
  if (!time_overlaps) {
    return false;
  }
  return box.s_max > stop_bar.s_stop;
}

}  // namespace autoware::safety_planner
