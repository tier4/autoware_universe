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

#include "compiled_constraints_utils.hpp"

#include <algorithm>
#include <cstddef>
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
        // LAT_ACCEL / LON_JERK / STEER_* are the NLP's job, not the views'
        break;
    }
  }
  return limits;
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
