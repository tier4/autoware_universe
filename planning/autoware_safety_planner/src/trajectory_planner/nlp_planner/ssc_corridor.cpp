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

#include "ssc_corridor.hpp"

#include <autoware_utils_visualization/marker_helper.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

constexpr double EPS = 1e-9;

SlBox to_reference_box(const SemanticCube & cube)
{
  SlBox box;
  box.s_min = cube.s0;
  box.s_max = cube.s1;
  box.l_min = cube.l0;
  box.l_max = cube.l1;
  return box;
}

}  // namespace

bool is_cube_free(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const SemanticCube & cube, const double margin_m)
{
  // A cube bounds the rear axle, so the test runs on the swept footprint plus the safety margin
  SlBox box = footprint_sl_box(context.vehicle_info, to_reference_box(cube));
  box.s_min -= margin_m;
  box.s_max += margin_m;
  box.l_min -= margin_m;
  box.l_max += margin_m;

  // Every entry narrows the cube; which constraints may be dropped is decided by the set
  for (const auto & bound : compiled_constraints.lateral_bounds) {
    if (violates_lateral_bound(bound, box)) {
      return false;
    }
  }
  for (const auto & occupancy : compiled_constraints.occupancies) {
    if (violates_occupancy(occupancy, compiled_constraints, box, cube.t0, cube.t1)) {
      return false;
    }
  }
  for (const auto & stop_bar : compiled_constraints.stop_bars) {
    if (violates_stop_bar(stop_bar, box, cube.t0, cube.t1)) {
      return false;
    }
  }
  return true;
}

std::vector<CorridorSeedPoint> make_corridor_seed(
  const PlannerContext & context, const RoughPlan & rough_plan)
{
  std::vector<CorridorSeedPoint> seed;
  const std::size_t n = std::min(rough_plan.points.size(), rough_plan.s.size());
  seed.reserve(n);
  for (std::size_t k = 0; k < n; ++k) {
    CorridorSeedPoint point;
    point.t = rough_plan.points[k].t;
    point.s = rough_plan.s[k];
    // The rough plan carries s, so only l is projected: the offset from the centerline point onto
    // the left normal, which needs no nearest point search
    point.l =
      lateral_offset_at(context.reference_path, point.s, rough_plan.points[k].pose.position);
    seed.push_back(point);
  }
  return seed;
}

std::vector<SemanticCube> generate_semantic_corridor(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const std::vector<CorridorSeedPoint> & seed, const SscCorridorParams & params)
{
  std::vector<SemanticCube> cubes;
  if (seed.size() < 2 || !(params.cube_duration_s > 0.0)) {
    return cubes;
  }

  const double horizon = seed.back().t - seed.front().t;
  if (!(horizon > EPS)) {
    return cubes;
  }
  // Divide [0, T] evenly. alpha is T/n rather than exactly the requested duration, so that the
  // last piece is not a remainder
  const int num_cubes = std::max(static_cast<int>(std::round(horizon / params.cube_duration_s)), 1);
  const double alpha = horizon / static_cast<double>(num_cubes);
  const double t_origin = seed.front().t;

  for (int j = 0; j < num_cubes; ++j) {
    SemanticCube cube;
    cube.t0 = t_origin + static_cast<double>(j) * alpha;
    cube.t1 = t_origin + static_cast<double>(j + 1) * alpha;

    // ---- the smallest box around the seed of this time span ----
    // Neighboring cubes share the value at their common time, so the seed is interpolated at both
    // ends of the span and always included, even when the t grid misses the cube boundary
    const auto interpolate = [&seed](const double t) {
      if (t <= seed.front().t) {
        return seed.front();
      }
      if (t >= seed.back().t) {
        return seed.back();
      }
      for (std::size_t k = 0; k + 1 < seed.size(); ++k) {
        if (t <= seed[k + 1].t) {
          const double span = seed[k + 1].t - seed[k].t;
          const double ratio = (span > EPS) ? (t - seed[k].t) / span : 0.0;
          CorridorSeedPoint point;
          point.t = t;
          point.s = seed[k].s * (1.0 - ratio) + seed[k + 1].s * ratio;
          point.l = seed[k].l * (1.0 - ratio) + seed[k + 1].l * ratio;
          return point;
        }
      }
      return seed.back();
    };

    const auto begin_point = interpolate(cube.t0);
    const auto end_point = interpolate(cube.t1);
    cube.s0 = std::min(begin_point.s, end_point.s);
    cube.s1 = std::max(begin_point.s, end_point.s);
    cube.l0 = std::min(begin_point.l, end_point.l);
    cube.l1 = std::max(begin_point.l, end_point.l);
    for (const auto & point : seed) {
      if (point.t < cube.t0 - EPS || point.t > cube.t1 + EPS) {
        continue;
      }
      cube.s0 = std::min(cube.s0, point.s);
      cube.s1 = std::max(cube.s1, point.s);
      cube.l0 = std::min(cube.l0, point.l);
      cube.l1 = std::max(cube.l1, point.l);
    }

    // ---- rejection of the seed (Algorithm 1 of SSC) ----
    // When the initial cube, the smallest box around the seed, is not collision free, the whole
    // corridor is rejected: SSC can only carve the free space around a seed that is already
    // collision free, and shrinking a blocked seed would break the homotopy. Emitting a stop plan
    // in a blocked cycle is the rough planner's job
    if (!is_cube_free(context, compiled_constraints, cube, params.margin_m)) {
      return {};
    }

    // ---- inflation: grow the four faces in turn until each one is blocked ----
    const double step = std::max(params.inflation_step_m, EPS);
    struct Face
    {
      double * value;
      double sign;  //!< +1 for an upper face (s1 / l1), -1 for a lower one
      double origin;
      double limit;  //!< how far this face may leave the seed
    };
    const double s_origin0 = cube.s0;
    const double s_origin1 = cube.s1;
    const double l_origin0 = cube.l0;
    const double l_origin1 = cube.l1;
    std::vector<Face> faces{
      {&cube.s1, +1.0, s_origin1, params.max_longitudinal_inflation_m},
      {&cube.s0, -1.0, s_origin0, params.max_longitudinal_inflation_m},
      {&cube.l1, +1.0, l_origin1, params.max_lateral_inflation_m},
      {&cube.l0, -1.0, l_origin0, params.max_lateral_inflation_m},
    };
    std::vector<bool> blocked(faces.size(), false);
    bool any_open = true;
    while (any_open) {
      any_open = false;
      for (std::size_t f = 0; f < faces.size(); ++f) {
        if (blocked[f]) {
          continue;
        }
        auto & face = faces[f];
        const double previous = *face.value;
        if (std::abs(previous + face.sign * step - face.origin) > face.limit) {
          blocked[f] = true;
          continue;
        }
        *face.value = previous + face.sign * step;
        if (!is_cube_free(context, compiled_constraints, cube, params.margin_m)) {
          *face.value = previous;  // blocked, so this face is settled
          blocked[f] = true;
          continue;
        }
        any_open = true;
      }
    }

    cubes.push_back(cube);
  }

  // ---- join the pieces: the upper t of a cube becomes exactly the lower t of the next ----
  for (std::size_t j = 0; j + 1 < cubes.size(); ++j) {
    cubes[j + 1].t0 = cubes[j].t1;
  }
  return cubes;
}

MarkerArray make_corridor_markers(
  const PlannerContext & context, const std::vector<SemanticCube> & cubes, const double z_base)
{
  MarkerArray markers;
  const auto & path = context.reference_path;
  for (std::size_t j = 0; j < cubes.size(); ++j) {
    const auto & cube = cubes[j];
    auto marker = autoware_utils_visualization::create_default_marker(
      "map", rclcpp::Time(context.odometry.header.stamp), "ssc_corridor", static_cast<int32_t>(j),
      Marker::LINE_STRIP, autoware_utils_visualization::create_marker_scale(0.05, 0.0, 0.0),
      autoware_utils_visualization::create_marker_color(0.2, 0.7, 1.0, 0.6));

    // The four corners in (s, l), converted to world coordinates. The edges are subdivided so
    // that the shape stays recognizable on a curve
    const std::vector<std::pair<double, double>> corners{
      {cube.s0, cube.l0},
      {cube.s1, cube.l0},
      {cube.s1, cube.l1},
      {cube.s0, cube.l1},
      {cube.s0, cube.l0}};
    constexpr int SUBDIVISION = 8;
    for (std::size_t c = 0; c + 1 < corners.size(); ++c) {
      for (int i = 0; i < SUBDIVISION; ++i) {
        const double ratio = static_cast<double>(i) / SUBDIVISION;
        const double s = corners[c].first * (1.0 - ratio) + corners[c + 1].first * ratio;
        const double l = corners[c].second * (1.0 - ratio) + corners[c + 1].second * ratio;
        const auto pose = to_world_pose(path, std::clamp(s, 0.0, path.length()), l);
        geometry_msgs::msg::Point point;
        point.x = pose.position.x();
        point.y = pose.position.y();
        point.z = z_base;
        marker.points.push_back(point);
      }
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
      markers.markers.push_back(marker);
    }
  }
  return markers;
}

}  // namespace autoware::safety_planner
