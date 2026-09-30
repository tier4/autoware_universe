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

#include "trajectory_postprocess.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <cstddef>

namespace autoware::safety_planner
{

Trajectory set_engage_speed(const Trajectory & trajectory, const double engage_velocity_mps)
{
  //! [m] A trajectory shorter than this is not a launch, and is left alone. It keeps the
  //! deceleration in front of the goal and a stop plan (all points at the same place) from engaging
  constexpr double MIN_ENGAGE_DIST_M = 0.5;

  Trajectory result = trajectory;
  if (result.points.empty() || !(engage_velocity_mps > 0.0)) {
    return result;
  }

  // The travelled distance is measured on the points themselves. After the time parameterization
  // the points of a stopped section sit on top of each other, so a stop plan lands near 0 here
  double length = 0.0;
  for (std::size_t k = 0; k + 1 < result.points.size(); ++k) {
    length += autoware_utils_geometry::calc_distance2d(
      result.points[k].pose.position, result.points[k + 1].pose.position);
  }
  if (length <= MIN_ENGAGE_DIST_M) {
    return result;
  }
  // A trajectory that never reaches the engage speed is a stop in progress, not a launch (an MPPI
  // output rolls a little past the stop point at a crawl, so its length alone does not tell)
  const bool reaches_engage_speed =
    std::any_of(result.points.begin(), result.points.end(), [&](const TrajectoryPoint & point) {
      return point.longitudinal_velocity_mps >= static_cast<float>(engage_velocity_mps);
    });
  if (!reaches_engage_speed) {
    return result;
  }

  for (auto & point : result.points) {
    if (point.longitudinal_velocity_mps >= static_cast<float>(engage_velocity_mps)) {
      break;  // from here on the trajectory is left as is, deceleration at the end included
    }
    point.longitudinal_velocity_mps = static_cast<float>(engage_velocity_mps);
  }
  return result;
}

}  // namespace autoware::safety_planner
