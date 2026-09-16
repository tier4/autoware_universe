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

#include "route_tracking.hpp"

#include <lanelet2_core/geometry/Lanelet.h>

#include <optional>
#include <utility>

namespace autoware::safety_planner
{

std::optional<RouteManager> track_current_lanelet(
  RouteManager && route_manager, const Pose & current_pose, const double dist_threshold_m,
  const double yaw_threshold_rad)
{
  // update_current_pose() only looks along the current lane, so a completed lane change has to
  // be detected here: base_link has left the current lanelet and lies inside another one on the
  // route. Until then the current lane is kept, which is what makes the path a return path.
  // TODO(odashima): change to centerpoint of vehicle
  const lanelet::BasicPoint2d ego{current_pose.position.x, current_pose.position.y};
  if (!lanelet::geometry::inside(route_manager.current_lanelet(), ego)) {
    const auto other = route_manager.get_closest_route_lanelet_within_constraints(
      current_pose, dist_threshold_m, yaw_threshold_rad);
    if (
      other && other->id() != route_manager.current_lanelet().id() &&
      lanelet::geometry::inside(*other, ego)) {
      return std::move(route_manager).commit_lane_change_success(current_pose);
    }
  }
  return std::move(route_manager)
    .update_current_pose(current_pose, dist_threshold_m, yaw_threshold_rad);
}

}  // namespace autoware::safety_planner
