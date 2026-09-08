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

#include "context.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace autoware::safety_planner
{

std::optional<double> PlannerContext::goal_arc_length() const
{
  //! [m] longitudinal distance within which the goal counts as connected
  constexpr double LON_DISTANCE_TOLERANCE_M = 0.1;

  if (reference_path.get_underlying_bases().empty()) {
    return std::nullopt;
  }

  // Signed longitudinal distance of the goal along the tangent at the end point. The projection of
  // closest() is not used: it clamps at the end and would report 0 for a goal beyond the path
  const double s_end = reference_path.length();
  const auto end_point = reference_path.compute(s_end).point.pose.position;
  const double end_yaw = reference_path.azimuth(s_end);

  const double dx = goal_pose.position.x - end_point.x;
  const double dy = goal_pose.position.y - end_point.y;
  const double lon_distance = std::cos(end_yaw) * dx + std::sin(end_yaw) * dy;

  if (std::abs(lon_distance) > LON_DISTANCE_TOLERANCE_M) {
    return std::nullopt;
  }
  return std::max(0.0, s_end + lon_distance);
}

bool PlannerContext::is_reference_path_connected_to_goal_pose() const
{
  return goal_arc_length().has_value();
}

}  // namespace autoware::safety_planner
