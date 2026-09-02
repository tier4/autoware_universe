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
  //! 接続とみなす縦距離 [m]
  constexpr double LON_DISTANCE_TOLERANCE_M = 0.1;

  if (reference_path.get_underlying_bases().empty()) {
    return std::nullopt;
  }

  // 終端点の接線方向に測った goal の符号付き縦距離。閉じた閾値との比較なので
  // closest() の射影 (終端でクランプされ、goal が経路の先にある場合に 0 になる) は使わない
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
