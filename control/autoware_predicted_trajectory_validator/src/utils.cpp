// Copyright 2023-2026 TIER IV, Inc.
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

#include "autoware/predicted_trajectory_validator/utils.hpp"

#include "autoware_utils/geometry/geometry.hpp"

#include <cmath>

namespace autoware::predicted_trajectory_validator
{

using geometry_msgs::msg::Pose;

void shift_pose(Pose & pose, double longitudinal)
{
  const auto yaw = tf2::getYaw(pose.orientation);
  pose.position.x += std::cos(yaw) * longitudinal;
  pose.position.y += std::sin(yaw) * longitudinal;
}

autoware_utils_geometry::Polygon2d transform_polygon_to_base_link(
  const autoware_utils_geometry::Polygon2d & polygon_map, const Pose & ego_pose)
{
  const auto yaw = tf2::getYaw(ego_pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  autoware_utils_geometry::Polygon2d polygon_base;
  polygon_base.outer().reserve(polygon_map.outer().size());
  for (const auto & pt : polygon_map.outer()) {
    const double dx = pt.x() - ego_pose.position.x;
    const double dy = pt.y() - ego_pose.position.y;
    polygon_base.outer().emplace_back(cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy);
  }
  return polygon_base;
}

}  // namespace autoware::predicted_trajectory_validator
