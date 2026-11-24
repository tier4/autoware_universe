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

#ifndef AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__UTILS_HPP_
#define AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__UTILS_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

namespace autoware::predicted_trajectory_validator
{

/**
 * @brief Shift pose along the yaw direction
 */
void shift_pose(geometry_msgs::msg::Pose & pose, double longitudinal);

/**
 * @brief Express a map polygon in the frame whose origin is ego_pose (base_link of that pose)
 * @param polygon_map polygon in the map frame
 * @param ego_pose ego pose in the map frame
 */
autoware_utils_geometry::Polygon2d transform_polygon_to_base_link(
  const autoware_utils_geometry::Polygon2d & polygon_map,
  const geometry_msgs::msg::Pose & ego_pose);

}  // namespace autoware::predicted_trajectory_validator

#endif  // AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__UTILS_HPP_
