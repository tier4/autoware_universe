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

#ifndef FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__TYPES_HPP_
#define FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__TYPES_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using Point2d = autoware_utils_geometry::Point2d;
using Polygon2d = autoware_utils_geometry::Polygon2d;

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__TYPES_HPP_
