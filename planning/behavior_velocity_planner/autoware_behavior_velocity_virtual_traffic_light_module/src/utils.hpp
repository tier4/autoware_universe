// Copyright 2024 Tier IV, Inc.
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <autoware/behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/virtual_traffic_light.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/system/time_keeper.hpp>

#include <tier4_v2x_msgs/msg/key_value.hpp>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::virtual_traffic_light
{
using Trajectory =
  experimental::trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>;

tier4_v2x_msgs::msg::KeyValue createKeyValue(const std::string & key, const std::string & value);

autoware_utils::LineString3d toAutowarePoints(const lanelet::ConstLineString3d & line_string);

std::optional<autoware_utils::LineString3d> toAutowarePoints(
  const lanelet::Optional<lanelet::ConstLineString3d> & line_string);

std::vector<autoware_utils::LineString3d> toAutowarePoints(
  const lanelet::ConstLineStrings3d & line_strings);

autoware_utils::Point3d calcCenter(const autoware_utils::LineString3d & line_string);

geometry_msgs::msg::Pose calcHeadPose(
  const geometry_msgs::msg::Pose & base_link_pose, const double base_link_to_front);

geometry_msgs::msg::Point convertToGeomPoint(const autoware_utils::Point3d & p);

std::optional<double> findLastCollisionBeforeEndLine(
  const Trajectory & path, const lanelet::ConstLineString3d & target_line, const double end_line_s);

std::optional<double> findLastCollisionBeforeEndLine(
  const Trajectory & path, const std::vector<lanelet::ConstLineString3d> & target_lines,
  const double end_line_s);

}  // namespace autoware::behavior_velocity_planner::virtual_traffic_light

#endif  // UTILS_HPP_
