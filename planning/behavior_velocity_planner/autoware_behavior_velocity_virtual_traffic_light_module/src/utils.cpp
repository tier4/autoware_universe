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

#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <limits>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::virtual_traffic_light
{

using autoware_utils::calc_distance2d;

tier4_v2x_msgs::msg::KeyValue createKeyValue(const std::string & key, const std::string & value)
{
  return tier4_v2x_msgs::build<tier4_v2x_msgs::msg::KeyValue>().key(key).value(value);
}

autoware_utils::LineString3d toAutowarePoints(const lanelet::ConstLineString3d & line_string)
{
  autoware_utils::LineString3d output;
  for (const auto & p : line_string) {
    output.emplace_back(p.x(), p.y(), p.z());
  }
  return output;
}

std::optional<autoware_utils::LineString3d> toAutowarePoints(
  const lanelet::Optional<lanelet::ConstLineString3d> & line_string)
{
  if (!line_string) {
    return {};
  }
  return toAutowarePoints(*line_string);
}

std::vector<autoware_utils::LineString3d> toAutowarePoints(
  const lanelet::ConstLineStrings3d & line_strings)
{
  std::vector<autoware_utils::LineString3d> output;
  for (const auto & line_string : line_strings) {
    output.emplace_back(toAutowarePoints(line_string));
  }
  return output;
}

autoware_utils::Point3d calcCenter(const autoware_utils::LineString3d & line_string)
{
  const auto p1 = line_string.front();
  const auto p2 = line_string.back();
  const auto p_center = (p1 + p2) / 2;
  return {p_center.x(), p_center.y(), p_center.z()};
}

geometry_msgs::msg::Pose calcHeadPose(
  const geometry_msgs::msg::Pose & base_link_pose, const double base_link_to_front)
{
  return autoware_utils::calc_offset_pose(base_link_pose, base_link_to_front, 0.0, 0.0);
}

geometry_msgs::msg::Point convertToGeomPoint(const autoware_utils::Point3d & p)
{
  geometry_msgs::msg::Point geom_p;
  geom_p.x = p.x();
  geom_p.y = p.y();

  return geom_p;
}

std::optional<double> findLastCollisionBeforeEndLine(
  const Trajectory & path, const lanelet::ConstLineString3d & target_line, const double end_line_s)
{
  auto cropped_path = path;
  cropped_path.crop(0., end_line_s);

  const auto collision = experimental::trajectory::crossed(cropped_path, target_line);
  if (collision.empty()) {
    return std::nullopt;
  }

  return collision.front();
}

std::optional<double> findLastCollisionBeforeEndLine(
  const Trajectory & path, const std::vector<lanelet::ConstLineString3d> & target_lines,
  const double end_line_s)
{
  for (const auto & target_line : target_lines) {
    if (const auto collision = findLastCollisionBeforeEndLine(path, target_line, end_line_s)) {
      return collision;
    }
  }

  return std::nullopt;
}

}  // namespace autoware::behavior_velocity_planner::virtual_traffic_light
