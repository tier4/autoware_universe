// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#ifndef FOOTPRINTS_HPP_
#define FOOTPRINTS_HPP_

#include "types.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <Eigen/src/Geometry/Rotation2D.h>
#include <Eigen/src/Geometry/RotationBase.h>

#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
inline run_out::TrajectoryCornerFootprint calculate_trajectory_corner_footprint(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  autoware::vehicle_info_utils::VehicleInfo vehicle_info, const run_out::Parameters & params)
{
  run_out::TrajectoryCornerFootprint trajectory_footprint;
  auto & footprint = trajectory_footprint.corner_footprint;
  const auto base_footprint =
    vehicle_info.createFootprint(params.ego_lateral_margin, params.ego_longitudinal_margin);
  for (const auto & p : trajectory) {
    const universe_utils::Point2d base_link(p.pose.position.x, p.pose.position.y);
    const auto angle = tf2::getYaw(p.pose.orientation);
    const Eigen::Rotation2Dd rotation(angle);
    const auto rotated_front_left_offset =
      rotation * base_footprint[vehicle_info_utils::VehicleInfo::FrontLeftIndex];
    const auto rotated_front_right_offset =
      rotation * base_footprint[vehicle_info_utils::VehicleInfo::FrontRightIndex];
    const auto rotated_rear_right_offset =
      rotation * base_footprint[vehicle_info_utils::VehicleInfo::RearRightIndex];
    const auto rotated_rear_left_offset =
      rotation * base_footprint[vehicle_info_utils::VehicleInfo::RearLeftIndex];
    footprint.front_left_ls.emplace_back(
      base_link.x() + rotated_front_left_offset.x(), base_link.y() + rotated_front_left_offset.y());
    footprint.front_right_ls.emplace_back(
      base_link.x() + rotated_front_right_offset.x(),
      base_link.y() + rotated_front_right_offset.y());
    footprint.rear_right_ls.emplace_back(
      base_link.x() + rotated_rear_right_offset.x(), base_link.y() + rotated_rear_right_offset.y());
    footprint.rear_left_ls.emplace_back(
      base_link.x() + rotated_rear_left_offset.x(), base_link.y() + rotated_rear_left_offset.y());
  }

  const auto push_to_polygon_fn = [](universe_utils::Polygon2d & poly) {
    return [&](const universe_utils::Point2d & p) { poly.outer().push_back(p); };
  };
  std::for_each(
    footprint.front_left_ls.begin(), footprint.front_left_ls.end(),
    push_to_polygon_fn(trajectory_footprint.front_polygon));
  std::for_each(
    footprint.front_right_ls.rbegin(), footprint.front_right_ls.rend(),
    push_to_polygon_fn(trajectory_footprint.front_polygon));
  boost::geometry::correct(trajectory_footprint.front_polygon);
  std::for_each(
    footprint.rear_left_ls.begin(), footprint.rear_left_ls.end(),
    push_to_polygon_fn(trajectory_footprint.rear_polygon));
  std::for_each(
    footprint.rear_right_ls.rbegin(), footprint.rear_right_ls.rend(),
    push_to_polygon_fn(trajectory_footprint.rear_polygon));
  boost::geometry::correct(trajectory_footprint.rear_polygon);
  return trajectory_footprint;
}
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // FOOTPRINTS_HPP_
