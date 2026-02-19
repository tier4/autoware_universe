// Copyright 2022 TIER IV, Inc.
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

#include "arrival_checker.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <tf2/utils.h>

#include <cmath>

namespace autoware::mission_planner_universe
{

ArrivalChecker::ArrivalChecker(rclcpp::Node * node) : vehicle_stop_checker_(node)
{
  const double angle_deg = node->declare_parameter<double>("arrival_check_angle_deg");
  angle_ = autoware_utils::deg2rad(angle_deg);
  if (node->has_parameter("arrival_check_distance_lateral") &&
      node->has_parameter("arrival_check_distance_longitudinal")) {
    distance_lateral_ = node->declare_parameter<double>("arrival_check_distance_lateral");
    distance_longitudinal_ = node->declare_parameter<double>("arrival_check_distance_longitudinal");
  } else {
    const double distance = node->declare_parameter<double>("arrival_check_distance");
    distance_lateral_ = distance;
    distance_longitudinal_ = distance;
  }
  duration_ = node->declare_parameter<double>("arrival_check_duration");
}

void ArrivalChecker::set_goal()
{
  // Ignore the modified goal after the route is cleared.
  goal_with_uuid_ = std::nullopt;
}

void ArrivalChecker::set_goal(const PoseWithUuidStamped & goal)
{
  // Ignore the modified goal for the previous route using uuid.
  goal_with_uuid_ = goal;
}

bool ArrivalChecker::is_arrived(const PoseStamped & pose) const
{
  if (!goal_with_uuid_) {
    return false;
  }
  const auto goal = goal_with_uuid_.value();

  // Check frame id
  if (goal.header.frame_id != pose.header.frame_id) {
    return false;
  }

  // Check distance.
  const double dx = goal.pose.position.x - pose.pose.position.x;
  const double dy = goal.pose.position.y - pose.pose.position.y;
  const double yaw_goal = tf2::getYaw(goal.pose.orientation);
  const double cos_yaw = std::cos(yaw_goal);
  const double sin_yaw = std::sin(yaw_goal);
  const double lon_distance = dx * cos_yaw + dy * sin_yaw;
  const double lat_distance = -dx * sin_yaw + dy * cos_yaw;
  if (std::fabs(lon_distance) > distance_longitudinal_) {
    return false;
  }
  if (std::fabs(lat_distance) > distance_lateral_) {
    return false;
  }

  // Check angle.
  const double yaw_pose = tf2::getYaw(pose.pose.orientation);
  const double yaw_goal = tf2::getYaw(goal.pose.orientation);
  const double yaw_diff = autoware_utils::normalize_radian(yaw_pose - yaw_goal);
  if (angle_ < std::fabs(yaw_diff)) {
    return false;
  }

  // Check vehicle stopped.
  return vehicle_stop_checker_.isVehicleStopped(duration_);
}

}  // namespace autoware::mission_planner_universe
