// Copyright 2025 TIER IV, Inc.
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
#ifndef CLASS_HELPER_HPP_
#define CLASS_HELPER_HPP_

#include "data_types.hpp"
#include "hermite_spline.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace autoware::time_to_space_trajectory_converter::helper
{
std::optional<std::string> check_odometry_msg(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_ptr, const rclcpp::Time & now,
  const double timeout_s);

std::optional<std::string> check_trajectory_msg(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & traj_ptr,
  const rclcpp::Time & now, const double timeout_s);

std::optional<std::string> has_non_monotonic(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points);

double solve_time_step(double current_s, double next_s, const HermiteSpline & sv);

SplineData resample(
  const SplineData & spline_data, const double resolution, const bool recompute_acceleration);

double calculate_yaw_at_index(const SplineData & spline, size_t i, double last_valid_yaw);

std::vector<PlannerPoint> convert_msg_to_planner_points(
  const autoware_planning_msgs::msg::Trajectory & traj);

autoware_planning_msgs::msg::Trajectory convert_spline_data_to_trajectory_msg(
  const SplineData & spline, const std_msgs::msg::Header & header);

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> convert_spline_data_to_trajectory_points(
  const SplineData & spline);

/**
 * @brief Converts odometry history buffer into PlannerPoints for spline generation.
 * Iterates backwards (Oldest -> Newest) to prepare for prepending to trajectory.
 */
std::vector<PlannerPoint> convert_odometry_history_to_planner_points(
  const std::deque<nav_msgs::msg::Odometry> & odom_history);
}  // namespace autoware::time_to_space_trajectory_converter::helper

#endif  // CLASS_HELPER_HPP_
