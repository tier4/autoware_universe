// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef SLOWDOWN_HPP_
#define SLOWDOWN_HPP_

#include "parameters.hpp"
#include "types.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common_universe/velocity_planning_result.hpp>
#include <autoware/motion_velocity_planner_common_universe/planner_data.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <algorithm>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
/// @brief calculate the interpolated point along the trajectory at the given time from start
/// @param trajectory ego trajectory starting from the ego pose (UB if empty)
/// @param time [s] requested time
/// @return trajectory point corresponding to the given time
inline geometry_msgs::msg::Point interpolated_point_at_time(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory, const double time)
{
  const auto & prev_it = std::find_if(
    trajectory.begin(), trajectory.end(),
    [&](const autoware_planning_msgs::msg::TrajectoryPoint & t) {
      return rclcpp::Duration(t.time_from_start).seconds() >= time;
    });
  const auto prev_time = rclcpp::Duration(prev_it->time_from_start).seconds();
  if (prev_time == time) {
    return prev_it->pose.position;
  }
  const auto next_time = rclcpp::Duration(std::next(prev_it)->time_from_start).seconds();
  const auto t_delta = next_time - prev_time;
  const auto t_diff = time - prev_time;
  const auto ratio = t_diff / t_delta;
  return universe_utils::calcInterpolatedPoint(
    prev_it->pose.position, std::next(prev_it)->pose.position, ratio);
}

/// @brief calculate the stop for the given decision history
/// @param [inout] history decision history
/// @param [in] trajectory ego trajectory starting from the current ego pose
/// @param [in] params module parameters
/// @return stop point
inline std::optional<geometry_msgs::msg::Point> calculate_stop_position(
  DecisionHistory & history,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const Parameters & params)
{
  const auto max_time = rclcpp::Duration(trajectory.back().time_from_start).seconds();
  std::optional<geometry_msgs::msg::Point> stop_position;
  auto & current_decision = history.decisions.back();
  if (current_decision.type == stop) {
    const auto t_coll = current_decision.collision->ego_collision_time;
    if(t_coll > max_time) {
      return stop_position;
    }
    const auto t_stop = std::max(0.0, t_coll);
    const auto base_link_point = interpolated_point_at_time(trajectory, t_stop);
    auto stop_point_length = motion_utils::calcSignedArcLength(trajectory, 0, base_link_point) -
                             params.stop_distance_buffer;
    current_decision.stop_point =
      motion_utils::calcInterpolatedPose(trajectory, stop_point_length).position;
    if (params.stop_calculate_earliest_within_history) {
      for (const auto & decision : history.decisions) {
        if (decision.stop_point) {
          stop_point_length = std::min(
            motion_utils::calcSignedArcLength(trajectory, 0, *decision.stop_point),
            stop_point_length);
        }
      }
      stop_position = motion_utils::calcInterpolatedPose(trajectory, stop_point_length).position;
    } else {
      stop_position = current_decision.stop_point;
    }
  }
  return stop_position;
}

/// @brief calculate the slowdown for the given decision history
/// @param [inout] history decision history
/// @param [in] trajectory ego trajectory starting from the current ego pose
/// @param [in] planner_data planner data with deceleration limits
/// @param [in] params module parameters
/// @return slowdown interval (from point, to point, velocity)
inline std::optional<SlowdownInterval> calculate_slowdown_interval(
  DecisionHistory & history,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const PlannerData & planner_data, const Parameters & params)
{
  std::optional<SlowdownInterval> interval;
  auto & current_decision = history.decisions.back();
  if (current_decision.type == slowdown) {
    const auto t_collision = current_decision.collision->ego_collision_time;
    const auto p_collision = interpolated_point_at_time(trajectory, t_collision);
    const auto min_slow_arc_length = planner_data.current_odometry.twist.twist.linear.x * 0.1;
    auto from_arc_length = std::max(
      min_slow_arc_length, motion_utils::calcSignedArcLength(trajectory, 0, p_collision) -
                             params.preventive_slowdown_distance_buffer);
    const auto p_slowdown =
      motion_utils::calcInterpolatedPose(trajectory, from_arc_length).position;
    // TODO(Maxime): add option to use decel limit (instead of comfortable decel)
    // safe velocity that guarantees we can stop before the collision
    const auto safe_velocity = std::sqrt(
      2.0 * -planner_data.velocity_smoother_->getMinDecel() *
      params.preventive_slowdown_distance_buffer);
    // smooth velocity we can reach by smoothly decelerating to the slowdown point
    const auto smooth_velocity = std::sqrt(
      2.0 * -planner_data.velocity_smoother_->getMinDecel() *
      planner_data.current_odometry.twist.twist.linear.x * from_arc_length);
    interval.emplace(p_slowdown, p_collision, std::max({0.0, safe_velocity, smooth_velocity}));
  }
  return interval;
}

/// @brief calculate slowdowns for the given decisions
/// @param [inout] decision_tracker decision history of all objects
/// @param [in] trajectory ego trajectory starting from current pose
/// @param [in] planner_data planner data with deceleration limits
/// @param [in] params module parameters
/// @return result with the calculated stop point and slowdown intervals
inline VelocityPlanningResult calculate_slowdowns(
  ObjectDecisionsTracker & decision_tracker,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const PlannerData & planner_data, const Parameters & params)
{
  VelocityPlanningResult result;
  for (auto & [object, history] : decision_tracker.history_per_object) {
    const auto stop_position = calculate_stop_position(history, trajectory, params);
    if (stop_position) {
      result.stop_points.push_back(*stop_position);
    }
    const auto slowdown_interval =
      calculate_slowdown_interval(history, trajectory, planner_data, params);
    if (slowdown_interval) {
      result.slowdown_intervals.push_back(*slowdown_interval);
    }
  }
  return result;
}
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // SLOWDOWN_HPP_
