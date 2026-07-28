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

#ifndef GOAL_PLANNER_HPP_
#define GOAL_PLANNER_HPP_

#include "type_alias.hpp"

#include <rclcpp/logger.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

struct GoalPlannerParams
{
  bool enable{false};
  double activation_lateral_offset{0.5};  // [m] goal offset from the path to trigger pull over
  double minimum_lateral_accel{0.2};      // [m/s^2] gentlest (longest) shift candidate
  double maximum_lateral_accel{1.0};      // [m/s^2] sharpest (shortest) shift candidate
  int lateral_accel_sampling_num{4};      // number of sampled shift candidates
  std::vector<double> collision_check_margins{2.0, 1.0, 0.5, 0.1};  // [m] tried large to small
  double object_velocity_threshold{1.0};  // [m/s] objects slower than this are static
  double object_search_radius{50.0};      // [m] objects farther from the goal are ignored
  double turn_signal_distance{30.0};      // [m] signal on when ego is this close to shift start
  // mirrors of existing params, used to compute the candidate shift length
  double minimum_shift_distance{5.0};  // [m] (path_shift.minimum_shift_distance)
  double expected_parking_speed{2.8};  // [m/s] (early_stop.expected_ego_speed_parking)
};

struct GoalPlannerResult
{
  enum class Status {
    NOT_APPLICABLE,  //!< goal is on the path (or not on this trajectory); early stop fallback
    PLANNED,         //!< a collision-free pull over candidate was adopted
    BLOCKED,         //!< pull over is required but every candidate collides; early stop fallback
  };

  Status status{Status::NOT_APPLICABLE};
  std::optional<Trajectory> trajectory{};  //!< set only when status == PLANNED
  //! ego has reached the pull over approach window (the same window the turn signal uses): the
  //! shift section it is about to drive is the morphed one, so the caller must stop re-connecting
  //! the trajectory to ego from here on
  bool in_pull_over_approach{false};
  uint8_t turn_indicators_command{autoware_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND};
  double selected_lateral_accel{0.0};  //!< set only when status == PLANNED
  double selected_margin{0.0};         //!< set only when status == PLANNED
};

/**
 * @brief Plans the arrival (pull over) maneuver: morphs the trajectory tail onto the laterally
 * offset goal pose with a quintic shift, sampling the shift length over lateral acceleration and
 * rejecting candidates whose shift section collides with static objects (first-fit; collision
 * margins are relaxed stepwise). The input trajectory must reach the goal projection (the early
 * stop crop is deferred by the caller while this planner is enabled).
 */
class GoalPlanner
{
public:
  GoalPlanner(const rclcpp::Logger & logger, const VehicleInfo & vehicle_info);

  GoalPlannerResult plan(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Pose & ego_pose, const PredictedObjects::ConstSharedPtr & objects,
    const GoalPlannerParams & params) const;

private:
  rclcpp::Logger logger_;
  mutable rclcpp::Clock clock_{RCL_ROS_TIME};  // mutable: throttled warn log in const plan()
  VehicleInfo vehicle_info_;
};

namespace goal_planner_utils
{

/**
 * @brief Morph the trajectory tail so that it leaves the reference line at (s_goal - shift_length)
 * and ends exactly at the goal pose, using a quintic (smoothstep) lateral profile. Points beyond
 * the goal projection are dropped.
 * @return nullopt when the goal does not project onto the trajectory interior
 */
std::optional<Trajectory> make_pull_over_candidate(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & goal_pose, double shift_length);

//! Cut the given length off the trajectory end (early stop fallback).
Trajectory crop_trajectory_end(const Trajectory & trajectory, double crop_length);

/**
 * @brief Re-pin the trajectory end onto end_pose: drop trailing points closer than
 * min_final_segment_length to it (a degenerate final segment makes the recomputed end yaw
 * unstable) and append a point at exactly end_pose. Used both when building the pull over
 * candidate and after smoothing stages that recompute orientations from point geometry.
 */
void pin_trajectory_end(
  Trajectory & trajectory, const geometry_msgs::msg::Pose & end_pose,
  double min_final_segment_length);

}  // namespace goal_planner_utils
}  // namespace autoware::minimum_rule_based_planner

#endif  // GOAL_PLANNER_HPP_
