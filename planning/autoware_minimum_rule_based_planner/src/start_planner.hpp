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

#ifndef START_PLANNER_HPP_
#define START_PLANNER_HPP_

#include "type_alias.hpp"

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <rclcpp/logger.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

struct StartPlannerParams
{
  bool enable{false};
  double ego_stopped_velocity{0.5};       // [m/s] pull out is considered only below this speed
  double activation_lateral_offset{0.5};  // [m] offset from reference path to trigger pull out
  double finish_lateral_offset{0.2};      // [m] offset below which the pull out is complete
  double minimum_lateral_accel{0.2};      // [m/s^2] gentlest (longest) shift candidate
  double maximum_lateral_accel{1.0};      // [m/s^2] sharpest (shortest) shift candidate
  int lateral_accel_sampling_num{4};      // number of sampled shift candidates
  std::vector<double> collision_check_margins{2.0, 1.0, 0.5, 0.1};  // [m] tried large to small
  double collision_check_extra_length{5.0};  // [m] checked distance beyond the shift section
  double object_velocity_threshold{1.0};     // [m/s] objects slower than this are static
  double object_search_radius{30.0};         // [m] objects farther from ego are ignored
  // mirrors of path_shift params, used to reproduce the candidate shift length
  double minimum_shift_distance{5.0};    // [m]
  double min_speed_for_curvature{2.77};  // [m/s]
};

struct StartPlannerResult
{
  enum class Status {
    NOT_APPLICABLE,  //!< not in a pull out situation; the normal shift applies
    PLANNED,         //!< a collision-free pull out candidate was adopted
    BLOCKED,         //!< pull out is required but every candidate collides; keep stopped
  };

  Status status{Status::NOT_APPLICABLE};
  std::optional<Trajectory> trajectory{};  //!< set only when status == PLANNED
  uint8_t turn_indicators_command{autoware_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND};
  double selected_lateral_accel{0.0};  //!< set only when status == PLANNED
  double selected_margin{0.0};         //!< set only when status == PLANNED
};

//! Generates a shift candidate for the given lateral acceleration limit
//! (wired to PathPlanner::shift_trajectory_to_ego by the node, faked in tests).
using ShiftCandidateGenerator = std::function<std::optional<Trajectory>(double lateral_accel)>;

/**
 * @brief Plans the departure (pull out) maneuver: samples shift candidates over lateral
 * acceleration, rejects candidates whose shift section collides with static objects, and adopts
 * the first passing one (first-fit). Collision margins are relaxed stepwise only when no candidate
 * passes with a larger margin.
 */
class StartPlanner
{
public:
  StartPlanner(const rclcpp::Logger & logger, const VehicleInfo & vehicle_info);

  /**
   * @param reference_trajectory trajectory on the lane centerline (before shifting to ego)
   * @param objects may be nullptr; collision check is skipped then
   */
  StartPlannerResult plan(
    const Trajectory & reference_trajectory, const geometry_msgs::msg::Pose & ego_pose,
    double ego_velocity, const PredictedObjects::ConstSharedPtr & objects,
    const ShiftCandidateGenerator & generate_candidate, const StartPlannerParams & params);

  bool is_pull_out_active() const { return pull_out_active_; }

private:
  rclcpp::Logger logger_;
  rclcpp::Clock clock_{RCL_ROS_TIME};
  VehicleInfo vehicle_info_;
  //! latched from activation (stopped & offset) until the offset converges below
  //! finish_lateral_offset, so the turn signal and collision gating persist while moving
  bool pull_out_active_{false};
};

namespace start_planner_utils
{

//! Extract footprint polygons of static objects near ego.
std::vector<autoware_utils::Polygon2d> get_static_object_polygons(
  const PredictedObjects & objects, const geometry_msgs::msg::Point & ego_position,
  double velocity_threshold, double search_radius);

//! Check whether the vehicle footprint swept along the first check_length [m] of the
//! trajectory comes closer than margin [m] to any of the object polygons.
bool has_collision(
  const TrajectoryPoints & points, double check_length, const VehicleInfo & vehicle_info,
  const std::vector<autoware_utils::Polygon2d> & object_polygons, double margin);

//! Range variant: check only the arc-length interval [check_start_length, check_end_length]
//! of the trajectory (also used by the goal planner for the pull over section).
bool has_collision(
  const TrajectoryPoints & points, double check_start_length, double check_end_length,
  const VehicleInfo & vehicle_info, const std::vector<autoware_utils::Polygon2d> & object_polygons,
  double margin);

}  // namespace start_planner_utils
}  // namespace autoware::minimum_rule_based_planner

#endif  // START_PLANNER_HPP_
