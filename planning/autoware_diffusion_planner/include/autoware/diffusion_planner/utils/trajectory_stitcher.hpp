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

#ifndef AUTOWARE__DIFFUSION_PLANNER__UTILS__TRAJECTORY_STITCHER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__UTILS__TRAJECTORY_STITCHER_HPP_

#include <rclcpp/time.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <deque>
#include <optional>
#include <string>

namespace autoware::diffusion_planner
{

struct TrajectoryStitcherParams
{
  bool enable{false};
  std::string history_mode{"real"};   // "real" | "on_plan"
  double reference_blend_gain{0.3};   // per-cycle blend of the new plan into the reference path
  double path_correction_gain{0.25};  // per-cycle pull toward the path projection (1 = raw)
  double time_offset_s{0.0};          // arc lead along the previous path at current speed [s]
  double lateral_deviation_threshold_m{0.3};
  double longitudinal_deviation_threshold_m{2.0};
  double lateral_rearm_threshold_m{0.1};
  double rearm_cooldown_s{1.0};
  double max_trajectory_age_s{0.35};
};

struct StitchingStatus
{
  geometry_msgs::msg::Pose planning_origin;
  bool stitched{false};
  std::string reset_reason;
  double lateral_deviation_m{0.0};
  double longitudinal_deviation_m{0.0};
};

/**
 * @class TrajectoryStitcher
 * @brief Decides the planning origin for each cycle: a virtual pose time-matched on the
 * previously generated trajectory (stitch), or the measured ego pose (reset).
 */
class TrajectoryStitcher
{
public:
  explicit TrajectoryStitcher(const TrajectoryStitcherParams & params);

  void update_params(const TrajectoryStitcherParams & params);

  StitchingStatus compute_planning_origin(
    const rclcpp::Time & frame_time, const nav_msgs::msg::Odometry & ego_odometry,
    const unique_identifier_msgs::msg::UUID & route_uuid, bool mppi_overwrite_active);

  void set_previous_trajectory(
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    const unique_identifier_msgs::msg::UUID & route_uuid);

  void push_planning_origin_history(
    const rclcpp::Time & frame_time, const geometry_msgs::msg::Pose & origin_pose, size_t max_size);

  const std::deque<nav_msgs::msg::Odometry> & planning_origin_history() const
  {
    return planning_origin_history_;
  }

  const std::optional<autoware_planning_msgs::msg::Trajectory> & previous_trajectory() const
  {
    return prev_trajectory_;
  }

  void reset();

private:
  void clear_previous_trajectory();

  TrajectoryStitcherParams params_;
  std::optional<autoware_planning_msgs::msg::Trajectory> prev_trajectory_;
  unique_identifier_msgs::msg::UUID prev_route_uuid_;
  std::deque<nav_msgs::msg::Odometry> planning_origin_history_;
  std::optional<rclcpp::Time> rearm_allowed_time_;
  std::optional<geometry_msgs::msg::Pose> filtered_origin_;
  std::optional<geometry_msgs::msg::Pose> last_ego_pose_;
  bool active_{false};
};

}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__UTILS__TRAJECTORY_STITCHER_HPP_
