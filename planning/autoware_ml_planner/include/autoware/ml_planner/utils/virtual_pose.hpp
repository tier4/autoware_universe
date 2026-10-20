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

#ifndef AUTOWARE__ML_PLANNER__UTILS__VIRTUAL_POSE_HPP_
#define AUTOWARE__ML_PLANNER__UTILS__VIRTUAL_POSE_HPP_

#include "autoware/ml_planner/utils/timed_buffer.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <optional>
#include <vector>

namespace autoware::ml_planner::utils
{

struct VirtualPoseParams
{
  bool enable{false};
  // The virtual pose is reset onto the vehicle when the vehicle is farther than these from it,
  // measured along / across the virtual heading, or when the headings differ by more.
  double max_longitudinal_error_m{5.0};
  double max_lateral_error_m{3.0};
  double max_yaw_error_deg{20.0};
  // At standstill (slower than hold_max_speed_mps) the virtual pose is held until the vehicle has
  // moved or turned beyond the tolerances, which cover the localization drift.
  double hold_max_speed_mps{0.1};
  double hold_position_tolerance_m{0.2};
  double hold_yaw_tolerance_deg{0.5};
};

/**
 * @brief Planning start taken from the previously published trajectory instead of the vehicle.
 *
 * Each cycle the model is given the point of its last published trajectory closest to the vehicle
 * (the virtual pose), so it plans as if that trajectory had been followed exactly and the
 * controller alone corrects the vehicle's deviation from it. The model's ego past is made of the
 * virtual poses of earlier cycles, matching the trajectories it planned from.
 *
 * The virtual pose falls back onto the vehicle when the vehicle is beyond the reset limits from
 * it, or when there is no trajectory to take it from. Such a jump restarts the past from the
 * measured poses and is reported by take_restart(), so the trajectory optimizer can forget its
 * previous solutions.
 */
class VirtualPose
{
public:
  using Odometry = nav_msgs::msg::Odometry;
  using Trajectory = autoware_planning_msgs::msg::Trajectory;

  explicit VirtualPose(double history_window_s);

  /**
   * @brief Compute the planning start of the current frame.
   *
   * @param measured_history Measured ego odometry, oldest first (not empty).
   * @param params Reset limits and standstill hold.
   */
  void update(const TimedBuffer<Odometry> & measured_history, const VirtualPoseParams & params);

  /**
   * @brief Ego past for the model, oldest first. The last entry is the planning start: the
   * measured odometry with the virtual pose.
   */
  const std::deque<Odometry> & history() const { return history_.msgs(); }

  /**
   * @brief Remember the trajectory published from the current planning start.
   */
  void set_published_trajectory(const Trajectory & trajectory);

  /**
   * @brief Start over from the vehicle (feature switched, Autoware took or gave up control, new
   * route).
   */
  void reset();

  /**
   * @brief Whether the planning start jumped onto the vehicle since the last call.
   */
  bool take_restart();

private:
  // Poses the virtual pose is searched on: earlier planning starts (prefix_count of them, oldest
  // first), then the planning start and the poses of the published trajectory.
  struct Reference
  {
    std::vector<geometry_msgs::msg::Pose> poses;
    size_t prefix_count;
  };

  void restart_from(const TimedBuffer<Odometry> & measured_history);
  Reference build_reference() const;

  TimedBuffer<Odometry> history_;
  // Planning start and poses of the last published trajectory.
  std::optional<geometry_msgs::msg::Pose> published_start_;
  std::vector<geometry_msgs::msg::Pose> published_poses_;
  // Measured pose at which the current standstill hold began.
  std::optional<geometry_msgs::msg::Pose> hold_anchor_;
  bool restart_{false};
};

}  // namespace autoware::ml_planner::utils

#endif  // AUTOWARE__ML_PLANNER__UTILS__VIRTUAL_POSE_HPP_
