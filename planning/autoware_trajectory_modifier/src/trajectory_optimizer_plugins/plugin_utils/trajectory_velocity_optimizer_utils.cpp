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

#include "autoware/trajectory_modifier/trajectory_optimizer_plugins/plugin_utils/trajectory_velocity_optimizer_utils.hpp"

#include "autoware/trajectory_modifier/trajectory_modifier_utils/set_max_velocity.hpp"
#include "autoware/trajectory_modifier/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/logging.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::trajectory_modifier::plugin::trajectory_velocity_optimizer_utils
{

void clamp_velocities(
  TrajectoryPoints & input_trajectory_array, const float min_velocity, const float min_acceleration)
{
  std::for_each(
    input_trajectory_array.begin(), input_trajectory_array.end(),
    [min_velocity, min_acceleration](TrajectoryPoint & point) {
      point.longitudinal_velocity_mps = std::max(point.longitudinal_velocity_mps, min_velocity);
      point.acceleration_mps2 = std::max(point.acceleration_mps2, min_acceleration);
    });
}

bool is_launch_trajectory(
  const TrajectoryPoints & input_trajectory_array, const float engage_velocity)
{
  //! [m] A trajectory shorter than this is not a launch. It keeps the deceleration in front of the
  //! goal and a stop plan (all points at the same place) from engaging
  constexpr double min_engage_dist_m = 0.5;

  if (input_trajectory_array.empty()) {
    return false;
  }

  // The travelled distance is measured on the points themselves. After the time parameterization
  // the points of a stopped section sit on top of each other, so a stop plan lands near 0 here
  double length = 0.0;
  for (size_t k = 0; k + 1 < input_trajectory_array.size(); ++k) {
    length += autoware_utils_geometry::calc_distance2d(
      input_trajectory_array[k].pose.position, input_trajectory_array[k + 1].pose.position);
  }
  if (length <= min_engage_dist_m) {
    return false;
  }

  // A trajectory that never reaches the engage speed is a stop in progress, not a launch
  return std::any_of(
    input_trajectory_array.begin(), input_trajectory_array.end(),
    [engage_velocity](const TrajectoryPoint & point) {
      return point.longitudinal_velocity_mps >= engage_velocity;
    });
}

void set_max_velocity(TrajectoryPoints & input_trajectory_array, const float max_velocity)
{
  // Shared with ExternalVelocityLimit. Lateral-accel limiting and jerk smoothing do not own this
  // clamp; it runs on its own whenever a hard speed cap is applied in place.
  autoware::trajectory_modifier::utils::set_max_velocity(input_trajectory_array, max_velocity);
}

void limit_lateral_acceleration(
  TrajectoryPoints & input_trajectory_array, std::vector<double> & max_velocity_per_point,
  const double max_lateral_accel_mps2, const double min_limited_speed_mps,
  const Odometry & current_odometry, const bool inplace)
{
  if (input_trajectory_array.empty()) {
    return;
  }

  const size_t traj_size = input_trajectory_array.size();

  auto get_delta_time = [](const auto & next, const auto & current) -> double {
    return next->time_from_start.sec + next->time_from_start.nanosec * 1e-9 -
           (current->time_from_start.sec + current->time_from_start.nanosec * 1e-9);
  };

  const auto & current_position = current_odometry.pose.pose.position;
  const auto start_index = motion_utils::findNearestIndex(input_trajectory_array, current_position);
  const size_t end_index = traj_size - 1;

  // Ensure we have a valid range
  if (start_index >= traj_size || start_index > end_index) {
    return;
  }

  const auto start_itr = std::next(
    input_trajectory_array.begin(),
    static_cast<std::vector<TrajectoryPoint>::difference_type>(start_index));

  size_t current_index = start_index;
  for (auto itr = start_itr; itr < std::prev(input_trajectory_array.end());
       ++itr, ++current_index) {
    const double current_speed = std::abs(itr->longitudinal_velocity_mps);
    if (current_speed < min_limited_speed_mps) {
      // No modification needed for low speeds
      continue;
    }

    const auto current_pose = itr->pose;
    const auto next_pose = std::next(itr)->pose;
    const auto delta_time = get_delta_time(std::next(itr), itr);
    constexpr double epsilon_dt = 1.0e-3;
    if (delta_time <= epsilon_dt) {
      continue;
    }

    tf2::Quaternion q_current;
    tf2::Quaternion q_next;

    const double yaw_current = autoware_utils_geometry::get_rpy(current_pose).z;
    const double yaw_next = autoware_utils_geometry::get_rpy(next_pose).z;
    const double delta_yaw = autoware_utils_geometry::normalize_radian(yaw_next - yaw_current);
    constexpr double epsilon_yaw_rate = 1.0e-5;
    const double yaw_rate = std::max(std::abs(delta_yaw / delta_time), epsilon_yaw_rate);
    const double lateral_acceleration = current_speed * yaw_rate;

    if (lateral_acceleration < max_lateral_accel_mps2) {
      // No modification needed
      continue;
    }

    // Compute velocity limit based on lateral acceleration constraint
    const double limited_velocity =
      std::max(max_lateral_accel_mps2 / yaw_rate, min_limited_speed_mps);

    // Update max velocity constraint for this point
    max_velocity_per_point.at(current_index) =
      std::min(max_velocity_per_point.at(current_index), limited_velocity);

    // Inplace trajectory update
    if (inplace) {
      itr->longitudinal_velocity_mps = static_cast<float>(max_velocity_per_point.at(current_index));
    }
  }
}

void filter_velocity(
  TrajectoryPoints & input_trajectory, const double nearest_dist_threshold_m,
  const double nearest_yaw_threshold_rad, const std::shared_ptr<ContinuousJerkSmoother> & smoother,
  const Odometry & current_odometry, const std::vector<double> & max_velocity_per_point)
{
  if (!smoother) {
    auto clock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("trajectory_velocity_optimizer"), *clock, 5000,
      "ContinuousJerkSmoother is not initialized");
    return;
  }

  if (input_trajectory.size() < 2) {
    return;
  }

  const size_t traj_closest = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    input_trajectory, current_odometry.pose.pose, nearest_dist_threshold_m,
    nearest_yaw_threshold_rad);

  // Clip trajectory from closest point
  TrajectoryPoints clipped;
  clipped.insert(
    clipped.end(),
    input_trajectory.begin() + static_cast<TrajectoryPoints::difference_type>(traj_closest),
    input_trajectory.end());
  input_trajectory = clipped;

  // Also clip the max velocity array if provided
  std::vector<double> clipped_max_vel;
  if (!max_velocity_per_point.empty() && max_velocity_per_point.size() > traj_closest) {
    clipped_max_vel.insert(
      clipped_max_vel.end(),
      max_velocity_per_point.begin() +
        static_cast<std::vector<double>::difference_type>(traj_closest),
      max_velocity_per_point.end());
  }

  // Apply continuous jerk smoother with per-point velocity constraints
  if (!smoother->apply(input_trajectory, input_trajectory, clipped_max_vel)) {
    auto clock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("trajectory_velocity_optimizer"), *clock, 5000,
      "Continuous jerk smoother optimization failed.");
  }
}

}  // namespace autoware::trajectory_modifier::plugin::trajectory_velocity_optimizer_utils
