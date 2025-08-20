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

#include "autoware/trajectory_optimizer/utils.hpp"

#include "autoware/trajectory/interpolator/akima_spline.hpp"
#include "autoware/trajectory/interpolator/interpolator.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/trajectory_point.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_planning_msgs/msg/detail/trajectory_point__struct.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>

namespace autoware::trajectory_optimizer::utils
{
using autoware::experimental::trajectory::interpolator::AkimaSpline;
using InterpolationTrajectory =
  autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;

rclcpp::Logger get_logger()
{
  return rclcpp::get_logger("trajectory_optimizer");
}

void smooth_trajectory_with_elastic_band(
  TrajectoryPoints & traj_points, const Odometry & current_odometry,
  const std::shared_ptr<EBPathSmoother> & eb_path_smoother_ptr)
{
  if (!eb_path_smoother_ptr) {
    RCLCPP_ERROR(get_logger(), "Elastic band path smoother is not initialized");
    return;
  }
  if (traj_points.empty()) {
    return;
  }
  traj_points = eb_path_smoother_ptr->smoothTrajectory(traj_points, current_odometry.pose.pose);
  eb_path_smoother_ptr->resetPreviousData();
}

void remove_invalid_points(TrajectoryPoints & input_trajectory)
{
  // remove points with nan or inf values
  input_trajectory.erase(
    std::remove_if(
      input_trajectory.begin(), input_trajectory.end(),
      [](const TrajectoryPoint & point) { return !validate_point(point); }),
    input_trajectory.end());

  utils::remove_close_proximity_points(input_trajectory, 1E-2);

  if (input_trajectory.size() < 2) {
    RCLCPP_ERROR(
      get_logger(),
      "No enough points in trajectory after removing close proximity points and invalid points");
    return;
  }
  const bool is_driving_forward = true;
  autoware::motion_utils::insertOrientation(input_trajectory, is_driving_forward);
  autoware::motion_utils::removeFirstInvalidOrientationPoints(input_trajectory);
  size_t previous_size{input_trajectory.size()};
  do {
    previous_size = input_trajectory.size();
    // Set the azimuth orientation to the next point at each point
    autoware::motion_utils::insertOrientation(input_trajectory, is_driving_forward);
    // Use azimuth orientation to remove points in reverse order
    autoware::motion_utils::removeFirstInvalidOrientationPoints(input_trajectory);
  } while (previous_size != input_trajectory.size());
}

void remove_close_proximity_points(TrajectoryPoints & input_trajectory_array, const double min_dist)
{
  if (std::size(input_trajectory_array) < 2) {
    return;
  }

  input_trajectory_array.erase(
    std::remove_if(
      std::next(input_trajectory_array.begin()),  // Start from second element
      input_trajectory_array.end(),
      [&](const TrajectoryPoint & point) {
        const auto prev_it = std::prev(&point);
        const auto dist = autoware_utils::calc_distance2d(point, *prev_it);
        return dist < min_dist;
      }),
    input_trajectory_array.end());
}

void clamp_velocities(
  TrajectoryPoints & input_trajectory_array, float min_velocity, float min_acceleration)
{
  std::for_each(
    input_trajectory_array.begin(), input_trajectory_array.end(),
    [min_velocity, min_acceleration](TrajectoryPoint & point) {
      point.longitudinal_velocity_mps = std::max(point.longitudinal_velocity_mps, min_velocity);
      point.acceleration_mps2 = std::max(point.acceleration_mps2, min_acceleration);
    });
}

void set_max_velocity(TrajectoryPoints & input_trajectory_array, const float max_velocity)
{
  std::for_each(
    input_trajectory_array.begin(), input_trajectory_array.end(),
    [max_velocity](TrajectoryPoint & point) {
      point.longitudinal_velocity_mps = std::min(point.longitudinal_velocity_mps, max_velocity);
    });
}

void limit_lateral_acceleration(
  TrajectoryPoints & input_trajectory_array, const TrajectoryOptimizerParams & params)
{
  if (input_trajectory_array.empty()) {
    return;
  }

  auto get_delta_time = [](const auto & next, const auto & current) -> double {
    return next->time_from_start.sec + next->time_from_start.nanosec * 1e-9 -
           (current->time_from_start.sec + current->time_from_start.nanosec * 1e-9);
  };

  const auto & current_position = params.current_odometry.pose.pose.position;
  motion_utils::calculate_time_from_start(input_trajectory_array, current_position);

  const auto closest_index =
    motion_utils::findNearestIndex(input_trajectory_array, current_position);
  const auto start_itr = std::next(
    input_trajectory_array.begin(),
    static_cast<std::vector<TrajectoryPoint>::difference_type>(closest_index));

  for (auto itr = start_itr; itr < std::prev(input_trajectory_array.end()); ++itr) {
    const auto current_pose = itr->pose;
    const auto next_pose = std::next(itr)->pose;
    const auto delta_time = get_delta_time(std::next(itr), itr);

    tf2::Quaternion q_current;
    tf2::Quaternion q_next;
    tf2::convert(current_pose.orientation, q_current);
    tf2::convert(next_pose.orientation, q_next);
    double delta_theta = q_current.angleShortestPath(q_next);
    // Handle wrap-around
    if (delta_theta > M_PI) {
      delta_theta -= 2.0 * M_PI;
    } else if (delta_theta < -M_PI) {
      delta_theta += 2.0 * M_PI;
    }

    const double yaw_rate = std::max(std::abs(delta_theta / delta_time), 1.0E-5);
    const double current_speed = std::abs(itr->longitudinal_velocity_mps);
    // Compute lateral acceleration
    const double lateral_acceleration = std::abs(current_speed * yaw_rate);
    if (lateral_acceleration < params.max_lateral_accel_mps2) continue;

    itr->longitudinal_velocity_mps = params.max_lateral_accel_mps2 / yaw_rate;
  }

  motion_utils::calculate_time_from_start(
    input_trajectory_array, params.current_odometry.pose.pose.position);
}

void filter_velocity(
  TrajectoryPoints & input_trajectory, const InitialMotion & initial_motion,
  const TrajectoryOptimizerParams & params, const std::shared_ptr<JerkFilteredSmoother> & smoother,
  const Odometry & current_odometry)
{
  if (!smoother) {
    RCLCPP_ERROR(get_logger(), "JerkFilteredSmoother is not initialized");
    return;
  }

  if (input_trajectory.size() < 2) {
    return;
  }
  // Lateral acceleration limit
  const auto & nearest_dist_threshold = params.nearest_dist_threshold_m;
  const auto & nearest_yaw_threshold = params.nearest_yaw_threshold_rad;
  const auto & initial_motion_speed = initial_motion.speed_mps;
  const auto & initial_motion_acc = initial_motion.acc_mps2;

  constexpr bool enable_smooth_limit = true;
  constexpr bool use_resampling = true;

  input_trajectory = smoother->applyLateralAccelerationFilter(
    input_trajectory, initial_motion_speed, initial_motion_acc, enable_smooth_limit,
    use_resampling);

  // Steering angle rate limit (Note: set use_resample = false since it is resampled above)
  input_trajectory = smoother->applySteeringRateLimit(input_trajectory, false);
  // Resample trajectory with ego-velocity based interval distance

  input_trajectory = smoother->resampleTrajectory(
    input_trajectory, initial_motion_speed, current_odometry.pose.pose, nearest_dist_threshold,
    nearest_yaw_threshold);

  const size_t traj_closest = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    input_trajectory, current_odometry.pose.pose, nearest_dist_threshold, nearest_yaw_threshold);

  // // Clip trajectory from closest point
  TrajectoryPoints clipped;
  clipped.insert(
    clipped.end(),
    input_trajectory.begin() + static_cast<TrajectoryPoints::difference_type>(traj_closest),
    input_trajectory.end());
  input_trajectory = clipped;

  std::vector<TrajectoryPoints> debug_trajectories;
  if (!smoother->apply(
        initial_motion_speed, initial_motion_acc, input_trajectory, input_trajectory,
        debug_trajectories, false)) {
    RCLCPP_WARN(get_logger(), "Fail to solve optimization.");
  }
}

bool validate_point(const TrajectoryPoint & point)
{
  return std::isfinite(point.longitudinal_velocity_mps) && std::isfinite(point.acceleration_mps2) &&
         std::isfinite(point.pose.position.x) && std::isfinite(point.pose.position.y) &&
         std::isfinite(point.pose.position.z) && std::isfinite(point.pose.orientation.x) &&
         std::isfinite(point.pose.orientation.y) && std::isfinite(point.pose.orientation.z) &&
         std::isfinite(point.pose.orientation.w) && !std::isnan(point.pose.position.x) &&
         !std::isnan(point.pose.position.y) && !std::isnan(point.pose.position.z) &&
         !std::isnan(point.pose.orientation.x) && !std::isnan(point.pose.orientation.y) &&
         !std::isnan(point.pose.orientation.z) && !std::isnan(point.pose.orientation.w) &&
         !std::isnan(point.longitudinal_velocity_mps) && !std::isnan(point.acceleration_mps2);
}

void apply_spline(TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params)
{
  if (traj_points.size() < 5) {
    RCLCPP_ERROR(get_logger(), "Not enough points in trajectory for akima spline interpolation");
    return;
  }
  auto trajectory_interpolation_util =
    InterpolationTrajectory::Builder{}
      .set_xy_interpolator<AkimaSpline>()  // Set interpolator for x-y plane
      .build(traj_points);
  if (!trajectory_interpolation_util) {
    RCLCPP_WARN(get_logger(), "Failed to build interpolation trajectory");
    return;
  }
  trajectory_interpolation_util->align_orientation_with_trajectory_direction();
  TrajectoryPoints output_points{traj_points.front()};
  constexpr double epsilon{1e-2};
  const auto ds = std::max(params.spline_interpolation_resolution_m, epsilon);
  output_points.reserve(static_cast<size_t>(trajectory_interpolation_util->length() / ds));

  for (auto s = ds; s <= trajectory_interpolation_util->length(); s += ds) {
    auto p = trajectory_interpolation_util->compute(s);
    if (!validate_point(p)) {
      continue;
    }
    output_points.push_back(p);
  }

  if (output_points.size() < 2) {
    RCLCPP_WARN(get_logger(), "Not enough points in trajectory after akima spline interpolation");
    return;
  }
  auto last_interpolated_point = output_points.back();
  auto & original_trajectory_last_point = traj_points.back();

  if (!validate_point(original_trajectory_last_point)) {
    RCLCPP_WARN(get_logger(), "Last point in original trajectory is invalid. Removing last point");
    traj_points = output_points;
    return;
  }

  auto d = autoware_utils::calc_distance2d(
    last_interpolated_point.pose.position, original_trajectory_last_point.pose.position);
  if (d > epsilon) {
    output_points.push_back(original_trajectory_last_point);
  };
  traj_points = output_points;
}

void interpolate_trajectory(
  TrajectoryPoints & traj_points, const Odometry & current_odometry,
  const AccelWithCovarianceStamped & current_acceleration, const TrajectoryOptimizerParams & params,
  const std::shared_ptr<JerkFilteredSmoother> & jerk_filtered_smoother,
  const std::shared_ptr<EBPathSmoother> & eb_path_smoother_ptr)
{
  // Remove overlap points and wrong orientation points
  if (params.fix_invalid_points) {
    remove_invalid_points(traj_points);
  }

  if (traj_points.size() < 2) {
    RCLCPP_ERROR(get_logger(), "No enough points in trajectory after overlap points removal");
    return;
  }

  const double & target_pull_out_speed_mps = params.target_pull_out_speed_mps;
  const double & target_pull_out_acc_mps2 = params.target_pull_out_acc_mps2;
  const double & max_speed_mps = params.max_speed_mps;

  const auto current_speed = current_odometry.twist.twist.linear.x;
  const auto current_linear_acceleration = current_acceleration.accel.accel.linear.x;
  auto initial_motion_speed =
    (current_speed > target_pull_out_speed_mps) ? current_speed : target_pull_out_speed_mps;
  auto initial_motion_acc = (current_speed > target_pull_out_speed_mps)
                              ? current_linear_acceleration
                              : target_pull_out_acc_mps2;

  // Set engage speed and acceleration
  if (params.set_engage_speed && (current_speed < target_pull_out_speed_mps)) {
    clamp_velocities(
      traj_points, static_cast<float>(initial_motion_speed),
      static_cast<float>(initial_motion_acc));
  }
  // Limit ego speed
  if (params.limit_speed) {
    set_max_velocity(traj_points, static_cast<float>(max_speed_mps));
  }

  // Smooth velocity profile
  if (params.smooth_velocities) {
    InitialMotion initial_motion{initial_motion_speed, initial_motion_acc};
    filter_velocity(traj_points, initial_motion, params, jerk_filtered_smoother, current_odometry);
  }
  // Apply spline to smooth the trajectory
  if (params.use_akima_spline_interpolation) {
    apply_spline(traj_points, params);
  }
  // Use elastic band to smooth the trajectory
  if (params.smooth_trajectories) {
    smooth_trajectory_with_elastic_band(traj_points, current_odometry, eb_path_smoother_ptr);
  }

  if (params.fix_invalid_points) {
    remove_invalid_points(traj_points);
  }
  // Recalculate timestamps
  motion_utils::calculate_time_from_start(traj_points, current_odometry.pose.pose.position);

  if (traj_points.size() < 2) {
    RCLCPP_ERROR(get_logger(), "Not enough points in trajectory after overlap points removal");
    return;
  }
}

void add_ego_state_to_trajectory(
  TrajectoryPoints & traj_points, const Odometry & current_odometry,
  const TrajectoryOptimizerParams & params)
{
  TrajectoryPoint ego_state;
  ego_state.pose = current_odometry.pose.pose;
  ego_state.longitudinal_velocity_mps = current_odometry.twist.twist.linear.x;

  if (traj_points.empty()) {
    traj_points.push_back(ego_state);
    return;
  }
  const auto & last_point = traj_points.back();
  const auto yaw_diff = std::abs(
    autoware_utils::normalize_degree(ego_state.pose.orientation.z - last_point.pose.orientation.z));
  const auto distance = autoware_utils::calc_distance2d(last_point, ego_state);
  constexpr double epsilon{1e-2};
  const bool is_change_small = distance < epsilon && yaw_diff < epsilon;
  if (is_change_small) {
    return;
  }

  const bool is_change_large =
    distance > params.nearest_dist_threshold_m || yaw_diff > params.nearest_yaw_threshold_rad;
  if (is_change_large) {
    traj_points = {ego_state};
    return;
  }

  traj_points.push_back(ego_state);

  size_t clip_idx = 0;
  double accumulated_length = 0.0;
  for (size_t i = traj_points.size() - 1; i > 0; i--) {
    accumulated_length += autoware_utils::calc_distance2d(traj_points.at(i - 1), traj_points.at(i));
    if (accumulated_length > params.backward_trajectory_extension_m) {
      clip_idx = i;
      break;
    }
  }
  traj_points.erase(traj_points.begin(), traj_points.begin() + static_cast<int>(clip_idx));
}

void expand_trajectory_with_ego_history(
  TrajectoryPoints & traj_points, const TrajectoryPoints & ego_history_points,
  [[maybe_unused]] const Odometry & current_odometry,
  [[maybe_unused]] const TrajectoryOptimizerParams & params)
{
  if (ego_history_points.empty()) {
    return;
  }

  if (traj_points.empty()) {
    traj_points.insert(traj_points.begin(), ego_history_points.begin(), ego_history_points.end());
    return;
  }

  const auto first_ego_history_point = ego_history_points.front();
  const auto first_ego_trajectory_point = traj_points.front();
  const auto first_ego_trajectory_point_arc_length = autoware::motion_utils::calcSignedArcLength(
    ego_history_points, first_ego_history_point.pose.position,
    first_ego_trajectory_point.pose.position);

  const auto ego_position = params.current_odometry.pose.pose.position;
  const auto distance_ego_to_first_trajectory_point =
    autoware_utils::calc_distance2d(first_ego_trajectory_point, ego_position);

  std::for_each(ego_history_points.rbegin(), ego_history_points.rend(), [&](const auto & point) {
    const auto point_arc_length = autoware::motion_utils::calcSignedArcLength(
      ego_history_points, first_ego_history_point.pose.position, point.pose.position);

    const bool is_ahead_of_first_point = point_arc_length > first_ego_trajectory_point_arc_length;
    const bool is_closer_than_first_trajectory_point =
      autoware_utils::calc_distance2d(point, ego_position) < distance_ego_to_first_trajectory_point;
    const bool is_point_already_in_trajectory =
      std::any_of(traj_points.begin(), traj_points.end(), [&](const TrajectoryPoint & traj_point) {
        return autoware_utils::calc_distance2d(traj_point, point) < 1e-1;
      });
    if (
      is_ahead_of_first_point || is_point_already_in_trajectory ||
      is_closer_than_first_trajectory_point) {
      return;
    }

    traj_points.insert(traj_points.begin(), point);
    traj_points.front().longitudinal_velocity_mps =
      first_ego_trajectory_point.longitudinal_velocity_mps;
  });
}

}  // namespace autoware::trajectory_optimizer::utils
