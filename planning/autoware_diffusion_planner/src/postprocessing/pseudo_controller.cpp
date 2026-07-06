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

#include "autoware/diffusion_planner/postprocessing/pseudo_controller.hpp"

#include <autoware_utils/math/normalization.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <vector>

namespace autoware::diffusion_planner::postprocess
{

namespace
{
double yaw_from_pose(const Eigen::Matrix4d & pose)
{
  return std::atan2(pose(1, 0), pose(0, 0));
}

Eigen::Matrix4d make_pose(const double x, const double y, const double z, const double yaw)
{
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose(0, 0) = std::cos(yaw);
  pose(0, 1) = -std::sin(yaw);
  pose(1, 0) = std::sin(yaw);
  pose(1, 1) = std::cos(yaw);
  pose(0, 3) = x;
  pose(1, 3) = y;
  pose(2, 3) = z;
  return pose;
}

Eigen::Matrix4d interpolate_pose(
  const Eigen::Matrix4d & from, const Eigen::Matrix4d & to, const double ratio)
{
  Eigen::Matrix4d interpolated = Eigen::Matrix4d::Identity();
  const Eigen::Vector3d from_translation = from.block<3, 1>(0, 3);
  const Eigen::Vector3d to_translation = to.block<3, 1>(0, 3);
  interpolated.block<3, 1>(0, 3) = from_translation + ratio * (to_translation - from_translation);

  const Eigen::Quaterniond from_quaternion(from.block<3, 3>(0, 0));
  const Eigen::Quaterniond to_quaternion(to.block<3, 3>(0, 0));
  interpolated.block<3, 3>(0, 0) = from_quaternion.slerp(ratio, to_quaternion).toRotationMatrix();
  return interpolated;
}

std::vector<double> integrate_travel_distances(const std::vector<float> & velocities)
{
  constexpr double dt = 0.1;
  std::vector<double> travel_distances(velocities.size());
  double distance = 0.0;
  for (size_t i = 0; i < velocities.size(); ++i) {
    distance += static_cast<double>(velocities[i]) * dt;
    travel_distances[i] = distance;
  }
  return travel_distances;
}

std::vector<double> calculate_path_distances(const std::vector<Eigen::Matrix4d> & poses)
{
  std::vector<double> path_distances(poses.size());
  if (poses.empty()) {
    return path_distances;
  }

  double prev_x = poses.front()(0, 3);
  double prev_y = poses.front()(1, 3);
  double prev_z = poses.front()(2, 3);
  double distance = 0.0;
  for (size_t i = 0; i < poses.size(); ++i) {
    distance +=
      std::hypot(poses[i](0, 3) - prev_x, poses[i](1, 3) - prev_y, poses[i](2, 3) - prev_z);
    path_distances[i] = distance;
    prev_x = poses[i](0, 3);
    prev_y = poses[i](1, 3);
    prev_z = poses[i](2, 3);
  }
  return path_distances;
}

/**
 * @brief Resamples the spatial path at the given cumulative travel distances (from integrating the
 * velocity plan), so each returned pose is the point the vehicle would reach at that time step if
 * it followed the path exactly at the planned speed.
 */
std::vector<Eigen::Matrix4d> sample_poses_by_travel_distance(
  const std::vector<Eigen::Matrix4d> & poses, const std::vector<double> & travel_distances)
{
  constexpr double max_path_distance = 80.0;
  const std::vector<double> path_distances = calculate_path_distances(poses);
  const double sample_limit = std::min(max_path_distance, path_distances.back());
  std::vector<Eigen::Matrix4d> sampled_poses;
  sampled_poses.reserve(travel_distances.size());

  for (const double travel_distance : travel_distances) {
    if (travel_distance > sample_limit) {
      sampled_poses.push_back(poses.back());
      continue;
    }

    const auto upper =
      std::lower_bound(path_distances.begin(), path_distances.end(), travel_distance);
    const size_t upper_index = static_cast<size_t>(std::distance(path_distances.begin(), upper));
    const double lower_distance = upper_index == 0 ? 0.0 : path_distances[upper_index - 1];
    const double upper_distance = path_distances[upper_index];
    const double ratio = upper_distance > lower_distance
                           ? (travel_distance - lower_distance) / (upper_distance - lower_distance)
                           : 0.0;
    const Eigen::Matrix4d & lower_pose = upper_index == 0 ? poses.front() : poses[upper_index - 1];
    sampled_poses.push_back(interpolate_pose(lower_pose, poses[upper_index], ratio));
  }

  return sampled_poses;
}

/**
 * @brief Reference state at the nearest point on a path.
 */
struct NearestReference
{
  double x;
  double y;
  double z;
  double yaw;
  double curvature;  // signed path curvature at the sample [1/m]
};

/**
 * @brief Finds the nearest point on `poses` to (query_x, query_y), interpolating within the
 * closest segment, and returns the interpolated reference pose plus the local path curvature.
 *
 * The query is projected onto every segment; the closest projection wins (ends clamp to the
 * first/last pose). Yaw is interpolated along the shortest angular difference and the curvature is
 * the segment's heading change over its arc length.
 */
NearestReference nearest_reference(
  const std::vector<Eigen::Matrix4d> & poses, const double query_x, const double query_y)
{
  const double front_yaw = yaw_from_pose(poses.front());
  NearestReference best{
    poses.front()(0, 3), poses.front()(1, 3), poses.front()(2, 3), front_yaw, 0.0};
  double best_squared_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    const double x0 = poses[i](0, 3);
    const double y0 = poses[i](1, 3);
    const double x1 = poses[i + 1](0, 3);
    const double y1 = poses[i + 1](1, 3);
    const double segment_dx = x1 - x0;
    const double segment_dy = y1 - y0;
    const double segment_squared_length = segment_dx * segment_dx + segment_dy * segment_dy;
    const double ratio =
      segment_squared_length > 0.0
        ? std::clamp(
            ((query_x - x0) * segment_dx + (query_y - y0) * segment_dy) / segment_squared_length,
            0.0, 1.0)
        : 0.0;

    const double proj_x = x0 + ratio * segment_dx;
    const double proj_y = y0 + ratio * segment_dy;
    const double dx = query_x - proj_x;
    const double dy = query_y - proj_y;
    const double squared_distance = dx * dx + dy * dy;
    if (squared_distance >= best_squared_distance) {
      continue;
    }
    best_squared_distance = squared_distance;

    const double yaw0 = yaw_from_pose(poses[i]);
    const double yaw_diff = autoware_utils::normalize_radian(yaw_from_pose(poses[i + 1]) - yaw0);
    const double z0 = poses[i](2, 3);
    const double z1 = poses[i + 1](2, 3);
    const double segment_length = std::sqrt(segment_squared_length);
    best.x = proj_x;
    best.y = proj_y;
    best.z = z0 + ratio * (z1 - z0);
    best.yaw = autoware_utils::normalize_radian(yaw0 + ratio * yaw_diff);
    best.curvature = segment_length > 0.0 ? yaw_diff / segment_length : 0.0;
  }

  return best;
}
}  // namespace

StanleyPseudoController::StanleyPseudoController(const StanleyControllerParams & params)
: params_(params)
{
}

std::vector<Eigen::Matrix4d> StanleyPseudoController::simulate(
  const std::vector<Eigen::Matrix4d> & path_poses, const std::vector<float> & velocities,
  const Eigen::Matrix4d & ego_pose, const double wheel_base, const double initial_steering) const
{
  constexpr double dt = 0.1;
  const double max_steer_rad = params_.max_steer_rad;
  const double max_steer_rate_rad_s = params_.max_steer_rate_rad_s;
  const double stop_velocity_threshold = params_.stop_velocity_threshold;
  const double heading_gain = params_.heading_gain;
  const double cross_track_gain = params_.cross_track_gain;
  const double effective_wheel_base = std::max(wheel_base, 0.1);

  std::vector<Eigen::Matrix4d> simulated;

  const auto travel_distances = integrate_travel_distances(velocities);
  const auto target_poses = sample_poses_by_travel_distance(path_poses, travel_distances);

  const size_t size = std::min(target_poses.size(), velocities.size());
  simulated.reserve(size);

  double x = ego_pose(0, 3);
  double y = ego_pose(1, 3);
  double yaw = yaw_from_pose(ego_pose);
  double steering = std::clamp(initial_steering, -max_steer_rad, max_steer_rad);

  for (size_t i = 0; i < size; ++i) {
    const double v = static_cast<double>(velocities[i]);
    const double target_yaw = yaw_from_pose(target_poses[i]);
    const double dx = target_poses[i](0, 3) - x;
    const double dy = target_poses[i](1, 3) - y;
    const double cross_track_error = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double heading_error = autoware_utils::normalize_radian(target_yaw - yaw);

    double cross_track_term = std::atan2(cross_track_gain * cross_track_error, std::abs(v));

    if (std::abs(v) < stop_velocity_threshold) {
      cross_track_term = 0.0;
    }

    const double desired_steering =
      std::clamp(heading_gain * heading_error + cross_track_term, -max_steer_rad, max_steer_rad);

    const double max_delta = max_steer_rate_rad_s * dt;
    steering += std::clamp(desired_steering - steering, -max_delta, max_delta);

    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
    yaw += (v / effective_wheel_base) * std::tan(steering) * dt;

    simulated.push_back(make_pose(x, y, target_poses[i](2, 3), yaw));
  }

  return simulated;
}

FeedbackLinearizationPseudoController::FeedbackLinearizationPseudoController(
  const FeedbackLinearizationControllerParams & params)
: params_(params)
{
}

std::vector<Eigen::Matrix4d> FeedbackLinearizationPseudoController::simulate(
  const std::vector<Eigen::Matrix4d> & path_poses, const std::vector<float> & velocities,
  const Eigen::Matrix4d & ego_pose, const double wheel_base, const double initial_steering) const
{
  constexpr double dt = 0.1;
  const double max_steer_rad = params_.max_steer_rad;
  const double max_steer_rate_rad_s = params_.max_steer_rate_rad_s;
  const double stop_velocity_threshold = params_.stop_velocity_threshold;
  const double k_y = params_.lateral_gain;
  const double k_psi = params_.heading_gain;
  const double effective_wheel_base = std::max(wheel_base, 0.1);

  std::vector<Eigen::Matrix4d> simulated;
  if (path_poses.empty()) {
    return simulated;
  }
  const auto travel_distances = integrate_travel_distances(velocities);
  const auto target_poses = sample_poses_by_travel_distance(path_poses, travel_distances);

  const size_t size = std::min(target_poses.size(), velocities.size());
  simulated.reserve(size);

  double x = ego_pose(0, 3);
  double y = ego_pose(1, 3);
  double yaw = yaw_from_pose(ego_pose);
  double steering = std::clamp(initial_steering, -max_steer_rad, max_steer_rad);

  for (size_t i = 0; i < size; ++i) {
    const double v_cmd = static_cast<double>(velocities[i]);
    // Reference state is taken from the nearest (interpolated) point on the resampled path to the
    // current simulated position, rather than the time-indexed target pose. The reference yaw
    // rate is the feed-forward omega_r = v_cmd * kappa from that point's path curvature, and the
    // tracking errors e_y / e_psi are measured against the same nearest point.
    const NearestReference reference = nearest_reference(target_poses, x, y);
    const double omega_r = v_cmd * reference.curvature;

    const double delta_x = reference.x - x;
    const double delta_y = reference.y - y;
    // Ego-frame tracking error. e_x (longitudinal) is not used by this control law, only e_y
    // (lateral) and e_psi (heading) feed back into the yaw-rate command.
    const double e_y = -std::sin(yaw) * delta_x + std::cos(yaw) * delta_y;
    const double e_psi = autoware_utils::normalize_radian(reference.yaw - yaw);

    const double omega_cmd = omega_r + v_cmd * (k_y * e_y + k_psi * std::sin(e_psi));
    const double desired_steering = std::clamp(
      std::atan2(effective_wheel_base * omega_cmd, v_cmd + stop_velocity_threshold), -max_steer_rad,
      max_steer_rad);

    const double max_delta = max_steer_rate_rad_s * dt;
    steering += std::clamp(desired_steering - steering, -max_delta, max_delta);

    x += v_cmd * std::cos(yaw) * dt;
    y += v_cmd * std::sin(yaw) * dt;
    yaw += (v_cmd / effective_wheel_base) * std::tan(steering) * dt;

    simulated.push_back(make_pose(x, y, reference.z, yaw));
  }

  return simulated;
}

}  // namespace autoware::diffusion_planner::postprocess
