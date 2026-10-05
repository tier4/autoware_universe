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

#include "autoware/ml_planner/utils/virtual_pose.hpp"

#include "autoware/trajectory/interpolator/cubic_spline.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/closest.hpp"

#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::ml_planner::utils
{
namespace
{
using autoware::experimental::trajectory::Trajectory;
using autoware::experimental::trajectory::interpolator::CubicSpline;

geometry_msgs::msg::Pose matrix4d_to_planar_pose(const Eigen::Matrix4d & matrix)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  // Planar: closest_with_constraint measures 3D distance and the arc length includes z, so a
  // trajectory at elevation would otherwise be compared against the query at the wrong height.
  pose.position.z = 0.0;
  const Eigen::Quaterniond q(matrix.block<3, 3>(0, 0));
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

// Leading vertices up to (excluding) the first one that coincides with its predecessor: a stop at
// the end of a prediction repeats its last point, which would give the spline zero-length bases.
std::vector<geometry_msgs::msg::Pose> leading_distinct_poses(
  const std::vector<Eigen::Matrix4d> & polyline)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(polyline.size());
  for (const auto & matrix : polyline) {
    const geometry_msgs::msg::Pose pose = matrix4d_to_planar_pose(matrix);
    if (
      !poses.empty() &&
      autoware::experimental::trajectory::is_almost_same(poses.back().position, pose.position)) {
      break;
    }
    poses.push_back(pose);
  }
  return poses;
}

// Circular mean of the spline tangent over a window kept symmetric around s (shrunk at either end
// so the mean is not biased toward the future path); nullopt when the window is too short.
std::optional<double> windowed_tangent_yaw(
  const Trajectory<geometry_msgs::msg::Pose> & trajectory, const double s,
  const double half_window_m, const double min_length_m)
{
  const double half = std::min({half_window_m, s, trajectory.length() - s});
  if (2.0 * half < min_length_m) {
    return std::nullopt;
  }
  constexpr double SAMPLE_TICK_M = 0.05;
  double sum_cos = 0.0;
  double sum_sin = 0.0;
  for (const double yaw :
       trajectory.azimuth(trajectory.base_arange({s - half, s + half}, SAMPLE_TICK_M))) {
    sum_cos += std::cos(yaw);
    sum_sin += std::sin(yaw);
  }
  return std::atan2(sum_sin, sum_cos);
}

// Savitzky-Golay smoothing of the trajectory points before the spline (the Diffusion-Planner
// recipe: window 11, cubic, on x and y over the point index; the window shrinks at the ends of a
// short trajectory, and the end points use the fit of the nearest full window). The spline then
// follows the trajectory without passing through its point-to-point jitter; point count and order
// are unchanged.
std::vector<geometry_msgs::msg::Pose> smooth_positions(
  const std::vector<geometry_msgs::msg::Pose> & poses)
{
  constexpr int ORDER = 3;
  const auto n = static_cast<int>(poses.size());
  int window = std::min(11, n % 2 == 0 ? n - 1 : n);
  if (window < ORDER + 2) {
    return poses;
  }
  const int half = window / 2;
  // Least-squares polynomial on the window [-half, half]; row r of the pseudo-inverse evaluates
  // the fit at offset (r - half).
  Eigen::MatrixXd vandermonde(window, ORDER + 1);
  for (int r = 0; r < window; ++r) {
    for (int c = 0; c <= ORDER; ++c) {
      vandermonde(r, c) = std::pow(static_cast<double>(r - half), c);
    }
  }
  const Eigen::MatrixXd fit =
    vandermonde * vandermonde.completeOrthogonalDecomposition().pseudoInverse();
  std::vector<geometry_msgs::msg::Pose> smoothed = poses;
  for (int i = 0; i < n; ++i) {
    const int start = std::clamp(i - half, 0, n - window);
    const int row = i - start;
    double x = 0.0;
    double y = 0.0;
    for (int j = 0; j < window; ++j) {
      x += fit(row, j) * poses[static_cast<size_t>(start + j)].position.x;
      y += fit(row, j) * poses[static_cast<size_t>(start + j)].position.y;
    }
    smoothed[static_cast<size_t>(i)].position.x = x;
    smoothed[static_cast<size_t>(i)].position.y = y;
  }
  return smoothed;
}

struct ClosestPoint
{
  Eigen::Vector2d position;
  std::optional<double> tangent_yaw;
};

std::optional<ClosestPoint> closest_point_on_previous_trajectory(
  const geometry_msgs::msg::Point & query, const std::vector<Eigen::Matrix4d> & polyline,
  const int64_t prefix_count, const VirtualPoseParams & params)
{
  // Only the raw model output is smoothed; the optimized trajectory is used as published.
  const std::vector<geometry_msgs::msg::Pose> distinct = leading_distinct_poses(polyline);
  const std::vector<geometry_msgs::msg::Pose> poses =
    params.reference == "raw" ? smooth_positions(distinct) : distinct;
  const auto prefix = static_cast<size_t>(prefix_count);
  if (poses.size() < prefix + 2) {
    return std::nullopt;
  }
  const auto trajectory =
    Trajectory<geometry_msgs::msg::Pose>::Builder{}.set_xy_interpolator<CubicSpline>().build(poses);
  if (!trajectory) {
    return std::nullopt;
  }
  // Search only the leading segments of the trajectory proper (vertex `prefix` onwards): a cycle
  // advances the vehicle by about one segment, and a far part (a U-turn's return leg) is excluded.
  const std::vector<double> bases = trajectory->get_underlying_bases();
  const double search_start_s = bases[prefix];
  const double search_end_s = bases[std::min<size_t>(
    prefix + static_cast<size_t>(params.max_search_segment_count), bases.size() - 1)];
  geometry_msgs::msg::Point planar_query;
  planar_query.x = query.x;
  planar_query.y = query.y;
  const std::optional<double> s = autoware::experimental::trajectory::closest_with_constraint(
    *trajectory, planar_query, [search_start_s, search_end_s](const double & arc) {
      return search_start_s <= arc && arc <= search_end_s;
    });
  if (!s) {
    return std::nullopt;
  }
  const geometry_msgs::msg::Pose closest = trajectory->compute(*s);
  return ClosestPoint{
    Eigen::Vector2d(closest.position.x, closest.position.y),
    windowed_tangent_yaw(
      *trajectory, *s, params.yaw_fit_half_window_m, params.yaw_fit_min_length_m)};
}
}  // namespace

std::optional<ElapsedTimePoint> point_at_elapsed_time(
  const Eigen::Matrix4d & frame_pose, const std::vector<Eigen::Matrix4d> & prediction,
  const std::vector<double> & times, const double elapsed_sec)
{
  if (prediction.empty() || prediction.size() != times.size()) {
    return std::nullopt;
  }
  Eigen::Vector2d start = frame_pose.block<2, 1>(0, 3);
  double start_time = 0.0;
  for (size_t i = 0; i < prediction.size(); ++i) {
    const Eigen::Vector2d end = prediction[i].block<2, 1>(0, 3);
    const double duration = times[i] - start_time;
    if (duration > 1.0e-6 && (elapsed_sec <= times[i] || i + 1 == prediction.size())) {
      const double ratio = std::clamp((elapsed_sec - start_time) / duration, 0.0, 1.0);
      return ElapsedTimePoint{start + ratio * (end - start), (end - start).norm() / duration};
    }
    start = end;
    start_time = times[i];
  }
  return ElapsedTimePoint{start, 0.0};
}

bool update_time_based_mode(
  const bool active, const bool engaged, const double speed_mps, const VirtualPoseParams & params)
{
  if (!params.time_based_enable || !engaged) {
    return false;
  }
  const double speed = std::abs(speed_mps);
  if (active) {
    return speed <= params.time_based_exit_speed_mps;
  }
  return speed < params.time_based_enter_speed_mps;
}

VirtualPoseResult compute_virtual_pose(
  const geometry_msgs::msg::Pose & measured_pose, const std::vector<Eigen::Matrix4d> & polyline,
  const int64_t prefix_count, const VirtualPoseParams & params)
{
  return compute_virtual_pose(
    measured_pose, measured_pose.position, polyline, prefix_count, params);
}

VirtualPoseResult compute_virtual_pose(
  const geometry_msgs::msg::Pose & measured_pose, const geometry_msgs::msg::Point & query,
  const std::vector<Eigen::Matrix4d> & polyline, const int64_t prefix_count,
  const VirtualPoseParams & params)
{
  const auto closest = closest_point_on_previous_trajectory(query, polyline, prefix_count, params);
  if (!closest) {
    return VirtualPoseResult{measured_pose, false, false, 0.0, 0.0};
  }

  const double measured_yaw = autoware_utils_geometry::get_rpy(measured_pose.orientation).z;
  const double virtual_yaw = closest->tangent_yaw.value_or(measured_yaw);
  const double yaw_change = autoware_utils::normalize_radian(virtual_yaw - measured_yaw);
  // Vehicle offset from the virtual pose, split along and across the virtual heading.
  const Eigen::Vector2d offset =
    Eigen::Vector2d(measured_pose.position.x, measured_pose.position.y) - closest->position;
  const double longitudinal_error_m =
    std::abs(std::cos(virtual_yaw) * offset.x() + std::sin(virtual_yaw) * offset.y());
  const double lateral_error_m =
    std::abs(-std::sin(virtual_yaw) * offset.x() + std::cos(virtual_yaw) * offset.y());
  const double position_error_m = offset.norm();
  const double yaw_error_deg = autoware_utils::rad2deg(std::abs(yaw_change));

  if (
    longitudinal_error_m > params.max_longitudinal_error_m ||
    lateral_error_m > params.max_lateral_error_m || yaw_error_deg > params.max_yaw_error_deg) {
    return VirtualPoseResult{measured_pose, true, true, position_error_m, yaw_error_deg};
  }

  // Rotate the measured orientation about the map z axis by the yaw change, so the vehicle's roll
  // and pitch are kept (the trajectory carries none); the height is the measured one.
  const Eigen::Quaterniond measured_q(
    measured_pose.orientation.w, measured_pose.orientation.x, measured_pose.orientation.y,
    measured_pose.orientation.z);
  const Eigen::Quaterniond virtual_q =
    (Eigen::Quaterniond(Eigen::AngleAxisd(yaw_change, Eigen::Vector3d::UnitZ())) * measured_q)
      .normalized();

  geometry_msgs::msg::Pose virtual_pose = measured_pose;
  virtual_pose.position.x = closest->position.x();
  virtual_pose.position.y = closest->position.y();
  virtual_pose.orientation.x = virtual_q.x();
  virtual_pose.orientation.y = virtual_q.y();
  virtual_pose.orientation.z = virtual_q.z();
  virtual_pose.orientation.w = virtual_q.w();
  return VirtualPoseResult{virtual_pose, true, false, position_error_m, yaw_error_deg};
}

}  // namespace autoware::ml_planner::utils
