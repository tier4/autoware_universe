// Copyright 2026 Autoware Foundation
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

#include "autoware/avoidance_target_detector/impl.hpp"

#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_geometry/pose_deviation.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::avoidance_target_detector
{

namespace
{

using autoware_perception_msgs::msg::PredictedObject;
using autoware_planning_msgs::msg::TrajectoryPoint;
namespace aw_trajectory = autoware::experimental::trajectory;

constexpr double k_s_position_epsilon_m = 1e-3;

/**
 * @brief Build an interpolated trajectory from a trajectory message.
 * @param trajectory_msg Input trajectory message.
 * @return Interpolated trajectory, or std::nullopt if build fails.
 */
std::optional<aw_trajectory::Trajectory<TrajectoryPoint>> build_trajectory(
  const Trajectory & trajectory_msg)
{
  if (trajectory_msg.points.size() < 2) {
    return std::nullopt;
  }

  const auto trajectory =
    aw_trajectory::Trajectory<TrajectoryPoint>::Builder{}.build(trajectory_msg.points);
  if (!trajectory) {
    return std::nullopt;
  }

  return *trajectory;
}

/**
 * @brief Get the reference point used for d-coordinate validation.
 * @param object Predicted object.
 * @return Object center position.
 */
geometry_msgs::msg::Point get_object_reference_point(const PredictedObject & object)
{
  return object.kinematics.initial_pose_with_covariance.pose.position;
}

/**
 * @brief Compute the arc-length range of an object footprint along the trajectory.
 * @param trajectory Interpolated reference trajectory.
 * @param object Predicted object.
 * @param s_min Output minimum s [m].
 * @param s_max Output maximum s [m].
 */
void get_footprint_s_range(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory, const PredictedObject & object,
  double & s_min, double & s_max)
{
  const auto footprint = autoware_utils_geometry::to_polygon2d(object);
  s_min = std::numeric_limits<double>::max();
  s_max = std::numeric_limits<double>::lowest();

  const auto update_s_range = [&](const geometry_msgs::msg::Point & point) {
    const double s = aw_trajectory::closest(trajectory, point);
    s_min = std::min(s_min, s);
    s_max = std::max(s_max, s);
  };

  if (footprint.outer().empty()) {
    update_s_range(get_object_reference_point(object));
    return;
  }

  for (const auto & footprint_point : footprint.outer()) {
    update_s_range(
      autoware_utils_geometry::create_point(footprint_point.x(), footprint_point.y(), 0.0));
  }
}

/**
 * @brief Check whether the object footprint lies beyond the trajectory end in s.
 * @param trajectory Interpolated reference trajectory.
 * @param object Predicted object.
 * @return True if the minimum footprint s exceeds trajectory.length().
 */
bool is_beyond_trajectory_end(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory, const PredictedObject & object)
{
  double s_min = 0.0;
  double s_max = 0.0;
  get_footprint_s_range(trajectory, object, s_min, s_max);
  return s_min > trajectory.length() + k_s_position_epsilon_m;
}

/**
 * @brief Check whether the object footprint lies within the trajectory s-range.
 * @param trajectory Interpolated reference trajectory.
 * @param object Predicted object.
 * @return True if footprint s is within [0, trajectory.length()].
 */
bool is_within_trajectory_s_range(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory, const PredictedObject & object)
{
  double s_min = 0.0;
  double s_max = 0.0;
  get_footprint_s_range(trajectory, object, s_min, s_max);
  return s_min >= -k_s_position_epsilon_m && s_max <= trajectory.length() + k_s_position_epsilon_m;
}

/**
 * @brief Get the last M arc-length samples from the trajectory.
 * @param trajectory Interpolated reference trajectory.
 * @return Vector of s values [m] (up to OnTrajectoryDValidationParams::sample_count_m).
 */
std::vector<double> get_last_m_s_samples(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory)
{
  const auto bases = trajectory.get_underlying_bases();
  if (bases.empty()) {
    return {};
  }

  const std::size_t sample_count =
    std::min(OnTrajectoryDValidationParams::sample_count_m, bases.size());
  return {bases.end() - static_cast<std::ptrdiff_t>(sample_count), bases.end()};
}

/**
 * @brief Get trajectory s samples near the object footprint within S meters.
 * @param trajectory Interpolated reference trajectory.
 * @param object Predicted object.
 * @return Vector of s values [m] in [s_min - S, s_max + S].
 */
std::vector<double> get_s_samples_near_object(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory, const PredictedObject & object)
{
  double s_min = 0.0;
  double s_max = 0.0;
  get_footprint_s_range(trajectory, object, s_min, s_max);

  const double s_lo = std::max(0.0, s_min - OnTrajectoryDValidationParams::near_s_range_m);
  const double s_hi =
    std::min(trajectory.length(), s_max + OnTrajectoryDValidationParams::near_s_range_m);

  if (s_hi < s_lo) {
    return {};
  }

  std::vector<double> samples;
  for (const double s : trajectory.get_underlying_bases()) {
    if (s >= s_lo && s <= s_hi) {
      samples.push_back(s);
    }
  }

  if (samples.size() < 2) {
    samples.clear();
    for (double s = s_lo; s <= s_hi + OnTrajectoryDValidationParams::s_sample_interval_m * 0.5;
         s += OnTrajectoryDValidationParams::s_sample_interval_m) {
      samples.push_back(std::min(s, s_hi));
    }
  }

  return samples;
}

/**
 * @brief Compute signed lateral d-coordinate at a given trajectory s.
 * @param trajectory Interpolated reference trajectory.
 * @param s Arc length on the trajectory [m].
 * @param object_point Object reference point.
 * @return Signed lateral deviation [m].
 */
double calc_lateral_d_at_s(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory, const double s,
  const geometry_msgs::msg::Point & object_point)
{
  const auto trajectory_point = trajectory.compute(s);
  return autoware_utils_geometry::calc_lateral_deviation(trajectory_point.pose, object_point);
}

/**
 * @brief Check whether |d(k)| and |d(k) - d(k-1)| are consistently small over s samples.
 * @param trajectory Interpolated reference trajectory.
 * @param object_point Object reference point.
 * @param s_samples Arc-length samples to evaluate.
 * @return True if all magnitudes and consecutive deviations are below thresholds.
 */
bool matches_small_d_pattern(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Point & object_point, const std::vector<double> & s_samples)
{
  if (s_samples.size() < 2) {
    return false;
  }

  std::vector<double> d_magnitudes;
  d_magnitudes.reserve(s_samples.size());
  for (const double s : s_samples) {
    d_magnitudes.push_back(std::abs(calc_lateral_d_at_s(trajectory, s, object_point)));
  }

  for (const double d_magnitude : d_magnitudes) {
    if (d_magnitude >= OnTrajectoryDValidationParams::magnitude_threshold_m) {
      return false;
    }
  }

  for (std::size_t k = 1; k < d_magnitudes.size(); ++k) {
    if (
      std::abs(d_magnitudes[k] - d_magnitudes[k - 1]) >=
      OnTrajectoryDValidationParams::deviation_threshold_m) {
      return false;
    }
  }

  return true;
}

std::vector<geometry_msgs::msg::Point> get_object_footprint_points(const PredictedObject & object)
{
  const auto footprint = autoware_utils_geometry::to_polygon2d(object);
  std::vector<geometry_msgs::msg::Point> footprint_points;
  footprint_points.reserve(footprint.outer().size());

  for (const auto & footprint_point : footprint.outer()) {
    footprint_points.push_back(
      autoware_utils_geometry::create_point(footprint_point.x(), footprint_point.y(), 0.0));
  }

  if (footprint_points.empty()) {
    footprint_points.push_back(get_object_reference_point(object));
  }

  return footprint_points;
}

geometry_msgs::msg::Point get_closest_point_on_polyline(
  const geometry_msgs::msg::Point & query, const std::vector<geometry_msgs::msg::Point> & polyline)
{
  if (polyline.empty()) {
    return query;
  }
  if (polyline.size() == 1) {
    return polyline.front();
  }

  geometry_msgs::msg::Point closest_point = polyline.front();
  double min_dist_sq = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    const auto & seg_start = polyline[i];
    const auto & seg_end = polyline[i + 1];

    const double dx = seg_end.x - seg_start.x;
    const double dy = seg_end.y - seg_start.y;
    const double len_sq = dx * dx + dy * dy;

    geometry_msgs::msg::Point projected_point = seg_start;
    if (len_sq > 1e-12) {
      const double t = std::clamp(
        ((query.x - seg_start.x) * dx + (query.y - seg_start.y) * dy) / len_sq, 0.0, 1.0);
      projected_point.x = seg_start.x + t * dx;
      projected_point.y = seg_start.y + t * dy;
      projected_point.z = seg_start.z + t * (seg_end.z - seg_start.z);
    }

    const double dist_sq = (query.x - projected_point.x) * (query.x - projected_point.x) +
                           (query.y - projected_point.y) * (query.y - projected_point.y);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_point = projected_point;
    }
  }

  return closest_point;
}

bool is_footprint_point_inside_drivable_area(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Point & footprint_point,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const double tolerance_m)
{
  const double s = aw_trajectory::closest(trajectory, footprint_point);
  const auto ref_pose = trajectory.compute(s).pose;

  const double lateral_offset =
    autoware_utils_geometry::calc_lateral_deviation(ref_pose, footprint_point);
  const auto left_closest = get_closest_point_on_polyline(ref_pose.position, left_bound);
  const auto right_closest = get_closest_point_on_polyline(ref_pose.position, right_bound);
  const double left_bound_offset =
    autoware_utils_geometry::calc_lateral_deviation(ref_pose, left_closest);
  const double right_bound_offset =
    autoware_utils_geometry::calc_lateral_deviation(ref_pose, right_closest);

  const double left_limit = std::max(left_bound_offset, right_bound_offset);
  const double right_limit = std::min(left_bound_offset, right_bound_offset);

  return (right_limit - tolerance_m) <= lateral_offset &&
         lateral_offset <= (left_limit + tolerance_m);
}

}  // namespace

bool is_object_beyond_trajectory_end(
  const Trajectory & trajectory_msg, const PredictedObject & object)
{
  const auto built_trajectory = build_trajectory(trajectory_msg);
  if (!built_trajectory) {
    return false;
  }

  return is_beyond_trajectory_end(*built_trajectory, object);
}

bool should_filter_out_on_trajectory_object(
  const Trajectory & trajectory_msg, const PredictedObject & object)
{
  const auto built_trajectory = build_trajectory(trajectory_msg);
  if (!built_trajectory) {
    return false;
  }

  const auto object_point = get_object_reference_point(object);

  if (is_beyond_trajectory_end(*built_trajectory, object)) {
    return matches_small_d_pattern(
      *built_trajectory, object_point, get_last_m_s_samples(*built_trajectory));
  }

  if (is_within_trajectory_s_range(*built_trajectory, object)) {
    return matches_small_d_pattern(
      *built_trajectory, object_point, get_s_samples_near_object(*built_trajectory, object));
  }

  return false;
}

bool should_filter_out_by_longitudinal_distance(
  const Trajectory & trajectory_msg, const PredictedObject & object,
  const LongitudinalDistanceFilterParams & params)
{
  const auto built_trajectory = build_trajectory(trajectory_msg);
  if (!built_trajectory) {
    return false;
  }

  const auto start_pose = built_trajectory->compute(0.0).pose;
  const auto end_pose = built_trajectory->compute(built_trajectory->length()).pose;
  const auto footprint_points = get_object_footprint_points(object);

  const bool all_before_start = std::all_of(
    footprint_points.begin(), footprint_points.end(),
    [&](const geometry_msgs::msg::Point & footprint_point) {
      return autoware_utils_geometry::calc_longitudinal_deviation(start_pose, footprint_point) <
             -params.tolerance_m;
    });

  const bool all_after_end = std::all_of(
    footprint_points.begin(), footprint_points.end(),
    [&](const geometry_msgs::msg::Point & footprint_point) {
      return autoware_utils_geometry::calc_longitudinal_deviation(end_pose, footprint_point) >
             params.tolerance_m;
    });

  return all_before_start || all_after_end;
}

bool should_filter_out_by_lateral_distance(
  const DrivableAreaResult & drivable_area, const Trajectory & trajectory_msg,
  const PredictedObject & object, const LateralDistanceFilterParams & params)
{
  if (drivable_area.left_bound.size() < 2 || drivable_area.right_bound.size() < 2) {
    return false;
  }

  const auto built_trajectory = build_trajectory(trajectory_msg);
  if (!built_trajectory) {
    return false;
  }

  const auto footprint_points = get_object_footprint_points(object);
  return std::all_of(
    footprint_points.begin(), footprint_points.end(),
    [&](const geometry_msgs::msg::Point & footprint_point) {
      return !is_footprint_point_inside_drivable_area(
        *built_trajectory, footprint_point, drivable_area.left_bound, drivable_area.right_bound,
        params.tolerance_m);
    });
}

}  // namespace autoware::avoidance_target_detector
