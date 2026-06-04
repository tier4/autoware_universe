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
#include <utility>
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
 * @brief Compute lateral and longitudinal deviations from the nearest trajectory point.
 * @param trajectory Interpolated reference trajectory.
 * @param point Query point.
 * @return Pair of absolute lateral and longitudinal deviations [m].
 */
std::pair<double, double> calc_sd_deviation(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Point & point)
{
  const double s = aw_trajectory::closest(trajectory, point);
  const auto nearest_point = trajectory.compute(s);
  const double lateral_distance =
    std::abs(autoware_utils_geometry::calc_lateral_deviation(nearest_point.pose, point));
  const double longitudinal_distance =
    std::abs(autoware_utils_geometry::calc_longitudinal_deviation(nearest_point.pose, point));
  return {lateral_distance, longitudinal_distance};
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

/**
 * @brief Decide whether to filter out an object aligned with the trajectory corridor.
 * @param trajectory Interpolated reference trajectory.
 * @param object Predicted object.
 * @return True if the object should be removed (on-path extension or in-lane alignment).
 */
bool should_filter_out_on_trajectory_object(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory, const PredictedObject & object)
{
  const auto object_point = get_object_reference_point(object);

  if (is_beyond_trajectory_end(trajectory, object)) {
    return matches_small_d_pattern(trajectory, object_point, get_last_m_s_samples(trajectory));
  }

  if (is_within_trajectory_s_range(trajectory, object)) {
    return matches_small_d_pattern(
      trajectory, object_point, get_s_samples_near_object(trajectory, object));
  }

  return false;
}

/**
 * @brief Check whether any footprint vertex is within proximity thresholds of the trajectory.
 * @param trajectory Interpolated reference trajectory.
 * @param object Predicted object.
 * @param thresholds Lateral and longitudinal distance thresholds.
 * @return True if the footprint is near the trajectory.
 */
bool is_footprint_near_trajectory(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory, const PredictedObject & object,
  const TrajectoryProximityThresholds & thresholds)
{
  const auto footprint = autoware_utils_geometry::to_polygon2d(object);
  if (footprint.outer().empty()) {
    return false;
  }

  double min_lateral_distance = std::numeric_limits<double>::max();
  double min_longitudinal_distance = std::numeric_limits<double>::max();

  for (const auto & footprint_point : footprint.outer()) {
    const auto point =
      autoware_utils_geometry::create_point(footprint_point.x(), footprint_point.y(), 0.0);
    const auto [lateral_distance, longitudinal_distance] = calc_sd_deviation(trajectory, point);
    min_lateral_distance = std::min(min_lateral_distance, lateral_distance);
    min_longitudinal_distance = std::min(min_longitudinal_distance, longitudinal_distance);
  }

  return min_lateral_distance <= thresholds.max_lateral_distance_m &&
         min_longitudinal_distance <= thresholds.max_longitudinal_distance_m;
}

}  // namespace

/**
 * @brief Detect avoidance targets from predicted objects.
 * @param input_objects Input predicted objects.
 * @param trajectory Optional reference trajectory for proximity filtering.
 * @param thresholds Proximity thresholds applied when trajectory is available.
 * @return Filtered predicted objects that qualify as avoidance targets.
 */
PredictedObjects detect_avoidance_targets(
  const PredictedObjects & input_objects, const std::optional<Trajectory> & trajectory,
  const TrajectoryProximityThresholds & thresholds)
{
  auto result = input_objects;
  RCLCPP_INFO(
    rclcpp::get_logger("avoidance_target_detector"), "%zu <- Original object size",
    result.objects.size());

  filter_out_objects_not_of_interest(result);
  RCLCPP_INFO(
    rclcpp::get_logger("avoidance_target_detector"),
    "    %zu <- After filter out objects not of interest", result.objects.size());

  filter_out_moving_objects(result);
  RCLCPP_INFO(
    rclcpp::get_logger("avoidance_target_detector"), "    %zu <- After filter out moving objects",
    result.objects.size());

  if (trajectory) {
    filter_out_objects_far_from_trajectory(result, *trajectory, thresholds);
    RCLCPP_INFO(
      rclcpp::get_logger("avoidance_target_detector"),
      "    %zu <- After filter out objects far from trajectory", result.objects.size());

    filter_out_objects_with_small_deviation(result, *trajectory);
    RCLCPP_INFO(
      rclcpp::get_logger("avoidance_target_detector"),
      "    %zu <- After filter out objects with small deviation", result.objects.size());
  }
  return result;
}

/**
 * @brief Remove objects whose classification is not of interest.
 * @param objects Predicted objects to filter in place.
 */
void filter_out_objects_not_of_interest(PredictedObjects & objects)
{
  const auto is_object_of_interest = [](const PredictedObject & object) {
    constexpr float probability_threshold = 0.1f;
    return std::any_of(
      object.classification.begin(), object.classification.end(),
      [](const ObjectClassification & classification) {
        if (classification.probability <= probability_threshold) {
          return false;
        }
        return std::find(
                 labels_of_interest.begin(), labels_of_interest.end(), classification.label) !=
               labels_of_interest.end();
      });
  };

  objects.objects.erase(
    std::remove_if(
      objects.objects.begin(), objects.objects.end(),
      [&](const PredictedObject & object) { return !is_object_of_interest(object); }),
    objects.objects.end());
}

/**
 * @brief Remove objects whose linear twist norm exceeds MovingObjectFilterParams threshold.
 * @param objects Predicted objects to filter in place.
 */
void filter_out_moving_objects(PredictedObjects & objects)
{
  const auto is_moving = [](const PredictedObject & object) {
    const auto & linear = object.kinematics.initial_twist_with_covariance.twist.linear;
    const double linear_velocity_norm = std::hypot(linear.x, linear.y, linear.z);
    return linear_velocity_norm > MovingObjectFilterParams::max_linear_velocity_mps;
  };

  objects.objects.erase(
    std::remove_if(
      objects.objects.begin(), objects.objects.end(),
      [&](const PredictedObject & object) { return is_moving(object); }),
    objects.objects.end());
}

/**
 * @brief Remove objects whose footprint is farther than the proximity thresholds from the
 * trajectory.
 * @param objects Predicted objects to filter in place.
 * @param trajectory Reference trajectory.
 * @param thresholds Lateral and longitudinal distance thresholds.
 */
void filter_out_objects_far_from_trajectory(
  PredictedObjects & objects, const Trajectory & trajectory,
  const TrajectoryProximityThresholds & thresholds)
{
  const auto built_trajectory = build_trajectory(trajectory);
  if (!built_trajectory) {
    return;
  }

  objects.objects.erase(
    std::remove_if(
      objects.objects.begin(), objects.objects.end(),
      [&](const PredictedObject & object) {
        return !is_footprint_near_trajectory(*built_trajectory, object, thresholds);
      }),
    objects.objects.end());
}

/**
 * @brief Remove objects that violate the on-trajectory d-coordinate thresholds.
 * @param objects Predicted objects to filter in place.
 * @param trajectory Reference trajectory.
 */
void filter_out_objects_with_small_deviation(
  PredictedObjects & objects, const Trajectory & trajectory)
{
  const auto built_trajectory = build_trajectory(trajectory);
  if (!built_trajectory) {
    return;
  }

  objects.objects.erase(
    std::remove_if(
      objects.objects.begin(), objects.objects.end(),
      [&](const PredictedObject & object) {
        return should_filter_out_on_trajectory_object(*built_trajectory, object);
      }),
    objects.objects.end());
}

}  // namespace autoware::avoidance_target_detector
