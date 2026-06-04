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

#ifndef AUTOWARE__AVOIDANCE_TARGET_DETECTOR__IMPL_HPP_
#define AUTOWARE__AVOIDANCE_TARGET_DETECTOR__IMPL_HPP_

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <array>
#include <cstddef>
#include <limits>
#include <optional>

namespace autoware::avoidance_target_detector
{

using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;

/** Object classification labels considered as avoidance targets. */
inline constexpr std::array<uint8_t, 6> labels_of_interest{
  ObjectClassification::CAR,     ObjectClassification::TRUCK,      ObjectClassification::BUS,
  ObjectClassification::TRAILER, ObjectClassification::MOTORCYCLE, ObjectClassification::BICYCLE};

/** Parameters for filtering out moving objects by linear twist norm. */
struct MovingObjectFilterParams
{
  static constexpr double max_linear_velocity_mps =
    0.5;  ///< Objects with linear velocity norm above this are removed [m/s].
};

/** Thresholds for footprint proximity to the reference trajectory. */
struct TrajectoryProximityThresholds
{
  double max_lateral_distance_m{5.5};        ///< Maximum lateral distance [m].
  double max_longitudinal_distance_m{10.0};  ///< Maximum longitudinal distance [m].
};

/** Parameters for on-trajectory d-coordinate validation (filter-out). */
struct OnTrajectoryDValidationParams
{
  static constexpr std::size_t sample_count_m = 5;      ///< Number of trajectory samples (M).
  static constexpr double magnitude_threshold_m = 1.5;  ///< Max |d(k)| threshold [m].
  static constexpr double deviation_threshold_m = 0.3;  ///< Max |d(k) - d(k-1)| threshold [m].
  static constexpr double near_s_range_m = 15.0;        ///< s-range around footprint (S) [m].
  static constexpr double s_sample_interval_m = 1.0;    ///< Fallback s sampling interval [m].
};

/**
 * @brief Detect avoidance targets from predicted objects.
 * @param input_objects Input predicted objects.
 * @param trajectory Optional reference trajectory for proximity filtering.
 * @param thresholds Proximity thresholds applied when trajectory is available.
 * @return Filtered predicted objects that qualify as avoidance targets.
 */
PredictedObjects detect_avoidance_targets(
  const PredictedObjects & input_objects, const std::optional<Trajectory> & trajectory,
  const TrajectoryProximityThresholds & thresholds = {});

/**
 * @brief Remove objects whose classification is not of interest.
 * @param objects Predicted objects to filter in place.
 */
void filter_out_objects_not_of_interest(PredictedObjects & objects);

/**
 * @brief Remove objects whose linear twist norm exceeds MovingObjectFilterParams threshold.
 * @param objects Predicted objects to filter in place.
 */
void filter_out_moving_objects(PredictedObjects & objects);

/**
 * @brief Remove objects whose footprint is farther than the proximity thresholds from the
 * trajectory.
 * @param objects Predicted objects to filter in place.
 * @param trajectory Reference trajectory.
 * @param thresholds Lateral and longitudinal distance thresholds.
 */
void filter_out_objects_far_from_trajectory(
  PredictedObjects & objects, const Trajectory & trajectory,
  const TrajectoryProximityThresholds & thresholds);

/**
 * @brief Remove objects that violate the on-trajectory d-coordinate thresholds.
 * @param objects Predicted objects to filter in place.
 * @param trajectory Reference trajectory.
 */
void filter_out_objects_with_small_deviation(
  PredictedObjects & objects, const Trajectory & trajectory);

}  // namespace autoware::avoidance_target_detector

#endif  // AUTOWARE__AVOIDANCE_TARGET_DETECTOR__IMPL_HPP_
