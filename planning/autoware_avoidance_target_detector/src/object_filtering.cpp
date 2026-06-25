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

#include "autoware/avoidance_target_detector/object_filtering.hpp"

#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_geometry/pose_deviation.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::avoidance_target_detector
{

namespace
{

using autoware_planning_msgs::msg::TrajectoryPoint;
namespace aw_trajectory = autoware::experimental::trajectory;

constexpr double k_s_position_epsilon_m = 1e-3;

constexpr double k_high_likelihood = 0.95;
constexpr double k_low_likelihood = 0.05;
constexpr double k_neutral_likelihood = 0.5;

bool is_object_of_interest(const PredictedObject & object)
{
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
}

double linear_velocity_norm(const PredictedObject & object)
{
  const auto & linear = object.kinematics.initial_twist_with_covariance.twist.linear;
  return std::hypot(linear.x, linear.y, linear.z);
}

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
 * @brief Get rear-left and rear-right corners of the object footprint.
 * @param object Predicted object.
 * @return Rear edge corner points in map frame.
 */
std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> get_object_rear_edge_points(
  const PredictedObject & object)
{
  if (object.shape.type != autoware_perception_msgs::msg::Shape::POLYGON) {
    const auto reference_point = get_object_reference_point(object);
    return {reference_point, reference_point};
  }

  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  const auto footprint = autoware_utils_geometry::to_polygon2d(object);

  if (footprint.outer().empty()) {
    const auto reference_point = get_object_reference_point(object);
    return {reference_point, reference_point};
  }

  double rear_longitudinal = std::numeric_limits<double>::max();
  for (const auto & footprint_point : footprint.outer()) {
    const auto point =
      autoware_utils_geometry::create_point(footprint_point.x(), footprint_point.y(), 0.0);
    rear_longitudinal = std::min(
      rear_longitudinal, autoware_utils_geometry::calc_longitudinal_deviation(object_pose, point));
  }

  constexpr double k_rear_edge_tolerance_m = 0.01;
  geometry_msgs::msg::Point rear_left;
  geometry_msgs::msg::Point rear_right;
  double rear_left_lateral = std::numeric_limits<double>::lowest();
  double rear_right_lateral = std::numeric_limits<double>::max();
  bool has_rear_left = false;
  bool has_rear_right = false;

  for (const auto & footprint_point : footprint.outer()) {
    const auto point =
      autoware_utils_geometry::create_point(footprint_point.x(), footprint_point.y(), 0.0);
    const double longitudinal =
      autoware_utils_geometry::calc_longitudinal_deviation(object_pose, point);
    if (rear_longitudinal + k_rear_edge_tolerance_m < longitudinal) {
      continue;
    }

    const double lateral = autoware_utils_geometry::calc_lateral_deviation(object_pose, point);
    if (lateral > rear_left_lateral) {
      rear_left_lateral = lateral;
      rear_left = point;
      has_rear_left = true;
    }
    if (lateral < rear_right_lateral) {
      rear_right_lateral = lateral;
      rear_right = point;
      has_rear_right = true;
    }
  }

  if (!has_rear_left || !has_rear_right) {
    const auto reference_point = get_object_reference_point(object);
    return {reference_point, reference_point};
  }

  return {rear_left, rear_right};
}

/**
 * @brief Compute |d| at a given trajectory s from the object rear edge.
 * @details When the rear edge straddles the trajectory (d_rear_left > 0 and d_rear_right < 0),
 *          |d| is 0. Otherwise |d| is taken from the rear corner closest to the trajectory.
 */
double calc_rear_edge_d_magnitude_at_s(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory, const double s,
  const geometry_msgs::msg::Point & rear_left, const geometry_msgs::msg::Point & rear_right)
{
  const auto trajectory_point = trajectory.compute(s);
  const double d_rear_left =
    autoware_utils_geometry::calc_lateral_deviation(trajectory_point.pose, rear_left);
  const double d_rear_right =
    autoware_utils_geometry::calc_lateral_deviation(trajectory_point.pose, rear_right);

  if (d_rear_left > 0.0 && d_rear_right < 0.0) {
    return 0.0;
  }
  if (d_rear_left > 0.0 && d_rear_right > 0.0) {
    return std::abs(d_rear_right);
  }
  if (d_rear_left < 0.0 && d_rear_right < 0.0) {
    return std::abs(d_rear_left);
  }

  return std::max(std::abs(d_rear_left), std::abs(d_rear_right));
}

/**
 * @brief Check whether |d(k)| and |d(k) - d(k-1)| are consistently small over s samples.
 * @param trajectory Interpolated reference trajectory.
 * @param rear_left Rear-left corner of the object footprint.
 * @param rear_right Rear-right corner of the object footprint.
 * @param s_samples Arc-length samples to evaluate.
 * @return True if all magnitudes and consecutive deviations are below thresholds.
 */
bool matches_small_d_pattern(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Point & rear_left, const geometry_msgs::msg::Point & rear_right,
  const std::vector<double> & s_samples)
{
  if (s_samples.size() < 2) {
    return false;
  }

  std::vector<double> d_magnitudes;
  d_magnitudes.reserve(s_samples.size());
  for (const double s : s_samples) {
    d_magnitudes.push_back(calc_rear_edge_d_magnitude_at_s(trajectory, s, rear_left, rear_right));
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

  const auto [rear_left, rear_right] = get_object_rear_edge_points(object);

  if (is_beyond_trajectory_end(*built_trajectory, object)) {
    return matches_small_d_pattern(
      *built_trajectory, rear_left, rear_right, get_last_m_s_samples(*built_trajectory));
  }

  if (is_within_trajectory_s_range(*built_trajectory, object)) {
    return matches_small_d_pattern(
      *built_trajectory, rear_left, rear_right,
      get_s_samples_near_object(*built_trajectory, object));
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

TwoClassFilter::TwoClassFilter(
  [[maybe_unused]] const PredictedObject & object, const rclcpp::Time & last_update_time)
: prior_{k_neutral_likelihood},
  posterior_{k_neutral_likelihood},
  is_initialized_{false},
  last_update_time_{last_update_time}
{
}

void TwoClassFilter::observe_and_update(
  const rclcpp::Time & current_time, const PredictedObject & object, const Trajectory & trajectory)
{
  calculate_likelihood(object, trajectory);

  if (!is_initialized_) {
    posterior_ = target_likelihood_;
    prior_ = posterior_;
    target_likelihood_ = 1.0;
    non_target_likelihood_ = 1.0;
    is_initialized_ = true;
    last_update_time_ = current_time;
    return;
  }

  apply_transition_to_prior(object, trajectory);
  apply_bayesian_update();
  last_update_time_ = current_time;

  target_likelihood_ = 1.0;
  non_target_likelihood_ = 1.0;
  prior_ = posterior_;
}

void TwoClassFilter::apply_transition_to_prior(
  const PredictedObject & object, const Trajectory & trajectory)
{
  const auto transition = transition_matrix(object, trajectory);
  const double non_target_prior = 1.0 - prior_;
  prior_ = transition[0][0] * prior_ + transition[1][0] * non_target_prior;
}

void TwoClassFilter::apply_bayesian_update()
{
  const double numerator = prior_ * target_likelihood_;
  const double denominator = numerator + (1.0 - prior_) * non_target_likelihood_;
  posterior_ = denominator > 0.0 ? numerator / denominator : prior_;
}

/** TargetFilter implementation */

void TargetFilter::calculate_likelihood(
  const PredictedObject & object, [[maybe_unused]] const Trajectory & trajectory)
{
  target_likelihood_ = is_object_of_interest(object) ? k_high_likelihood : k_low_likelihood;
  non_target_likelihood_ = 1.0 - target_likelihood_;
}

TwoClassFilter::Matrix2x2 TargetFilter::transition_matrix(
  [[maybe_unused]] const PredictedObject & object,
  [[maybe_unused]] const Trajectory & trajectory) const
{
  constexpr double state_persistence = 0.95;
  constexpr double switch_probability = 1.0 - state_persistence;
  return {{
    {state_persistence, switch_probability},
    {switch_probability, state_persistence},
  }};
}

/** StationaryFilter implementation */

void StationaryFilter::calculate_likelihood(
  const PredictedObject & object, [[maybe_unused]] const Trajectory & trajectory)
{
  const double pre_clamp_probability =
    -(linear_velocity_norm(object) - MovingObjectFilterParams::max_linear_velocity_mps) /
    (MovingObjectFilterParams::max_linear_velocity_mps -
     MovingObjectFilterParams::promising_stop_velocity_mps);
  target_likelihood_ = std::clamp(pre_clamp_probability, 0.1, 0.99);
  non_target_likelihood_ = 1.0 - target_likelihood_;
}

TwoClassFilter::Matrix2x2 StationaryFilter::transition_matrix(
  [[maybe_unused]] const PredictedObject & object,
  [[maybe_unused]] const Trajectory & trajectory) const
{
  const double velocity_norm = linear_velocity_norm(object);

  if (velocity_norm < MovingObjectFilterParams::max_linear_velocity_mps) {
    return {{
      {0.9, 0.1},
      {0.5, 0.5},
    }};
  }

  return {{
    {0.05, 0.95},
    {0.95, 0.05},
  }};
}

/** DeviationFilter implementation */

void DeviationFilter::calculate_likelihood(
  const PredictedObject & object, [[maybe_unused]] const Trajectory & trajectory)
{
  target_likelihood_ = should_filter_out_on_trajectory_object(trajectory, object)
                         ? k_low_likelihood
                         : k_high_likelihood;
  non_target_likelihood_ = 1.0 - target_likelihood_;
}

TwoClassFilter::Matrix2x2 DeviationFilter::transition_matrix(
  const PredictedObject & object, const Trajectory & trajectory) const
{
  if (is_object_beyond_trajectory_end(trajectory, object)) {
    return {{
      {0.8, 0.2},
      {0.4, 0.6},
    }};
  }

  return {{
    {0.95, 0.05},
    {0.2, 0.8},
  }};
}

FilterManager::FilterManager(const PredictedObject & object, const rclcpp::Time & last_update_time)
: target_filter_{std::make_unique<TargetFilter>(object, last_update_time)},
  stationary_filter_{std::make_unique<StationaryFilter>(object, last_update_time)},
  deviation_filter_{std::make_unique<DeviationFilter>(object, last_update_time)},
  is_target_stamped_{last_update_time, false},
  stale_check_time_{last_update_time}
{
}

void FilterManager::observe_and_update_all(
  const rclcpp::Time & current_time, const PredictedObject & object, const Trajectory & trajectory)
{
  stale_check_time_ = current_time;

  target_filter_->observe_and_update(current_time, object, trajectory);
  stationary_filter_->observe_and_update(current_time, object, trajectory);
  deviation_filter_->observe_and_update(current_time, object, trajectory);

  const bool is_target_now = is_object_of_interest() && is_stationary() && is_deviated();

  if (!is_initialized_) {
    is_target_stamped_.first = current_time;
    is_target_stamped_.second = is_target_now;
    is_initialized_ = true;
    return;
  }

  state_change_count_ = (is_target_now != is_target_stamped_.second) ? state_change_count_ + 1 : 0;

  if (
    rclcpp::Time(current_time) - rclcpp::Time(is_target_stamped_.first) <
    rclcpp::Duration::from_seconds(FilterManagerParams::hysteresis_seconds)) {
    return;
  }

  if (state_change_count_ < FilterManagerParams::count_threshold) {
    return;
  }

  if (is_target_now && !is_target_stamped_.second) {
    debug_log_ = std::string("Turned to target on due to \n") +
                 "is_object_of_interest: " + std::to_string(get_is_target_probability()) + "\n" +
                 "is_stationary: " + std::to_string(get_is_stationary_probability()) + "\n" +
                 "is_deviated: " + std::to_string(get_is_deviated_probability());
  } else if (!is_target_now && is_target_stamped_.second) {
    debug_log_ = std::string("Turned to non-target on due to \n") +
                 "is_object_of_interest: " + std::to_string(get_is_target_probability()) + "\n" +
                 "is_stationary: " + std::to_string(get_is_stationary_probability()) + "\n" +
                 "is_deviated: " + std::to_string(get_is_deviated_probability());
  }

  is_target_stamped_.first = current_time;
  is_target_stamped_.second = is_target_now;
}

PredictedObjects ObjectSelector::get_avoidance_targets(
  const rclcpp::Time & current_time, const PredictedObjects & objects,
  const Trajectory & trajectory, const std::optional<DrivableAreaResult> & drivable_area)
{
  for (const auto & object : objects.objects) {
    const auto object_id_str = autoware_utils_uuid::to_hex_string(object.object_id);
    const auto it = object_filters_.try_emplace(object_id_str, object, current_time).first;
    it->second.observe_and_update_all(current_time, object, trajectory);
  }

  for (auto it = object_filters_.begin(); it != object_filters_.end();) {
    if (it->second.is_stale(current_time)) {
      it = object_filters_.erase(it);
    } else {
      ++it;
    }
  }

  for (auto & [object_id_str, filter_manager] : object_filters_) {
    if (filter_manager.get_debug_log().empty()) {
      continue;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("autoware_avoidance_target_detector"), "Object ID: %s, Debug Log: %s",
      object_id_str.c_str(), filter_manager.get_debug_log().c_str());
    filter_manager.clear_debug_log();
  }

  PredictedObjects avoidance_targets = objects;
  avoidance_targets.objects.erase(
    std::remove_if(
      avoidance_targets.objects.begin(), avoidance_targets.objects.end(),
      [&](const PredictedObject & object) {
        const auto it = object_filters_.find(autoware_utils_uuid::to_hex_string(object.object_id));
        return it == object_filters_.end() || !it->second.is_target();
      }),
    avoidance_targets.objects.end());

  avoidance_targets.objects.erase(
    std::remove_if(
      avoidance_targets.objects.begin(), avoidance_targets.objects.end(),
      [&](const PredictedObject & object) {
        if (should_filter_out_by_longitudinal_distance(
              trajectory, object, LongitudinalDistanceFilterParams{})) {
          return true;
        }
        if (
          drivable_area && should_filter_out_by_lateral_distance(
                             *drivable_area, trajectory, object, LateralDistanceFilterParams{})) {
          return true;
        }
        return false;
      }),
    avoidance_targets.objects.end());

  return avoidance_targets;
}

}  // namespace autoware::avoidance_target_detector
