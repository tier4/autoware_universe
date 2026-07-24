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

#include "autoware/trajectory_validator/filters/safety/trajectory_feasibility_filter.hpp"

#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <builtin_interfaces/msg/duration.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <angles/angles/angles.h>
#include <tf2/utils.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
namespace
{
/**
 * @brief Convert a Duration message to seconds.
 */
double to_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}

/**
 * @brief Convert TrajectoryPoint to speed (m/s)
 */
double to_speed(const TrajectoryPoint & point)
{
  return std::sqrt(
    point.longitudinal_velocity_mps * point.longitudinal_velocity_mps +
    point.lateral_velocity_mps * point.lateral_velocity_mps);
}

/**
 * @brief Convert TrajectoryPoints to steering angle (rad)
 */
double to_steering_angle(
  const TrajectoryPoint & prev_point, const TrajectoryPoint & curr_point,
  const TrajectoryPoint & next_point, const VehicleInfo & vehicle_info)
{
  const auto & prev_p = prev_point.pose.position;
  const auto & curr_p = curr_point.pose.position;
  const auto & next_p = next_point.pose.position;

  try {
    const double curvature = autoware_utils_geometry::calc_curvature(prev_p, curr_p, next_p);
    return std::atan(vehicle_info.wheel_base_m * curvature);
  } catch (...) {
    return 0.0;  // throw exception if three points are too close
  }
}

/**
 * @brief Convert TrajectoryPoints to a vector of steering angles (rad), with optional smoothing.
 */
std::vector<std::optional<double>> to_steering_angles(
  const TrajectoryPoints & traj_points, const VehicleInfo & vehicle_info, int smoothing_window_size)
{
  std::vector<std::optional<double>> steering_angles(traj_points.size(), std::nullopt);
  if (traj_points.size() < 3) {
    return steering_angles;
  }

  for (size_t i = 1; i + 1 < traj_points.size(); ++i) {
    steering_angles[i] =
      to_steering_angle(traj_points[i - 1], traj_points[i], traj_points[i + 1], vehicle_info);
  }

  if (smoothing_window_size < 1) {
    return steering_angles;
  }

  const size_t radius = static_cast<size_t>(std::max(1, smoothing_window_size) / 2);
  std::vector<std::optional<double>> smoothed_angles(traj_points.size(), std::nullopt);
  for (size_t i = radius; i < traj_points.size() - radius; ++i) {
    double sum = 0.0;
    size_t count = 0;
    const size_t start_index = (i > radius) ? i - radius : 1;
    const size_t end_index = std::min(traj_points.size() - 2, i + radius);
    for (size_t sample_index = start_index; sample_index <= end_index; ++sample_index) {
      if (!steering_angles[sample_index].has_value()) {
        continue;
      }
      sum += steering_angles[sample_index].value();
      ++count;
    }
    if (count > 0) {
      smoothed_angles[i] = sum / static_cast<double>(count);
    }
  }
  return smoothed_angles;
}

autoware_planning_msgs::msg::Trajectory to_trajectory(const TrajectoryPoints & traj_points)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.points = traj_points;
  return trajectory;
}

/**
 * @brief Convert a lanelet's speed limit attribute to m/s, if it exists.
 */
std::optional<double> to_lanelet_speed_limit_mps(const lanelet::ConstLanelet & lanelet)
{
  constexpr char attribute_name[] = "speed_limit";

  const auto result = lanelet.attribute(attribute_name).as<double>().map([](const auto v) {
    return autoware_utils_math::kmph2mps(v);
  });

  return result.has_value() ? std::make_optional(result.value()) : std::nullopt;
}

/**
 * @brief Find the speed limit of the nearest lanelet to a given pose, in m/s.
 */
std::optional<double> find_nearest_lanelet_speed_limit_mps(
  const lanelet::LaneletMap & lanelet_map, const geometry_msgs::msg::Pose & search_pose)
{
  const auto nearest_lanelets =
    autoware::experimental::lanelet2_utils::find_nearest(lanelet_map.laneletLayer, search_pose, 10);
  for (const auto & [_, nearest_lanelet] : nearest_lanelets) {
    if (const auto speed_limit_mps = to_lanelet_speed_limit_mps(nearest_lanelet)) {
      return speed_limit_mps;
    }
  }

  return std::nullopt;
}
}  // namespace

TrajectoryFeasibilityFilter::TrajectoryFeasibilityFilter()
: ValidatorInterface("trajectory_feasibility_filter")
{
}

void TrajectoryFeasibilityFilter::update_parameters(const validator::Params & params)
{
  params_ = params.trajectory_feasibility;
}

TrajectoryFeasibilityFilter::result_t TrajectoryFeasibilityFilter::is_feasible(
  const CandidateTrajectory & candidate_trajectory, const FilterContext & context)
{
  const auto & traj_points = candidate_trajectory.points;
  if (!vehicle_info_ptr_) {
    return tl::make_unexpected("Vehicle info not set");
  }

  // NOTE: Feasibility decision logic might be more complex in the future, but for now we just
  // check all constraints and return false if any are violated
  bool is_feasible = true;
  std::vector<MetricReport> metrics;
  for (const auto & checker : checkers_) {
    auto report = (this->*checker)(traj_points, context);
    is_feasible &= report.risk.level == RiskLevel::SAFE;
    metrics.push_back(report);
  }

  return ValidationResult{is_feasible, std::move(metrics)};
}

MetricReport TrajectoryFeasibilityFilter::check_speed(
  const TrajectoryPoints & traj_points, const FilterContext &) const
{
  const auto [max_observed, is_ok] = is_speed_ok(traj_points, params_.max_speed);

  RiskLevel risk_level;
  risk_level.level = is_ok ? RiskLevel::SAFE : RiskLevel::HIGH_CAUTION;
  return autoware_trajectory_validator::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("speed")
    .metric_value(max_observed)
    .risk(risk_level);
}

MetricReport TrajectoryFeasibilityFilter::check_lanelet_speed_limit(
  const TrajectoryPoints & traj_points, const FilterContext & context) const
{
  double observed_speed = 0.0;
  bool is_ok = true;

  if (!traj_points.empty() && context.odometry && context.lanelet_map) {
    const auto nearest_idx =
      autoware::motion_utils::findNearestIndex(traj_points, context.odometry->pose.pose.position);
    const auto & nearest_point = traj_points.at(nearest_idx);
    observed_speed = to_speed(nearest_point);

    if (
      const auto speed_limit_mps =
        find_nearest_lanelet_speed_limit_mps(*context.lanelet_map, nearest_point.pose)) {
      is_ok = observed_speed <= *speed_limit_mps;
    }
  }

  RiskLevel risk_level;
  risk_level.level = is_ok ? RiskLevel::SAFE : RiskLevel::HIGH_CAUTION;
  return autoware_trajectory_validator::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("lanelet_speed_limit")
    .metric_value(observed_speed)
    .risk(risk_level);
}

MetricReport TrajectoryFeasibilityFilter::check_acceleration(
  const TrajectoryPoints & traj_points, const FilterContext &) const
{
  const auto [max_observed, is_ok] = is_acceleration_ok(traj_points, params_.max_acceleration);

  RiskLevel risk_level;
  risk_level.level = is_ok ? RiskLevel::SAFE : RiskLevel::HIGH_CAUTION;
  return autoware_trajectory_validator::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("acceleration")
    .metric_value(max_observed)
    .risk(risk_level);
}

MetricReport TrajectoryFeasibilityFilter::check_deceleration(
  const TrajectoryPoints & traj_points, const FilterContext &) const
{
  const auto [max_observed, is_ok] = is_deceleration_ok(traj_points, params_.max_deceleration);

  RiskLevel risk_level;
  risk_level.level = is_ok ? RiskLevel::SAFE : RiskLevel::HIGH_CAUTION;
  return autoware_trajectory_validator::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("deceleration")
    .metric_value(max_observed)
    .risk(risk_level);
}

MetricReport TrajectoryFeasibilityFilter::check_distance_deviation(
  const TrajectoryPoints & traj_points, const FilterContext & context) const
{
  const auto [max_observed, is_ok] =
    is_distance_deviation_ok(traj_points, context, params_.max_distance_deviation);

  RiskLevel risk_level;
  risk_level.level = is_ok ? RiskLevel::SAFE : RiskLevel::HIGH_CAUTION;
  return autoware_trajectory_validator::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("distance_deviation")
    .metric_value(max_observed)
    .risk(risk_level);
}

MetricReport TrajectoryFeasibilityFilter::check_lateral_acceleration(
  const TrajectoryPoints & traj_points, const FilterContext &) const
{
  const auto [max_observed, is_ok] =
    is_lateral_acceleration_ok(traj_points, params_.max_lateral_acceleration);

  RiskLevel risk_level;
  risk_level.level = is_ok ? RiskLevel::SAFE : RiskLevel::HIGH_CAUTION;
  return autoware_trajectory_validator::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("lateral_acceleration")
    .metric_value(max_observed)
    .risk(risk_level);
}

MetricReport TrajectoryFeasibilityFilter::check_velocity_deviation(
  const TrajectoryPoints & traj_points, const FilterContext & context) const
{
  const auto [max_observed, is_ok] =
    is_velocity_deviation_ok(traj_points, context, params_.max_velocity_deviation);

  RiskLevel risk_level;
  risk_level.level = is_ok ? RiskLevel::SAFE : RiskLevel::HIGH_CAUTION;
  return autoware_trajectory_validator::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("velocity_deviation")
    .metric_value(max_observed)
    .risk(risk_level);
}

MetricReport TrajectoryFeasibilityFilter::check_yaw_deviation(
  const TrajectoryPoints & traj_points, const FilterContext & context) const
{
  const auto [max_observed, is_ok] =
    is_yaw_deviation_ok(traj_points, context, params_.max_yaw_deviation);

  RiskLevel risk_level;
  risk_level.level = is_ok ? RiskLevel::SAFE : RiskLevel::HIGH_CAUTION;
  return autoware_trajectory_validator::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("yaw_deviation")
    .metric_value(max_observed)
    .risk(risk_level);
}

MetricReport TrajectoryFeasibilityFilter::check_steering_angle(
  const TrajectoryPoints & traj_points, const FilterContext &) const
{
  const auto [max_observed, is_ok] =
    is_steering_angle_ok(traj_points, *vehicle_info_ptr_, params_.max_steering_angle);

  RiskLevel risk_level;
  risk_level.level = is_ok ? RiskLevel::SAFE : RiskLevel::HIGH_CAUTION;
  return autoware_trajectory_validator::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("steering_angle")
    .metric_value(max_observed)
    .risk(risk_level);
}

MetricReport TrajectoryFeasibilityFilter::check_steering_rate(
  const TrajectoryPoints & traj_points, const FilterContext &) const
{
  const auto [max_observed, is_ok] =
    is_steering_rate_ok(traj_points, *vehicle_info_ptr_, params_.max_steering_rate);

  RiskLevel risk_level;
  risk_level.level = is_ok ? RiskLevel::SAFE : RiskLevel::HIGH_CAUTION;
  return autoware_trajectory_validator::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("steering_rate")
    .metric_value(max_observed)
    .risk(risk_level);
}

// --- Helper functions for constraint checks ---

std::pair<double, bool> is_speed_ok(const TrajectoryPoints & traj_points, double max_speed)
{
  double max_observed = 0.0;
  bool is_ok = true;
  for (const auto & point : traj_points) {
    double speed = to_speed(point);
    if (speed > max_speed) {
      max_observed = std::max(max_observed, speed);
      is_ok = false;
    }
  }
  return {max_observed, is_ok};
}

std::pair<double, bool> is_acceleration_ok(
  const TrajectoryPoints & traj_points, double max_acceleration)
{
  double max_observed = 0.0;
  bool is_ok = true;
  for (const auto & point : traj_points) {
    const auto acc = static_cast<double>(point.acceleration_mps2);
    if (acc > 0 && acc > max_acceleration) {
      max_observed = std::max(max_observed, acc);
      is_ok = false;
    }
  }
  return {max_observed, is_ok};
}

std::pair<double, bool> is_deceleration_ok(
  const TrajectoryPoints & traj_points, double max_deceleration)
{
  double max_observed = 0.0;
  bool is_ok = true;
  for (const auto & point : traj_points) {
    const auto dec = static_cast<double>(point.acceleration_mps2);
    if (dec < 0 && std::abs(dec) > max_deceleration) {
      max_observed = std::max(max_observed, std::abs(dec));
      is_ok = false;
    }
  }
  return {max_observed, is_ok};
}

std::pair<double, bool> is_distance_deviation_ok(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  double max_distance_deviation)
{
  if (!context.odometry || traj_points.size() < 2) {
    return {0.0, true};
  }
  const auto nearest_idx = autoware::motion_utils::findNearestSegmentIndex(
    traj_points, context.odometry->pose.pose.position);
  const double distance_deviation = std::abs(
    autoware::motion_utils::calcLateralOffset(
      traj_points, context.odometry->pose.pose.position, nearest_idx));
  return {distance_deviation, distance_deviation <= max_distance_deviation};
}

std::pair<double, bool> is_lateral_acceleration_ok(
  const TrajectoryPoints & traj_points, double max_lateral_acceleration)
{
  double max_observed = 0.0;
  bool is_ok = true;
  if (traj_points.size() < 3) {
    return {max_observed, is_ok};
  }
  for (size_t i = 1; i + 1 < traj_points.size(); ++i) {
    const auto & prev_p = traj_points[i - 1].pose.position;
    const auto & curr_p = traj_points[i].pose.position;
    const auto & next_p = traj_points[i + 1].pose.position;
    try {
      const double curvature = autoware_utils_geometry::calc_curvature(prev_p, curr_p, next_p);
      const double longitudinal_velocity = traj_points[i].longitudinal_velocity_mps;
      const double lateral_acceleration =
        std::abs(longitudinal_velocity * longitudinal_velocity * curvature);
      if (lateral_acceleration > max_lateral_acceleration) {
        max_observed = std::max(max_observed, lateral_acceleration);
        is_ok = false;
      }
    } catch (...) {  // skip if three points are too close
      continue;
    }
  }
  return {max_observed, is_ok};
}

std::pair<double, bool> is_velocity_deviation_ok(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  double max_velocity_deviation)
{
  if (!context.odometry || traj_points.empty()) {
    return {0.0, true};
  }
  const auto nearest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    traj_points, context.odometry->pose.pose);
  const double ego_speed = context.odometry->twist.twist.linear.x;
  const double velocity_deviation =
    std::abs(traj_points.at(nearest_idx).longitudinal_velocity_mps - ego_speed);
  return {velocity_deviation, velocity_deviation <= max_velocity_deviation};
}

std::pair<double, bool> is_yaw_deviation_ok(
  const TrajectoryPoints & traj_points, const FilterContext & context, double max_yaw_deviation)
{
  if (!context.odometry || traj_points.empty()) {
    return {0.0, true};
  }
  const auto trajectory = to_trajectory(traj_points);
  const auto & ego_pose = context.odometry->pose.pose;
  const auto interpolated_trajectory_point =
    autoware::motion_utils::calcInterpolatedPoint(trajectory, ego_pose);
  const double yaw_deviation = std::abs(
    angles::shortest_angular_distance(
      tf2::getYaw(interpolated_trajectory_point.pose.orientation),
      tf2::getYaw(ego_pose.orientation)));
  return {yaw_deviation, yaw_deviation <= max_yaw_deviation};
}

std::pair<double, bool> is_steering_angle_ok(
  const TrajectoryPoints & traj_points, const VehicleInfo & vehicle_info, double max_steering_angle)
{
  double max_observed = 0.0;
  bool is_ok = true;
  constexpr int smoothing_window_size = 5;
  const auto steering_angles = to_steering_angles(traj_points, vehicle_info, smoothing_window_size);
  for (size_t i = 1; i + 1 < traj_points.size(); ++i) {
    if (!steering_angles[i].has_value()) {
      continue;
    }
    if (std::abs(steering_angles[i].value()) > max_steering_angle) {
      max_observed = std::max(max_observed, std::abs(steering_angles[i].value()));
      is_ok = false;
    }
  }
  return {max_observed, is_ok};
}

std::pair<double, bool> is_steering_rate_ok(
  const TrajectoryPoints & traj_points, const VehicleInfo & vehicle_info, double max_steering_rate)
{
  double max_observed = 0.0;
  bool is_ok = true;
  constexpr int smoothing_window_size = 5;
  const auto steering_angles = to_steering_angles(traj_points, vehicle_info, smoothing_window_size);
  for (size_t i = 2; i + 1 < traj_points.size(); ++i) {
    if (!steering_angles[i].has_value() || !steering_angles[i - 1].has_value()) {
      continue;
    }
    const double dt =
      to_seconds(traj_points[i].time_from_start) - to_seconds(traj_points[i - 1].time_from_start);
    const auto steering_rate =
      dt > 0.0 ? std::abs(steering_angles[i].value() - steering_angles[i - 1].value()) / dt : 0.0;
    if (steering_rate > max_steering_rate) {
      max_observed = std::max(max_observed, steering_rate);
      is_ok = false;
    }
  }
  return {max_observed, is_ok};
}
}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::TrajectoryFeasibilityFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
