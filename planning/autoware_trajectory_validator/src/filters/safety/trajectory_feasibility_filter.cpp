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

#include <autoware/lanelet2_utils/kind.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <builtin_interfaces/msg/duration.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <angles/angles.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <optional>
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
 * @brief Minimum arc length between the three points used to estimate the curvature (m).
 *
 * Using immediately adjacent points is numerically unstable when the trajectory is densely sampled
 * (e.g. right after the vehicle starts moving, where the point interval can be a few centimeters):
 * a sub-millimeter lateral jitter then yields a huge curvature. Spreading the three points over a
 * fixed arc length makes the estimate independent of the sampling interval.
 */
constexpr double curvature_distance = 1.0;

/**
 * @brief Curvature at a trajectory point together with the indices of the two points it was
 * estimated from.
 */
struct CurvatureSample
{
  double curvature;   //!< Curvature at the point (1/m)
  size_t prev_index;  //!< Index of the preceding point used for the estimate
  size_t next_index;  //!< Index of the following point used for the estimate
};

/**
 * @brief Estimate the curvature at each trajectory point from three points that are at least
 * `curvature_distance` apart in arc length.
 *
 * Points near the ends of the trajectory that have no sufficiently distant neighbor reuse the
 * sample of the nearest point that does. If no point satisfies the distance requirement (the
 * trajectory is shorter than `2 * curvature_distance`), every sample is `std::nullopt`.
 */
std::vector<std::optional<CurvatureSample>> to_curvature_samples(
  const TrajectoryPoints & traj_points)
{
  std::vector<std::optional<CurvatureSample>> samples(traj_points.size(), std::nullopt);
  if (traj_points.size() < 3) {
    return samples;
  }

  std::vector<double> arc_length(traj_points.size(), 0.0);
  for (size_t i = 1; i < traj_points.size(); ++i) {
    arc_length[i] =
      arc_length[i - 1] + autoware_utils_geometry::calc_distance2d(
                            traj_points[i - 1].pose.position, traj_points[i].pose.position);
  }

  std::optional<size_t> first_valid_index;
  std::optional<size_t> last_valid_index;
  for (size_t i = 1; i + 1 < traj_points.size(); ++i) {
    std::optional<size_t> prev_index;
    for (size_t j = i; j-- > 0;) {
      if (arc_length[i] - arc_length[j] >= curvature_distance) {
        prev_index = j;
        break;
      }
    }
    if (!prev_index) {
      continue;
    }

    std::optional<size_t> next_index;
    for (size_t j = i + 1; j < traj_points.size(); ++j) {
      if (arc_length[j] - arc_length[i] >= curvature_distance) {
        next_index = j;
        break;
      }
    }
    if (!next_index) {
      break;  // no later point can satisfy the requirement either
    }

    double curvature = 0.0;
    try {
      curvature = autoware_utils_geometry::calc_curvature(
        traj_points[*prev_index].pose.position, traj_points[i].pose.position,
        traj_points[*next_index].pose.position);
    } catch (...) {
      curvature = 0.0;  // points are too close, treat as straight
    }
    samples[i] = CurvatureSample{curvature, *prev_index, *next_index};
    if (!first_valid_index) {
      first_valid_index = i;
    }
    last_valid_index = i;
  }

  if (!first_valid_index) {
    return samples;
  }

  // Extend the first/last valid sample to the points where the distance is not enough.
  for (size_t i = 0; i < *first_valid_index; ++i) {
    samples[i] = samples[*first_valid_index];
  }
  for (size_t i = *last_valid_index + 1; i < traj_points.size(); ++i) {
    samples[i] = samples[*last_valid_index];
  }
  return samples;
}

/**
 * @brief Convert curvature to the front-wheel steering angle (rad) with the bicycle model.
 */
double to_steering_angle(const double curvature, const VehicleInfo & vehicle_info)
{
  return std::atan(vehicle_info.wheel_base_m * curvature);
}

/**
 * @brief Convert a lanelet's speed limit attribute to m/s, if it exists.
 */
std::optional<double> to_lanelet_speed_limit_mps(const lanelet::ConstLanelet & lanelet)
{
  constexpr char attribute_name[] = "speed_limit";

  // NOTE: `attribute()` throws NoSuchAttributeError if the attribute is not present.
  if (!lanelet.hasAttribute(attribute_name)) {
    return std::nullopt;
  }

  const auto result = lanelet.attribute(attribute_name).as<double>().map([](const auto v) {
    return autoware_utils_math::kmph2mps(v);
  });

  return result.has_value() ? std::make_optional(result.value()) : std::nullopt;
}

/**
 * @brief Find the maximum speed limit from the nearest lanelets to a given pose, in m/s.
 * @note By searching for lanelets within a 0.5 meter radius, we only consider lanelets that are
 * likely to be so close to the search pose.
 */
std::optional<double> find_nearest_lanelet_speed_limit_mps(
  const lanelet::LaneletMap & lanelet_map, const geometry_msgs::msg::Pose & search_pose)
{
  constexpr size_t max_search_count = 10;
  constexpr double search_r_range = 0.5;
  constexpr double search_z_range = 2.0;

  const auto nearest_lanelets = autoware::experimental::lanelet2_utils::find_nearest(
    lanelet_map.laneletLayer, search_pose, max_search_count, search_r_range, search_z_range);

  std::optional<double> max_speed_limit_mps;
  for (const auto & [_, nearest_lanelet] : nearest_lanelets) {
    if (!autoware::experimental::lanelet2_utils::is_road_lane(nearest_lanelet)) {
      continue;
    }

    if (const auto speed_limit_mps = to_lanelet_speed_limit_mps(nearest_lanelet)) {
      max_speed_limit_mps = max_speed_limit_mps.has_value()
                              ? std::max(max_speed_limit_mps.value(), speed_limit_mps.value())
                              : speed_limit_mps;
    }
  }

  return max_speed_limit_mps;
}

autoware_planning_msgs::msg::Trajectory to_trajectory(const TrajectoryPoints & traj_points)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.points = traj_points;
  return trajectory;
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

  // Each checker reports its own risk level. The core derives feasibility from the worst risk
  // level of these metrics, so a violated constraint reports HIGH_CAUTION and does not reject the
  // trajectory on its own.
  std::vector<MetricReport> metrics;
  for (const auto & checker : checkers_) {
    auto report = (this->*checker)(traj_points, context);
    metrics.push_back(report);
  }

  return ValidationResult{std::move(metrics)};
}

MetricReport TrajectoryFeasibilityFilter::check_speed(
  const TrajectoryPoints & traj_points, const FilterContext &) const
{
  const auto [max_observed, is_ok] = is_speed_ok(traj_points, params_.max_speed);

  RiskLevel risk_level;
  risk_level.level = is_ok ? RiskLevel::SAFE : RiskLevel::HIGH_CAUTION;
  return autoware_internal_planning_msgs::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("speed")
    .metric_value(max_observed)
    .risk(risk_level);
}

MetricReport TrajectoryFeasibilityFilter::check_lanelet_speed_limit(
  const TrajectoryPoints & traj_points, const FilterContext & context) const
{
  const auto [observed_speed, is_ok] = is_lanelet_speed_limit_ok(traj_points, context);

  RiskLevel risk_level;
  risk_level.level = is_ok ? RiskLevel::SAFE : RiskLevel::HIGH_CAUTION;
  return autoware_internal_planning_msgs::build<MetricReport>()
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
  return autoware_internal_planning_msgs::build<MetricReport>()
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
  return autoware_internal_planning_msgs::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("deceleration")
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
  return autoware_internal_planning_msgs::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("yaw_deviation")
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
  return autoware_internal_planning_msgs::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("velocity_deviation")
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
  return autoware_internal_planning_msgs::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("lateral_acceleration")
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
  return autoware_internal_planning_msgs::build<MetricReport>()
    .validator_name(get_name())
    .validator_category(category())
    .metric_name("distance_deviation")
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
  return autoware_internal_planning_msgs::build<MetricReport>()
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
  return autoware_internal_planning_msgs::build<MetricReport>()
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

std::pair<double, bool> is_lanelet_speed_limit_ok(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  if (traj_points.empty() || !context.odometry || !context.lanelet_map) {
    return {0.0, true};
  }

  const auto nearest_idx =
    autoware::motion_utils::findNearestIndex(traj_points, context.odometry->pose.pose.position);
  const auto & nearest_point = traj_points.at(nearest_idx);
  const double observed_speed = to_speed(nearest_point);

  const auto speed_limit_mps =
    find_nearest_lanelet_speed_limit_mps(*context.lanelet_map, nearest_point.pose);
  if (!speed_limit_mps) {
    return {observed_speed, true};
  }

  return {observed_speed, observed_speed <= *speed_limit_mps};
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

std::pair<double, bool> is_lateral_acceleration_ok(
  const TrajectoryPoints & traj_points, double max_lateral_acceleration)
{
  double max_observed = 0.0;
  bool is_ok = true;

  const auto samples = to_curvature_samples(traj_points);
  for (size_t i = 0; i < traj_points.size(); ++i) {
    if (!samples[i].has_value()) {
      continue;
    }
    const double longitudinal_velocity = traj_points[i].longitudinal_velocity_mps;
    const double lateral_acceleration =
      std::abs(longitudinal_velocity * longitudinal_velocity * samples[i]->curvature);
    if (lateral_acceleration > max_lateral_acceleration) {
      max_observed = std::max(max_observed, lateral_acceleration);
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

std::pair<double, bool> is_steering_angle_ok(
  const TrajectoryPoints & traj_points, const VehicleInfo & vehicle_info, double max_steering_angle)
{
  double max_observed = 0.0;
  bool is_ok = true;

  const auto samples = to_curvature_samples(traj_points);
  for (const auto & sample : samples) {
    if (!sample.has_value()) {
      continue;
    }
    const double steering_angle = std::abs(to_steering_angle(sample->curvature, vehicle_info));
    if (steering_angle > max_steering_angle) {
      max_observed = std::max(max_observed, steering_angle);
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

  // The steering rate at a point is the change of steering angle between the two points used to
  // estimate its curvature, divided by the time between them. Taking the difference over this arc
  // length span (instead of between adjacent points) keeps the rate from blowing up where the
  // trajectory is densely sampled and the time between adjacent points is tiny.
  const auto samples = to_curvature_samples(traj_points);
  for (const auto & sample : samples) {
    if (!sample.has_value()) {
      continue;
    }
    const auto & prev_sample = samples[sample->prev_index];
    const auto & next_sample = samples[sample->next_index];
    if (!prev_sample.has_value() || !next_sample.has_value()) {
      continue;
    }
    const double dt = to_seconds(traj_points[sample->next_index].time_from_start) -
                      to_seconds(traj_points[sample->prev_index].time_from_start);
    if (dt <= 0.0) {
      continue;
    }
    const double steering_rate = std::abs(
                                   to_steering_angle(next_sample->curvature, vehicle_info) -
                                   to_steering_angle(prev_sample->curvature, vehicle_info)) /
                                 dt;
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
