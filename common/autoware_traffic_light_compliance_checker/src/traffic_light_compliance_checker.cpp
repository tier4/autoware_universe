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

#include "autoware/traffic_light_compliance_checker/traffic_light_compliance_checker.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/duration.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
using autoware::traffic_light_compliance_checker::PreparedTrajectory;
using autoware::traffic_light_compliance_checker::RouteTrafficLightIndex;
using autoware::traffic_light_compliance_checker::StopLineInfo;

/// @brief map each traffic light of the route to the route lanelet that holds its stop line
RouteTrafficLightIndex build_route_traffic_light_index(
  const lanelet::LaneletMap & lanelet_map, const autoware_planning_msgs::msg::LaneletRoute & route)
{
  RouteTrafficLightIndex route_lanelet_id_per_traffic_light_id;
  for (const auto & segment : route.segments) {
    for (const auto & tl : lanelet_map.laneletLayer.get(segment.preferred_primitive.id)
                             .regulatoryElementsAs<lanelet::TrafficLight>()) {
      route_lanelet_id_per_traffic_light_id.emplace(tl->id(), segment.preferred_primitive.id);
    }
  }
  return route_lanelet_id_per_traffic_light_id;
}

/// @brief get stop lines where ego need to stop, and their corresponding signals from the given
/// traffic light groups
/// @param route_traffic_light_index route lanelet id per traffic light id, from
/// build_route_traffic_light_index()
std::vector<std::pair<StopLineInfo, autoware_perception_msgs::msg::TrafficLightGroup>>
collect_stop_lines(
  const lanelet::LaneletMap & lanelet_map, const RouteTrafficLightIndex & route_traffic_light_index,
  const std::vector<autoware_perception_msgs::msg::TrafficLightGroup> & traffic_light_groups,
  const bool collect_traffic_stop_signals = true)
{
  std::vector<std::pair<StopLineInfo, autoware_perception_msgs::msg::TrafficLightGroup>> stop_lines;
  for (const auto & signal : traffic_light_groups) {
    const auto hit = route_traffic_light_index.find(signal.traffic_light_group_id);
    if (hit == route_traffic_light_index.end()) {
      continue;
    }
    const auto traffic_light_it =
      lanelet_map.regulatoryElementLayer.find(signal.traffic_light_group_id);
    if (traffic_light_it == lanelet_map.regulatoryElementLayer.end()) {
      continue;
    }

    const auto route_stop_line_lanelet = lanelet_map.laneletLayer.get(hit->second);
    const bool is_stop_signal =
      autoware::traffic_light_utils::isTrafficSignalStop(route_stop_line_lanelet, signal);
    if (collect_traffic_stop_signals != is_stop_signal) continue;

    const auto traffic_light =
      std::dynamic_pointer_cast<const lanelet::TrafficLight>(*traffic_light_it);
    if (!traffic_light || !traffic_light->stopLine().has_value()) {
      continue;
    }

    stop_lines.emplace_back(
      StopLineInfo{
        lanelet::utils::to2D(traffic_light->stopLine()->basicLineString()),
        signal.traffic_light_group_id, hit->second},
      signal);
  }
  return stop_lines;
}

/// @brief prepare the part of the trajectory to check against stop lines
/// @param max_length stop the scan after this arc length, or scan the full trajectory if not set
/// @return nullopt if the prepared trajectory has less than two points, as it cannot cross a stop
/// line
std::optional<PreparedTrajectory> prepare_trajectory(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & input_trajectory,
  const std::optional<double> max_length, const double stopped_velocity_threshold,
  const double max_longitudinal_offset)
{
  if (input_trajectory.empty()) {
    return std::nullopt;
  }

  PreparedTrajectory prepared_trajectory;
  auto length = 0.0;
  auto last_p = input_trajectory.front();
  for (const auto & p : input_trajectory) {
    // skip points behind ego
    if (rclcpp::Duration(p.time_from_start).seconds() < 0.0) {
      prepared_trajectory.backward_length +=
        autoware_utils_geometry::calc_distance2d(last_p.pose, p.pose);
      last_p = p;
      continue;
    }

    const lanelet::BasicPoint2d lanelet_p(p.pose.position.x, p.pose.position.y);
    if (!prepared_trajectory.linestring.empty())
      length += lanelet::geometry::distance2d(prepared_trajectory.linestring.back(), lanelet_p);

    prepared_trajectory.points.push_back(p);
    prepared_trajectory.linestring.emplace_back(lanelet_p);

    // search for a stop point beyond the current ego position
    if (length > 0.0 && p.longitudinal_velocity_mps <= stopped_velocity_threshold) {
      prepared_trajectory.stop_point = prepared_trajectory.linestring.back();
      break;
    }
    if (max_length.has_value() && length > *max_length) break;
  }

  if (prepared_trajectory.linestring.size() < 2) {
    return std::nullopt;
  }

  if (max_longitudinal_offset > 0.0) {
    // extend the trajectory linestring by the vehicle's longitudinal offset
    const auto offset_pose = autoware_utils_geometry::calc_offset_pose(
      prepared_trajectory.points.back().pose, max_longitudinal_offset, 0.0, 0.0);
    const lanelet::BasicPoint2d offset_point(offset_pose.position.x, offset_pose.position.y);
    prepared_trajectory.linestring.emplace_back(offset_point);
    if (prepared_trajectory.stop_point.has_value())
      prepared_trajectory.stop_point.value() = offset_point;
  }
  return prepared_trajectory;
}

/// @brief return true if a prediction turns the signal to stop less than time_limit seconds from
/// now. Predictions in the past are ignored, because they come from a stale feed or an unset stamp.
bool turns_red_within(
  const std::vector<autoware_perception_msgs::msg::PredictedTrafficLightState> & predictions,
  const lanelet::ConstLanelet & lanelet, const rclcpp::Time & current_time, const double time_limit)
{
  // tolerance for small clock differences between the V2I source and ego [s]
  constexpr double past_prediction_tolerance = 0.5;
  return std::any_of(predictions.begin(), predictions.end(), [&](const auto & prediction) {
    if (!autoware::traffic_light_utils::isTrafficSignalStop(
          lanelet, prediction.simultaneous_elements)) {
      return false;
    }
    const auto time_to_red = (rclcpp::Time(prediction.predicted_stamp) - current_time).seconds();
    if (time_to_red < -past_prediction_tolerance) {
      return false;
    }
    return time_to_red < time_limit;
  });
}
}  // namespace

namespace autoware::traffic_light_compliance_checker
{

TrafficLightComplianceChecker::TrafficLightComplianceChecker(
  const Parameters & parameters, const vehicle_info_utils::VehicleInfo & vehicle_info)
: params_(parameters),
  vehicle_info_(vehicle_info),
  status_tracker_(std::make_unique<TrafficLightStatusTracker>(parameters.status_tracker_parameters))
{
}

TrafficLightComplianceChecker::~TrafficLightComplianceChecker() = default;

void TrafficLightComplianceChecker::update_parameters(const Parameters & parameters)
{
  params_ = parameters;
  status_tracker_->update_parameters(parameters.status_tracker_parameters);
}

tl::expected<ComplianceResult, std::string> TrafficLightComplianceChecker::check(
  const Inputs & input, const bool check_red_lights, const bool check_amber_lights,
  const bool use_v2i_remaining_time)
{
  if (input.map == nullptr) {
    return tl::make_unexpected("Lanelet map is not set");
  }

  const auto route_traffic_light_index = build_route_traffic_light_index(*input.map, input.route);

  const bool is_ego_stopped =
    std::abs(input.current_velocity) < params_.ego_stopped_velocity_threshold;
  const auto filtered_signals =
    status_tracker_->filter_signals(input.signals, input.current_time, is_ego_stopped);

  const auto force_reject_amber_ids =
    get_force_reject_amber_ids(input.current_time, is_ego_stopped);

  ego_stopping_distance_ = autoware::motion_utils::calculate_stop_distance(
    input.current_velocity, input.current_acceleration,
    params_.checked_trajectory_length.deceleration_limit,
    params_.checked_trajectory_length.jerk_limit, params_.delay_response_time);

  auto result = check_with_filtered_signals(
    input, filtered_signals, route_traffic_light_index, force_reject_amber_ids, check_red_lights,
    check_amber_lights);

  const auto v2i_result =
    handle_v2i(input, filtered_signals, route_traffic_light_index, use_v2i_remaining_time);
  result.violations.insert(
    result.violations.end(), v2i_result.violations.begin(), v2i_result.violations.end());

  cleanup_amber_rejection_history(input.current_time);

  return result;
}

std::vector<int64_t> TrafficLightComplianceChecker::get_force_reject_amber_ids(
  const rclcpp::Time & current_time, const bool is_ego_stopped) const
{
  std::vector<int64_t> force_reject_amber_ids;
  if (is_ego_stopped) {
    return force_reject_amber_ids;
  }
  for (const auto & [id, rejected_time] : amber_rejection_history_) {
    if ((current_time - rejected_time).seconds() <= params_.amber_rejection.hysteresis_duration) {
      force_reject_amber_ids.push_back(id);
    }
  }
  return force_reject_amber_ids;
}

void TrafficLightComplianceChecker::cleanup_amber_rejection_history(
  const rclcpp::Time & current_time)
{
  for (auto it = amber_rejection_history_.begin(); it != amber_rejection_history_.end();) {
    if ((current_time - it->second).seconds() > params_.amber_rejection.hysteresis_duration) {
      it = amber_rejection_history_.erase(it);
    } else {
      ++it;
    }
  }
}

Violations TrafficLightComplianceChecker::get_red_light_violations(
  const std::vector<StopLineInfo> & red_stop_lines,
  const lanelet::BasicLineString2d & trajectory_ls,
  const std::optional<lanelet::BasicPoint2d> & stop_point, const double distance_offset) const
{
  Violations violations;
  for (const auto & red_stop_line : red_stop_lines) {
    auto distance_to_stop_line = 0.0;
    lanelet::BasicPoints2d intersection_points;
    for (size_t i = 0; i + 1 < trajectory_ls.size(); ++i) {
      const lanelet::BasicLineString2d segment{trajectory_ls[i], trajectory_ls[i + 1]};
      boost::geometry::intersection(segment, red_stop_line.line, intersection_points);
      if (!intersection_points.empty()) {
        distance_to_stop_line += static_cast<double>(
          boost::geometry::distance(segment.front(), intersection_points.front()));
        break;
      }
      distance_to_stop_line += static_cast<double>(boost::geometry::length(segment));
    }
    if (
      intersection_points.empty() ||
      is_stop_point_within_margin_from_stop_line(stop_point, red_stop_line.line) ||
      is_allow_if_cannot_stop(distance_to_stop_line))
      continue;

    violations.emplace_back(
      ViolationType::RED_LIGHT, red_stop_line.line, red_stop_line.traffic_light_id,
      intersection_points.front(), distance_to_stop_line + distance_offset);
  }
  return violations;
}

Violations TrafficLightComplianceChecker::get_amber_light_violations(
  const std::vector<StopLineInfo> & amber_stop_lines,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const lanelet::BasicLineString2d & trajectory_ls,
  const std::optional<lanelet::BasicPoint2d> & stop_point,
  const std::vector<int64_t> & force_reject_amber_ids, const rclcpp::Time & current_time,
  const double distance_offset) const
{
  Violations violations;
  for (const auto & amber_stop_line : amber_stop_lines) {
    auto distance_to_stop_line = 0.0;
    std::optional<double> amber_stop_line_crossing_time;
    lanelet::BasicPoint2d intersection_point;
    for (size_t i = 0; i + 1 < trajectory.size(); ++i) {
      lanelet::BasicPoints2d intersection_points;
      const lanelet::BasicLineString2d segment{trajectory_ls[i], trajectory_ls[i + 1]};
      const auto segment_length = static_cast<double>(boost::geometry::length(segment));
      boost::geometry::intersection(segment, amber_stop_line.line, intersection_points);
      if (intersection_points.empty()) {
        distance_to_stop_line += segment_length;
        continue;
      }
      const auto distance_to_intersection =
        boost::geometry::distance(segment.front(), intersection_points.front());
      distance_to_stop_line += distance_to_intersection;
      const auto ratio = distance_to_intersection / segment_length;
      amber_stop_line_crossing_time = autoware::interpolation::lerp(
        rclcpp::Duration(trajectory[i].time_from_start).seconds(),
        rclcpp::Duration(trajectory[i + 1].time_from_start).seconds(), ratio);
      intersection_point = intersection_points.front();
      break;
    }

    if (
      !amber_stop_line_crossing_time ||
      is_stop_point_within_margin_from_stop_line(stop_point, amber_stop_line.line)) {
      if (stop_point.has_value() && params_.amber_rejection.reject_if_stop_detected)
        amber_rejection_history_[amber_stop_line.traffic_light_id] = current_time;
      continue;
    }

    if (is_allow_if_cannot_stop(distance_to_stop_line)) {
      continue;
    }

    const auto current_velocity = trajectory.front().longitudinal_velocity_mps;
    const auto current_acceleration = trajectory.front().acceleration_mps2;

    bool is_force_reject = std::find(
                             force_reject_amber_ids.begin(), force_reject_amber_ids.end(),
                             amber_stop_line.traffic_light_id) != force_reject_amber_ids.end();

    bool can_pass = can_pass_amber_light(
      amber_stop_line.traffic_light_id, distance_to_stop_line, current_velocity,
      current_acceleration, *amber_stop_line_crossing_time);

    if (!is_force_reject && can_pass) continue;

    if (!can_pass) amber_rejection_history_[amber_stop_line.traffic_light_id] = current_time;

    violations.emplace_back(
      ViolationType::AMBER_LIGHT, amber_stop_line.line, amber_stop_line.traffic_light_id,
      intersection_point, distance_to_stop_line + distance_offset);
  }
  return violations;
}

ComplianceResult TrafficLightComplianceChecker::check_with_filtered_signals(
  const Inputs & input,
  const autoware_perception_msgs::msg::TrafficLightGroupArray & filtered_signals,
  const RouteTrafficLightIndex & route_traffic_light_index,
  const std::vector<int64_t> & force_reject_amber_ids, const bool check_red_lights,
  const bool check_amber_lights) const
{
  if (input.trajectory.empty() || (!check_red_lights && !check_amber_lights)) {
    return ComplianceResult{};
  }

  // Floor by min_lookahead_distance so low ego speed still covers nearby stop lines,
  // while keeping the comfortable-stop cap so far lights are not over-checked
  // (important for traffic_light_filter which rejects trajectories).
  // stop_overshoot_margin extends the scan so a stop just past the line is not truncated.
  const auto max_trajectory_length = std::max(
    params_.min_lookahead_distance,
    ego_stopping_distance_.value_or(0.0) + params_.stop_overshoot_margin);
  const auto prepared_trajectory = prepare_trajectory(
    input.trajectory, max_trajectory_length, params_.ego_stopped_velocity_threshold,
    vehicle_info_.max_longitudinal_offset_m);
  if (!prepared_trajectory) {
    return ComplianceResult{};  // allow empty or stopped trajectories as they do not cross traffic
                                // lights
  }

  const auto [red_stop_lines, amber_stop_lines] =
    get_stop_lines(*input.map, route_traffic_light_index, filtered_signals);

  ComplianceResult result;
  if (check_red_lights) {
    result.violations = get_red_light_violations(
      red_stop_lines, prepared_trajectory->linestring, prepared_trajectory->stop_point,
      prepared_trajectory->backward_length);
  }
  if (check_amber_lights) {
    const auto amber_light_violations = get_amber_light_violations(
      amber_stop_lines, prepared_trajectory->points, prepared_trajectory->linestring,
      prepared_trajectory->stop_point, force_reject_amber_ids, input.current_time,
      prepared_trajectory->backward_length);
    result.violations.insert(
      result.violations.end(), amber_light_violations.begin(), amber_light_violations.end());
  }

  return result;
}

std::pair<std::vector<StopLineInfo>, std::vector<StopLineInfo>>
TrafficLightComplianceChecker::get_stop_lines(
  const lanelet::LaneletMap & lanelet_map, const RouteTrafficLightIndex & route_traffic_light_index,
  const autoware_perception_msgs::msg::TrafficLightGroupArray & traffic_lights) const
{
  std::vector<StopLineInfo> red_stop_lines;
  std::vector<StopLineInfo> amber_stop_lines;
  for (const auto & [stop_line_info, signal] : collect_stop_lines(
         lanelet_map, route_traffic_light_index, traffic_lights.traffic_light_groups)) {
    const bool is_red = autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
      signal.elements, autoware_perception_msgs::msg::TrafficLightElement::CIRCLE,
      autoware_perception_msgs::msg::TrafficLightElement::RED);
    const bool is_amber = autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
      signal.elements, autoware_perception_msgs::msg::TrafficLightElement::CIRCLE,
      autoware_perception_msgs::msg::TrafficLightElement::AMBER);
    const bool is_unknown = autoware::traffic_light_utils::hasTrafficLightColor(
      signal.elements, autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN);

    const auto is_treated_as_red = is_red ||
                                   (is_unknown && params_.treat_unknown_light_as_red_light) ||
                                   (is_amber && params_.treat_amber_light_as_red_light);
    if (is_treated_as_red) {
      red_stop_lines.push_back(stop_line_info);
    } else if (is_amber) {
      amber_stop_lines.push_back(stop_line_info);
    }
  }
  return {red_stop_lines, amber_stop_lines};
}

bool TrafficLightComplianceChecker::is_stop_point_within_margin_from_stop_line(
  const std::optional<lanelet::BasicPoint2d> & stop_point,
  const lanelet::BasicLineString2d & stop_line) const
{
  if (stop_point.has_value()) {
    if (boost::geometry::distance(*stop_point, stop_line) <= params_.stop_overshoot_margin) {
      return true;
    }
  }
  return false;
}

bool TrafficLightComplianceChecker::can_pass_amber_light(
  const int64_t traffic_light_id, const double distance_to_stop_line, const double current_velocity,
  const double current_acceleration, const double time_to_cross_stop_line) const
{
  const double decel_limit = params_.deceleration_limit;
  const double jerk_limit = params_.jerk_limit;
  const double delay_response_time = params_.delay_response_time;
  const auto distance_for_ego_to_stop = autoware::motion_utils::calculate_stop_distance(
    current_velocity, current_acceleration, decel_limit, jerk_limit, delay_response_time);

  const auto crossing_time_limit =
    std::max(0.0, params_.crossing_time_limit - status_tracker_->get_duration(traffic_light_id));

  const bool can_stop =
    distance_for_ego_to_stop.has_value() && *distance_for_ego_to_stop <= distance_to_stop_line;
  const bool can_pass_in_time = time_to_cross_stop_line <= crossing_time_limit;
  const bool can_pass = !can_stop && can_pass_in_time;
  return can_pass;
}

bool TrafficLightComplianceChecker::is_allow_if_cannot_stop(
  const double distance_to_cross_point) const
{
  if (!ego_stopping_distance_.has_value() || params_.allow_if_cannot_stop_distance < 1e-3)
    return false;
  const auto distance_from_ego_front =
    distance_to_cross_point - vehicle_info_.max_longitudinal_offset_m;
  return distance_from_ego_front < params_.allow_if_cannot_stop_distance &&
         distance_from_ego_front < *ego_stopping_distance_ - params_.stop_overshoot_margin;
}

ComplianceResult TrafficLightComplianceChecker::handle_v2i(
  const Inputs & input,
  const autoware_perception_msgs::msg::TrafficLightGroupArray & filtered_signals,
  const RouteTrafficLightIndex & route_traffic_light_index, const bool use_v2i_remaining_time) const
{
  if (!use_v2i_remaining_time) {
    return ComplianceResult{};
  }

  // scan the full trajectory, as a V2I prediction can concern a stop line far ahead of ego
  const auto prepared = prepare_trajectory(
    input.trajectory, std::nullopt, params_.ego_stopped_velocity_threshold,
    vehicle_info_.max_longitudinal_offset_m);
  if (!prepared) {
    return ComplianceResult{};
  }
  const auto & trajectory = prepared->points;
  const auto & trajectory_ls = prepared->linestring;
  const auto & v2i_params = params_.v2i_handling;

  ComplianceResult result;
  for (const auto & [stop_line, traffic_signal] : collect_stop_lines(
         *input.map, route_traffic_light_index, filtered_signals.traffic_light_groups, false)) {
    auto distance_to_stop_line = 0.0;
    std::optional<double> stop_line_crossing_time;
    lanelet::BasicPoint2d intersection_point;
    for (size_t i = 0; i + 1 < trajectory_ls.size(); ++i) {
      lanelet::BasicPoints2d intersection_points;
      const lanelet::BasicLineString2d segment{trajectory_ls[i], trajectory_ls[i + 1]};
      const auto segment_length = static_cast<double>(boost::geometry::length(segment));
      boost::geometry::intersection(segment, stop_line.line, intersection_points);
      if (intersection_points.empty()) {
        distance_to_stop_line += segment_length;
        continue;
      }
      const auto distance_to_intersection =
        boost::geometry::distance(segment.front(), intersection_points.front());
      distance_to_stop_line += distance_to_intersection;
      const auto ratio = distance_to_intersection / segment_length;
      // the last segment models the vehicle body at the last trajectory point. it has no time of
      // its own, so use the time of that point.
      const auto next_index = std::min(i + 1, trajectory.size() - 1);
      stop_line_crossing_time = autoware::interpolation::lerp(
        rclcpp::Duration(trajectory[i].time_from_start).seconds(),
        rclcpp::Duration(trajectory[next_index].time_from_start).seconds(), ratio);
      intersection_point = intersection_points.front();
      break;
    }

    if (
      !stop_line_crossing_time ||
      is_stop_point_within_margin_from_stop_line(prepared->stop_point, stop_line.line) ||
      is_allow_if_cannot_stop(distance_to_stop_line)) {
      continue;
    }

    // a slow ego needs a fixed departure time instead of the trajectory crossing time
    const double time_to_cross_stop_line = input.current_velocity > v2i_params.velocity_threshold
                                             ? *stop_line_crossing_time
                                             : v2i_params.required_time_to_departure;

    const auto stop_lanelet = input.map->laneletLayer.get(stop_line.traffic_light_lanelet_id);
    const double last_time_allowed_to_pass =
      v2i_params.get_last_time_allowed_to_pass_from_map
        ? stop_lanelet.attributeOr(
            "v2i_last_time_allowed_to_pass", v2i_params.last_time_allowed_to_pass)
        : v2i_params.last_time_allowed_to_pass;
    const double required_time_to_pass = time_to_cross_stop_line + last_time_allowed_to_pass;

    if (!turns_red_within(
          traffic_signal.predictions, stop_lanelet, input.current_time, required_time_to_pass)) {
      continue;
    }

    result.violations.emplace_back(
      ViolationType::V2I, stop_line.line, stop_line.traffic_light_id, intersection_point,
      distance_to_stop_line + prepared->backward_length);
  }
  return result;
}

}  // namespace autoware::traffic_light_compliance_checker
