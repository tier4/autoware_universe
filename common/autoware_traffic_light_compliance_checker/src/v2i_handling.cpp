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

// V2I handling of the traffic light compliance checker, ported from v4.4.0.1-hotfix (#3499).
// It is kept in its own file so that the rest of the checker stays identical to feat/v0.64/e2e.

#include "autoware/traffic_light_compliance_checker/traffic_light_compliance_checker.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/duration.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <optional>
#include <unordered_map>
#include <vector>

namespace
{
using autoware::traffic_light_compliance_checker::StopLineInfo;
using autoware_perception_msgs::msg::PredictedTrafficLightState;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_planning_msgs::msg::TrajectoryPoint;

/// @brief stop line of a light that does not require a stop now, with its route lanelet and signal
struct V2IStopLine
{
  StopLineInfo info;
  lanelet::ConstLanelet lanelet;
  TrafficLightGroup signal;
};

/// @brief part of the trajectory ahead of ego, up to the first stop point
struct V2ITrajectory
{
  std::vector<TrajectoryPoint> points;
  lanelet::BasicLineString2d linestring;
  std::optional<lanelet::BasicPoint2d> stop_point;
  double backward_length{0.0};
};

/// @brief crossing of a stop line by the trajectory
struct StopLineCrossing
{
  double distance;
  double time;
  lanelet::BasicPoint2d point;
};

/// @brief get the stop lines on the route whose light does not require a stop now
std::vector<V2IStopLine> collect_v2i_stop_lines(
  const lanelet::LaneletMap & lanelet_map, const autoware_planning_msgs::msg::LaneletRoute & route,
  const std::vector<TrafficLightGroup> & traffic_light_groups)
{
  std::unordered_map<lanelet::Id, lanelet::Id> route_lanelet_id_per_traffic_light_id;
  for (const auto & segment : route.segments) {
    for (const auto & tl : lanelet_map.laneletLayer.get(segment.preferred_primitive.id)
                             .regulatoryElementsAs<lanelet::TrafficLight>()) {
      route_lanelet_id_per_traffic_light_id.emplace(tl->id(), segment.preferred_primitive.id);
    }
  }

  std::vector<V2IStopLine> stop_lines;
  for (const auto & signal : traffic_light_groups) {
    const auto hit = route_lanelet_id_per_traffic_light_id.find(signal.traffic_light_group_id);
    if (hit == route_lanelet_id_per_traffic_light_id.end()) {
      continue;
    }
    const auto traffic_light_it =
      lanelet_map.regulatoryElementLayer.find(signal.traffic_light_group_id);
    if (traffic_light_it == lanelet_map.regulatoryElementLayer.end()) {
      continue;
    }
    const lanelet::ConstLanelet lanelet = lanelet_map.laneletLayer.get(hit->second);
    if (autoware::traffic_light_utils::isTrafficSignalStop(lanelet, signal)) {
      continue;  // already handled by the red and amber checks
    }
    const auto traffic_light =
      std::dynamic_pointer_cast<const lanelet::TrafficLight>(*traffic_light_it);
    if (!traffic_light || !traffic_light->stopLine().has_value()) {
      continue;
    }
    stop_lines.push_back(
      {StopLineInfo{
         lanelet::utils::to2D(traffic_light->stopLine()->basicLineString()),
         signal.traffic_light_group_id},
       lanelet, signal});
  }
  return stop_lines;
}

/// @brief prepare the full trajectory ahead of ego, as a V2I prediction can concern a stop line far
/// ahead. Return nullopt if it has less than two points, as it cannot cross a stop line.
std::optional<V2ITrajectory> prepare_v2i_trajectory(
  const std::vector<TrajectoryPoint> & input_trajectory, const double stopped_velocity_threshold,
  const double max_longitudinal_offset, const bool is_ego_stopped)
{
  if (input_trajectory.empty()) {
    return std::nullopt;
  }

  // same forward offset as the red and amber checks, to allow more gap when stopped
  const auto forward_stop_point_offset = is_ego_stopped ? 0.25 : 0.0;

  V2ITrajectory trajectory;
  auto length = 0.0;
  auto last_p = input_trajectory.front();
  for (const auto & p : input_trajectory) {
    // skip points behind ego
    if (rclcpp::Duration(p.time_from_start).seconds() < 0.0) {
      trajectory.backward_length += autoware_utils_geometry::calc_distance2d(last_p.pose, p.pose);
      last_p = p;
      continue;
    }
    const lanelet::BasicPoint2d lanelet_p(p.pose.position.x, p.pose.position.y);
    if (!trajectory.linestring.empty())
      length += lanelet::geometry::distance2d(trajectory.linestring.back(), lanelet_p);
    trajectory.points.push_back(p);
    trajectory.linestring.emplace_back(lanelet_p);

    if (
      length > forward_stop_point_offset &&
      p.longitudinal_velocity_mps <= stopped_velocity_threshold) {
      trajectory.stop_point = trajectory.linestring.back();
      break;
    }
  }

  if (trajectory.linestring.size() < 2) {
    return std::nullopt;
  }

  if (max_longitudinal_offset > 0.0) {
    // extend the trajectory linestring by the vehicle's longitudinal offset
    const auto offset_pose = autoware_utils_geometry::calc_offset_pose(
      trajectory.points.back().pose, max_longitudinal_offset, 0.0, 0.0);
    const lanelet::BasicPoint2d offset_point(offset_pose.position.x, offset_pose.position.y);
    trajectory.linestring.emplace_back(offset_point);
    if (trajectory.stop_point.has_value()) trajectory.stop_point.value() = offset_point;
  }
  return trajectory;
}

/// @brief return where and when the trajectory crosses the stop line, or nullopt if it does not
std::optional<StopLineCrossing> find_stop_line_crossing(
  const V2ITrajectory & trajectory, const lanelet::BasicLineString2d & stop_line)
{
  const auto & points = trajectory.points;
  const auto & ls = trajectory.linestring;
  auto distance = 0.0;
  for (size_t i = 0; i + 1 < ls.size(); ++i) {
    lanelet::BasicPoints2d intersection_points;
    const lanelet::BasicLineString2d segment{ls[i], ls[i + 1]};
    const auto segment_length = static_cast<double>(boost::geometry::length(segment));
    boost::geometry::intersection(segment, stop_line, intersection_points);
    if (intersection_points.empty()) {
      distance += segment_length;
      continue;
    }
    const auto distance_to_intersection =
      boost::geometry::distance(segment.front(), intersection_points.front());
    const auto ratio = distance_to_intersection / segment_length;
    // the last segment models the vehicle body at the last trajectory point. it has no time of its
    // own, so use the time of that point.
    const auto next_index = std::min(i + 1, points.size() - 1);
    const auto time = autoware::interpolation::lerp(
      rclcpp::Duration(points[i].time_from_start).seconds(),
      rclcpp::Duration(points[next_index].time_from_start).seconds(), ratio);
    return StopLineCrossing{distance + distance_to_intersection, time, intersection_points.front()};
  }
  return std::nullopt;
}

/// @brief return true if a prediction turns the signal to stop less than time_limit seconds from
/// now. Predictions in the past are ignored, because they come from a stale feed or an unset stamp.
bool turns_red_within(
  const std::vector<PredictedTrafficLightState> & predictions,
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

Violations TrafficLightComplianceChecker::get_v2i_violations(
  const Inputs & input,
  const autoware_perception_msgs::msg::TrafficLightGroupArray & filtered_signals,
  const bool is_ego_stopped) const
{
  const auto trajectory = prepare_v2i_trajectory(
    input.trajectory, params_.ego_stopped_velocity_threshold,
    vehicle_info_.max_longitudinal_offset_m, is_ego_stopped);
  if (!trajectory) {
    return {};
  }

  const auto & v2i_params = params_.v2i_handling;
  Violations violations;
  for (const auto & stop_line :
       collect_v2i_stop_lines(*input.map, input.route, filtered_signals.traffic_light_groups)) {
    const auto crossing = find_stop_line_crossing(*trajectory, stop_line.info.line);
    const auto ego_front_to_stop_line =
      crossing ? crossing->distance - vehicle_info_.max_longitudinal_offset_m : 0.0;
    if (
      !crossing ||
      is_stop_point_within_margin_from_stop_line(trajectory->stop_point, stop_line.info.line) ||
      is_allow_if_cannot_stop(ego_front_to_stop_line)) {
      continue;
    }

    // a slow ego needs a fixed departure time instead of the trajectory crossing time
    const double time_to_cross_stop_line = input.current_velocity > v2i_params.velocity_threshold
                                             ? crossing->time
                                             : v2i_params.required_time_to_departure;
    const double last_time_allowed_to_pass =
      v2i_params.get_last_time_allowed_to_pass_from_map
        ? stop_line.lanelet.attributeOr(
            "v2i_last_time_allowed_to_pass", v2i_params.last_time_allowed_to_pass)
        : v2i_params.last_time_allowed_to_pass;
    if (!turns_red_within(
          stop_line.signal.predictions, stop_line.lanelet, input.current_time,
          time_to_cross_stop_line + last_time_allowed_to_pass)) {
      continue;
    }

    violations.emplace_back(
      ViolationType::V2I, stop_line.info.line, stop_line.info.traffic_light_id, crossing->point,
      crossing->distance + trajectory->backward_length);
  }
  return violations;
}

}  // namespace autoware::traffic_light_compliance_checker
