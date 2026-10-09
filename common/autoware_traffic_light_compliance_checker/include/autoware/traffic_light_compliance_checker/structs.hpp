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

#ifndef AUTOWARE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER__STRUCTS_HPP_
#define AUTOWARE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER__STRUCTS_HPP_

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace autoware::traffic_light_compliance_checker
{

/// @brief route lanelet id per traffic light id, for the traffic lights of the current route
using RouteTrafficLightIndex = std::unordered_map<lanelet::Id, lanelet::Id>;

/// @brief input data for traffic light compliance check
struct Inputs
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory;
  lanelet::LaneletMapPtr map;
  autoware_planning_msgs::msg::LaneletRoute route;
  autoware_perception_msgs::msg::TrafficLightGroupArray signals;
  rclcpp::Time current_time;
  double current_velocity;
  double current_acceleration;
};

/// @brief information about a stop line and its associated traffic light
struct StopLineInfo
{
  lanelet::BasicLineString2d line;
  int64_t traffic_light_id;
  int64_t traffic_light_lanelet_id;
};

/// @brief type of traffic light violation
enum class ViolationType { RED_LIGHT, AMBER_LIGHT, V2I };

/// @brief violation detail
struct Violation
{
  ViolationType type;
  lanelet::BasicLineString2d stop_line;
  int64_t traffic_light_id;
  lanelet::BasicPoint2d cross_point;
  double arc_length_to_cross_point;

  Violation(
    ViolationType type, lanelet::BasicLineString2d stop_line, int64_t traffic_light_id,
    lanelet::BasicPoint2d cross_point, double arc_length_to_cross_point)
  : type(type),
    stop_line(stop_line),
    traffic_light_id(traffic_light_id),
    cross_point(cross_point),
    arc_length_to_cross_point(arc_length_to_cross_point)
  {
  }

  bool operator<(const Violation & other) const
  {
    return arc_length_to_cross_point < other.arc_length_to_cross_point;
  }
};
using Violations = std::vector<Violation>;

/// @brief result of compliance check
struct ComplianceResult
{
  Violations violations;
};

/// @brief parameters for traffic light signal status tracking
struct StatusTrackerParameters
{
  double stable_duration_threshold_red;
  double stable_duration_threshold_amber;
  double stable_duration_threshold_unknown;
};

/// @brief part of the trajectory to check against stop lines
struct PreparedTrajectory
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;
  lanelet::BasicLineString2d linestring;
  std::optional<lanelet::BasicPoint2d> stop_point;
  double backward_length{0.0};
};

/// @brief parameters for traffic light compliance check
struct Parameters
{
  double deceleration_limit;
  double jerk_limit;
  double delay_response_time;
  double crossing_time_limit;
  bool treat_amber_light_as_red_light;
  bool treat_unknown_light_as_red_light;
  double stop_overshoot_margin;
  double allow_if_cannot_stop_distance;
  double min_lookahead_distance;
  double ego_stopped_velocity_threshold;
  StatusTrackerParameters status_tracker_parameters;
  struct CheckedTrajectoryLength
  {
    double deceleration_limit;
    double jerk_limit;
  } checked_trajectory_length;
  struct AmberRejection
  {
    double hysteresis_duration;
    bool reject_if_stop_detected;
  } amber_rejection;
  struct V2IHandling
  {
    bool get_last_time_allowed_to_pass_from_map;
    double last_time_allowed_to_pass;
    double velocity_threshold;
    double required_time_to_departure;
  } v2i_handling;
};

}  // namespace autoware::traffic_light_compliance_checker

#endif  // AUTOWARE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER__STRUCTS_HPP_
