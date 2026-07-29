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

#ifndef TURN_INDICATOR_DECIDER_HPP_
#define TURN_INDICATOR_DECIDER_HPP_

#include "path_planner.hpp"
#include "type_alias.hpp"
#include "utils/turn_indicator_utils.hpp"

#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

//! Geometry / intermediate values of the most recent decision, for debug visualization only.
struct TurnIndicatorDebug
{
  turn_indicator::TurnDirection command{turn_indicator::TurnDirection::NONE};  //!< after BlinkHold
  turn_indicator::ManeuverKind kind{turn_indicator::ManeuverKind::NONE};
  bool has_segment{false};
  geometry_msgs::msg::Point start_point;  //!< map-frame position of the maneuver segment start
  geometry_msgs::msg::Point end_point;    //!< map-frame position of the maneuver segment end
  double start_yaw{0.0};                  //!< [rad] path heading at the segment start (map frame)
  double exit_yaw{0.0};                   //!< [rad] path heading the maneuver ends on
  double dist_to_start{0.0};              //!< [m] arc length from ego to the segment start
  double ego_lateral_offset{0.0};         //!< [m] ego offset from the nearest route lane centerline
  double goal_lateral_offset{0.0};        //!< [m] goal offset from the goal lane centerline
  double distance_to_goal{0.0};           //!< [m]
};

//! Decides the turn-signal command written into every candidate trajectory.
//!
//! Covers the three maneuvers the planner is responsible for (see
//! my_docs/TASK_mrbp_turn_indicator.md): intersection turns, private-area exits, and
//! departure/arrival at an off-centerline stop. Lane changes and avoidance are out of scope and
//! must NOT raise a signal even though ego is laterally offset while the upstream planner runs one.
class TurnIndicatorDecider
{
public:
  explicit TurnIndicatorDecider(const turn_indicator::TurnSignalParams & params);

  void update_params(const turn_indicator::TurnSignalParams & params);

  TurnIndicatorsCommand decide(
    const PathWithLaneId & path, const RouteContext & route_context,
    const geometry_msgs::msg::Pose & ego_pose, double ego_velocity, const rclcpp::Time & stamp);

  //! Debug values from the most recent decide() call (for visualization).
  const TurnIndicatorDebug & debug() const { return debug_; }

private:
  //! Map attributes for `lane_ids`, memoised across cycles (invalidated when the map changes).
  const turn_indicator::LaneAttributeMap & lane_attributes(
    const std::vector<turn_indicator::PathPointLite> & points, const RouteContext & route_context);

  //! The lane ego is expected to track plus its lateral neighbours, so that a lane change (whose
  //! target lane is not the path's lane) still reports a near-zero offset. Memoised per lane id.
  const lanelet::ConstLanelets & lanes_with_neighbours(
    int64_t lane_id, const RouteContext & route_context);

  turn_indicator::TurnSignalParams params_;
  turn_indicator::BlinkHold blink_hold_;
  turn_indicator::DepartureLatch departure_latch_;
  turn_indicator::ArrivalState arrival_state_;
  TurnIndicatorDebug debug_;

  // --- caches (perf: the maps below are rebuilt only when the map or route changes) ---
  const lanelet::LaneletMap * cached_map_{nullptr};
  turn_indicator::LaneAttributeMap lane_attribute_cache_;
  std::unordered_map<int64_t, lanelet::ConstLanelets> neighbour_cache_;
  geometry_msgs::msg::Pose cached_goal_pose_;
  bool has_cached_goal_{false};
  double cached_goal_offset_{0.0};
};

//! Debug markers for the decision above (empty when nothing is lit and no segment was found).
visualization_msgs::msg::MarkerArray create_turn_indicator_markers(
  const TurnIndicatorDebug & debug, const std_msgs::msg::Header & header);

}  // namespace autoware::minimum_rule_based_planner

#endif  // TURN_INDICATOR_DECIDER_HPP_
