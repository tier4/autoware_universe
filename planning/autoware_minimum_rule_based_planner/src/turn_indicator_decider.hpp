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

#include <optional>

namespace autoware::minimum_rule_based_planner
{

//! The most recent decision, for debug visualization only.
struct TurnIndicatorDebug
{
  turn_indicator::Signal signal;          //!< direction after BlinkHold, plus what raised it
  bool has_segment{false};                //!< whether the two points below hold a maneuver
  geometry_msgs::msg::Point start_point;  //!< map-frame start of the maneuver that lit the signal
  geometry_msgs::msg::Point end_point;    //!< map-frame end of it
};

//! Decides the turn-signal command written into every candidate trajectory.
//!
//! Covers the maneuvers this planner is responsible for (see
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
  turn_indicator::TurnSignalParams params_;
  turn_indicator::BlinkHold blink_hold_;
  //! direction the departure latch holds; NONE while not latched
  turn_indicator::TurnDirection pull_out_latch_{turn_indicator::TurnDirection::NONE};
  bool arrived_at_goal_{false};
  //! goal the two states above were latched against; both are re-armed when it changes
  std::optional<geometry_msgs::msg::Pose> latched_goal_pose_;
  TurnIndicatorDebug debug_;
};

//! Debug markers for the decision above (the arrow is omitted when no maneuver lit the signal).
visualization_msgs::msg::MarkerArray create_turn_indicator_markers(
  const TurnIndicatorDebug & debug, const std_msgs::msg::Header & header);

}  // namespace autoware::minimum_rule_based_planner

#endif  // TURN_INDICATOR_DECIDER_HPP_
