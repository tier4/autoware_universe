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
#include "turn_signal_logic.hpp"
#include "type_alias.hpp"

#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace autoware::minimum_rule_based_planner
{

//! Geometry of the detected intersection turn segment, for debug visualization only.
struct TurnIndicatorDebug
{
  bool has_segment{false};
  turn_signal::TurnDir direction{turn_signal::TurnDir::kNone};
  geometry_msgs::msg::Point start_point;  //!< map-frame position of the turn segment start
  geometry_msgs::msg::Point end_point;    //!< map-frame position of the turn segment end
  double start_yaw{0.0};                  //!< [rad] path heading at the segment start (map frame)
};

/**
 * @brief Decides the turn-signal command for the minimum rule based planner.
 *
 * This is the lanelet/ROS-aware glue around the pure logic in turn_signal_logic.
 * It reads `turn_direction` and `road_shoulder` lanelet attributes, measures the
 * path's lateral shift, then delegates every numeric decision to the pure layer.
 *
 * Three use cases are covered:
 *   - intersection right/left turn (lanelet `turn_direction` attribute)
 *   - pull-out from a road shoulder (ego on a road_shoulder lanelet)
 *   - pull-over to a road shoulder (goal on a road_shoulder lanelet)
 *
 * Fallback: when no lanelet map is available (e.g. test_path_with_lane_id mode),
 * intersection detection is skipped if turn directions cannot be resolved and the
 * shoulder cases are skipped entirely, degrading to DISABLE.
 */
class TurnIndicatorDecider
{
public:
  explicit TurnIndicatorDecider(const turn_signal::TurnSignalParams & params);

  void update_params(const turn_signal::TurnSignalParams & params);

  TurnIndicatorsCommand decide(
    const PathWithLaneId & path, const RouteContext & route_context,
    const geometry_msgs::msg::Pose & ego_pose, double ego_velocity, const rclcpp::Time & stamp);

  //! Debug geometry from the most recent decide() call (for visualization).
  const TurnIndicatorDebug & debug() const { return debug_; }

private:
  turn_signal::TurnSignalParams params_;
  turn_signal::BlinkHold blink_hold_;
  TurnIndicatorDebug debug_;
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // TURN_INDICATOR_DECIDER_HPP_
