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

#ifndef STOP_PLANNER_HPP_
#define STOP_PLANNER_HPP_

#include "path_planner.hpp"

#include <rclcpp/logger.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <optional>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

/**
 * @brief Type of a map-defined stop target, classified by the map element that induces it.
 *
 * The type determines whether the vehicle must stop (mandatory stop target, 停止対象箇所) or only
 * may need to stop (possibility stop target, 停止可能性対象箇所). See is_possibility_type().
 */
enum class StopLineType {
  StopLine,      //!< 一時停止線: painted stop line / stop sign            -> mandatory
  Walkway,       //!< 歩道: walkway the route crosses                      -> mandatory
  Crosswalk,     //!< 横断歩道: crosswalk the route crosses                -> possibility
  TrafficLight,  //!< 信号: stop line referenced by a traffic light        -> possibility
  Intersection,  //!< 交差点: lanelet with a turn_direction attribute      -> possibility
  PrivateArea,   //!< 私有地入退出: private-area entry/exit transition      -> possibility
};

/**
 * @brief A map-defined stop line together with its classified type.
 */
struct StopLine
{
  lanelet::ConstLineString3d line;
  StopLineType type;
};

/**
 * @brief Whether the stop target is a "possibility" target (停止可能性対象箇所) rather than a
 *        mandatory stop target (停止対象箇所).
 *
 * Mandatory (go trajectory stops): 一時停止線 (StopLine), 歩道 (Walkway).
 * Possibility (only the stop trajectory additionally stops): 横断歩道 (Crosswalk), 信号
 * (TrafficLight), 交差点 (Intersection), 私有地入退出 (PrivateArea).
 */
bool is_possibility_type(StopLineType type);

/**
 * @brief Parameters used to select a feasible stop point along a trajectory.
 */
struct StopSelectionParams
{
  double max_deceleration;  //!< maximum deceleration used for the braking-distance check [m/s^2]
  double stop_margin_distance;  //!< distance to stop before the stop line crossing [m]
  double base_link_to_front;    //!< vehicle front offset from base_link [m]
};

/**
 * @brief Plans stops at map-defined locations where the vehicle may need to stop
 *        (stop lines, crosswalks, private-area exits, etc.).
 *
 * Extracts candidate stop lines from the map, classifies them by type, and selects the nearest
 * reachable stop point along the trajectory.
 */
class StopPlanner
{
public:
  explicit StopPlanner(const rclcpp::Logger & logger);

  /**
   * @brief Collect stop targets along the given route lanelets, tagged by type.
   *
   * Detection sources:
   *  - RoadMarking (type=stop_line) + traffic sign reference lines -> StopLine
   *  - crosswalk / walkway lanelets that geometrically cross the route path (searched over the
   *    whole map layer, since walkways carry no regulatory element) -> Crosswalk / Walkway,
   *    using only the entry-side bound as the stop line
   *  - traffic light stop lines -> TrafficLight
   *  - lanelets with a turn_direction attribute -> Intersection (entry edge)
   *  - private-area (location=private) entry/exit transitions along the preferred lane sequence
   *    -> PrivateArea (entry edge)
   *
   * Duplicate targets (shared between lanelets) are returned only once.
   */
  std::vector<StopLine> collect_stop_lines(const RouteContext & route_context) const;

  /**
   * @brief Keep only the stop lines that intersect the given trajectory (in 2D).
   *
   * Used to narrow the map-defined stop lines down to the ones actually crossed
   * by the currently planned trajectory.
   */
  std::vector<StopLine> filter_stop_lines_on_trajectory(
    const std::vector<StopLine> & stop_lines,
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points) const;

  /**
   * @brief Select the arc length (from the trajectory start) of the nearest reachable stop point.
   *
   * For every stop line whose type is allowed (all types when @p include_possibility is true,
   * mandatory types only otherwise), the nearest 2D crossing is converted to a stop point by
   * subtracting the vehicle front offset and the stop margin. The nearest positive stop point is
   * returned only if it is reachable given @p ego_velocity and the maximum deceleration (braking
   * distance). Returns nullopt when no reachable stop point exists.
   */
  std::optional<double> select_stop_arc_length(
    const std::vector<StopLine> & stop_lines,
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
    const double ego_velocity, const StopSelectionParams & params,
    const bool include_possibility) const;

  /**
   * @brief Build a MarkerArray visualizing the collected stop lines.
   *
   * The markers are published in the "map" frame.
   */
  visualization_msgs::msg::MarkerArray create_stop_line_marker_array(
    const std::vector<StopLine> & stop_lines) const;

private:
  rclcpp::Logger logger_;
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // STOP_PLANNER_HPP_
