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

#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/logger.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <memory>
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
  //! true when the geometry is a crosswalk/walkway bound used because the map has no explicit
  //! stop line for it; such targets use the larger stop_distance_from_crosswalk margin
  //! (upstream behavior_velocity semantics) instead of stop_margin_distance.
  bool without_explicit_stop_line{false};
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
  double max_jerk;          //!< maximum jerk used for the braking-distance check [m/s^3]
  double stop_margin_distance;  //!< distance to stop before the stop line crossing [m]
  //! distance to stop before a crosswalk/walkway without an explicit stop line, added to
  //! stop_margin_distance [m]
  double stop_distance_from_crosswalk;
  //! distance to stop before a private-area boundary, added to stop_margin_distance [m]
  double stop_distance_from_private_area;
  double base_link_to_front;  //!< vehicle front offset from base_link [m]
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
  //! @param time_keeper processing-time tracker; a private no-op instance is created when omitted
  //! (e.g. in unit tests), so the tracking scopes are always valid.
  explicit StopPlanner(
    const rclcpp::Logger & logger,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper = nullptr);

  /**
   * @brief Collect stop targets along the given route lanelets, tagged by type.
   *
   * Detection sources:
   *  - RoadMarking (type=stop_line) + traffic sign reference lines -> StopLine
   *  - crosswalk / walkway lanelets overlapping the route lanelets -> Crosswalk / Walkway, found
   *    via the map's spatial index (RTree) per route lanelet plus a polygon-overlap check, since
   *    walkways carry no regulatory element. The stop line is, in priority order: the crosswalk
   *    regulatory element's stop line, a RoadMarking bound to the crosswalk by its "crosswalk_id"
   *    attribute, or the entry-side bound of the crosswalk lanelet.
   *  - traffic light stop lines -> TrafficLight
   *  - lanelets with a turn_direction attribute -> Intersection (entry edge)
   *  - private-area (location=private) runs along the preferred lane sequence -> PrivateArea.
   *    The stop line is placed where the route path leaves the public road (entry) or enters it
   *    again (exit, upstream merge_from_private semantics), found as crossings with the outlines
   *    of non-private road lanelets overlapping the connector lanelet; the connector's entry
   *    edge is used as a fallback when no overlapping road is found.
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
   * subtracting the vehicle front offset and the stop margin. Reachability is judged on the
   * remaining distance from @p ego_pose (not from the trajectory start, which lies behind the
   * vehicle): a candidate is kept only while the remaining distance is positive and at least the
   * jerk-aware braking distance from @p ego_velocity and @p ego_acceleration under the maximum
   * deceleration and jerk, so a stop point the vehicle can no longer stop for (or has already
   * passed) is dropped. Returns the nearest such candidate, or nullopt when none exists.
   */
  std::optional<double> select_stop_arc_length(
    const std::vector<StopLine> & stop_lines,
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
    const geometry_msgs::msg::Pose & ego_pose, const double ego_velocity,
    const double ego_acceleration, const StopSelectionParams & params,
    const bool include_possibility) const;

  /**
   * @brief Build a MarkerArray visualizing the collected stop lines.
   *
   * Besides the line markers, each stop line gets a text label with its type name, floating 1 m
   * above the line (namespace "stop_line_type"). The markers are published in the "map" frame.
   */
  visualization_msgs::msg::MarkerArray create_stop_line_marker_array(
    const std::vector<StopLine> & stop_lines) const;

private:
  rclcpp::Logger logger_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // STOP_PLANNER_HPP_
