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
 * @brief Type of a map-defined stop line, classified by its collection source.
 *
 * The type determines whether the vehicle must stop (mandatory) or only may need to stop
 * (possibility). See is_possibility_type().
 */
enum class StopLineType {
  RoadMarking,   //!< explicit stop-line road marking (temporary stop line): mandatory
  TrafficLight,  //!< stop line referenced by a traffic light (signal): possibility
  TrafficSign,   //!< reference line of a traffic sign (e.g. stop sign): mandatory
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
 * @brief Whether the stop line is a "possibility" target (the vehicle may need to stop) rather
 *        than a mandatory stop target.
 *
 * Signals (traffic lights) are treated as possibility targets; painted stop lines and stop signs
 * are mandatory. The go trajectory only stops at mandatory targets, while the stop trajectory
 * stops at both mandatory and possibility targets.
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
   * @brief Collect stop lines from the map along the given route lanelets, tagged by type.
   *
   * Sources:
   *  - RoadMarking regulatory elements whose line string type is "stop_line" -> RoadMarking
   *  - stop lines referenced by traffic lights -> TrafficLight
   *  - reference lines of traffic signs (e.g. stop signs) -> TrafficSign
   *
   * Duplicate line strings (shared between lanelets) are returned only once.
   */
  std::vector<StopLine> collect_stop_lines(const lanelet::ConstLanelets & route_lanelets) const;

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
