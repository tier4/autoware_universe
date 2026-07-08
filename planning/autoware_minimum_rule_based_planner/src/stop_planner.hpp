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

#include <vector>

namespace autoware::minimum_rule_based_planner
{

/**
 * @brief Plans stops at map-defined locations where the vehicle may need to stop
 *        (stop lines, crosswalks, private-area exits, etc.).
 *
 * As a first step this only extracts candidate stop lines from the map and
 * exposes them for visualization. Stop-distance computation and velocity
 * insertion are added in later steps.
 */
class StopPlanner
{
public:
  explicit StopPlanner(const rclcpp::Logger & logger);

  /**
   * @brief Collect stop lines from the map along the given route lanelets.
   *
   * Sources:
   *  - RoadMarking regulatory elements whose line string type is "stop_line"
   *    (e.g. painted stop lines, crosswalk stop lines)
   *  - stop lines referenced by traffic lights
   *  - reference lines of traffic signs (e.g. stop signs at intersections)
   *
   * Duplicate line strings (shared between lanelets) are returned only once.
   */
  std::vector<lanelet::ConstLineString3d> collect_stop_lines(
    const lanelet::ConstLanelets & route_lanelets) const;

  /**
   * @brief Keep only the stop lines that intersect the given trajectory (in 2D).
   *
   * Used to narrow the map-defined stop lines down to the ones actually crossed
   * by the currently planned trajectory.
   */
  std::vector<lanelet::ConstLineString3d> filter_stop_lines_on_trajectory(
    const std::vector<lanelet::ConstLineString3d> & stop_lines,
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points) const;

  /**
   * @brief Build a MarkerArray visualizing the collected stop lines.
   *
   * The markers are published in the "map" frame.
   */
  visualization_msgs::msg::MarkerArray create_stop_line_marker_array(
    const std::vector<lanelet::ConstLineString3d> & stop_lines) const;

private:
  rclcpp::Logger logger_;
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // STOP_PLANNER_HPP_
