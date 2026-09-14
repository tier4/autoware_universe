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

#ifndef MAP_BASED_STOP_PLANNER_HPP_
#define MAP_BASED_STOP_PLANNER_HPP_

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

//! Type of a map-defined stop target, classified by the map element that induces it.
enum class StopLineType {
  StopLine,      //!< stop line referenced by a traffic sign -> mandatory
  Walkway,       //!< walkway the route crosses              -> mandatory
  Crosswalk,     //!< crosswalk the route crosses            -> possibility
  TrafficLight,  //!< stop line referenced by traffic light  -> possibility
  Intersection,  //!< lanelet with turn_direction attribute  -> possibility
  PrivateArea,   //!< private-area entry/exit transition     -> possibility
  RoadShoulder,  //!< crossing a road-shoulder boundary either way  -> possibility
};

//! A map-defined stop line together with its classified type.
struct StopLine
{
  lanelet::ConstLineString3d line;
  StopLineType type;
  //! true when the geometry is a crosswalk/walkway bound used because the map has no explicit
  //! stop line; such targets use the larger stop_distance_from_crosswalk margin.
  bool without_explicit_stop_line{false};
};

/**
 * @brief Whether the stop target is a "possibility" target rather than a mandatory one.
 *
 * Mandatory targets (the go trajectory also stops): StopLine, Walkway.
 * Possibility targets (only the stop trajectory additionally stops): Crosswalk, TrafficLight,
 * Intersection, PrivateArea, RoadShoulder.
 */
bool is_possibility_type(StopLineType type);

//! Parameters used to select a feasible stop point along a trajectory.
struct StopSelectionParams
{
  double max_deceleration;      //!< max deceleration for the braking-distance check [m/s^2]
  double max_jerk;              //!< max jerk for the braking-distance check [m/s^3]
  double stop_margin_distance;  //!< distance to stop before the stop line crossing [m]
  //! extra stop distance for a crosswalk/walkway without an explicit stop line [m]
  double stop_distance_from_crosswalk;
  //! extra stop distance before a private-area boundary [m]
  double stop_distance_from_private_area;
  //! extra stop distance before the first conflicting priority lane in an intersection [m]
  double stop_distance_from_intersection;
  //! extra stop distance before the vehicle footprint crosses a road-shoulder boundary, in either
  //! direction [m]
  double stop_distance_from_road_shoulder;
  double base_link_to_front;  //!< vehicle front offset from base_link [m]
  //! vehicle dimensions; used only by the road-shoulder stop, which sweeps the footprint
  //! along the trajectory instead of offsetting a line crossing by base_link_to_front
  VehicleInfo vehicle_info;
  //! min arc-length difference for the stop trajectory's stop point to be treated as different
  //! from the go trajectory's [m]
  double stop_point_diff_threshold;
};

//! Intersection lanelet groups collected during stop-line collection, for debug visualization.
struct IntersectionDebugLanelets
{
  lanelet::ConstLanelets conflicting;  //!< routing-graph conflicting lanelets
  lanelet::ConstLanelets yield;        //!< lanes yielding to ego (RightOfWay) and their upstreams
  lanelet::ConstLanelets ego;          //!< ego's own lanes (assigned, upstreams, sibling branches)
  //! priority lanes ego must yield to (conflicting - yield - ego); the stop target
  lanelet::ConstLanelets attention;
};

/**
 * @brief Plans stops at map-defined locations (stop lines, crosswalks, intersections, etc.).
 *
 * Extracts candidate stop lines from the map, classifies them by type, and plans the go/stop
 * trajectories with the nearest reachable stop point embedded. set_planner_data() + plan() is
 * the node-facing interface; the collect/filter/select building blocks are also public for
 * node-independent testing.
 */
class MapBasedStopPlanner
{
public:
  //! Result of plan().
  struct Result
  {
    //! stops at mandatory targets only (equals the input trajectory when there is none)
    Trajectory go_trajectory;
    //! additionally stops at possibility targets; set only when its stop point differs from the
    //! go trajectory's
    std::optional<Trajectory> stop_trajectory;
    visualization_msgs::msg::MarkerArray stop_line_markers;
  };

  //! @param time_keeper processing-time tracker; a private no-op instance is created when omitted
  //! (e.g. in unit tests), so the tracking scopes are always valid.
  explicit MapBasedStopPlanner(
    const rclcpp::Logger & logger,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper = nullptr);

  //! Update the cached stop lines. Collection depends only on the map and the route, so it is
  //! re-run only when either changes (pointer identity).
  void set_planner_data(
    const LaneletMapBin::ConstSharedPtr & lanelet_map_bin_ptr,
    const LaneletRoute::ConstSharedPtr & route_ptr, const RouteContext & route_context);

  /**
   * @brief Plan the go and stop trajectories with map-defined stop points embedded.
   *
   * The go trajectory stops only at mandatory targets; the stop trajectory additionally stops at
   * possibility targets and is set only when its stop point differs from the go trajectory's.
   * A stop point is not inserted when an upstream stop (e.g. obstacle stop) already stops the
   * vehicle before it. Velocities are not planned here; the returned trajectories are shape-only
   * inputs for the velocity optimization.
   */
  Result plan(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & ego_pose,
    const double ego_velocity, const double ego_acceleration,
    const StopSelectionParams & params) const;

  /**
   * @brief Collect stop targets along the given route, tagged by type.
   *
   * Detection sources:
   *  - traffic sign reference lines (vm-02-02 stop sign) -> StopLine. Stop lines referenced
   *    only by RoadMarking regulatory elements (vm-03-14 intersection guidance lines) are not
   *    mandatory stops and are intentionally not collected.
   *  - crosswalk / walkway lanelets overlapping the route lanelets -> Crosswalk / Walkway.
   *    The stop line is, in priority order: the crosswalk regulatory element's stop line, a
   *    RoadMarking bound to the crosswalk by its "crosswalk_id" attribute, or the entry-side
   *    bound of the crosswalk lanelet.
   *  - traffic light stop lines -> TrafficLight
   *  - lanelets with a turn_direction attribute -> Intersection, placed where the route first
   *    enters a conflicting priority lane (routing-graph conflicting lanelets minus the
   *    RightOfWay yield lanes and ego lanes); no line when ego has priority. Falls back to the
   *    lanelet entry edge when no routing graph is available.
   *  - location=private runs along the preferred lane sequence -> PrivateArea, placed where the
   *    route leaves the public road (entry) or enters it again (exit).
   *
   * Duplicate targets (shared between lanelets) are returned only once.
   *
   * When @p debug_lanelets is given, the intersection lanelet groups (conflicting / yield / ego)
   * are accumulated into it for visualization.
   */
  std::vector<StopLine> collect_stop_lines(
    const RouteContext & route_context, IntersectionDebugLanelets * debug_lanelets = nullptr) const;

  //! Keep only the stop lines that intersect the given trajectory (in 2D).
  std::vector<StopLine> filter_stop_lines_on_trajectory(
    const std::vector<StopLine> & stop_lines,
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points) const;

  /**
   * @brief Select the arc length (from the trajectory start) of the nearest reachable stop point.
   *
   * Each crossing of an allowed stop line (all types when @p include_possibility is true,
   * mandatory types only otherwise) is converted to a stop point by subtracting the vehicle front
   * offset and the stop margin. A candidate is kept only while the remaining distance from
   * @p ego_pose is positive and at least the jerk-aware braking distance, so a stop point the
   * vehicle has passed or can no longer stop for is dropped in favor of a farther reachable one.
   * Returns nullopt when no reachable candidate exists.
   */
  std::optional<double> select_stop_arc_length(
    const std::vector<StopLine> & stop_lines,
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
    const geometry_msgs::msg::Pose & ego_pose, const double ego_velocity,
    const double ego_acceleration, const StopSelectionParams & params,
    const bool include_possibility) const;

  /**
   * @brief Arc length of the stop point placed before ego crosses a road-shoulder boundary.
   *
   * Two symmetric cases, selected from where the ego footprint currently is:
   * - Departing: the footprint is clear of every road lane and overlaps a shoulder lane. The stop
   *   is placed a margin before the first pose whose footprint reaches a road lane.
   * - Entering: the footprint overlaps a road lane and no shoulder lane. The stop is placed a
   *   margin before the first pose whose footprint touches a shoulder lane, but only when a later
   *   pose is wholly inside the shoulder — clipping a shoulder while staying in the road lane is
   *   normal driving and must not stop the vehicle.
   *
   * A footprint that already straddles both kinds returns nullopt, so a handover from another
   * generator mid-crossing completes the crossing without stopping. The footprint is swept along
   * the trajectory rather than intersecting a base_link line, which would be several metres late
   * at the shallow angles the shift produces. Clamped to ego's own arc length so the stop is never
   * behind ego.
   */
  std::optional<double> select_road_shoulder_stop_arc_length(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
    const geometry_msgs::msg::Pose & ego_pose, const StopSelectionParams & params) const;

  //! Build a MarkerArray (frame "map") of the stop lines with a per-type text label above each.
  visualization_msgs::msg::MarkerArray create_stop_line_marker_array(
    const std::vector<StopLine> & stop_lines) const;

private:
  //! One stop pass: the nearest reachable stop point among the allowed types, embedded on a copy.
  struct SingleStopResult
  {
    double stop_point_arc_length;
    Trajectory trajectory;
  };
  //! @param road_shoulder_stop_arc_length possibility stop from
  //! select_road_shoulder_stop_arc_length, considered alongside the map stop lines only when
  //! include_possibility is true
  std::optional<SingleStopResult> plan_single_stop(
    const std::vector<StopLine> & stop_lines, const Trajectory & trajectory,
    const geometry_msgs::msg::Pose & ego_pose, const double ego_velocity,
    const double ego_acceleration, const StopSelectionParams & params,
    const bool include_possibility,
    const std::optional<double> & road_shoulder_stop_arc_length) const;

  rclcpp::Logger logger_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;

  //! Stop lines collected from the map along the route, cached by set_planner_data().
  std::vector<StopLine> stop_lines_;
  //! Intersection lanelet groups collected alongside stop_lines_, for debug markers.
  IntersectionDebugLanelets intersection_debug_lanelets_;
  LaneletMapBin::ConstSharedPtr stop_lines_map_ptr_;
  LaneletRoute::ConstSharedPtr stop_lines_route_ptr_;
  //! Map used by the road-shoulder stop, which is re-evaluated every cycle.
  lanelet::LaneletMapPtr lanelet_map_ptr_;
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // MAP_BASED_STOP_PLANNER_HPP_
