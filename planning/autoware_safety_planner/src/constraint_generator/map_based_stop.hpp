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

#ifndef CONSTRAINT_GENERATOR__MAP_BASED_STOP_HPP_
#define CONSTRAINT_GENERATOR__MAP_BASED_STOP_HPP_

#include "constraint_generator_interface.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner::experimental
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
 * Mandatory targets (Certainty::DEFINITE): StopLine, Walkway.
 * Possibility targets (Certainty::POSSIBLE): Crosswalk, TrafficLight, Intersection, PrivateArea,
 * RoadShoulder.
 */
bool is_possibility_type(StopLineType type);

/**
 * @brief Emits the stops at map-defined locations (stop lines, crosswalks, intersections, etc.),
 * as the map_based_stop of autoware_minimum_rule_based_planner does.
 *
 * The nearest reachable mandatory stop point is emitted as a DEFINITE Gate, and the nearest
 * reachable possibility stop point as a POSSIBLE Gate.
 */
class MapBasedStopConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "map_based_stop"; }
  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;

private:
  //! Intersection lanelet groups collected during stop-line collection, for debug visualization.
  struct IntersectionDebugLanelets
  {
    lanelet::ConstLanelets conflicting;  //!< routing-graph conflicting lanelets
    lanelet::ConstLanelets yield;        //!< lanes yielding to ego and their upstreams
    lanelet::ConstLanelets ego;          //!< ego's own lanes (assigned, upstreams, siblings)
    //! priority lanes ego must yield to (conflicting - yield - ego); the stop target
    lanelet::ConstLanelets attention;
  };

  //! The stop point selected for one certainty.
  struct StopPoint
  {
    double arc_length;  //!< [m] base_link arc length from the start of the trajectory
    std::string target_id;
    StopLineType type;
  };

  //! Re-collect the stop lines when the map or the lanelet sequence on the route has changed.
  void update_stop_lines(const RouteManager & route_manager);

  /**
   * @brief Collect stop targets along the given lanelet sequence, tagged by type. The
   * intersection lanelet groups are accumulated into @p debug_lanelets for visualization.
   *
   * Detection sources:
   *  - traffic sign reference lines (vm-02-02 stop sign) -> StopLine. Stop lines referenced
   *    only by RoadMarking regulatory elements (vm-03-14 intersection guidance lines) are not
   *    mandatory stops and are intentionally not collected.
   *  - crosswalk / walkway lanelets overlapping the lanelets -> Crosswalk / Walkway.
   *    The stop line is, in priority order: the crosswalk regulatory element's stop line, a
   *    RoadMarking bound to the crosswalk by its "crosswalk_id" attribute, or the entry-side
   *    bound of the crosswalk lanelet.
   *  - traffic light stop lines -> TrafficLight
   *  - lanelets with a turn_direction attribute -> Intersection, placed where the route first
   *    enters a conflicting priority lane (routing-graph conflicting lanelets minus the
   *    RightOfWay yield lanes and ego lanes); no line when ego has priority. Falls back to the
   *    lanelet entry edge when no routing graph is available.
   *  - location=private runs along the lanelets -> PrivateArea, placed where the route leaves
   *    the public road (entry) or enters it again (exit).
   *
   * Duplicate targets (shared between lanelets) are returned only once.
   */
  std::vector<StopLine> collect_stop_lines(
    const lanelet::ConstLanelets & lanelets, const lanelet::LaneletMapConstPtr & lanelet_map,
    const lanelet::routing::RoutingGraphConstPtr & routing_graph,
    IntersectionDebugLanelets & debug_lanelets) const;

  /**
   * @brief Select the nearest reachable stop point among the stop lines of the given certainty.
   *
   * Each crossing of a stop line is converted to a stop point by subtracting the vehicle front
   * offset and the stop margin. A candidate is kept only while the remaining distance from ego is
   * positive and at least the jerk-aware braking distance, so a stop point the vehicle has passed
   * or can no longer stop for is dropped in favor of a farther reachable one.
   *
   * @param possibility true selects among the possibility types, false among the mandatory ones
   */
  std::optional<StopPoint> select_stop_point(
    const std::vector<StopLine> & stop_lines, const TrajectoryPoints & traj_points,
    const geometry_msgs::msg::Pose & ego_pose, const double ego_velocity,
    const double ego_acceleration, const VehicleInfo & vehicle_info, const bool possibility) const;

  /**
   * @brief Arc length of the stop point placed before ego crosses a road-shoulder boundary.
   *
   * Two symmetric cases, selected from where the ego footprint currently is:
   * - Departing: the footprint is clear of every road lane and overlaps a shoulder lane. The stop
   *   is placed a margin before the first pose whose footprint reaches a road lane.
   * - Entering: the footprint overlaps a road lane and no shoulder lane. The stop is placed a
   *   margin before the first pose whose footprint touches a shoulder lane, but only when a later
   *   pose is wholly inside the shoulder.
   *
   * A footprint that already straddles both kinds returns nullopt. Clamped to ego's own arc length
   * so the stop is never behind ego.
   */
  std::optional<double> select_road_shoulder_stop_arc_length(
    const TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & ego_pose,
    const VehicleInfo & vehicle_info, const lanelet::LaneletMapConstPtr & lanelet_map) const;

  //! Stop lines collected along the route, cached by update_stop_lines().
  std::vector<StopLine> stop_lines_;
  //! Intersection lanelet groups collected alongside stop_lines_, for debug markers.
  IntersectionDebugLanelets intersection_debug_lanelets_;
  lanelet::LaneletMapConstPtr stop_lines_map_ptr_;
  std::vector<lanelet::Id> stop_lines_lanelet_ids_;
};

}  // namespace autoware::safety_planner::experimental

#endif  // CONSTRAINT_GENERATOR__MAP_BASED_STOP_HPP_
