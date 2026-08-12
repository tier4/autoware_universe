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

#include "map_based_stop_planner.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <rclcpp/logger.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace
{
using autoware_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoint make_point(double x, double y)
{
  TrajectoryPoint pt;
  pt.pose.position.x = x;
  pt.pose.position.y = y;
  pt.pose.orientation.w = 1.0;
  pt.longitudinal_velocity_mps = 5.0f;
  return pt;
}

// A straight trajectory along +x at y = 0, 1 m spacing.
std::vector<TrajectoryPoint> make_straight_trajectory(size_t num_points)
{
  std::vector<TrajectoryPoint> points;
  for (size_t i = 0; i < num_points; ++i) {
    points.push_back(make_point(static_cast<double>(i), 0.0));
  }
  return points;
}

// A vertical (crossing) stop line at arc length x_cross, spanning y in [-2, 2].
StopLine make_crossing_stop_line(
  lanelet::Id id, double x_cross, StopLineType type = StopLineType::StopLine)
{
  lanelet::LineString3d line(
    id, {lanelet::Point3d(id * 10 + 1, x_cross, -2.0, 0.0),
         lanelet::Point3d(id * 10 + 2, x_cross, 2.0, 0.0)});
  return StopLine{line, type};
}

StopSelectionParams make_params()
{
  StopSelectionParams params;
  params.max_deceleration = 4.0;
  params.max_jerk = 5.0;
  params.stop_margin_distance = 1.0;
  params.stop_distance_from_crosswalk = 3.5;
  params.stop_distance_from_private_area = 3.0;
  params.stop_distance_from_intersection = 1.0;
  params.base_link_to_front = 4.0;
  params.stop_point_diff_threshold = 0.5;
  return params;
}

}  // namespace

// ============================================================
// is_possibility_type
// ============================================================

TEST(MapBasedStopPlannerTest, PossibilityTypeClassification)
{
  // Mandatory stop targets: the go trajectory also stops.
  EXPECT_FALSE(is_possibility_type(StopLineType::StopLine));
  EXPECT_FALSE(is_possibility_type(StopLineType::Walkway));
  // Possibility stop targets: only the stop trajectory additionally stops.
  EXPECT_TRUE(is_possibility_type(StopLineType::Crosswalk));
  EXPECT_TRUE(is_possibility_type(StopLineType::TrafficLight));
  EXPECT_TRUE(is_possibility_type(StopLineType::Intersection));
  EXPECT_TRUE(is_possibility_type(StopLineType::PrivateArea));
}

// ============================================================
// filter_stop_lines_on_trajectory
// ============================================================

TEST(MapBasedStopPlannerTest, FilterKeepsOnlyCrossingLines)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  const auto trajectory = make_straight_trajectory(20);

  std::vector<StopLine> stop_lines;
  stop_lines.push_back(make_crossing_stop_line(1, 10.0));  // crosses the trajectory
  // A stop line far to the side that does not cross the trajectory.
  lanelet::LineString3d off_line(
    2, {lanelet::Point3d(21, 5.0, 10.0, 0.0), lanelet::Point3d(22, 5.0, 12.0, 0.0)});
  stop_lines.push_back(StopLine{off_line, StopLineType::StopLine});

  const auto filtered = planner.filter_stop_lines_on_trajectory(stop_lines, trajectory);
  ASSERT_EQ(filtered.size(), 1u);
  EXPECT_EQ(filtered.front().line.id(), 1);
}

// ============================================================
// select_stop_arc_length
// ============================================================

TEST(MapBasedStopPlannerTest, SelectAppliesFrontOffsetAndMargin)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  const auto trajectory = make_straight_trajectory(30);
  const std::vector<StopLine> stop_lines{make_crossing_stop_line(1, 20.0)};

  // Ego stopped: braking distance is 0, so the stop point is reachable.
  const auto arc = planner.select_stop_point(
    stop_lines, trajectory, /*ego_arc_length=*/0.0, /*ego_velocity=*/0.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  ASSERT_TRUE(arc.has_value());
  // crossing (20) - front offset (4) - margin (1) = 15
  EXPECT_NEAR(arc->arc_length, 15.0, 1e-6);
}

TEST(MapBasedStopPlannerTest, SelectAppliesCrosswalkMarginWithoutExplicitStopLine)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  const auto trajectory = make_straight_trajectory(30);

  // A walkway bound used as the stop line because the map has no explicit stop line: the
  // stop_distance_from_crosswalk margin applies on top of stop_margin_distance.
  auto stop_line = make_crossing_stop_line(1, 20.0, StopLineType::Walkway);
  stop_line.without_explicit_stop_line = true;
  const std::vector<StopLine> stop_lines{stop_line};

  const auto arc = planner.select_stop_point(
    stop_lines, trajectory, /*ego_arc_length=*/0.0, /*ego_velocity=*/0.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  ASSERT_TRUE(arc.has_value());
  // crossing (20) - front offset (4) - (stop_margin (1) + stop_distance_from_crosswalk (3.5))
  EXPECT_NEAR(arc->arc_length, 11.5, 1e-6);
}

TEST(MapBasedStopPlannerTest, SelectAppliesIntersectionMargin)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  const auto trajectory = make_straight_trajectory(30);

  // An intersection conflict line: the stop_distance_from_intersection margin applies on top of
  // stop_margin_distance so the vehicle keeps a gap to the priority lane.
  const std::vector<StopLine> stop_lines{
    make_crossing_stop_line(1, 20.0, StopLineType::Intersection)};

  const auto arc = planner.select_stop_point(
    stop_lines, trajectory, /*ego_arc_length=*/0.0, /*ego_velocity=*/0.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/true);
  ASSERT_TRUE(arc.has_value());
  // crossing (20) - front offset (4) - (stop_margin (1) + stop_distance_from_intersection (1))
  EXPECT_NEAR(arc->arc_length, 14.0, 1e-6);
}

TEST(MapBasedStopPlannerTest, SelectAppliesPrivateAreaMargin)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  const auto trajectory = make_straight_trajectory(30);

  // A private-area boundary line: the stop_distance_from_private_area margin applies on top of
  // stop_margin_distance so the vehicle keeps a gap to the boundary.
  const std::vector<StopLine> stop_lines{
    make_crossing_stop_line(1, 20.0, StopLineType::PrivateArea)};

  const auto arc = planner.select_stop_point(
    stop_lines, trajectory, /*ego_arc_length=*/0.0, /*ego_velocity=*/0.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/true);
  ASSERT_TRUE(arc.has_value());
  // crossing (20) - front offset (4) - (stop_margin (1) + stop_distance_from_private_area (3))
  EXPECT_NEAR(arc->arc_length, 12.0, 1e-6);
}

TEST(MapBasedStopPlannerTest, SelectPicksNearest)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  const auto trajectory = make_straight_trajectory(60);
  const std::vector<StopLine> stop_lines{
    make_crossing_stop_line(1, 40.0), make_crossing_stop_line(2, 20.0)};

  const auto arc = planner.select_stop_point(
    stop_lines, trajectory, /*ego_arc_length=*/0.0, /*ego_velocity=*/0.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  ASSERT_TRUE(arc.has_value());
  // nearest crossing is 20 -> 20 - 4 - 1 = 15
  EXPECT_NEAR(arc->arc_length, 15.0, 1e-6);
}

TEST(MapBasedStopPlannerTest, SelectRejectsUnreachableStopPoint)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  const auto trajectory = make_straight_trajectory(30);
  const std::vector<StopLine> stop_lines{make_crossing_stop_line(1, 20.0)};

  // Stop point arc = 15 m. The jerk-aware braking distance at 20 m/s (a = 4, j = 5) is about
  // 57.9 m > 15 m, so the vehicle cannot stop in time and no stop point should be selected.
  const auto arc = planner.select_stop_point(
    stop_lines, trajectory, /*ego_arc_length=*/0.0, /*ego_velocity=*/20.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  EXPECT_FALSE(arc.has_value());
}

TEST(MapBasedStopPlannerTest, SelectSkipsUnreachableNearestForReachableFarther)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  const auto trajectory = make_straight_trajectory(80);
  // Nearest crossing at 20 m (stop point 15 m) is unreachable at 20 m/s (jerk-aware braking
  // distance ~57.9 m); a farther crossing at 70 m (stop point 65 m) is reachable and selected.
  const std::vector<StopLine> stop_lines{
    make_crossing_stop_line(1, 20.0), make_crossing_stop_line(2, 70.0)};

  const auto arc = planner.select_stop_point(
    stop_lines, trajectory, /*ego_arc_length=*/0.0, /*ego_velocity=*/20.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  ASSERT_TRUE(arc.has_value());
  // 70 - 4 - 1 = 65
  EXPECT_NEAR(arc->arc_length, 65.0, 1e-6);
}

TEST(MapBasedStopPlannerTest, SelectExcludesPossibilityTargetsForGoTrajectory)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  const auto trajectory = make_straight_trajectory(30);
  // Only a signal (possibility) target crosses the trajectory.
  const std::vector<StopLine> stop_lines{
    make_crossing_stop_line(1, 20.0, StopLineType::TrafficLight)};

  // Go trajectory: possibility targets are ignored -> no stop.
  EXPECT_FALSE(planner
                 .select_stop_point(
                   stop_lines, trajectory, /*ego_arc_length=*/0.0, /*ego_velocity=*/0.0,
                   /*ego_acceleration=*/0.0, make_params(), /*include_possibility=*/false)
                 .has_value());
  // Stop trajectory: possibility targets are considered -> stop.
  EXPECT_TRUE(planner
                .select_stop_point(
                  stop_lines, trajectory, /*ego_arc_length=*/0.0, /*ego_velocity=*/0.0,
                  /*ego_acceleration=*/0.0, make_params(), /*include_possibility=*/true)
                .has_value());
}

TEST(MapBasedStopPlannerTest, SelectMeasuresBrakingDistanceFromEgo)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  // The trajectory starts behind the vehicle (backward path length), so arc lengths from the
  // trajectory start must not be used as the remaining distance.
  const auto trajectory = make_straight_trajectory(30);
  const std::vector<StopLine> stop_lines{make_crossing_stop_line(1, 20.0)};

  // Stop point arc = 15 m from the trajectory start, but ego is at x = 10, so the remaining
  // distance is only 5 m. The jerk-aware braking distance at 8 m/s (a = 4, j = 5) is ~11.1 m
  // > 5 m -> must be skipped. (Judged from the trajectory start, 15 m > 11.1 m would wrongly
  // keep it.)
  const auto arc = planner.select_stop_point(
    stop_lines, trajectory, /*ego_arc_length=*/10.0, /*ego_velocity=*/8.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  EXPECT_FALSE(arc.has_value());

  // With enough remaining distance (ego at x = 2 -> 13 m > 11.1 m) the stop point is kept.
  const auto arc_reachable = planner.select_stop_point(
    stop_lines, trajectory, /*ego_arc_length=*/2.0, /*ego_velocity=*/8.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  ASSERT_TRUE(arc_reachable.has_value());
  EXPECT_NEAR(arc_reachable->arc_length, 15.0, 1e-6);
}

TEST(MapBasedStopPlannerTest, SelectUsesJerkAwareBrakingDistance)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  const auto trajectory = make_straight_trajectory(30);
  const std::vector<StopLine> stop_lines{make_crossing_stop_line(1, 20.0)};

  // Remaining distance from ego (x = 5) to the stop point (arc 15) is 10 m. The constant-decel
  // braking distance 8^2/(2*4) = 8 m would keep the candidate, but the jerk-limited ramp-up to
  // the maximum deceleration (j = 5) stretches the real braking distance to ~11.1 m, so the
  // candidate must be skipped.
  const auto arc = planner.select_stop_point(
    stop_lines, trajectory, /*ego_arc_length=*/5.0, /*ego_velocity=*/8.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  EXPECT_FALSE(arc.has_value());
}

TEST(MapBasedStopPlannerTest, SelectSkipsStopPointPassedByEgo)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  const auto trajectory = make_straight_trajectory(30);
  const std::vector<StopLine> stop_lines{make_crossing_stop_line(1, 20.0)};

  // Stop point arc = 15 m; ego (base_link) is already at x = 16, past the stop point. Even though
  // the arc length from the trajectory start is positive, the candidate must be dropped.
  const auto arc = planner.select_stop_point(
    stop_lines, trajectory, /*ego_arc_length=*/16.0, /*ego_velocity=*/1.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  EXPECT_FALSE(arc.has_value());
}

// ============================================================
// collect_stop_lines
// ============================================================

namespace
{
// A stop line crossing the straight test road at x = 5.
lanelet::LineString3d make_map_stop_line(lanelet::Id id)
{
  lanelet::LineString3d stop_line(
    id, {lanelet::Point3d(id + 1, 5.0, -1.0, 0.0), lanelet::Point3d(id + 2, 5.0, 1.0, 0.0)});
  stop_line.attributes()[lanelet::AttributeNamesString::Type] =
    lanelet::AttributeValueString::StopLine;
  return stop_line;
}

// A traffic sign regulatory element (subtype stop_sign) whose ref_line is the given stop line.
lanelet::RegulatoryElementPtr make_stop_sign_regelem(
  lanelet::Id id, const lanelet::LineString3d & stop_line)
{
  lanelet::LineString3d sign(
    id + 10, {lanelet::Point3d(id + 11, 5.0, 2.0, 0.0), lanelet::Point3d(id + 12, 5.0, 2.5, 0.0)});
  sign.attributes()[lanelet::AttributeNamesString::Type] = "traffic_sign";
  sign.attributes()[lanelet::AttributeNamesString::Subtype] = "stop_sign";
  return lanelet::TrafficSign::make(
    id, lanelet::AttributeMap(), {{sign}, "stop_sign"}, {}, {stop_line});
}
}  // namespace

TEST(MapBasedStopPlannerTest, CollectTagsTrafficSignRefLine)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  // Attach a stop sign whose ref_line is the stop line to a lanelet (vm-02-02).
  const auto stop_line = make_map_stop_line(100);
  lanelet::LineString3d left(
    200, {lanelet::Point3d(201, 0.0, 1.0, 0.0), lanelet::Point3d(202, 10.0, 1.0, 0.0)});
  lanelet::LineString3d right(
    300, {lanelet::Point3d(301, 0.0, -1.0, 0.0), lanelet::Point3d(302, 10.0, -1.0, 0.0)});
  lanelet::Lanelet lanelet(400, left, right);
  lanelet.addRegulatoryElement(make_stop_sign_regelem(500, stop_line));

  RouteContext ctx;
  ctx.route_lanelets = {lanelet};
  const auto stop_lines = planner.collect_stop_lines(ctx);

  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().line.id(), 100);
  EXPECT_EQ(stop_lines.front().type, StopLineType::StopLine);
}

TEST(MapBasedStopPlannerTest, CollectIgnoresRoadMarkingStopLine)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  // A stop line referenced only by a RoadMarking regulatory element (vm-03-14 intersection
  // guidance line) is not a legally mandatory stop and must not be collected.
  const auto stop_line = make_map_stop_line(100);
  lanelet::LineString3d left(
    200, {lanelet::Point3d(201, 0.0, 1.0, 0.0), lanelet::Point3d(202, 10.0, 1.0, 0.0)});
  lanelet::LineString3d right(
    300, {lanelet::Point3d(301, 0.0, -1.0, 0.0), lanelet::Point3d(302, 10.0, -1.0, 0.0)});
  lanelet::Lanelet lanelet(400, left, right);
  lanelet.addRegulatoryElement(
    lanelet::autoware::RoadMarking::make(500, lanelet::AttributeMap(), stop_line));

  RouteContext ctx;
  ctx.route_lanelets = {lanelet};
  const auto stop_lines = planner.collect_stop_lines(ctx);

  EXPECT_TRUE(stop_lines.empty());
}

namespace
{
// Build a straight road lanelet along +x, spanning [x_start, x_start + length] in x, 4 m wide.
lanelet::Lanelet make_road_lanelet(lanelet::Id id, double x_start, double length)
{
  const double x_end = x_start + length;
  const lanelet::Id base = id * 100;
  lanelet::LineString3d left(
    base + 1,
    {lanelet::Point3d(base + 2, x_start, 2.0, 0.0), lanelet::Point3d(base + 3, x_end, 2.0, 0.0)});
  lanelet::LineString3d right(
    base + 4,
    {lanelet::Point3d(base + 5, x_start, -2.0, 0.0), lanelet::Point3d(base + 6, x_end, -2.0, 0.0)});
  return lanelet::Lanelet(id, left, right);
}
}  // namespace

TEST(MapBasedStopPlannerTest, CollectFallsBackToIntersectionEntryEdgeWithoutRoutingGraph)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  auto approach = make_road_lanelet(1, 0.0, 10.0);
  auto intersection = make_road_lanelet(2, 10.0, 10.0);
  intersection.attributes()["turn_direction"] = "right";

  // Without a routing graph the conflict topology is unknown: the entry edge is used.
  RouteContext ctx;
  ctx.preferred_lanelets = {approach, intersection};
  const auto stop_lines = planner.collect_stop_lines(ctx);

  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::Intersection);
  // Entry edge is at the intersection lanelet start (x = 10).
  EXPECT_NEAR(stop_lines.front().line.front().x(), 10.0, 1e-6);
  // Synthesized edges carry a unique id (negated source lanelet id): the marker visualization
  // de-duplicates line strings by id, so sharing InvalId would drop every synthesized line but the
  // first.
  EXPECT_EQ(stop_lines.front().line.id(), -2);
}

namespace
{
// A road lanelet the German vehicle traffic rules accept, so it takes part in the routing graph.
lanelet::Lanelet make_routable_lanelet(lanelet::Id id, double x_start, double length)
{
  auto lanelet = make_road_lanelet(id, x_start, length);
  lanelet.attributes()[lanelet::AttributeNamesString::Subtype] =
    lanelet::AttributeValueString::Road;
  lanelet.attributes()[lanelet::AttributeNamesString::Location] =
    lanelet::AttributeValueString::Urban;
  lanelet.attributes()["one_way"] = "yes";
  return lanelet;
}

// A routable lanelet along +y spanning x in [x_start, x_start + 4], crossing the +x road.
lanelet::Lanelet make_crossing_road_lanelet(lanelet::Id id, double x_start)
{
  const lanelet::Id base = id * 100;
  lanelet::LineString3d left(
    base + 1, {lanelet::Point3d(base + 2, x_start + 4.0, -10.0, 0.0),
               lanelet::Point3d(base + 3, x_start + 4.0, 10.0, 0.0)});
  lanelet::LineString3d right(
    base + 4, {lanelet::Point3d(base + 5, x_start, -10.0, 0.0),
               lanelet::Point3d(base + 6, x_start, 10.0, 0.0)});
  lanelet::Lanelet lanelet(id, left, right);
  lanelet.attributes()[lanelet::AttributeNamesString::Subtype] =
    lanelet::AttributeValueString::Road;
  lanelet.attributes()[lanelet::AttributeNamesString::Location] =
    lanelet::AttributeValueString::Urban;
  lanelet.attributes()["one_way"] = "yes";
  return lanelet;
}

// A route context whose routing graph is built from the given lanelets (German vehicle rules).
RouteContext make_routing_graph_context(const std::vector<lanelet::Lanelet> & lanelets)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  for (const auto & lanelet : lanelets) {
    map->add(lanelet);
  }
  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  RouteContext ctx;
  ctx.lanelet_map_ptr = map;
  ctx.routing_graph_ptr = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
  return ctx;
}
}  // namespace

TEST(MapBasedStopPlannerTest, CollectStopsBeforeConflictingPriorityLane)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  auto approach = make_routable_lanelet(1, 0.0, 10.0);
  auto intersection = make_routable_lanelet(2, 10.0, 10.0);
  intersection.attributes()["turn_direction"] = "right";
  // A priority lane crossing the intersection lanelet at x in [14, 18]; no RightOfWay grants ego
  // priority over it, so ego is non-priority and must stop before entering it.
  auto crossing = make_crossing_road_lanelet(3, 14.0);

  auto ctx = make_routing_graph_context({approach, intersection, crossing});
  ctx.preferred_lanelets = {approach, intersection};
  const auto stop_lines = planner.collect_stop_lines(ctx);

  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::Intersection);
  // The stop line sits where the route centerline enters the crossing lane (x = 14), not at the
  // intersection lanelet entry (x = 10).
  const auto & line = stop_lines.front().line;
  EXPECT_NEAR(0.5 * (line.front().x() + line.back().x()), 14.0, 0.1);
  EXPECT_NEAR(0.5 * (line.front().y() + line.back().y()), 0.0, 0.1);
}

TEST(MapBasedStopPlannerTest, CollectSkipsIntersectionWhenEgoHasPriority)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  auto approach = make_routable_lanelet(1, 0.0, 10.0);
  auto intersection = make_routable_lanelet(2, 10.0, 10.0);
  intersection.attributes()["turn_direction"] = "right";
  auto crossing = make_crossing_road_lanelet(3, 14.0);
  // A RightOfWay regulatory element grants ego priority: the crossing lane yields to ego.
  intersection.addRegulatoryElement(
    lanelet::RightOfWay::make(999, lanelet::AttributeMap(), {intersection}, {crossing}));

  auto ctx = make_routing_graph_context({approach, intersection, crossing});
  ctx.preferred_lanelets = {approach, intersection};
  const auto stop_lines = planner.collect_stop_lines(ctx);

  EXPECT_TRUE(stop_lines.empty());
}

TEST(MapBasedStopPlannerTest, CollectSkipsIntersectionWithoutConflictingLane)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  // A branching turn (turn_direction present) that crosses no other lane: no stop line.
  auto approach = make_routable_lanelet(1, 0.0, 10.0);
  auto intersection = make_routable_lanelet(2, 10.0, 10.0);
  intersection.attributes()["turn_direction"] = "left";

  auto ctx = make_routing_graph_context({approach, intersection});
  ctx.preferred_lanelets = {approach, intersection};
  const auto stop_lines = planner.collect_stop_lines(ctx);

  EXPECT_TRUE(stop_lines.empty());
}

TEST(MapBasedStopPlannerTest, CollectSkipsSignalizedStraightIntersection)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  auto approach = make_routable_lanelet(1, 0.0, 10.0);
  auto intersection = make_routable_lanelet(2, 10.0, 10.0);
  intersection.attributes()["turn_direction"] = "straight";
  auto crossing = make_crossing_road_lanelet(3, 14.0);
  // A signalized straight lane is arbitrated by the signal: no intersection stop line even
  // though a conflicting lane exists.
  lanelet::LineString3d light_bar(
    800, {lanelet::Point3d(801, 12.0, -2.0, 3.0), lanelet::Point3d(802, 12.0, 2.0, 3.0)});
  intersection.addRegulatoryElement(
    lanelet::TrafficLight::make(998, lanelet::AttributeMap(), {light_bar}));

  auto ctx = make_routing_graph_context({approach, intersection, crossing});
  ctx.preferred_lanelets = {approach, intersection};
  const auto stop_lines = planner.collect_stop_lines(ctx);

  EXPECT_TRUE(stop_lines.empty());
}

TEST(MapBasedStopPlannerTest, CollectDetectsPrivateAreaTransition)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  auto public_lanelet = make_road_lanelet(1, 0.0, 10.0);
  auto private_lanelet = make_road_lanelet(2, 10.0, 10.0);
  private_lanelet.attributes()[lanelet::AttributeNamesString::Location] =
    lanelet::AttributeValueString::Private;

  // Private-area detection scans the preferred (single-chain) lane sequence.
  RouteContext ctx;
  ctx.preferred_lanelets = {public_lanelet, private_lanelet};
  const auto stop_lines = planner.collect_stop_lines(ctx);

  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::PrivateArea);
  EXPECT_NEAR(stop_lines.front().line.front().x(), 10.0, 1e-6);
  EXPECT_EQ(stop_lines.front().line.id(), -2);
}

TEST(MapBasedStopPlannerTest, CollectPlacesPrivateEntryAtRoadDeparture)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  // Route: public road x[0,10], then a private connector branching off to the upper side.
  auto route_road = make_road_lanelet(1, 0.0, 10.0);
  lanelet::LineString3d conn_left(
    200, {lanelet::Point3d(201, 10.0, 2.0, 0.0), lanelet::Point3d(202, 20.0, 12.0, 0.0)});
  lanelet::LineString3d conn_right(
    210, {lanelet::Point3d(211, 10.0, -2.0, 0.0), lanelet::Point3d(212, 24.0, 8.0, 0.0)});
  lanelet::Lanelet connector(2, conn_left, conn_right);
  connector.attributes()[lanelet::AttributeNamesString::Location] =
    lanelet::AttributeValueString::Private;

  // The straight public road continues (map only, not on the route) and overlaps the connector.
  auto straight_road = make_road_lanelet(9, 10.0, 20.0);
  straight_road.attributes()[lanelet::AttributeNamesString::Subtype] =
    lanelet::AttributeValueString::Road;

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(straight_road);

  RouteContext ctx;
  ctx.lanelet_map_ptr = map;
  ctx.preferred_lanelets = {route_road, connector};
  const auto stop_lines = planner.collect_stop_lines(ctx);

  // The stop line sits where the connector centerline leaves the straight road (y = 2), not at
  // the connector start edge (x = 10, on the roadway).
  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::PrivateArea);
  const auto & line = stop_lines.front().line;
  const double mid_x = 0.5 * (line.front().x() + line.back().x());
  const double mid_y = 0.5 * (line.front().y() + line.back().y());
  EXPECT_NEAR(mid_y, 2.0, 0.2);
  EXPECT_GT(mid_x, 10.5);
  EXPECT_LT(mid_x, 20.0);
}

TEST(MapBasedStopPlannerTest, CollectPlacesPrivateExitAtRoadEntry)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  // Route: a private connector descending from the premises into the road, then the public road.
  // (travel direction is down-right, so the +x-shifted bound is the left one)
  lanelet::LineString3d conn_left(
    200, {lanelet::Point3d(201, 6.0, 10.0, 0.0), lanelet::Point3d(202, 14.0, -2.0, 0.0)});
  lanelet::LineString3d conn_right(
    210, {lanelet::Point3d(211, 2.0, 10.0, 0.0), lanelet::Point3d(212, 10.0, -2.0, 0.0)});
  lanelet::Lanelet connector(2, conn_left, conn_right);
  connector.attributes()[lanelet::AttributeNamesString::Location] =
    lanelet::AttributeValueString::Private;
  auto merged_road = make_road_lanelet(3, 12.0, 18.0);

  // The public road being entered (map only) overlaps the connector's lower part.
  auto road = make_road_lanelet(9, 0.0, 30.0);
  road.attributes()[lanelet::AttributeNamesString::Subtype] = lanelet::AttributeValueString::Road;

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(road);

  RouteContext ctx;
  ctx.lanelet_map_ptr = map;
  ctx.preferred_lanelets = {connector, merged_road};
  const auto stop_lines = planner.collect_stop_lines(ctx);

  // The stop line sits where the connector centerline first enters the road (y = 2), i.e. before
  // merging — upstream merge_from_private semantics — not at the merged lanelet start (x = 12).
  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::PrivateArea);
  const auto & line = stop_lines.front().line;
  const double mid_x = 0.5 * (line.front().x() + line.back().x());
  const double mid_y = 0.5 * (line.front().y() + line.back().y());
  EXPECT_NEAR(mid_y, 2.0, 0.3);
  EXPECT_GT(mid_x, 5.0);
  EXPECT_LT(mid_x, 13.0);
}

TEST(MapBasedStopPlannerTest, CollectDetectsWalkwayCrossingEntrySideOnly)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  // Road along +x; the ego drives from x=0 toward x=20.
  auto road = make_road_lanelet(1, 0.0, 20.0);

  // A walkway lanelet crossing the road at x ~ 10, its two bounds at x=9 (near) and x=11 (far).
  lanelet::LineString3d near_bound(
    700, {lanelet::Point3d(701, 9.0, -3.0, 0.0), lanelet::Point3d(702, 9.0, 3.0, 0.0)});
  lanelet::LineString3d far_bound(
    710, {lanelet::Point3d(711, 11.0, -3.0, 0.0), lanelet::Point3d(712, 11.0, 3.0, 0.0)});
  lanelet::Lanelet walkway(720, near_bound, far_bound);
  walkway.attributes()[lanelet::AttributeNamesString::Subtype] =
    lanelet::AttributeValueString::Walkway;

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(walkway);

  RouteContext ctx;
  ctx.lanelet_map_ptr = map;
  ctx.preferred_lanelets = {road};
  const auto stop_lines = planner.collect_stop_lines(ctx);

  // Only the entry-side (near, x=9) bound is used, not both bounds. The bound is a fallback for a
  // missing explicit stop line, so the crosswalk stop distance applies.
  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::Walkway);
  EXPECT_EQ(stop_lines.front().line.id(), 700);
  EXPECT_TRUE(stop_lines.front().without_explicit_stop_line);
}

namespace
{
// A walkway lanelet crossing the +x road at x in [9, 11].
lanelet::Lanelet make_crossing_walkway(lanelet::Id id)
{
  const lanelet::Id base = id * 10;
  lanelet::LineString3d near_bound(
    base, {lanelet::Point3d(base + 1, 9.0, -3.0, 0.0), lanelet::Point3d(base + 2, 9.0, 3.0, 0.0)});
  lanelet::LineString3d far_bound(
    base + 3,
    {lanelet::Point3d(base + 4, 11.0, -3.0, 0.0), lanelet::Point3d(base + 5, 11.0, 3.0, 0.0)});
  lanelet::Lanelet walkway(id, near_bound, far_bound);
  walkway.attributes()[lanelet::AttributeNamesString::Subtype] =
    lanelet::AttributeValueString::Walkway;
  return walkway;
}
}  // namespace

TEST(MapBasedStopPlannerTest, CollectPrefersCrosswalkRegelemStopLine)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  auto road = make_road_lanelet(1, 0.0, 20.0);
  auto walkway = make_crossing_walkway(720);

  // Explicit stop line provided by a crosswalk regulatory element attached to the road lanelet.
  lanelet::LineString3d explicit_stop_line(
    800, {lanelet::Point3d(801, 7.0, -2.0, 0.0), lanelet::Point3d(802, 7.0, 2.0, 0.0)});
  lanelet::Polygon3d crosswalk_area;
  road.addRegulatoryElement(
    lanelet::autoware::Crosswalk::make(
      900, lanelet::AttributeMap(), walkway, crosswalk_area, {explicit_stop_line}));

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(walkway);

  RouteContext ctx;
  ctx.lanelet_map_ptr = map;
  ctx.route_lanelets = {road};
  ctx.preferred_lanelets = {road};
  const auto stop_lines = planner.collect_stop_lines(ctx);

  // The regelem stop line (x=7) wins over the walkway's entry-side bound (x=9).
  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::Walkway);
  EXPECT_EQ(stop_lines.front().line.id(), 800);
  EXPECT_FALSE(stop_lines.front().without_explicit_stop_line);
}

TEST(MapBasedStopPlannerTest, CollectPrefersCrosswalkIdRoadMarkingStopLine)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  auto road = make_road_lanelet(1, 0.0, 20.0);
  auto walkway = make_crossing_walkway(720);

  // Explicit stop line bound to the walkway via a RoadMarking with a crosswalk_id attribute.
  lanelet::LineString3d explicit_stop_line(
    800, {lanelet::Point3d(801, 7.5, -2.0, 0.0), lanelet::Point3d(802, 7.5, 2.0, 0.0)});
  explicit_stop_line.attributes()[lanelet::AttributeNamesString::Type] =
    lanelet::AttributeValueString::StopLine;
  explicit_stop_line.attributes()["crosswalk_id"] = 720;
  walkway.addRegulatoryElement(
    lanelet::autoware::RoadMarking::make(900, lanelet::AttributeMap(), explicit_stop_line));

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(walkway);

  RouteContext ctx;
  ctx.lanelet_map_ptr = map;
  ctx.preferred_lanelets = {road};
  const auto stop_lines = planner.collect_stop_lines(ctx);

  // The crosswalk_id road marking (x=7.5) wins over the walkway's entry-side bound (x=9).
  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::Walkway);
  EXPECT_EQ(stop_lines.front().line.id(), 800);
}

// ============================================================
// set_planner_data / plan
// ============================================================

namespace
{
// Seed the planner's stop line cache with a stop-sign ref_line (mandatory) crossing the
// straight test trajectory at x = 20.
void set_mandatory_stop_line_data(MapBasedStopPlanner & planner)
{
  lanelet::LineString3d stop_line(
    100, {lanelet::Point3d(101, 20.0, -2.0, 0.0), lanelet::Point3d(102, 20.0, 2.0, 0.0)});
  stop_line.attributes()[lanelet::AttributeNamesString::Type] =
    lanelet::AttributeValueString::StopLine;
  auto lanelet = make_road_lanelet(400, 0.0, 30.0);
  lanelet.addRegulatoryElement(make_stop_sign_regelem(500, stop_line));

  RouteContext ctx;
  ctx.route_lanelets = {lanelet};
  planner.set_planner_data(
    std::make_shared<LaneletMapBin>(), std::make_shared<LaneletRoute>(), ctx);
}

Trajectory make_straight_trajectory_msg(size_t num_points)
{
  Trajectory trajectory;
  trajectory.points = make_straight_trajectory(num_points);
  return trajectory;
}
}  // namespace

TEST(MapBasedStopPlannerTest, PlanEmbedsMandatoryStopInGoTrajectory)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  set_mandatory_stop_line_data(planner);

  const auto trajectory = make_straight_trajectory_msg(30);
  const auto result = planner.plan(
    trajectory, /*ego_arc_length=*/0.0, /*ego_velocity=*/0.0, /*ego_acceleration=*/0.0,
    make_params());

  // The go trajectory ends with a zero-velocity point at the stop point (20 - 4 - 1 = 15).
  ASSERT_FALSE(result.go_trajectory.points.empty());
  EXPECT_NEAR(result.go_trajectory.points.back().pose.position.x, 15.0, 1e-3);
  EXPECT_FLOAT_EQ(result.go_trajectory.points.back().longitudinal_velocity_mps, 0.0f);
  // The mandatory stop is shared by both passes, so no distinct stop trajectory is produced.
  EXPECT_FALSE(result.stop_trajectory.has_value());
  EXPECT_FALSE(result.stop_line_markers.markers.empty());
}

TEST(MapBasedStopPlannerTest, PlanReturnsStopTrajectoryOnlyForDistinctStopPoint)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  // Only a possibility target (intersection entry edge at x = 10, routing-graph-less fallback)
  // crosses the trajectory.
  auto intersection = make_road_lanelet(2, 10.0, 10.0);
  intersection.attributes()["turn_direction"] = "right";
  RouteContext ctx;
  ctx.preferred_lanelets = {intersection};
  planner.set_planner_data(
    std::make_shared<LaneletMapBin>(), std::make_shared<LaneletRoute>(), ctx);

  const auto trajectory = make_straight_trajectory_msg(30);
  const auto result = planner.plan(
    trajectory, /*ego_arc_length=*/0.0, /*ego_velocity=*/0.0, /*ego_acceleration=*/0.0,
    make_params());

  // The go trajectory ignores possibility targets and stays unchanged.
  ASSERT_EQ(result.go_trajectory.points.size(), trajectory.points.size());
  EXPECT_FLOAT_EQ(result.go_trajectory.points.back().longitudinal_velocity_mps, 5.0f);
  // The stop trajectory embeds the possibility stop (10 - 4 - (1 + 1) = 4), distinct from go.
  ASSERT_TRUE(result.stop_trajectory.has_value());
  EXPECT_NEAR(result.stop_trajectory->points.back().pose.position.x, 4.0, 1e-3);
  EXPECT_FLOAT_EQ(result.stop_trajectory->points.back().longitudinal_velocity_mps, 0.0f);
}

}  // namespace autoware::minimum_rule_based_planner
