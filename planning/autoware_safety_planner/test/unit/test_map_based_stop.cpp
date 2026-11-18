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

#include "constraint_generator/map_based_stop.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/route_manager.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <autoware_utils_debug/time_keeper.hpp>

#include <autoware_planning_msgs/msg/lanelet_segment.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner::experimental
{
namespace
{
using autoware::experimental::lanelet2_utils::RouteManager;

TrajectoryPoint make_point(double x, double y)
{
  TrajectoryPoint pt;
  pt.pose.position.x = x;
  pt.pose.position.y = y;
  pt.pose.orientation.w = 1.0;
  return pt;
}

// A straight trajectory along +x at y = 0, 1 m spacing.
TrajectoryPoints make_straight_trajectory(size_t num_points)
{
  TrajectoryPoints points;
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

// 5 m long, 2 m wide box with base_link 1 m behind the rear bumper.
VehicleInfo make_vehicle_info()
{
  VehicleInfo info;
  info.max_longitudinal_offset_m = 4.0;
  info.min_longitudinal_offset_m = -1.0;
  info.max_lateral_offset_m = 1.0;
  info.min_lateral_offset_m = -1.0;
  info.vehicle_width_m = 2.0;
  return info;
}

// The margins the original MapBasedStopPlanner tests judge the stop point with.
Params make_params()
{
  Params params;
  params.map_based_stop.stop_margin_distance = 1.0;
  params.map_based_stop.stop_distance_from_crosswalk = 3.5;
  params.map_based_stop.stop_distance_from_private_area = 3.0;
  params.map_based_stop.stop_distance_from_intersection = 1.0;
  params.map_based_stop.stop_distance_from_road_shoulder = 0.5;
  params.map_based_stop.max_deceleration = 4.0;
  params.map_based_stop.max_jerk = 5.0;
  return params;
}

MapBasedStopConstraintGenerator make_generator()
{
  MapBasedStopConstraintGenerator generator;
  generator.on_initialize(std::make_shared<autoware_utils_debug::TimeKeeper>(), make_params());
  return generator;
}

// Ego pose on the straight trajectory (y = 0, facing +x).
geometry_msgs::msg::Pose make_ego_pose(double x)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.orientation.w = 1.0;
  return pose;
}

std::optional<MapBasedStopConstraintGenerator::StopPoint> select_stop_point(
  MapBasedStopConstraintGenerator & generator, const std::vector<StopLine> & stop_lines,
  const TrajectoryPoints & trajectory, const geometry_msgs::msg::Pose & ego_pose, double velocity,
  double acceleration, bool possibility)
{
  return generator.select_stop_point(
    stop_lines, trajectory, ego_pose, velocity, acceleration, make_vehicle_info(), possibility);
}

lanelet::LaneletMapConstPtr to_const_shared(lanelet::LaneletMapUPtr map)
{
  return lanelet::LaneletMapConstPtr{std::move(map)};
}

std::vector<StopLine> collect_stop_lines(
  MapBasedStopConstraintGenerator & generator, const lanelet::ConstLanelets & lanelets,
  const lanelet::LaneletMapConstPtr & map = nullptr,
  const lanelet::routing::RoutingGraphConstPtr & graph = nullptr)
{
  MapBasedStopConstraintGenerator::IntersectionDebugLanelets debug;
  return generator.collect_stop_lines(lanelets, map, graph, debug);
}

}  // namespace

// ============================================================
// is_possibility_type
// ============================================================

TEST(MapBasedStopConstraintGeneratorTest, PossibilityTypeClassification)
{
  // Mandatory stop targets: emitted as Certainty::DEFINITE.
  EXPECT_FALSE(is_possibility_type(StopLineType::StopLine));
  EXPECT_FALSE(is_possibility_type(StopLineType::Walkway));
  // Possibility stop targets: emitted as Certainty::POSSIBLE.
  EXPECT_TRUE(is_possibility_type(StopLineType::Crosswalk));
  EXPECT_TRUE(is_possibility_type(StopLineType::TrafficLight));
  EXPECT_TRUE(is_possibility_type(StopLineType::Intersection));
  EXPECT_TRUE(is_possibility_type(StopLineType::PrivateArea));
  EXPECT_TRUE(is_possibility_type(StopLineType::RoadShoulder));
}

// ============================================================
// filter_stop_lines_on_trajectory
// ============================================================

TEST(MapBasedStopConstraintGeneratorTest, FilterKeepsOnlyCrossingLines)
{
  auto generator = make_generator();
  const auto trajectory = make_straight_trajectory(20);

  std::vector<StopLine> stop_lines;
  stop_lines.push_back(make_crossing_stop_line(1, 10.0));  // crosses the trajectory
  // A stop line far to the side that does not cross the trajectory.
  lanelet::LineString3d off_line(
    2, {lanelet::Point3d(21, 5.0, 10.0, 0.0), lanelet::Point3d(22, 5.0, 12.0, 0.0)});
  stop_lines.push_back(StopLine{off_line, StopLineType::StopLine});

  const auto filtered = generator.filter_stop_lines_on_trajectory(stop_lines, trajectory);
  ASSERT_EQ(filtered.size(), 1u);
  EXPECT_EQ(filtered.front().line.id(), 1);
}

// ============================================================
// select_stop_point
// ============================================================

TEST(MapBasedStopConstraintGeneratorTest, SelectAppliesFrontOffsetAndMargin)
{
  auto generator = make_generator();
  const auto trajectory = make_straight_trajectory(30);
  const std::vector<StopLine> stop_lines{make_crossing_stop_line(1, 20.0)};

  // Ego stopped: braking distance is 0, so the stop point is reachable.
  const auto stop = select_stop_point(
    generator, stop_lines, trajectory, make_ego_pose(0.0), /*velocity=*/0.0,
    /*acceleration=*/0.0, /*possibility=*/false);
  ASSERT_TRUE(stop.has_value());
  // crossing (20) - front offset (4) - margin (1) = 15
  EXPECT_NEAR(stop->arc_length, 15.0, 1e-6);
  EXPECT_EQ(stop->type, StopLineType::StopLine);
}

TEST(MapBasedStopConstraintGeneratorTest, SelectAppliesCrosswalkMarginWithoutExplicitStopLine)
{
  auto generator = make_generator();
  const auto trajectory = make_straight_trajectory(30);

  // A walkway bound used as the stop line because the map has no explicit stop line: the
  // stop_distance_from_crosswalk margin applies on top of stop_margin_distance.
  auto stop_line = make_crossing_stop_line(1, 20.0, StopLineType::Walkway);
  stop_line.without_explicit_stop_line = true;
  const std::vector<StopLine> stop_lines{stop_line};

  const auto stop = select_stop_point(
    generator, stop_lines, trajectory, make_ego_pose(0.0), 0.0, 0.0, /*possibility=*/false);
  ASSERT_TRUE(stop.has_value());
  // crossing (20) - front offset (4) - (stop_margin (1) + stop_distance_from_crosswalk (3.5))
  EXPECT_NEAR(stop->arc_length, 11.5, 1e-6);
}

TEST(MapBasedStopConstraintGeneratorTest, SelectAppliesIntersectionMargin)
{
  auto generator = make_generator();
  const auto trajectory = make_straight_trajectory(30);

  // An intersection conflict line: the stop_distance_from_intersection margin applies on top of
  // stop_margin_distance so the vehicle keeps a gap to the priority lane.
  const std::vector<StopLine> stop_lines{
    make_crossing_stop_line(1, 20.0, StopLineType::Intersection)};

  const auto stop = select_stop_point(
    generator, stop_lines, trajectory, make_ego_pose(0.0), 0.0, 0.0, /*possibility=*/true);
  ASSERT_TRUE(stop.has_value());
  // crossing (20) - front offset (4) - (stop_margin (1) + stop_distance_from_intersection (1))
  EXPECT_NEAR(stop->arc_length, 14.0, 1e-6);
}

TEST(MapBasedStopConstraintGeneratorTest, SelectAppliesPrivateAreaMargin)
{
  auto generator = make_generator();
  const auto trajectory = make_straight_trajectory(30);

  // A private-area boundary line: the stop_distance_from_private_area margin applies on top of
  // stop_margin_distance so the vehicle keeps a gap to the boundary.
  const std::vector<StopLine> stop_lines{
    make_crossing_stop_line(1, 20.0, StopLineType::PrivateArea)};

  const auto stop = select_stop_point(
    generator, stop_lines, trajectory, make_ego_pose(0.0), 0.0, 0.0, /*possibility=*/true);
  ASSERT_TRUE(stop.has_value());
  // crossing (20) - front offset (4) - (stop_margin (1) + stop_distance_from_private_area (3))
  EXPECT_NEAR(stop->arc_length, 12.0, 1e-6);
}

TEST(MapBasedStopConstraintGeneratorTest, SelectPicksNearest)
{
  auto generator = make_generator();
  const auto trajectory = make_straight_trajectory(60);
  const std::vector<StopLine> stop_lines{
    make_crossing_stop_line(1, 40.0), make_crossing_stop_line(2, 20.0)};

  const auto stop =
    select_stop_point(generator, stop_lines, trajectory, make_ego_pose(0.0), 0.0, 0.0, false);
  ASSERT_TRUE(stop.has_value());
  // nearest crossing is 20 -> 20 - 4 - 1 = 15
  EXPECT_NEAR(stop->arc_length, 15.0, 1e-6);
}

TEST(MapBasedStopConstraintGeneratorTest, SelectRejectsUnreachableStopPoint)
{
  auto generator = make_generator();
  const auto trajectory = make_straight_trajectory(30);
  const std::vector<StopLine> stop_lines{make_crossing_stop_line(1, 20.0)};

  // Stop point arc = 15 m. The jerk-aware braking distance at 20 m/s (a = 4, j = 5) is about
  // 57.9 m > 15 m, so the vehicle cannot stop in time and no stop point should be selected.
  const auto stop = select_stop_point(
    generator, stop_lines, trajectory, make_ego_pose(0.0), /*velocity=*/20.0, 0.0, false);
  EXPECT_FALSE(stop.has_value());
}

TEST(MapBasedStopConstraintGeneratorTest, SelectSkipsUnreachableNearestForReachableFarther)
{
  auto generator = make_generator();
  const auto trajectory = make_straight_trajectory(80);
  // Nearest crossing at 20 m (stop point 15 m) is unreachable at 20 m/s (jerk-aware braking
  // distance ~57.9 m); a farther crossing at 70 m (stop point 65 m) is reachable and selected.
  const std::vector<StopLine> stop_lines{
    make_crossing_stop_line(1, 20.0), make_crossing_stop_line(2, 70.0)};

  const auto stop = select_stop_point(
    generator, stop_lines, trajectory, make_ego_pose(0.0), /*velocity=*/20.0, 0.0, false);
  ASSERT_TRUE(stop.has_value());
  // 70 - 4 - 1 = 65
  EXPECT_NEAR(stop->arc_length, 65.0, 1e-6);
}

TEST(MapBasedStopConstraintGeneratorTest, SelectSplitsMandatoryAndPossibilityTargets)
{
  auto generator = make_generator();
  const auto trajectory = make_straight_trajectory(30);
  // Only a signal (possibility) target crosses the trajectory.
  const std::vector<StopLine> stop_lines{
    make_crossing_stop_line(1, 20.0, StopLineType::TrafficLight)};

  // Mandatory pass: possibility targets are ignored -> no stop.
  EXPECT_FALSE(
    select_stop_point(generator, stop_lines, trajectory, make_ego_pose(0.0), 0.0, 0.0, false)
      .has_value());
  // Possibility pass: the signal is considered -> stop.
  EXPECT_TRUE(
    select_stop_point(generator, stop_lines, trajectory, make_ego_pose(0.0), 0.0, 0.0, true)
      .has_value());
}

TEST(MapBasedStopConstraintGeneratorTest, SelectMeasuresBrakingDistanceFromEgo)
{
  auto generator = make_generator();
  // The trajectory starts behind the vehicle, so arc lengths from the trajectory start must not
  // be used as the remaining distance.
  const auto trajectory = make_straight_trajectory(30);
  const std::vector<StopLine> stop_lines{make_crossing_stop_line(1, 20.0)};

  // Stop point arc = 15 m from the trajectory start, but ego is at x = 10, so the remaining
  // distance is only 5 m. The jerk-aware braking distance at 8 m/s (a = 4, j = 5) is ~11.1 m
  // > 5 m -> must be skipped. (Judged from the trajectory start, 15 m > 11.1 m would wrongly
  // keep it.)
  const auto stop = select_stop_point(
    generator, stop_lines, trajectory, make_ego_pose(10.0), /*velocity=*/8.0, 0.0, false);
  EXPECT_FALSE(stop.has_value());

  // With enough remaining distance (ego at x = 2 -> 13 m > 11.1 m) the stop point is kept.
  const auto reachable = select_stop_point(
    generator, stop_lines, trajectory, make_ego_pose(2.0), /*velocity=*/8.0, 0.0, false);
  ASSERT_TRUE(reachable.has_value());
  EXPECT_NEAR(reachable->arc_length, 15.0, 1e-6);
}

TEST(MapBasedStopConstraintGeneratorTest, SelectUsesJerkAwareBrakingDistance)
{
  auto generator = make_generator();
  const auto trajectory = make_straight_trajectory(30);
  const std::vector<StopLine> stop_lines{make_crossing_stop_line(1, 20.0)};

  // Remaining distance from ego (x = 5) to the stop point (arc 15) is 10 m. The constant-decel
  // braking distance 8^2/(2*4) = 8 m would keep the candidate, but the jerk-limited ramp-up to
  // the maximum deceleration (j = 5) stretches the real braking distance to ~11.1 m, so the
  // candidate must be skipped.
  const auto stop = select_stop_point(
    generator, stop_lines, trajectory, make_ego_pose(5.0), /*velocity=*/8.0, 0.0, false);
  EXPECT_FALSE(stop.has_value());
}

TEST(MapBasedStopConstraintGeneratorTest, SelectSkipsStopPointPassedByEgo)
{
  auto generator = make_generator();
  const auto trajectory = make_straight_trajectory(30);
  const std::vector<StopLine> stop_lines{make_crossing_stop_line(1, 20.0)};

  // Stop point arc = 15 m; ego (base_link) is already at x = 16, past the stop point. Even though
  // the arc length from the trajectory start is positive, the candidate must be dropped.
  const auto stop = select_stop_point(
    generator, stop_lines, trajectory, make_ego_pose(16.0), /*velocity=*/1.0, 0.0, false);
  EXPECT_FALSE(stop.has_value());
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

// A routing graph built from the given lanelets (German vehicle rules).
lanelet::routing::RoutingGraphPtr make_routing_graph(const lanelet::LaneletMap & map)
{
  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  return lanelet::routing::RoutingGraph::build(map, *traffic_rules);
}

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

TEST(MapBasedStopConstraintGeneratorTest, CollectTagsTrafficSignRefLine)
{
  auto generator = make_generator();

  // Attach a stop sign whose ref_line is the stop line to a lanelet (vm-02-02).
  const auto stop_line = make_map_stop_line(100);
  auto lanelet = make_road_lanelet(400, 0.0, 10.0);
  lanelet.addRegulatoryElement(make_stop_sign_regelem(500, stop_line));

  const auto stop_lines = collect_stop_lines(generator, {lanelet});

  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().line.id(), 100);
  EXPECT_EQ(stop_lines.front().type, StopLineType::StopLine);
}

TEST(MapBasedStopConstraintGeneratorTest, CollectIgnoresRoadMarkingStopLine)
{
  auto generator = make_generator();

  // A stop line referenced only by a RoadMarking regulatory element (vm-03-14 intersection
  // guidance line) is not a legally mandatory stop and must not be collected.
  const auto stop_line = make_map_stop_line(100);
  auto lanelet = make_road_lanelet(400, 0.0, 10.0);
  lanelet.addRegulatoryElement(
    lanelet::autoware::RoadMarking::make(500, lanelet::AttributeMap(), stop_line));

  const auto stop_lines = collect_stop_lines(generator, {lanelet});

  EXPECT_TRUE(stop_lines.empty());
}

TEST(
  MapBasedStopConstraintGeneratorTest, CollectFallsBackToIntersectionEntryEdgeWithoutRoutingGraph)
{
  auto generator = make_generator();

  auto approach = make_road_lanelet(1, 0.0, 10.0);
  auto intersection = make_road_lanelet(2, 10.0, 10.0);
  intersection.attributes()["turn_direction"] = "right";

  // Without a routing graph the conflict topology is unknown: the entry edge is used.
  const auto stop_lines = collect_stop_lines(generator, {approach, intersection});

  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::Intersection);
  // Entry edge is at the intersection lanelet start (x = 10).
  EXPECT_NEAR(stop_lines.front().line.front().x(), 10.0, 1e-6);
  // Synthesized edges carry a unique id (negated source lanelet id): the marker visualization
  // de-duplicates line strings by id, so sharing InvalId would drop every synthesized line but the
  // first.
  EXPECT_EQ(stop_lines.front().line.id(), -2);
}

TEST(MapBasedStopConstraintGeneratorTest, CollectStopsBeforeConflictingPriorityLane)
{
  auto generator = make_generator();

  auto approach = make_routable_lanelet(1, 0.0, 10.0);
  auto intersection = make_routable_lanelet(2, 10.0, 10.0);
  intersection.attributes()["turn_direction"] = "right";
  // A priority lane crossing the intersection lanelet at x in [14, 18]; no RightOfWay grants ego
  // priority over it, so ego is non-priority and must stop before entering it.
  auto crossing = make_crossing_road_lanelet(3, 14.0);

  auto map = to_const_shared(lanelet::utils::createMap({approach, intersection, crossing}));
  const auto stop_lines =
    collect_stop_lines(generator, {approach, intersection}, map, make_routing_graph(*map));

  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::Intersection);
  // The stop line sits where the route centerline enters the crossing lane (x = 14), not at the
  // intersection lanelet entry (x = 10).
  const auto & line = stop_lines.front().line;
  EXPECT_NEAR(0.5 * (line.front().x() + line.back().x()), 14.0, 0.1);
  EXPECT_NEAR(0.5 * (line.front().y() + line.back().y()), 0.0, 0.1);
}

TEST(MapBasedStopConstraintGeneratorTest, CollectSkipsIntersectionWhenEgoHasPriority)
{
  auto generator = make_generator();

  auto approach = make_routable_lanelet(1, 0.0, 10.0);
  auto intersection = make_routable_lanelet(2, 10.0, 10.0);
  intersection.attributes()["turn_direction"] = "right";
  auto crossing = make_crossing_road_lanelet(3, 14.0);
  // A RightOfWay regulatory element grants ego priority: the crossing lane yields to ego.
  intersection.addRegulatoryElement(
    lanelet::RightOfWay::make(999, lanelet::AttributeMap(), {intersection}, {crossing}));

  auto map = to_const_shared(lanelet::utils::createMap({approach, intersection, crossing}));
  const auto stop_lines =
    collect_stop_lines(generator, {approach, intersection}, map, make_routing_graph(*map));

  EXPECT_TRUE(stop_lines.empty());
}

TEST(MapBasedStopConstraintGeneratorTest, CollectSkipsIntersectionWithoutConflictingLane)
{
  auto generator = make_generator();

  // A branching turn (turn_direction present) that crosses no other lane: no stop line.
  auto approach = make_routable_lanelet(1, 0.0, 10.0);
  auto intersection = make_routable_lanelet(2, 10.0, 10.0);
  intersection.attributes()["turn_direction"] = "left";

  auto map = to_const_shared(lanelet::utils::createMap({approach, intersection}));
  const auto stop_lines =
    collect_stop_lines(generator, {approach, intersection}, map, make_routing_graph(*map));

  EXPECT_TRUE(stop_lines.empty());
}

TEST(MapBasedStopConstraintGeneratorTest, CollectSkipsSignalizedStraightIntersection)
{
  auto generator = make_generator();

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

  auto map = to_const_shared(lanelet::utils::createMap({approach, intersection, crossing}));
  const auto stop_lines =
    collect_stop_lines(generator, {approach, intersection}, map, make_routing_graph(*map));

  EXPECT_TRUE(stop_lines.empty());
}

TEST(MapBasedStopConstraintGeneratorTest, CollectDetectsPrivateAreaTransition)
{
  auto generator = make_generator();

  auto public_lanelet = make_road_lanelet(1, 0.0, 10.0);
  auto private_lanelet = make_road_lanelet(2, 10.0, 10.0);
  private_lanelet.attributes()[lanelet::AttributeNamesString::Location] =
    lanelet::AttributeValueString::Private;

  const auto stop_lines = collect_stop_lines(generator, {public_lanelet, private_lanelet});

  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::PrivateArea);
  EXPECT_NEAR(stop_lines.front().line.front().x(), 10.0, 1e-6);
  EXPECT_EQ(stop_lines.front().line.id(), -2);
}

TEST(MapBasedStopConstraintGeneratorTest, CollectPlacesPrivateEntryAtRoadDeparture)
{
  auto generator = make_generator();

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

  auto map = to_const_shared(lanelet::utils::createMap({straight_road}));
  const auto stop_lines = collect_stop_lines(generator, {route_road, connector}, map);

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

TEST(MapBasedStopConstraintGeneratorTest, CollectPlacesPrivateExitAtRoadEntry)
{
  auto generator = make_generator();

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

  auto map = to_const_shared(lanelet::utils::createMap({road}));
  const auto stop_lines = collect_stop_lines(generator, {connector, merged_road}, map);

  // The stop line sits where the connector centerline first enters the road (y = 2), i.e. before
  // merging, not at the merged lanelet start (x = 12).
  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::PrivateArea);
  const auto & line = stop_lines.front().line;
  const double mid_x = 0.5 * (line.front().x() + line.back().x());
  const double mid_y = 0.5 * (line.front().y() + line.back().y());
  EXPECT_NEAR(mid_y, 2.0, 0.3);
  EXPECT_GT(mid_x, 5.0);
  EXPECT_LT(mid_x, 13.0);
}

TEST(MapBasedStopConstraintGeneratorTest, CollectDetectsWalkwayCrossingEntrySideOnly)
{
  auto generator = make_generator();

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

  auto map = to_const_shared(lanelet::utils::createMap({walkway}));
  const auto stop_lines = collect_stop_lines(generator, {road}, map);

  // Only the entry-side (near, x=9) bound is used, not both bounds. The bound is a fallback for a
  // missing explicit stop line, so the crosswalk stop distance applies.
  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::Walkway);
  EXPECT_EQ(stop_lines.front().line.id(), 700);
  EXPECT_TRUE(stop_lines.front().without_explicit_stop_line);
}

TEST(MapBasedStopConstraintGeneratorTest, CollectPrefersCrosswalkRegelemStopLine)
{
  auto generator = make_generator();

  auto road = make_road_lanelet(1, 0.0, 20.0);
  auto walkway = make_crossing_walkway(720);

  // Explicit stop line provided by a crosswalk regulatory element attached to the road lanelet.
  lanelet::LineString3d explicit_stop_line(
    800, {lanelet::Point3d(801, 7.0, -2.0, 0.0), lanelet::Point3d(802, 7.0, 2.0, 0.0)});
  lanelet::Polygon3d crosswalk_area;
  road.addRegulatoryElement(
    lanelet::autoware::Crosswalk::make(
      900, lanelet::AttributeMap(), walkway, crosswalk_area, {explicit_stop_line}));

  auto map = to_const_shared(lanelet::utils::createMap({walkway}));
  const auto stop_lines = collect_stop_lines(generator, {road}, map);

  // The regelem stop line (x=7) wins over the walkway's entry-side bound (x=9).
  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::Walkway);
  EXPECT_EQ(stop_lines.front().line.id(), 800);
  EXPECT_FALSE(stop_lines.front().without_explicit_stop_line);
}

TEST(MapBasedStopConstraintGeneratorTest, CollectPrefersCrosswalkIdRoadMarkingStopLine)
{
  auto generator = make_generator();

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

  auto map = to_const_shared(lanelet::utils::createMap({walkway}));
  const auto stop_lines = collect_stop_lines(generator, {road}, map);

  // The crosswalk_id road marking (x=7.5) wins over the walkway's entry-side bound (x=9).
  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::Walkway);
  EXPECT_EQ(stop_lines.front().line.id(), 800);
}

// ============================================================
// select_road_shoulder_stop_arc_length
// ============================================================

namespace
{
// A shoulder lanelet on x in [0, 20] followed by a road lanelet on x in [20, 50], both spanning
// y in [-2, 2], registered in a map so the layer search finds them.
lanelet::LaneletMapPtr make_shoulder_to_road_map()
{
  auto shoulder = make_road_lanelet(10, 0.0, 20.0);
  shoulder.attributes()[lanelet::AttributeNamesString::Subtype] = "road_shoulder";
  auto road = make_road_lanelet(20, 20.0, 30.0);
  road.attributes()[lanelet::AttributeNamesString::Subtype] = lanelet::AttributeValueString::Road;
  return lanelet::utils::createMap({shoulder, road});
}

// A road lanelet on x in [0, 25] followed by a shoulder lanelet on x in [25, 55], both spanning
// y in [-2, 2].
lanelet::LaneletMapPtr make_road_to_shoulder_map()
{
  auto road = make_road_lanelet(30, 0.0, 25.0);
  road.attributes()[lanelet::AttributeNamesString::Subtype] = lanelet::AttributeValueString::Road;
  auto shoulder = make_road_lanelet(40, 25.0, 30.0);
  shoulder.attributes()[lanelet::AttributeNamesString::Subtype] = "road_shoulder";
  return lanelet::utils::createMap({road, shoulder});
}

// A road lanelet on x in [0, 50] with a shoulder lanelet laid over x in [20, 25]: the footprint
// touches the shoulder but never leaves the road lane.
lanelet::LaneletMapPtr make_road_with_clipped_shoulder_map()
{
  auto road = make_road_lanelet(50, 0.0, 50.0);
  road.attributes()[lanelet::AttributeNamesString::Subtype] = lanelet::AttributeValueString::Road;
  auto shoulder = make_road_lanelet(60, 20.0, 5.0);
  shoulder.attributes()[lanelet::AttributeNamesString::Subtype] = "road_shoulder";
  return lanelet::utils::createMap({road, shoulder});
}
}  // namespace

TEST(MapBasedStopConstraintGeneratorTest, RoadShoulderStopBeforeEnteringShoulder)
{
  auto generator = make_generator();

  // The footprint first touches the shoulder (x = 25) at base_link x = 21, and is wholly inside it
  // from base_link x = 27, so the stop keeps the margins ahead of the first touch: 21 - 1.0 - 0.5.
  const auto arc_length = generator.select_road_shoulder_stop_arc_length(
    make_straight_trajectory(30), make_ego_pose(0.0), make_vehicle_info(),
    make_road_to_shoulder_map());

  ASSERT_TRUE(arc_length.has_value());
  EXPECT_NEAR(*arc_length, 19.5, 1e-3);
}

TEST(MapBasedStopConstraintGeneratorTest, RoadShoulderStopInactiveWhenShoulderIsOnlyClipped)
{
  auto generator = make_generator();

  // The footprint never leaves the road lane, so passing the shoulder must not insert a stop.
  EXPECT_FALSE(generator
                 .select_road_shoulder_stop_arc_length(
                   make_straight_trajectory(40), make_ego_pose(0.0), make_vehicle_info(),
                   make_road_with_clipped_shoulder_map())
                 .has_value());
}

TEST(MapBasedStopConstraintGeneratorTest, RoadShoulderStopKeepsFootprintOutOfRoadLane)
{
  auto generator = make_generator();

  // The footprint reaches the road lanelet (x = 20) at base_link x = 16, so the stop point keeps
  // the configured margins ahead of it: 16 - 1.0 - 0.5.
  const auto arc_length = generator.select_road_shoulder_stop_arc_length(
    make_straight_trajectory(30), make_ego_pose(0.0), make_vehicle_info(),
    make_shoulder_to_road_map());

  ASSERT_TRUE(arc_length.has_value());
  EXPECT_NEAR(*arc_length, 14.5, 1e-3);
}

TEST(MapBasedStopConstraintGeneratorTest, RoadShoulderStopInactiveOnceFootprintReachesRoadLane)
{
  auto generator = make_generator();

  // Ego already straddles the road lanelet: the trajectory must return to the lane without a stop.
  EXPECT_FALSE(generator
                 .select_road_shoulder_stop_arc_length(
                   make_straight_trajectory(30), make_ego_pose(18.0), make_vehicle_info(),
                   make_shoulder_to_road_map())
                 .has_value());
}

// ============================================================
// generate_constraints: certainty of the emitted gates
// ============================================================

namespace
{
// Owns the input PlannerContext references. context() stays valid while this object does.
struct Scene
{
  SafetyPlannerInput input;
  PathPointTrajectory path;

  Scene(
    const lanelet::LaneletMapConstPtr & map, const std::vector<lanelet::Id> & route_ids,
    double ego_x)
  {
    input.vehicle_info = make_vehicle_info();
    input.odometry.pose.pose = make_ego_pose(ego_x);
    input.odometry.twist.twist.linear.x = 0.0;
    input.acceleration.accel.accel.linear.x = 0.0;

    LaneletRoute route;
    for (const auto id : route_ids) {
      autoware_planning_msgs::msg::LaneletSegment segment;
      autoware_planning_msgs::msg::LaneletPrimitive primitive;
      primitive.id = id;
      primitive.primitive_type = "lane";
      segment.primitives.push_back(primitive);
      segment.preferred_primitive = primitive;
      route.segments.push_back(segment);
    }
    auto manager = RouteManager::create(
      autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map), route,
      input.odometry.pose.pose);
    if (!manager) {
      throw std::runtime_error("failed to build the route manager for the test map");
    }
    input.route_manager = std::move(*manager);

    std::vector<PathPointWithLaneId> points;
    for (int i = 0; i <= 40; ++i) {
      auto & point = points.emplace_back();
      point.point.pose.position.x = static_cast<double>(i);
      point.point.pose.orientation.w = 1.0;
    }
    path = *PathPointTrajectory::Builder{}.build(points);
  }

  PlannerContext context() const { return PlannerContext(input, path); }
};

const Constraint * find_constraint(const std::vector<Constraint> & constraints, Certainty certainty)
{
  const Constraint * found = nullptr;
  for (const auto & constraint : constraints) {
    if (constraint.certainty != certainty) continue;
    found = &constraint;
  }
  return found;
}

double gate_mid_x(const Constraint & constraint)
{
  const auto & gate = std::get<Gate>(constraint.payload);
  return 0.5 * (gate.line.first.x() + gate.line.second.x());
}
}  // namespace

TEST(MapBasedStopConstraintGeneratorTest, GenerateEmitsDefiniteGateForMandatoryStop)
{
  auto lanelet = make_routable_lanelet(1, 0.0, 40.0);
  lanelet::LineString3d stop_line(
    100, {lanelet::Point3d(101, 20.0, -2.0, 0.0), lanelet::Point3d(102, 20.0, 2.0, 0.0)});
  lanelet.addRegulatoryElement(make_stop_sign_regelem(500, stop_line));
  auto map = to_const_shared(lanelet::utils::createMap({lanelet}));

  Scene scene(map, {lanelet.id()}, /*ego_x=*/0.0);
  auto generator = make_generator();
  const auto output = generator.generate_constraints(scene.context());

  const auto * definite = find_constraint(output.constraints, Certainty::DEFINITE);
  ASSERT_NE(definite, nullptr);
  EXPECT_EQ(definite->source.detail, "stop_line");
  // The gate is where the front bumper stops: crossing (20) - margin (1) = 19.
  EXPECT_NEAR(gate_mid_x(*definite), 19.0, 1e-2);
  EXPECT_EQ(find_constraint(output.constraints, Certainty::POSSIBLE), nullptr);
  ASSERT_TRUE(output.debug_markers.has_value());
  EXPECT_FALSE(output.debug_markers->markers.empty());
}

TEST(MapBasedStopConstraintGeneratorTest, GenerateEmitsPossibleGateForPossibilityStop)
{
  auto lanelet = make_routable_lanelet(1, 0.0, 40.0);
  lanelet.attributes()["turn_direction"] = "right";
  // No routing-graph conflict is needed: a traffic-light stop line is a possibility target.
  lanelet::LineString3d stop_line(
    100, {lanelet::Point3d(101, 10.0, -2.0, 0.0), lanelet::Point3d(102, 10.0, 2.0, 0.0)});
  lanelet::LineString3d light(
    200, {lanelet::Point3d(201, 10.0, 3.0, 3.0), lanelet::Point3d(202, 10.0, 3.5, 3.0)});
  lanelet.addRegulatoryElement(
    lanelet::autoware::AutowareTrafficLight::make(
      500, lanelet::AttributeMap(), {light}, stop_line));
  auto map = to_const_shared(lanelet::utils::createMap({lanelet}));

  Scene scene(map, {lanelet.id()}, /*ego_x=*/0.0);
  auto generator = make_generator();
  const auto output = generator.generate_constraints(scene.context());

  // The turn has a routing graph and no conflicting lane, so it adds no intersection stop; only
  // the traffic-light stop line remains, and it is a possibility.
  const auto * possible = find_constraint(output.constraints, Certainty::POSSIBLE);
  ASSERT_NE(possible, nullptr);
  EXPECT_EQ(possible->source.detail, "traffic_light");
  // crossing (10) - margin (1) = 9
  EXPECT_NEAR(gate_mid_x(*possible), 9.0, 1e-2);
  EXPECT_EQ(find_constraint(output.constraints, Certainty::DEFINITE), nullptr);
}

TEST(MapBasedStopConstraintGeneratorTest, GenerateEmitsBothCertainties)
{
  auto lanelet = make_routable_lanelet(1, 0.0, 40.0);
  lanelet::LineString3d mandatory_line(
    100, {lanelet::Point3d(101, 25.0, -2.0, 0.0), lanelet::Point3d(102, 25.0, 2.0, 0.0)});
  lanelet.addRegulatoryElement(make_stop_sign_regelem(500, mandatory_line));
  lanelet::LineString3d signal_line(
    110, {lanelet::Point3d(111, 12.0, -2.0, 0.0), lanelet::Point3d(112, 12.0, 2.0, 0.0)});
  lanelet::LineString3d light(
    200, {lanelet::Point3d(201, 12.0, 3.0, 3.0), lanelet::Point3d(202, 12.0, 3.5, 3.0)});
  lanelet.addRegulatoryElement(
    lanelet::autoware::AutowareTrafficLight::make(
      600, lanelet::AttributeMap(), {light}, signal_line));
  auto map = to_const_shared(lanelet::utils::createMap({lanelet}));

  Scene scene(map, {lanelet.id()}, /*ego_x=*/0.0);
  auto generator = make_generator();
  const auto output = generator.generate_constraints(scene.context());

  const auto * definite = find_constraint(output.constraints, Certainty::DEFINITE);
  const auto * possible = find_constraint(output.constraints, Certainty::POSSIBLE);
  ASSERT_NE(definite, nullptr);
  ASSERT_NE(possible, nullptr);
  // mandatory crossing 25 - margin 1 = 24; signal crossing 12 - margin 1 = 11
  EXPECT_NEAR(gate_mid_x(*definite), 24.0, 1e-2);
  EXPECT_NEAR(gate_mid_x(*possible), 11.0, 1e-2);
}

TEST(MapBasedStopConstraintGeneratorTest, GenerateEmitsNothingWithoutRoute)
{
  auto generator = make_generator();
  SafetyPlannerInput input;
  input.vehicle_info = make_vehicle_info();
  input.odometry.pose.pose = make_ego_pose(0.0);
  std::vector<PathPointWithLaneId> points;
  for (int i = 0; i <= 10; ++i) {
    auto & point = points.emplace_back();
    point.point.pose.position.x = static_cast<double>(i);
    point.point.pose.orientation.w = 1.0;
  }
  const PlannerContext context(input, *PathPointTrajectory::Builder{}.build(points));

  const auto output = generator.generate_constraints(context);
  EXPECT_TRUE(output.constraints.empty());
}

}  // namespace autoware::safety_planner::experimental
