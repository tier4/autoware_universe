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
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
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
  params.base_link_to_front = 4.0;
  params.stop_point_diff_threshold = 0.5;
  return params;
}

// Ego pose on the straight trajectory (y = 0, facing +x).
geometry_msgs::msg::Pose make_ego_pose(double x)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.orientation.w = 1.0;
  return pose;
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
  const auto arc = planner.select_stop_arc_length(
    stop_lines, trajectory, make_ego_pose(0.0), /*ego_velocity=*/0.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  ASSERT_TRUE(arc.has_value());
  // crossing (20) - front offset (4) - margin (1) = 15
  EXPECT_NEAR(*arc, 15.0, 1e-6);
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

  const auto arc = planner.select_stop_arc_length(
    stop_lines, trajectory, make_ego_pose(0.0), /*ego_velocity=*/0.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  ASSERT_TRUE(arc.has_value());
  // crossing (20) - front offset (4) - (stop_margin (1) + stop_distance_from_crosswalk (3.5))
  EXPECT_NEAR(*arc, 11.5, 1e-6);
}

TEST(MapBasedStopPlannerTest, SelectAppliesPrivateAreaMargin)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  const auto trajectory = make_straight_trajectory(30);

  // A private-area boundary line: the stop_distance_from_private_area margin applies on top of
  // stop_margin_distance so the vehicle keeps a gap to the boundary.
  const std::vector<StopLine> stop_lines{
    make_crossing_stop_line(1, 20.0, StopLineType::PrivateArea)};

  const auto arc = planner.select_stop_arc_length(
    stop_lines, trajectory, make_ego_pose(0.0), /*ego_velocity=*/0.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/true);
  ASSERT_TRUE(arc.has_value());
  // crossing (20) - front offset (4) - (stop_margin (1) + stop_distance_from_private_area (3))
  EXPECT_NEAR(*arc, 12.0, 1e-6);
}

TEST(MapBasedStopPlannerTest, SelectPicksNearest)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  const auto trajectory = make_straight_trajectory(60);
  const std::vector<StopLine> stop_lines{
    make_crossing_stop_line(1, 40.0), make_crossing_stop_line(2, 20.0)};

  const auto arc = planner.select_stop_arc_length(
    stop_lines, trajectory, make_ego_pose(0.0), 0.0, 0.0, make_params(),
    /*include_possibility=*/false);
  ASSERT_TRUE(arc.has_value());
  // nearest crossing is 20 -> 20 - 4 - 1 = 15
  EXPECT_NEAR(*arc, 15.0, 1e-6);
}

TEST(MapBasedStopPlannerTest, SelectRejectsUnreachableStopPoint)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));
  const auto trajectory = make_straight_trajectory(30);
  const std::vector<StopLine> stop_lines{make_crossing_stop_line(1, 20.0)};

  // Stop point arc = 15 m. The jerk-aware braking distance at 20 m/s (a = 4, j = 5) is about
  // 57.9 m > 15 m, so the vehicle cannot stop in time and no stop point should be selected.
  const auto arc = planner.select_stop_arc_length(
    stop_lines, trajectory, make_ego_pose(0.0), /*ego_velocity=*/20.0, /*ego_acceleration=*/0.0,
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

  const auto arc = planner.select_stop_arc_length(
    stop_lines, trajectory, make_ego_pose(0.0), /*ego_velocity=*/20.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  ASSERT_TRUE(arc.has_value());
  // 70 - 4 - 1 = 65
  EXPECT_NEAR(*arc, 65.0, 1e-6);
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
                 .select_stop_arc_length(
                   stop_lines, trajectory, make_ego_pose(0.0), 0.0, 0.0, make_params(),
                   /*include_possibility=*/false)
                 .has_value());
  // Stop trajectory: possibility targets are considered -> stop.
  EXPECT_TRUE(planner
                .select_stop_arc_length(
                  stop_lines, trajectory, make_ego_pose(0.0), 0.0, 0.0, make_params(),
                  /*include_possibility=*/true)
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
  const auto arc = planner.select_stop_arc_length(
    stop_lines, trajectory, make_ego_pose(10.0), /*ego_velocity=*/8.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  EXPECT_FALSE(arc.has_value());

  // With enough remaining distance (ego at x = 2 -> 13 m > 11.1 m) the stop point is kept.
  const auto arc_reachable = planner.select_stop_arc_length(
    stop_lines, trajectory, make_ego_pose(2.0), /*ego_velocity=*/8.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  ASSERT_TRUE(arc_reachable.has_value());
  EXPECT_NEAR(*arc_reachable, 15.0, 1e-6);
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
  const auto arc = planner.select_stop_arc_length(
    stop_lines, trajectory, make_ego_pose(5.0), /*ego_velocity=*/8.0, /*ego_acceleration=*/0.0,
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
  const auto arc = planner.select_stop_arc_length(
    stop_lines, trajectory, make_ego_pose(16.0), /*ego_velocity=*/1.0, /*ego_acceleration=*/0.0,
    make_params(), /*include_possibility=*/false);
  EXPECT_FALSE(arc.has_value());
}

// ============================================================
// collect_stop_lines
// ============================================================

TEST(MapBasedStopPlannerTest, CollectTagsRoadMarkingStopLine)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  // Build a stop-line road marking and attach it to a lanelet as a regulatory element.
  lanelet::LineString3d stop_line(
    100, {lanelet::Point3d(101, 5.0, -1.0, 0.0), lanelet::Point3d(102, 5.0, 1.0, 0.0)});
  stop_line.attributes()[lanelet::AttributeNamesString::Type] =
    lanelet::AttributeValueString::StopLine;

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

  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().line.id(), 100);
  EXPECT_EQ(stop_lines.front().type, StopLineType::StopLine);
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

TEST(MapBasedStopPlannerTest, CollectDetectsIntersectionEntry)
{
  MapBasedStopPlanner planner(rclcpp::get_logger("test_map_based_stop_planner"));

  auto approach = make_road_lanelet(1, 0.0, 10.0);
  auto intersection = make_road_lanelet(2, 10.0, 10.0);
  intersection.attributes()["turn_direction"] = "right";

  RouteContext ctx;
  ctx.route_lanelets = {approach, intersection};
  const auto stop_lines = planner.collect_stop_lines(ctx);

  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().type, StopLineType::Intersection);
  // Entry edge is at the intersection lanelet start (x = 10).
  EXPECT_NEAR(stop_lines.front().line.front().x(), 10.0, 1e-6);
  // Synthesized edges carry a unique id (negated source lanelet id): the marker visualization
  // dedups line strings by id, so sharing InvalId would drop every synthesized line but the first.
  EXPECT_EQ(stop_lines.front().line.id(), -2);
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
// Seed the planner's stop line cache with a RoadMarking stop line (mandatory) crossing the
// straight test trajectory at x = 20.
void set_mandatory_stop_line_data(MapBasedStopPlanner & planner)
{
  lanelet::LineString3d stop_line(
    100, {lanelet::Point3d(101, 20.0, -2.0, 0.0), lanelet::Point3d(102, 20.0, 2.0, 0.0)});
  stop_line.attributes()[lanelet::AttributeNamesString::Type] =
    lanelet::AttributeValueString::StopLine;
  auto lanelet = make_road_lanelet(400, 0.0, 30.0);
  lanelet.addRegulatoryElement(
    lanelet::autoware::RoadMarking::make(500, lanelet::AttributeMap(), stop_line));

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
    trajectory, make_ego_pose(0.0), /*ego_velocity=*/0.0, /*ego_acceleration=*/0.0, make_params());

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

  // Only a possibility target (intersection entry edge at x = 10) crosses the trajectory.
  auto intersection = make_road_lanelet(2, 10.0, 10.0);
  intersection.attributes()["turn_direction"] = "right";
  RouteContext ctx;
  ctx.route_lanelets = {intersection};
  planner.set_planner_data(
    std::make_shared<LaneletMapBin>(), std::make_shared<LaneletRoute>(), ctx);

  const auto trajectory = make_straight_trajectory_msg(30);
  const auto result = planner.plan(
    trajectory, make_ego_pose(0.0), /*ego_velocity=*/0.0, /*ego_acceleration=*/0.0, make_params());

  // The go trajectory ignores possibility targets and stays unchanged.
  ASSERT_EQ(result.go_trajectory.points.size(), trajectory.points.size());
  EXPECT_FLOAT_EQ(result.go_trajectory.points.back().longitudinal_velocity_mps, 5.0f);
  // The stop trajectory embeds the possibility stop (10 - 4 - 1 = 5), distinct from go.
  ASSERT_TRUE(result.stop_trajectory.has_value());
  EXPECT_NEAR(result.stop_trajectory->points.back().pose.position.x, 5.0, 1e-3);
  EXPECT_FLOAT_EQ(result.stop_trajectory->points.back().longitudinal_velocity_mps, 0.0f);
}

}  // namespace autoware::minimum_rule_based_planner
