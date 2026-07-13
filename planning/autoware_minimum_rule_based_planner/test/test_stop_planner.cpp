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

#include "stop_planner.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <rclcpp/logger.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

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
  lanelet::Id id, double x_cross, StopLineType type = StopLineType::RoadMarking)
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
  params.stop_margin_distance = 1.0;
  params.base_link_to_front = 4.0;
  return params;
}

}  // namespace

// ============================================================
// is_possibility_type
// ============================================================

TEST(StopPlannerTest, PossibilityTypeClassification)
{
  // Signals are possibility targets; painted stop lines and stop signs are mandatory.
  EXPECT_TRUE(is_possibility_type(StopLineType::TrafficLight));
  EXPECT_FALSE(is_possibility_type(StopLineType::RoadMarking));
  EXPECT_FALSE(is_possibility_type(StopLineType::TrafficSign));
}

// ============================================================
// filter_stop_lines_on_trajectory
// ============================================================

TEST(StopPlannerTest, FilterKeepsOnlyCrossingLines)
{
  StopPlanner planner(rclcpp::get_logger("test_stop_planner"));
  const auto trajectory = make_straight_trajectory(20);

  std::vector<StopLine> stop_lines;
  stop_lines.push_back(make_crossing_stop_line(1, 10.0));  // crosses the trajectory
  // A stop line far to the side that does not cross the trajectory.
  lanelet::LineString3d off_line(
    2, {lanelet::Point3d(21, 5.0, 10.0, 0.0), lanelet::Point3d(22, 5.0, 12.0, 0.0)});
  stop_lines.push_back(StopLine{off_line, StopLineType::RoadMarking});

  const auto filtered = planner.filter_stop_lines_on_trajectory(stop_lines, trajectory);
  ASSERT_EQ(filtered.size(), 1u);
  EXPECT_EQ(filtered.front().line.id(), 1);
}

// ============================================================
// select_stop_arc_length
// ============================================================

TEST(StopPlannerTest, SelectAppliesFrontOffsetAndMargin)
{
  StopPlanner planner(rclcpp::get_logger("test_stop_planner"));
  const auto trajectory = make_straight_trajectory(30);
  const std::vector<StopLine> stop_lines{make_crossing_stop_line(1, 20.0)};

  // Ego stopped: braking distance is 0, so the stop point is reachable.
  const auto arc = planner.select_stop_arc_length(
    stop_lines, trajectory, /*ego_velocity=*/0.0, make_params(), /*include_possibility=*/false);
  ASSERT_TRUE(arc.has_value());
  // crossing (20) - front offset (4) - margin (1) = 15
  EXPECT_NEAR(*arc, 15.0, 1e-6);
}

TEST(StopPlannerTest, SelectPicksNearest)
{
  StopPlanner planner(rclcpp::get_logger("test_stop_planner"));
  const auto trajectory = make_straight_trajectory(60);
  const std::vector<StopLine> stop_lines{
    make_crossing_stop_line(1, 40.0), make_crossing_stop_line(2, 20.0)};

  const auto arc = planner.select_stop_arc_length(
    stop_lines, trajectory, 0.0, make_params(), /*include_possibility=*/false);
  ASSERT_TRUE(arc.has_value());
  // nearest crossing is 20 -> 20 - 4 - 1 = 15
  EXPECT_NEAR(*arc, 15.0, 1e-6);
}

TEST(StopPlannerTest, SelectRejectsUnreachableStopPoint)
{
  StopPlanner planner(rclcpp::get_logger("test_stop_planner"));
  const auto trajectory = make_straight_trajectory(30);
  const std::vector<StopLine> stop_lines{make_crossing_stop_line(1, 20.0)};

  // Stop point arc = 15 m. Braking distance at 20 m/s with a = 4 is 20^2/(2*4) = 50 m > 15 m,
  // so the vehicle cannot stop in time and no stop point should be selected.
  const auto arc = planner.select_stop_arc_length(
    stop_lines, trajectory, /*ego_velocity=*/20.0, make_params(), /*include_possibility=*/false);
  EXPECT_FALSE(arc.has_value());
}

TEST(StopPlannerTest, SelectExcludesPossibilityTargetsForGoTrajectory)
{
  StopPlanner planner(rclcpp::get_logger("test_stop_planner"));
  const auto trajectory = make_straight_trajectory(30);
  // Only a signal (possibility) target crosses the trajectory.
  const std::vector<StopLine> stop_lines{
    make_crossing_stop_line(1, 20.0, StopLineType::TrafficLight)};

  // Go trajectory: possibility targets are ignored -> no stop.
  EXPECT_FALSE(planner
                 .select_stop_arc_length(
                   stop_lines, trajectory, 0.0, make_params(), /*include_possibility=*/false)
                 .has_value());
  // Stop trajectory: possibility targets are considered -> stop.
  EXPECT_TRUE(planner
                .select_stop_arc_length(
                  stop_lines, trajectory, 0.0, make_params(), /*include_possibility=*/true)
                .has_value());
}

// ============================================================
// collect_stop_lines
// ============================================================

TEST(StopPlannerTest, CollectTagsRoadMarkingStopLine)
{
  StopPlanner planner(rclcpp::get_logger("test_stop_planner"));

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

  const lanelet::ConstLanelets route_lanelets{lanelet};
  const auto stop_lines = planner.collect_stop_lines(route_lanelets);

  ASSERT_EQ(stop_lines.size(), 1u);
  EXPECT_EQ(stop_lines.front().line.id(), 100);
  EXPECT_EQ(stop_lines.front().type, StopLineType::RoadMarking);
}

}  // namespace autoware::minimum_rule_based_planner
