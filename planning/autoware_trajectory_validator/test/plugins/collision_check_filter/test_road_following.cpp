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

#include "../../../src/filters/safety/collision_check_filter/parameter.hpp"
#include "../../../src/filters/safety/collision_check_filter/road_following.hpp"
#include "../../../src/filters/safety/collision_check_filter/trajectory_utils.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry/algorithms/area.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
namespace
{
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using IdLists = std::vector<std::vector<lanelet::Id>>;

constexpr double k_half_width = 1.5;

// Builds a small lanelet map in code. Equal coordinates give the same point object, and equal
// start and end points give the same bound object. So a lanelet that starts at the end points of
// another lanelet follows it in the routing graph, and two lanelets side by side share a bound.
class TestMap
{
public:
  // Adds a lanelet with straight bounds. `left` and `right` are the start and end points.
  lanelet::Lanelet add_lanelet(
    const lanelet::Id id, const std::pair<lanelet::BasicPoint2d, lanelet::BasicPoint2d> & left,
    const std::pair<lanelet::BasicPoint2d, lanelet::BasicPoint2d> & right,
    const std::string & turn_direction = "")
  {
    lanelet::Lanelet lanelet(id, bound(left), bound(right));
    lanelet.attributes()[lanelet::AttributeNamesString::Subtype] =
      lanelet::AttributeValueString::Road;
    lanelet.attributes()[lanelet::AttributeNamesString::Location] =
      lanelet::AttributeValueString::Urban;
    lanelet.attributes()["one_way"] = "yes";
    if (!turn_direction.empty()) {
      lanelet.attributes()["turn_direction"] = turn_direction;
    }
    map_->add(lanelet);
    return lanelet;
  }

  // Adds a lanelet that runs along the x axis from `x_start` to `x_end`, centered on `y`.
  lanelet::Lanelet add_straight_lanelet(
    const lanelet::Id id, const double x_start, const double x_end, const double y = 0.0)
  {
    return add_lanelet(
      id, {{x_start, y + k_half_width}, {x_end, y + k_half_width}},
      {{x_start, y - k_half_width}, {x_end, y - k_half_width}});
  }

  [[nodiscard]] const lanelet::LaneletMap & map() const { return *map_; }

  [[nodiscard]] lanelet::routing::RoutingGraphConstPtr routing_graph() const
  {
    return autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(map_)
      .first;
  }

private:
  lanelet::Point3d point(const lanelet::BasicPoint2d & p)
  {
    const auto key = std::make_pair(
      static_cast<int64_t>(std::llround(p.x() * 1000.0)),
      static_cast<int64_t>(std::llround(p.y() * 1000.0)));
    const auto it = points_.find(key);
    if (it != points_.end()) {
      return it->second;
    }
    const lanelet::Point3d new_point(next_id_++, p.x(), p.y(), 0.0);
    points_.emplace(key, new_point);
    return new_point;
  }

  lanelet::LineString3d bound(const std::pair<lanelet::BasicPoint2d, lanelet::BasicPoint2d> & ends)
  {
    const auto start = point(ends.first);
    const auto end = point(ends.second);
    const auto key = std::make_pair(start.id(), end.id());
    const auto it = bounds_.find(key);
    if (it != bounds_.end()) {
      return it->second;
    }
    const lanelet::LineString3d new_bound(next_id_++, {start, end});
    bounds_.emplace(key, new_bound);
    return new_bound;
  }

  lanelet::LaneletMapPtr map_{std::make_shared<lanelet::LaneletMap>()};
  std::map<std::pair<int64_t, int64_t>, lanelet::Point3d> points_;
  std::map<std::pair<lanelet::Id, lanelet::Id>, lanelet::LineString3d> bounds_;
  lanelet::Id next_id_{1000};
};

// Each inner vector is one route segment. Its first id is the preferred lanelet.
LaneletRoute make_route(const IdLists & segments)
{
  LaneletRoute route;
  for (const auto & ids : segments) {
    LaneletSegment segment;
    segment.preferred_primitive.id = ids.front();
    segment.preferred_primitive.primitive_type = "lane";
    for (const auto id : ids) {
      LaneletPrimitive primitive;
      primitive.id = id;
      primitive.primitive_type = "lane";
      segment.primitives.push_back(primitive);
    }
    route.segments.push_back(segment);
  }
  return route;
}

// A sample map, not a real map: a lane shift at a branch point on a road with two lanes. All
// lanes are 3 m wide. The left lane is lanelets 1, 3, and 6. The right lane is lanelets 5, 7,
// and 4. Lanelets side by side share a bound. Lanelet 1 ends at a branch point. Lanelet 2
// (route) starts there and shifts one lane to the right in 10 m, so it ends where lanelet 4
// starts. Lanelet 2 overlaps lanelet 3 (branch) in one wedge and lanelet 7 in another wedge.
TestMap make_lane_shift_map()
{
  TestMap test_map;
  test_map.add_straight_lanelet(1, 0.0, 20.0);
  test_map.add_lanelet(
    2, {{20.0, k_half_width}, {30.0, -k_half_width}},
    {{20.0, -k_half_width}, {30.0, -3.0 * k_half_width}});
  test_map.add_straight_lanelet(3, 20.0, 30.0);
  test_map.add_straight_lanelet(4, 30.0, 50.0, -2.0 * k_half_width);
  test_map.add_straight_lanelet(5, 0.0, 20.0, -2.0 * k_half_width);
  test_map.add_straight_lanelet(6, 30.0, 50.0);
  test_map.add_straight_lanelet(7, 20.0, 30.0, -2.0 * k_half_width);
  return test_map;
}

// Returns the lanelet ids of each list, with the lists sorted, so a test does not depend on
// the order of the lists.
IdLists to_sorted_ids(const std::vector<lanelet::ConstLanelets> & lists)
{
  IdLists ids;
  ids.reserve(lists.size());
  for (const auto & list : lists) {
    std::vector<lanelet::Id> list_ids;
    list_ids.reserve(list.size());
    for (const auto & lanelet : list) {
      list_ids.push_back(lanelet.id());
    }
    ids.push_back(list_ids);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

TEST(RoadFollowingParams, CopiesEveryNodeParameter)
{
  // The values differ from the defaults, so a constructor that ignores its input fails.
  validator::Params::CollisionCheck::RoadFollowing node_params;
  node_params.ego_footprint_margin.lateral = -0.2;
  node_params.ego_footprint_margin.front = 0.3;
  node_params.ego_footprint_margin.rear = 0.4;
  node_params.max_lateral_deviation = 0.6;
  node_params.max_heading_deviation = 0.15;
  node_params.window_distance = 40.0;
  node_params.window_time = 4.0;

  const RoadFollowingParams params(node_params);

  EXPECT_DOUBLE_EQ(params.ego_footprint_margin.lateral, -0.2);
  EXPECT_DOUBLE_EQ(params.ego_footprint_margin.front, 0.3);
  EXPECT_DOUBLE_EQ(params.ego_footprint_margin.rear, 0.4);
  EXPECT_DOUBLE_EQ(params.max_lateral_deviation, 0.6);
  EXPECT_DOUBLE_EQ(params.max_heading_deviation, 0.15);
  EXPECT_DOUBLE_EQ(params.window_distance, 40.0);
  EXPECT_DOUBLE_EQ(params.window_time, 4.0);
}

TEST(BuildLaneletLists, OneSegmentWithOneLaneletGivesOneList)
{
  TestMap test_map;
  test_map.add_straight_lanelet(1, 0.0, 10.0);

  const auto result = road_following::build_lanelet_lists(
    make_route({{1}}), test_map.map(), test_map.routing_graph());

  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(to_sorted_ids(*result), (IdLists{{1}}));
}

TEST(BuildLaneletLists, ThreeStraightSegmentsGiveOneListWithThreeLanelets)
{
  TestMap test_map;
  test_map.add_straight_lanelet(1, 0.0, 10.0);
  test_map.add_straight_lanelet(2, 10.0, 20.0);
  test_map.add_straight_lanelet(3, 20.0, 30.0);

  const auto result = road_following::build_lanelet_lists(
    make_route({{1}, {2}, {3}}), test_map.map(), test_map.routing_graph());

  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(to_sorted_ids(*result), (IdLists{{1, 2, 3}}));
}

TEST(BuildLaneletLists, TurnLaneletEndsTheListAndIsInNoList)
{
  TestMap test_map;
  test_map.add_straight_lanelet(1, 0.0, 10.0);
  test_map.add_lanelet(
    2, {{10.0, k_half_width}, {20.0, k_half_width}}, {{10.0, -k_half_width}, {20.0, -k_half_width}},
    "right");
  test_map.add_straight_lanelet(3, 20.0, 30.0);

  const auto result = road_following::build_lanelet_lists(
    make_route({{1}, {2}, {3}}), test_map.map(), test_map.routing_graph());

  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(to_sorted_ids(*result), (IdLists{{1}, {3}}));
}

TEST(BuildLaneletLists, KeepsListAndCopiesItPerBranch)
{
  // The end of lanelet 1 is a branch point. Its branches are lanelet 2 (straight) and lanelet 3
  // (bends to the left).
  TestMap test_map;
  test_map.add_straight_lanelet(1, 0.0, 10.0);
  test_map.add_straight_lanelet(2, 10.0, 20.0);
  test_map.add_lanelet(
    3, {{10.0, k_half_width}, {20.0, 3.0 * k_half_width}},
    {{10.0, -k_half_width}, {20.0, k_half_width}});

  const auto result = road_following::build_lanelet_lists(
    make_route({{1}, {2, 3}}), test_map.map(), test_map.routing_graph());

  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(to_sorted_ids(*result), (IdLists{{1}, {1, 2}, {1, 3}}));
}

TEST(BuildLaneletLists, LaneChangeTargetStartsANewList)
{
  // Lanes 1-2 and 3-4 run side by side. The route changes lane from 1 to 4, so lanelet 4 is in
  // the second segment, but its previous lanelet 3 is not in the route.
  TestMap test_map;
  test_map.add_straight_lanelet(1, 0.0, 10.0);
  test_map.add_straight_lanelet(2, 10.0, 20.0);
  test_map.add_straight_lanelet(3, 0.0, 10.0, 2.0 * k_half_width);
  test_map.add_straight_lanelet(4, 10.0, 20.0, 2.0 * k_half_width);

  const auto result = road_following::build_lanelet_lists(
    make_route({{1}, {2, 4}}), test_map.map(), test_map.routing_graph());

  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(to_sorted_ids(*result), (IdLists{{1, 2}, {4}}));
}

TEST(BuildLaneletLists, RouteLaneletNotInMapGivesErrorWithItsId)
{
  TestMap test_map;
  test_map.add_straight_lanelet(1, 0.0, 10.0);

  const auto result = road_following::build_lanelet_lists(
    make_route({{1}, {99}}), test_map.map(), test_map.routing_graph());

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), "route lanelet 99 is not in the map");
}

// Lanelet2 declares its 2D polygons as clockwise and open, so boost::geometry reads them as they
// are.
double shape_area(const road_following::LaneChain & chain)
{
  return boost::geometry::area(chain.shape.polygon2d().basicPolygon());
}

TEST(BuildLaneChain, OneLaneletShapeHasItsArea)
{
  // Lanelet 1 is 10 m long and 3 m wide.
  TestMap test_map;
  const lanelet::ConstLanelets lanelets{test_map.add_straight_lanelet(1, 0.0, 10.0)};

  const auto result = road_following::build_lane_chain(lanelets);

  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_NEAR(shape_area(*result), 30.0, 1e-6);
}

TEST(BuildLaneChain, TwoLaneletsGiveSumOfAreas)
{
  // Lanelet 2 follows lanelet 1. Each one is 10 m long and 3 m wide.
  TestMap test_map;
  const lanelet::ConstLanelets lanelets{
    test_map.add_straight_lanelet(1, 0.0, 10.0), test_map.add_straight_lanelet(2, 10.0, 20.0)};

  const auto result = road_following::build_lane_chain(lanelets);

  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(to_sorted_ids({result->lanelets}), (IdLists{{1, 2}}));
  EXPECT_NEAR(shape_area(*result), 60.0, 1e-6);
}

TEST(BuildLaneletLists, LaneShiftAtBranchPoint)
{
  const auto test_map = make_lane_shift_map();
  const auto & layer = test_map.map().laneletLayer;
  for (const auto & [left_id, right_id] :
       {std::pair<lanelet::Id, lanelet::Id>{1, 5}, std::pair<lanelet::Id, lanelet::Id>{3, 7},
        std::pair<lanelet::Id, lanelet::Id>{6, 4}}) {
    ASSERT_EQ(layer.get(left_id).rightBound().id(), layer.get(right_id).leftBound().id());
  }

  const auto result = road_following::build_lanelet_lists(
    make_route({{1}, {2}, {4}}), test_map.map(), test_map.routing_graph());

  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(to_sorted_ids(*result), (IdLists{{1}, {1, 2, 4}}));
}

using XyPoints = std::vector<std::pair<double, double>>;

XyPoints to_xy(const lanelet::ConstLineString2d & bound)
{
  XyPoints points;
  for (const auto & point : bound) {
    points.emplace_back(point.x(), point.y());
  }
  return points;
}

TEST(BuildLaneChain, LaneShiftChainHasNoBranch)
{
  // The footprint edges are tested against these bounds. Through both wedges, the bounds follow
  // lanelet 2. They take no bound of lanelets 3, 6, or 7.
  const auto test_map = make_lane_shift_map();
  const auto & layer = test_map.map().laneletLayer;
  const lanelet::ConstLanelets lanelets{layer.get(1), layer.get(2), layer.get(4)};

  const auto result = road_following::build_lane_chain(lanelets);

  ASSERT_TRUE(result.has_value()) << result.error();
  constexpr double h = k_half_width;
  EXPECT_EQ(
    to_xy(result->shape.leftBound2d()), (XyPoints{{0.0, h}, {20.0, h}, {30.0, -h}, {50.0, -h}}));
  EXPECT_EQ(
    to_xy(result->shape.rightBound2d()),
    (XyPoints{{0.0, -h}, {20.0, -h}, {30.0, -3.0 * h}, {50.0, -3.0 * h}}));
}

TEST(BuildLaneChain, EmptyListGivesError)
{
  const auto result = road_following::build_lane_chain(lanelet::ConstLanelets{});

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), "cannot combine an empty lanelet list");
}

autoware_perception_msgs::msg::Shape make_box_shape(const double length, const double width)
{
  autoware_perception_msgs::msg::Shape shape;
  shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  shape.dimensions.x = length;
  shape.dimensions.y = width;
  return shape;
}

// Returns `count` samples on the x axis at a constant `speed`, one sample every `time_step`
// seconds, with 1 m by 1 m footprints. The first sample is at x = 0 and has the time `start_time`.
// The test helper `create_trajectory_data` in test_trajectory_utilities.cpp builds its data the
// same way.
TrajectoryData make_straight_trajectory(
  const double speed, const double time_step, const size_t count, const double start_time = 0.0)
{
  TimeTrajectory times;
  TravelDistanceTrajectory distances;
  PoseTrajectory poses;
  for (size_t i = 0; i < count; ++i) {
    const double time = static_cast<double>(i) * time_step;
    times.push_back(start_time + time);
    distances.push_back(speed * time);
    geometry_msgs::msg::Pose pose;
    pose.position.x = speed * time;
    pose.orientation.w = 1.0;
    poses.push_back(pose);
  }
  auto footprints =
    trajectory::footprint::compute_footprint_trajectory(poses, make_box_shape(1.0, 1.0));
  return TrajectoryData{
    TrajectoryIdentification{"test"}, std::move(times), std::move(distances), std::move(poses),
    std::move(footprints)};
}

TEST(FindWindowEnd, LowSpeedGivesFirstSampleOver30m)
{
  // 4 m/s, one sample per second. Sample 4 is the first over 3 s (16 m). Sample 8 is the first
  // over 30 m (32 m, 8 s), so the 30 m limit sets the end.
  const auto trajectory = make_straight_trajectory(4.0, 1.0, 12);

  EXPECT_EQ(road_following::find_window_end(trajectory, RoadFollowingParams{}), 8U);
}

TEST(FindWindowEnd, HighSpeedGivesFirstSampleOver3s)
{
  // 20 m/s, one sample per 0.7 s. Sample 3 is the first over 30 m (42 m). Sample 5 is the first
  // over 3 s (3.5 s, 70 m), so the 3 s limit sets the end.
  const auto trajectory = make_straight_trajectory(20.0, 0.7, 8);

  EXPECT_EQ(road_following::find_window_end(trajectory, RoadFollowingParams{}), 5U);
}

TEST(FindWindowEnd, NoSampleOverBothGivesSize)
{
  // 20 m/s, one sample per 0.5 s. The last sample is at 40 m and 2 s, so no sample is over 3 s.
  const auto trajectory = make_straight_trajectory(20.0, 0.5, 5);

  EXPECT_EQ(road_following::find_window_end(trajectory, RoadFollowingParams{}), trajectory.size());
}

TEST(FindWindowEnd, UsesTimeFromFirstSample)
{
  // The high speed case with the first sample at 10 s. Each time is over 3 s, so a function that
  // does not subtract the first time returns 3, the first sample over 30 m.
  const auto trajectory = make_straight_trajectory(20.0, 0.7, 8, 10.0);

  EXPECT_EQ(road_following::find_window_end(trajectory, RoadFollowingParams{}), 5U);
}

geometry_msgs::msg::Pose make_pose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  return pose;
}

// Returns the corners of a `length` by `width` box centered on `pose`, made by the CCF footprint
// helper.
std::array<Point2d, 4> make_box_corners(
  const geometry_msgs::msg::Pose & pose, const double length, const double width)
{
  const auto footprints = trajectory::footprint::compute_footprint_trajectory(
    PoseTrajectory{pose}, make_box_shape(length, width));
  return std::get<QuadTrajectory>(footprints).front();
}

// Returns a TrajectoryData with one sample for each pose, 0.1 s apart, and the given footprints.
TrajectoryData make_trajectory(const PoseTrajectory & poses, const QuadTrajectory & footprints)
{
  TimeTrajectory times;
  TravelDistanceTrajectory distances;
  for (size_t i = 0; i < poses.size(); ++i) {
    times.push_back(0.1 * static_cast<double>(i));
    distances.push_back(autoware::motion_utils::calcSignedArcLength(poses, 0, i));
  }
  return TrajectoryData{TrajectoryIdentification{"test"}, times, distances, poses, footprints};
}

// Returns a TrajectoryData with a 1 m by 1 m footprint on each pose. The corners of a footprint
// are at most 0.71 m from its pose, so a pose 0.79 m or more from both bounds of the 3 m lane
// gives a footprint that crosses no bound.
TrajectoryData make_small_footprint_trajectory(const PoseTrajectory & poses)
{
  QuadTrajectory footprints;
  for (const auto & pose : poses) {
    footprints.push_back(make_box_corners(pose, 1.0, 1.0));
  }
  return make_trajectory(poses, footprints);
}

// The chain {1, 2, 4} of the lane shift map. Lanelet 1 runs on the x axis from x = 0 to 20 m.
road_following::LaneChain make_lane_shift_chain(const TestMap & test_map)
{
  const auto & layer = test_map.map().laneletLayer;
  return road_following::build_lane_chain({layer.get(1), layer.get(2), layer.get(4)}).value();
}

TEST(FollowsChain, SamplesInsideAllLimitsGiveTrue)
{
  // The samples are on the chain centerline, with its heading, in lanelets 1, 2, and 4.
  const auto test_map = make_lane_shift_map();
  const auto chain = make_lane_shift_chain(test_map);
  const double shift_yaw = std::atan2(-3.0, 10.0);
  const auto trajectory = make_small_footprint_trajectory(
    {make_pose(5.0, 0.0, 0.0), make_pose(15.0, 0.0, 0.0), make_pose(25.0, -1.5, shift_yaw),
     make_pose(40.0, -3.0, 0.0)});

  EXPECT_TRUE(
    road_following::follows_chain(
      chain, test_map.map().laneletLayer.get(1), trajectory, trajectory.size(),
      RoadFollowingParams{}));
}

TEST(FollowsChain, FootprintCrossingBoundGivesFalse)
{
  // The sample is on the centerline of lanelet 1, so only the bound test can fail. Each box is
  // 1.2 m wide and reaches 0.2 m past one bound of lanelet 1 (y = 1.5 m or y = -1.5 m).
  const auto test_map = make_lane_shift_map();
  const auto chain = make_lane_shift_chain(test_map);
  const auto & start_lanelet = test_map.map().laneletLayer.get(1);
  const PoseTrajectory poses{make_pose(10.0, 0.0, 0.0)};
  const auto over_left =
    make_trajectory(poses, {make_box_corners(make_pose(10.0, 1.1, 0.0), 2.0, 1.2)});
  const auto over_right =
    make_trajectory(poses, {make_box_corners(make_pose(10.0, -1.1, 0.0), 2.0, 1.2)});

  EXPECT_FALSE(
    road_following::follows_chain(chain, start_lanelet, over_left, 1U, RoadFollowingParams{}));
  EXPECT_FALSE(
    road_following::follows_chain(chain, start_lanelet, over_right, 1U, RoadFollowingParams{}));
}

TEST(FollowsChain, LateralOverLimitGivesFalse)
{
  // Small boxes cross no bound, so only the lateral test (limit 0.5 m) can fail.
  const auto test_map = make_lane_shift_map();
  const auto chain = make_lane_shift_chain(test_map);
  const auto & start_lanelet = test_map.map().laneletLayer.get(1);
  const auto within_limit = make_small_footprint_trajectory({make_pose(10.0, 0.4, 0.0)});
  const auto over_limit = make_small_footprint_trajectory({make_pose(10.0, 0.6, 0.0)});

  EXPECT_TRUE(
    road_following::follows_chain(chain, start_lanelet, within_limit, 1U, RoadFollowingParams{}));
  EXPECT_FALSE(
    road_following::follows_chain(chain, start_lanelet, over_limit, 1U, RoadFollowingParams{}));
}

TEST(FollowsChain, HeadingOverLimitGivesFalse)
{
  // Small boxes cross no bound, so only the heading test (limit 0.1 rad) can fail.
  const auto test_map = make_lane_shift_map();
  const auto chain = make_lane_shift_chain(test_map);
  const auto & start_lanelet = test_map.map().laneletLayer.get(1);
  const auto within_limit = make_small_footprint_trajectory({make_pose(10.0, 0.0, 0.05)});
  const auto over_limit = make_small_footprint_trajectory({make_pose(10.0, 0.0, 0.15)});

  EXPECT_TRUE(
    road_following::follows_chain(chain, start_lanelet, within_limit, 1U, RoadFollowingParams{}));
  EXPECT_FALSE(
    road_following::follows_chain(chain, start_lanelet, over_limit, 1U, RoadFollowingParams{}));
}

TEST(FollowsChain, SkipsBoundsBeforeStartLanelet)
{
  // The sample is on the centerline of lanelet 2. The 0.4 m box lies on the left bound of
  // lanelet 1 (y = 1.5 m, x up to 20 m) and touches no bound of lanelets 2 and 4. A walk from
  // lanelet 2 skips that bound. A walk from lanelet 1 tests it.
  const auto test_map = make_lane_shift_map();
  const auto chain = make_lane_shift_chain(test_map);
  const auto & layer = test_map.map().laneletLayer;
  const auto trajectory = make_trajectory(
    {make_pose(25.0, -1.5, std::atan2(-3.0, 10.0))},
    {make_box_corners(make_pose(19.0, 1.5, 0.0), 0.4, 0.4)});

  EXPECT_TRUE(
    road_following::follows_chain(chain, layer.get(2), trajectory, 1U, RoadFollowingParams{}));
  EXPECT_FALSE(
    road_following::follows_chain(chain, layer.get(1), trajectory, 1U, RoadFollowingParams{}));
}

TEST(FollowsChain, StartOutsideChainGivesFalse)
{
  // Lanelet 3, the branch lanelet, is not in the chain {1, 2, 4}. The sample passes all three
  // tests with a walk from lanelet 1.
  const auto test_map = make_lane_shift_map();
  const auto chain = make_lane_shift_chain(test_map);
  const auto & layer = test_map.map().laneletLayer;
  const auto trajectory = make_small_footprint_trajectory({make_pose(10.0, 0.0, 0.0)});

  EXPECT_TRUE(
    road_following::follows_chain(chain, layer.get(1), trajectory, 1U, RoadFollowingParams{}));
  EXPECT_FALSE(
    road_following::follows_chain(chain, layer.get(3), trajectory, 1U, RoadFollowingParams{}));
}

TEST(FollowsChain, EmptyWindowGivesTrue)
{
  // The only sample has a footprint that crosses the left bound of lanelet 1. A window of one
  // sample tests it. An empty window tests nothing.
  const auto test_map = make_lane_shift_map();
  const auto chain = make_lane_shift_chain(test_map);
  const auto & start_lanelet = test_map.map().laneletLayer.get(1);
  const auto trajectory = make_trajectory(
    {make_pose(10.0, 0.0, 0.0)}, {make_box_corners(make_pose(10.0, 1.1, 0.0), 2.0, 1.2)});

  EXPECT_FALSE(
    road_following::follows_chain(chain, start_lanelet, trajectory, 1U, RoadFollowingParams{}));
  EXPECT_TRUE(
    road_following::follows_chain(chain, start_lanelet, trajectory, 0U, RoadFollowingParams{}));
}

TEST(FollowsChain, SkipsSamplesFromWindowEnd)
{
  // Sample 0 is inside the lane. Sample 1 has a footprint that crosses the left bound of
  // lanelet 1. A window that ends at sample 1 does not test it.
  const auto test_map = make_lane_shift_map();
  const auto chain = make_lane_shift_chain(test_map);
  const auto & start_lanelet = test_map.map().laneletLayer.get(1);
  const PoseTrajectory poses{make_pose(5.0, 0.0, 0.0), make_pose(10.0, 0.0, 0.0)};
  const auto trajectory = make_trajectory(
    poses, {make_box_corners(poses.at(0), 1.0, 1.0),
            make_box_corners(make_pose(10.0, 1.1, 0.0), 2.0, 1.2)});

  EXPECT_TRUE(
    road_following::follows_chain(chain, start_lanelet, trajectory, 1U, RoadFollowingParams{}));
  EXPECT_FALSE(
    road_following::follows_chain(chain, start_lanelet, trajectory, 2U, RoadFollowingParams{}));
}
}  // namespace
}  // namespace autoware::trajectory_validator::plugin::safety
