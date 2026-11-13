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

#include <autoware/lanelet2_utils/conversion.hpp>

#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
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

// Builds a small lanelet map in code. Equal coordinates give the same point object. So a
// lanelet that starts at the end points of another lanelet follows it in the routing graph.
class TestMap
{
public:
  // Adds a lanelet with straight bounds. `left` and `right` are the start and end points.
  lanelet::Lanelet add_lanelet(
    const lanelet::Id id, const std::pair<lanelet::BasicPoint2d, lanelet::BasicPoint2d> & left,
    const std::pair<lanelet::BasicPoint2d, lanelet::BasicPoint2d> & right,
    const std::string & turn_direction = "")
  {
    lanelet::LineString3d left_bound(next_id_++, {point(left.first), point(left.second)});
    lanelet::LineString3d right_bound(next_id_++, {point(right.first), point(right.second)});
    lanelet::Lanelet lanelet(id, left_bound, right_bound);
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

  lanelet::LaneletMapPtr map_{std::make_shared<lanelet::LaneletMap>()};
  std::map<std::pair<int64_t, int64_t>, lanelet::Point3d> points_;
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
}  // namespace
}  // namespace autoware::trajectory_validator::plugin::safety
