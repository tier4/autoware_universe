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

// Tests for the lane-following check (LaneFollowingChecker) — the rules and patterns from
// docs/lane_following.md, on both synthetic maps and the real test map (test/map). The check
// lives outside the tracker: the LaneTracker supplies the map, routing graph and reference lane,
// and LaneFollowingChecker::evaluate makes the following decision.

#include "synthetic_lanelet_maps.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lane_event_classifier/detail/lane_tracker.hpp>
#include <autoware_lane_event_classifier/lane_following/checker.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <iostream>
#include <vector>

namespace autoware::lane_event_classifier
{

namespace
{
// Runs the lane-following check for the ego reference point against the tracker's held reference
// lane.
LaneFollowingResult check_following(
  const LaneFollowingChecker & checker, const LaneTracker & tracker,
  const lanelet::BasicPoint2d & ego_point)
{
  return checker.evaluate(
    tracker.lanelet_map_ptr(), tracker.routing_graph_ptr(),
    tracker.reference_lane().reference_lane_id, ego_point);
}

// Departure onset is simply "not lane following" for the ego reference point.
bool departed(
  const LaneFollowingChecker & checker, const LaneTracker & tracker,
  const lanelet::BasicPoint2d & ego_point)
{
  return !check_following(checker, tracker, ego_point).is_following;
}
}  // namespace

// ── Synthetic rule tests ─────────────────────────────────────────────────────

// distance_to_lane backs the held-reference departure reset: it is zero while the ego is inside
// the lane and grows with how far the ego has strayed outside it (nullopt for an unknown lane).
TEST(LaneTrackerTest, distance_to_lane_measures_departure_from_the_lane)
{
  lanelet::Id lane_id{};
  auto map = test_maps::make_single_lane_map(lane_id);  // x=[0,10], y=[-2,2]

  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(map).has_value());

  ASSERT_TRUE(tracker.distance_to_lane(lane_id, {5.0, 0.0}).has_value());
  EXPECT_NEAR(*tracker.distance_to_lane(lane_id, {5.0, 0.0}), 0.0, 1e-6);   // inside
  EXPECT_NEAR(*tracker.distance_to_lane(lane_id, {5.0, 5.0}), 3.0, 1e-6);   // 3 m past the y=2 edge
  EXPECT_NEAR(*tracker.distance_to_lane(lane_id, {15.0, 0.0}), 5.0, 1e-6);  // 5 m past the x=10 end
  EXPECT_FALSE(tracker.distance_to_lane(lanelet::InvalId, {5.0, 0.0}).has_value());
}

// Entering a next lane (forward driving across a lanelet boundary) is not a lateral departure;
// moving into a lane that is neither the reference lane nor a next lane is.
TEST(LaneFollowingTest, next_lane_is_not_departure_lateral_is)
{
  lanelet::Id id_a{};
  lanelet::Id id_b{};
  auto map = test_maps::make_next_lane_map(id_a, id_b);

  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(map).has_value());

  tracker.update(test_maps::make_input({id_a, id_b}, 5.0, 0.0, 0, test_maps::ms(0)));
  ASSERT_EQ(tracker.reference_lane().reference_lane_id, id_a);
  ASSERT_TRUE(tracker.reference_lane().is_reference_lane_on_route);
  tracker
    .hold_reference_lane();  // hold the reference lane so we can probe the relation test directly

  LaneFollowingChecker checker;
  EXPECT_FALSE(departed(checker, tracker, {15.0, 0.0}));  // inside next lane lane_b
  EXPECT_TRUE(departed(checker, tracker, {5.0, 5.0}));  // lateral: neither reference lane nor next
}

// An off-route reference lane still departs on a lateral exit (lane_a has no next lane here, so the
// exit cannot be forward driving).
TEST(LaneFollowingTest, off_route_reference_lane_departs_on_lateral_exit)
{
  lanelet::Id id_a{};
  lanelet::Id id_b{};
  auto map = test_maps::make_parallel_map(id_a, id_b);

  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(map).has_value());

  // Route runs down lane_b; ego sits in lane_a, which is therefore off-route.
  tracker.update(test_maps::make_input({id_b}, 5.0, 0.0, 0, test_maps::ms(0)));
  ASSERT_EQ(tracker.reference_lane().reference_lane_id, id_a);
  ASSERT_FALSE(tracker.reference_lane().is_reference_lane_on_route);
  tracker.hold_reference_lane();

  LaneFollowingChecker checker;
  EXPECT_FALSE(departed(checker, tracker, {5.0, 0.0}));  // still inside off-route reference lane
  EXPECT_TRUE(departed(checker, tracker, {5.0, 3.0}));   // lateral exit → departure
}

// An off-route reference lane driving forward into its OWN next lane is lane following, not a
// departure.
TEST(LaneFollowingTest, off_route_forward_into_next_lane_is_not_departure)
{
  lanelet::Id id_a{};
  lanelet::Id id_b{};
  auto map = test_maps::make_next_lane_map(id_a, id_b);  // lane_a -> lane_b, connected end-to-end

  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(map).has_value());

  // Route runs on lane_b only, so the ego's lane (lane_a) is off-route.
  tracker.update(test_maps::make_input({id_b}, 5.0, 0.0, 0, test_maps::ms(0)));
  ASSERT_EQ(tracker.reference_lane().reference_lane_id, id_a);
  ASSERT_FALSE(tracker.reference_lane().is_reference_lane_on_route);
  tracker.hold_reference_lane();

  LaneFollowingChecker checker;
  EXPECT_FALSE(departed(checker, tracker, {15.0, 0.0}));  // in lane_a's own next lane → forward
}

// A reference point in the reference lane's PREVIOUS lane is still on the corridor, not a
// departure.
TEST(LaneFollowingTest, reference_point_in_previous_lane_is_not_departure)
{
  lanelet::Id id_a{};
  lanelet::Id id_b{};
  auto map = test_maps::make_next_lane_map(id_a, id_b);  // lane_a (x=[0,10]) -> lane_b (x=[10,20])

  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(map).has_value());

  // Ego centre in lane_b, so lane_b is the reference lane and lane_a is its previous lane.
  tracker.update(test_maps::make_input({id_a, id_b}, 12.0, 0.0, 0, test_maps::ms(0)));
  ASSERT_EQ(tracker.reference_lane().reference_lane_id, id_b);
  tracker.hold_reference_lane();

  LaneFollowingChecker checker;
  EXPECT_FALSE(departed(checker, tracker, {8.0, 0.0}));  // in lane_a — the previous lane
}

// within_lateral_tolerance: a reference point just outside the lane but within the lateral
// tolerance is still following; beyond the tolerance it departs.
TEST(LaneFollowingTest, within_lateral_tolerance_is_not_departure)
{
  lanelet::Id id{};
  auto map = test_maps::make_single_lane_map(id);

  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(map).has_value());

  tracker.update(test_maps::make_input({id}, 5.0, 0.0, 0, test_maps::ms(0)));
  tracker.hold_reference_lane();

  LaneFollowingConfig config;
  config.lateral_tolerance_m = 0.5;
  LaneFollowingChecker checker(config);

  EXPECT_FALSE(departed(checker, tracker, {5.0, 2.3}));  // 0.3 m outside — within tolerance
  EXPECT_TRUE(departed(checker, tracker, {5.0, 2.8}));   // 0.8 m outside — beyond tolerance
}

// road_shoulder_exempt: a reference point overlapping a road shoulder is following.
TEST(LaneFollowingTest, overlapping_road_shoulder_is_not_departure)
{
  lanelet::Id road_id{};
  lanelet::Id shoulder_id{};
  auto map = test_maps::make_road_and_shoulder_map(road_id, shoulder_id);

  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(map).has_value());

  tracker.update(test_maps::make_input({road_id}, 5.0, 0.0, 0, test_maps::ms(0)));
  ASSERT_EQ(tracker.reference_lane().reference_lane_id, road_id);
  tracker.hold_reference_lane();

  LaneFollowingChecker checker;                          // default: road-shoulder exemption on
  EXPECT_FALSE(departed(checker, tracker, {5.0, 3.0}));  // inside the road shoulder → following

  LaneFollowingConfig config;
  config.enable_road_shoulder_exemption = false;
  LaneFollowingChecker checker_no_shoulder(config);
  EXPECT_TRUE(departed(checker_no_shoulder, tracker, {5.0, 3.0}));  // exemption off → departs
}

// turn_lane_exempt: while in a turn / intersection lane, going out of the lane is following.
TEST(LaneFollowingTest, turn_lane_out_of_lane_is_not_departure)
{
  lanelet::Id id{};
  auto map = test_maps::make_turn_lane_map(id);

  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(map).has_value());

  tracker.update(test_maps::make_input({id}, 5.0, 0.0, 0, test_maps::ms(0)));
  tracker.hold_reference_lane();

  LaneFollowingChecker checker;                           // default: turn-lane exemption on
  EXPECT_FALSE(departed(checker, tracker, {5.0, 10.0}));  // far out of lane, but turn lane

  LaneFollowingConfig config;
  config.enable_turn_lane_exemption = false;
  LaneFollowingChecker checker_no_turn(config);
  EXPECT_TRUE(departed(checker_no_turn, tracker, {5.0, 10.0}));  // exemption off → departs
}

// virtual_boundary_exempt: crossing a virtual boundary is following; the solid boundary departs.
TEST(LaneFollowingTest, crossing_virtual_boundary_is_not_departure)
{
  lanelet::Id id{};
  auto map = test_maps::make_virtual_left_bound_map(id);

  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(map).has_value());

  tracker.update(test_maps::make_input({id}, 5.0, 0.0, 0, test_maps::ms(0)));
  tracker.hold_reference_lane();

  LaneFollowingChecker checker;  // default: virtual-boundary exemption on
  EXPECT_FALSE(
    departed(checker, tracker, {5.0, 3.0}));  // across the virtual LEFT bound → following
  EXPECT_TRUE(departed(checker, tracker, {5.0, -3.0}));  // across the solid RIGHT bound → departs

  LaneFollowingConfig config;
  config.enable_virtual_boundary_exemption = false;
  LaneFollowingChecker checker_no_virtual(config);
  EXPECT_TRUE(departed(checker_no_virtual, tracker, {5.0, 3.0}));  // exemption off → departs
}

// ── Real map tests (test/map/lanelet2_map.osm) ───────────────────────────────

namespace
{
lanelet::LaneletMapPtr load_test_map()
{
  const auto map_const =
    autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(TEST_MAP_PATH);
  return map_const ? autoware::experimental::lanelet2_utils::remove_const(map_const) : nullptr;
}

lanelet::BasicPoint2d centerline_point(const lanelet::LaneletMapPtr & map, lanelet::Id id)
{
  const auto centerline = map->laneletLayer.get(id).centerline2d();
  const auto & point = centerline[centerline.size() / 2];
  return {point.x(), point.y()};
}

struct Scenario
{
  const char * name;
  double ego_x;
  double ego_y;
  std::vector<lanelet::BasicPoint2d> footprint;  // exact corners from the bag log
};
}  // namespace

// Regression: the three logged bag positions that used to fire a false LANE_CHANGING are all lane
// following once the reference lane is anchored to the lanelet the ego is actually in.
TEST(LaneFollowingMapTest, logged_false_positive_positions_are_following)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map)) << "failed to load " << TEST_MAP_PATH;

  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(map).has_value());
  LaneFollowingChecker checker;

  const std::vector<Scenario> scenarios = {
    {"case1 (log reference lane=52)",
     89434.43,
     42635.48,
     {{89431.27, 42640.39},
      {89433.54, 42641.25},
      {89434.73, 42638.14},
      {89436.12, 42634.49},
      {89433.85, 42633.63},
      {89432.46, 42637.27}}},
    {"case2 (log reference lane=51)",
     89438.56,
     42624.91,
     {{89435.53, 42629.89},
      {89437.81, 42630.70},
      {89438.93, 42627.56},
      {89440.23, 42623.88},
      {89437.94, 42623.07},
      {89436.64, 42626.75}}},
    {"case3 (junction, reference lane=1172)",
     89358.68,
     42817.13,
     {{89360.14, 42815.86}, {89356.24, 42816.05}}},
  };

  for (const auto & scenario : scenarios) {
    const lanelet::BasicPoint2d ego{scenario.ego_x, scenario.ego_y};
    const auto ego_lane_ids = tracker.lanelet_ids_at(ego);
    std::cout << "\n=== " << scenario.name << " === ego_now=["
              << fmt::format("{}", fmt::join(ego_lane_ids, ",")) << "] footprint_lanes=["
              << fmt::format("{}", fmt::join(tracker.footprint_lane_ids(scenario.footprint), ","))
              << "]\n";
    ASSERT_FALSE(ego_lane_ids.empty()) << "ego is not inside any lanelet";

    tracker.release_reference_lane();
    tracker.update(
      test_maps::make_input({ego_lane_ids.front()}, scenario.ego_x, scenario.ego_y, 0, 0));
    tracker.hold_reference_lane();

    const auto result = check_following(checker, tracker, ego);
    std::cout << "reference lane=" << tracker.reference_lane().reference_lane_id
              << " is_following=" << result.is_following << " (" << to_string(result.reason)
              << ")\n";
    EXPECT_TRUE(result.is_following) << scenario.name;
  }
}

// inside_connected_sequence: the connected lane sequence spans multiple hops. Reference lane 47,
// ego two hops down in 51 (47 -> 1167 -> 51). Full sequence length → following; a short sequence
// (exemptions off to isolate the connected-sequence check) → departs.
TEST(LaneFollowingMapTest, connected_sequence_spans_multiple_hops)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(map).has_value());

  const auto point_in_47 = centerline_point(map, 47);
  const auto point_in_51 = centerline_point(map, 51);

  tracker.update(test_maps::make_input({47}, point_in_47.x(), point_in_47.y(), 0, 0));
  ASSERT_EQ(tracker.reference_lane().reference_lane_id, 47);
  tracker.hold_reference_lane();

  LaneFollowingChecker checker;
  EXPECT_TRUE(check_following(checker, tracker, point_in_51).is_following);  // sequence reaches 51

  LaneFollowingConfig config;
  config.connected_sequence_length_m = 1.0;
  config.enable_turn_lane_exemption = false;
  config.enable_virtual_boundary_exemption = false;
  LaneFollowingChecker short_checker(config);
  EXPECT_FALSE(
    check_following(short_checker, tracker, point_in_51).is_following);  // 51 unreachable
}

// Raw prompt: route runs on 1167 -> 51, but the ego drives the parallel corridor 1169 -> 52
// (off-route). This is lane following, not a lane change.
TEST(LaneFollowingMapTest, off_route_parallel_corridor_is_following)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(map).has_value());
  LaneFollowingChecker checker;

  const auto point_in_1169 = centerline_point(map, 1169);
  const auto point_in_52 = centerline_point(map, 52);
  const std::vector<lanelet::Id> route{1167, 51};

  // Ego in 1169 (parallel to the route's 1167).
  tracker.update(test_maps::make_input(route, point_in_1169.x(), point_in_1169.y(), 0, 0));
  EXPECT_TRUE(check_following(checker, tracker, point_in_1169).is_following);

  // Ego advances to 52 (parallel to the route's 51).
  tracker.update(test_maps::make_input(route, point_in_52.x(), point_in_52.y(), 0, 0));
  EXPECT_TRUE(check_following(checker, tracker, point_in_52).is_following);
}

// Same idea but isolated from the turn / virtual exemptions: 51 and 52 are parallel non-turn road
// lanes with solid bounds. With the route on 51 and the ego in 52, the reference lane anchors to
// the ego's actual lane (52) and it is following its own connected sequence — reason
// inside_connected_sequence, not a turn / virtual exemption.
TEST(LaneFollowingMapTest, off_route_parallel_non_turn_lane_is_following)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(map).has_value());
  LaneFollowingChecker checker;

  const auto point_in_52 = centerline_point(map, 52);

  // Route on 51; ego drives the parallel lane 52 (off-route).
  tracker.update(test_maps::make_input({51}, point_in_52.x(), point_in_52.y(), 0, 0));
  ASSERT_EQ(
    tracker.reference_lane().reference_lane_id, 52);  // reference lane = ego's lane, not route 51

  const auto result = check_following(checker, tracker, point_in_52);
  EXPECT_TRUE(result.is_following);
  EXPECT_EQ(result.reason, LaneFollowingReason::inside_connected_sequence);
}

}  // namespace autoware::lane_event_classifier
