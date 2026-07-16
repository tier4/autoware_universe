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

#include "synthetic_lanelet_maps.hpp"

#include <autoware_lane_event_classifier/detail/lane_tracker.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace autoware::lane_event_classifier
{
namespace
{
using test_maps::make_input;
using test_maps::make_next_lane_map;
using test_maps::make_parallel_map;
using test_maps::make_single_lane_map;

// A lane id that no synthetic map ever mints, for the "unknown id" queries.
constexpr lanelet::Id kUnknownLaneId = 999999;

bool contains(const std::vector<lanelet::Id> & ids, lanelet::Id id)
{
  return std::find(ids.cbegin(), ids.cend(), id) != ids.cend();
}
}  // namespace

TEST(LaneTrackerTest, set_lanelet_map_rejects_null_and_empty)
{
  LaneTracker tracker;

  EXPECT_FALSE(tracker.set_lanelet_map(nullptr).has_value());
  EXPECT_FALSE(tracker.has_lanelet_map());

  const auto empty_map = std::make_shared<lanelet::LaneletMap>();
  EXPECT_FALSE(tracker.set_lanelet_map(empty_map).has_value());
  EXPECT_FALSE(tracker.has_lanelet_map());
}

TEST(LaneTrackerTest, set_lanelet_map_builds_routing_graph)
{
  lanelet::Id lane_id = lanelet::InvalId;
  LaneTracker tracker;

  ASSERT_TRUE(tracker.set_lanelet_map(make_single_lane_map(lane_id)).has_value());
  EXPECT_TRUE(tracker.has_lanelet_map());
  EXPECT_NE(tracker.lanelet_map_ptr(), nullptr);
  EXPECT_NE(tracker.routing_graph_ptr(), nullptr);
}

TEST(LaneTrackerTest, update_anchors_reference_to_route_lane)
{
  lanelet::Id lane_id = lanelet::InvalId;
  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(make_single_lane_map(lane_id)).has_value());

  tracker.update(make_input({lane_id}, 5.0, 0.0, 0, 0));

  EXPECT_EQ(tracker.reference_lane().reference_lane_id, lane_id);
  EXPECT_TRUE(tracker.reference_lane().is_reference_lane_on_route);
  EXPECT_EQ(tracker.last_selected_lane_id(), lane_id);
}

TEST(LaneTrackerTest, update_anchors_off_route_lane_when_no_route_match)
{
  lanelet::Id lane_id = lanelet::InvalId;
  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(make_single_lane_map(lane_id)).has_value());

  // Empty route: the ego still sits inside a lane, so it becomes an off-route reference lane.
  tracker.update(make_input({}, 5.0, 0.0, 0, 0));

  EXPECT_EQ(tracker.reference_lane().reference_lane_id, lane_id);
  EXPECT_FALSE(tracker.reference_lane().is_reference_lane_on_route);
}

TEST(LaneTrackerTest, reference_reanchors_on_forward_progress)
{
  lanelet::Id id_a = lanelet::InvalId;
  lanelet::Id id_b = lanelet::InvalId;
  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(make_next_lane_map(id_a, id_b)).has_value());

  tracker.update(make_input({id_a, id_b}, 5.0, 0.0, 0, 0));
  ASSERT_EQ(tracker.reference_lane().reference_lane_id, id_a);

  // Ego advances into lane_b, a next lane of lane_a: the reference lane follows it forward.
  tracker.update(make_input({id_a, id_b}, 15.0, 0.0, 1, 0));
  EXPECT_EQ(tracker.reference_lane().reference_lane_id, id_b);
  EXPECT_EQ(tracker.last_selected_lane_id(), id_b);
  EXPECT_FALSE(tracker.is_last_reanchor_blocked());
}

TEST(LaneTrackerTest, reference_holds_across_lateral_move)
{
  lanelet::Id id_a = lanelet::InvalId;
  lanelet::Id id_b = lanelet::InvalId;
  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(make_parallel_map(id_a, id_b)).has_value());

  tracker.update(make_input({id_a, id_b}, 5.0, 0.0, 0, 0));
  ASSERT_EQ(tracker.reference_lane().reference_lane_id, id_a);

  // Ego moves sideways into the parallel lane_b (not a next lane): the reference lane must not
  // advance, and the blocked-reanchor diagnostic fires.
  tracker.update(make_input({id_a, id_b}, 5.0, 4.0, 1, 0));
  EXPECT_EQ(tracker.reference_lane().reference_lane_id, id_a);
  EXPECT_EQ(tracker.last_selected_lane_id(), id_b);
  EXPECT_TRUE(tracker.is_last_reanchor_blocked());
}

TEST(LaneTrackerTest, hold_freezes_reference_until_released)
{
  lanelet::Id id_a = lanelet::InvalId;
  lanelet::Id id_b = lanelet::InvalId;
  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(make_next_lane_map(id_a, id_b)).has_value());

  tracker.update(make_input({id_a, id_b}, 5.0, 0.0, 0, 0));
  ASSERT_EQ(tracker.reference_lane().reference_lane_id, id_a);

  tracker.hold_reference_lane();
  EXPECT_TRUE(tracker.is_reference_lane_held());

  // While held the reference lane is frozen even as the ego advances into lane_b.
  tracker.update(make_input({id_a, id_b}, 15.0, 0.0, 1, 0));
  EXPECT_EQ(tracker.reference_lane().reference_lane_id, id_a);

  // Releasing clears the reference so the next update re-anchors to the ego's current lane.
  tracker.release_reference_lane();
  EXPECT_FALSE(tracker.is_reference_lane_held());
  tracker.update(make_input({id_a, id_b}, 15.0, 0.0, 2, 0));
  EXPECT_EQ(tracker.reference_lane().reference_lane_id, id_b);
}

TEST(LaneTrackerTest, distance_to_lane_reports_inside_outside_and_unknown)
{
  lanelet::Id lane_id = lanelet::InvalId;
  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(make_single_lane_map(lane_id)).has_value());

  // Lane spans x=[0,10], y=[-2,2].
  const auto inside = tracker.distance_to_lane(lane_id, {5.0, 0.0});
  ASSERT_TRUE(inside.has_value());
  EXPECT_DOUBLE_EQ(*inside, 0.0);

  const auto outside = tracker.distance_to_lane(lane_id, {5.0, 10.0});
  ASSERT_TRUE(outside.has_value());
  EXPECT_NEAR(*outside, 8.0, 1e-6);

  EXPECT_FALSE(tracker.distance_to_lane(kUnknownLaneId, {5.0, 0.0}).has_value());
}

TEST(LaneTrackerTest, lanelet_lookup_helpers)
{
  lanelet::Id lane_id = lanelet::InvalId;
  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(make_single_lane_map(lane_id)).has_value());

  EXPECT_TRUE(tracker.lanelet_exists(lane_id));
  EXPECT_TRUE(tracker.get_lanelet(lane_id).has_value());

  EXPECT_FALSE(tracker.lanelet_exists(lanelet::InvalId));
  EXPECT_FALSE(tracker.lanelet_exists(kUnknownLaneId));
  EXPECT_FALSE(tracker.get_lanelet(kUnknownLaneId).has_value());

  EXPECT_TRUE(contains(tracker.lanelet_ids_at({5.0, 0.0}), lane_id));
  EXPECT_TRUE(tracker.lanelet_ids_at({5.0, 10.0}).empty());
}

TEST(LaneTrackerTest, footprint_fully_inside_lane)
{
  lanelet::Id lane_id = lanelet::InvalId;
  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(make_single_lane_map(lane_id)).has_value());

  const std::vector<lanelet::BasicPoint2d> inside_footprint{
    {2.0, -1.0}, {2.0, 1.0}, {8.0, 1.0}, {8.0, -1.0}};
  EXPECT_TRUE(tracker.is_footprint_fully_inside_lane(lane_id, inside_footprint));

  // One corner at y=3 is outside the lane (y in [-2,2]).
  const std::vector<lanelet::BasicPoint2d> straddling_footprint{
    {2.0, -1.0}, {2.0, 3.0}, {8.0, 3.0}, {8.0, -1.0}};
  EXPECT_FALSE(tracker.is_footprint_fully_inside_lane(lane_id, straddling_footprint));

  EXPECT_FALSE(tracker.is_footprint_fully_inside_lane(kUnknownLaneId, inside_footprint));
}

TEST(LaneTrackerTest, next_lane_ids_follow_routing_graph)
{
  lanelet::Id id_a = lanelet::InvalId;
  lanelet::Id id_b = lanelet::InvalId;
  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(make_next_lane_map(id_a, id_b)).has_value());

  EXPECT_TRUE(contains(tracker.next_lane_ids(id_a), id_b));
  EXPECT_TRUE(tracker.next_lane_ids(id_b).empty());
}

TEST(LaneTrackerTest, straight_lane_sequence_spans_connected_lanes)
{
  lanelet::Id id_a = lanelet::InvalId;
  lanelet::Id id_b = lanelet::InvalId;
  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(make_next_lane_map(id_a, id_b)).has_value());

  const auto lane_a = tracker.get_lanelet(id_a);
  ASSERT_TRUE(lane_a.has_value());
  const auto & sequence_ids = tracker.straight_lane_sequence_ids(*lane_a, 100.0);
  EXPECT_NE(sequence_ids.count(id_a), 0u);
  EXPECT_NE(sequence_ids.count(id_b), 0u);

  // A repeat query with the same arguments returns the memoized set unchanged.
  const auto & cached_ids = tracker.straight_lane_sequence_ids(*lane_a, 100.0);
  EXPECT_EQ(cached_ids, sequence_ids);
}

TEST(LaneTrackerTest, route_primitive_cache_tracks_current_route)
{
  lanelet::Id id_a = lanelet::InvalId;
  lanelet::Id id_b = lanelet::InvalId;
  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(make_next_lane_map(id_a, id_b)).has_value());

  tracker.update(make_input({id_a}, 5.0, 0.0, 0, 0));
  EXPECT_TRUE(tracker.is_route_primitive(id_a));
  EXPECT_FALSE(tracker.is_route_primitive(id_b));
  EXPECT_NE(tracker.route_primitive_ids().count(id_a), 0u);
}

}  // namespace autoware::lane_event_classifier
