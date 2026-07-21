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

// Trajectory-driven intentional-lane-crossing tests (see docs/lane_crossing.md) on the real test
// map (test/map/lanelet2_map.osm). The primitive sequence 47 -> 1167 -> 51 -> 55 runs in the left
// lane of the three-lane bundle 47|48|50 (left->right); 48 is the off-route neighbour to the right
// of 47. A crossing is the ego dodging an object in 47 by poking toward 48 and returning: the
// planned path leaves the lane sequence around the object (out before it, back after it) and comes
// back. Trajectories are built from the map's centerlines; the ego is driven through cycles by
// advancing the message stamp.

#include "synthetic_lanelet_maps.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lane_event_classifier/detail/lane_tracker.hpp>
#include <autoware_lane_event_classifier/lane_crossing/classifier.hpp>
#include <autoware_lane_event_classifier/msg/driving_state.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::lane_event_classifier
{

namespace
{
using DS = autoware_lane_event_classifier::msg::DrivingState;
using TurnIndicatorsReport = autoware_vehicle_msgs::msg::TurnIndicatorsReport;

constexpr std::array<lanelet::Id, 4> route_ids()
{
  return {47, 1167, 51, 55};  // preferred-primitive sequence
}

lanelet::LaneletMapPtr load_test_map()
{
  const auto map_const =
    autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(TEST_MAP_PATH);
  return map_const ? autoware::experimental::lanelet2_utils::remove_const(map_const) : nullptr;
}

std::vector<lanelet::BasicPoint2d> centerline_points(
  const lanelet::LaneletMapPtr & map, lanelet::Id id)
{
  const auto centerline = map->laneletLayer.get(id).centerline2d();
  std::vector<lanelet::BasicPoint2d> points;
  points.reserve(centerline.size());
  for (const auto & point : centerline) {
    points.emplace_back(point.x(), point.y());
  }
  return points;
}

lanelet::BasicPoint2d point_at_fraction(
  const std::vector<lanelet::BasicPoint2d> & points, double fraction)
{
  const auto index = static_cast<std::size_t>(
    std::clamp(fraction, 0.0, 1.0) * static_cast<double>(points.size() - 1));
  return points[index];
}

lanelet::BasicPoint2d nearest_point_on(
  const std::vector<lanelet::BasicPoint2d> & points, const lanelet::BasicPoint2d & query)
{
  lanelet::BasicPoint2d nearest = points.front();
  double nearest_distance_sq = std::numeric_limits<double>::max();
  for (const auto & point : points) {
    const double distance_sq = (point - query).squaredNorm();
    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest = point;
    }
  }
  return nearest;
}

// Builds a trajectory that starts on lane_ids.front() and sweeps laterally through the listed lanes
// (in order) while advancing forward along the first lane. A single sweep {47, 48} commits to the
// neighbour and never returns - the lane-change signature, not a crossing.
std::vector<lanelet::BasicPoint2d> build_lane_trajectory(
  const lanelet::LaneletMapPtr & map, const std::vector<lanelet::Id> & lane_ids, double start_frac,
  double end_frac, std::size_t point_count)
{
  std::vector<std::vector<lanelet::BasicPoint2d>> centerlines;
  centerlines.reserve(lane_ids.size());
  for (const auto id : lane_ids) {
    centerlines.push_back(centerline_points(map, id));
  }
  const auto & base = centerlines.front();
  const double lane_span = static_cast<double>(lane_ids.size() - 1);

  std::vector<lanelet::BasicPoint2d> trajectory;
  trajectory.reserve(point_count);
  for (std::size_t index = 0; index < point_count; ++index) {
    const double progress = static_cast<double>(index) / static_cast<double>(point_count - 1);
    const double longitudinal_fraction = start_frac + (end_frac - start_frac) * progress;
    const lanelet::BasicPoint2d anchor = point_at_fraction(base, longitudinal_fraction);

    const double lane_position = progress * lane_span;
    const auto lane_index = static_cast<std::size_t>(lane_position);
    const auto next_lane_index = std::min(lane_index + 1, lane_ids.size() - 1);
    const double lateral_fraction = lane_position - static_cast<double>(lane_index);

    const lanelet::BasicPoint2d point_a = nearest_point_on(centerlines[lane_index], anchor);
    const lanelet::BasicPoint2d point_b = nearest_point_on(centerlines[next_lane_index], anchor);
    trajectory.push_back(point_a + (point_b - point_a) * lateral_fraction);
  }
  return trajectory;
}

// Builds an out-and-back poke: the ego advances along base_id while the lateral offset follows a
// triangle (0 -> 1 -> 0) toward poke_id, so the path crosses the shared boundary on the way out and
// again on the way back without committing to the neighbour lane. This is the closed departure the
// bracket onset detects (exit, then re-enter), unlike a lane change whose path commits fully.
std::vector<lanelet::BasicPoint2d> build_out_and_back_trajectory(
  const lanelet::LaneletMapPtr & map, lanelet::Id base_id, lanelet::Id poke_id, double start_frac,
  double end_frac, std::size_t point_count)
{
  const auto base = centerline_points(map, base_id);
  const auto poke = centerline_points(map, poke_id);
  std::vector<lanelet::BasicPoint2d> trajectory;
  trajectory.reserve(point_count);
  for (std::size_t index = 0; index < point_count; ++index) {
    const double progress = static_cast<double>(index) / static_cast<double>(point_count - 1);
    const double longitudinal_fraction = start_frac + (end_frac - start_frac) * progress;
    const lanelet::BasicPoint2d anchor = point_at_fraction(base, longitudinal_fraction);
    const double lateral_fraction = 1.0 - std::abs(2.0 * progress - 1.0);  // triangle 0 -> 1 -> 0
    const lanelet::BasicPoint2d base_point = nearest_point_on(base, anchor);
    const lanelet::BasicPoint2d poke_point = nearest_point_on(poke, anchor);
    trajectory.push_back(base_point + (poke_point - base_point) * lateral_fraction);
  }
  return trajectory;
}

// A small footprint square around a point (well within a road lane's width).
std::vector<lanelet::BasicPoint2d> footprint_box(const lanelet::BasicPoint2d & center)
{
  constexpr double half = 0.25;
  return {
    {center.x() - half, center.y() - half},
    {center.x() + half, center.y() - half},
    {center.x() + half, center.y() + half},
    {center.x() - half, center.y() + half}};
}

// Builds a footprint quad straddling lane 47's right boundary (shared with 48): its far edge sits
// far_m into 48 (the overshoot the physical source measures) and its near edge near_m back inside
// 47. Used to drive the physical footprint crossing directly, independent of the trajectory.
std::vector<lanelet::BasicPoint2d> build_straddle_footprint(
  const lanelet::LaneletMapPtr & map, double near_m, double far_m)
{
  const auto right_bound = map->laneletLayer.get(47).rightBound2d();
  const auto centerline_47 = centerline_points(map, 47);
  const std::size_t index = right_bound.size() / 2;
  const lanelet::BasicPoint2d b1{right_bound[index - 1].x(), right_bound[index - 1].y()};
  const lanelet::BasicPoint2d b2{right_bound[index].x(), right_bound[index].y()};
  const lanelet::BasicPoint2d tangent = (b2 - b1).normalized();
  lanelet::BasicPoint2d normal{-tangent.y(), tangent.x()};
  // Point the normal away from lane 47's centerline (toward the neighbour lane 48).
  const lanelet::BasicPoint2d near_center = nearest_point_on(centerline_47, b1);
  if (normal.dot(lanelet::BasicPoint2d{b1 - near_center}) < 0.0) {
    normal = -normal;
  }
  return {b1 - normal * near_m, b1 + normal * far_m, b2 + normal * far_m, b2 - normal * near_m};
}

// Drives one cycle exactly as the node does: update the tracker, run the classifier, then hold or
// release the reference lane based on whether a crossing is active.
class Simulator
{
public:
  Simulator(lanelet::LaneletMapPtr map, LaneCrossingConfig config)
  : classifier_{
      true, config, tracker_,
      LaneCrossingGeometry{config.crossing_look_ahead_m, config.footprint_boundary_overshoot_m},
      LaneCrossingObjects{config.object_longitudinal_window_m}}
  {
    const auto result = tracker_.set_lanelet_map(map);
    EXPECT_TRUE(result.has_value());
  }

  uint8_t step(const LaneEventInput & input)
  {
    tracker_.update(input);
    classifier_.update(input);
    const uint8_t state = classifier_.get_state();
    const bool is_active = state == DS::INTENTIONAL_LANE_CROSSING;
    if (is_active && !tracker_.is_reference_lane_held()) {
      tracker_.hold_reference_lane();
    } else if (!is_active && tracker_.is_reference_lane_held()) {
      tracker_.release_reference_lane();
    }
    return state;
  }

  [[nodiscard]] lanelet::Id reference_lane_id() const
  {
    return tracker_.reference_lane().reference_lane_id;
  }

private:
  LaneTracker tracker_;
  IntentionalCrossingClassifier classifier_;
};

LaneCrossingConfig make_config()
{
  LaneCrossingConfig config;
  config.enable_classifier = true;
  config.crossing_look_ahead_m = 50.0;
  config.crossing_persist_duration_s = 0.3;
  config.crossing_position_tolerance_m = 2.0;
  config.footprint_boundary_overshoot_m = 0.5;
  config.settle_confirm_duration_s = 0.5;
  config.confidence_factor = 0.5;
  config.object_longitudinal_window_m = 50.0;
  config.object_qualifying_memory_s = 3.0;
  return config;
}

// Splits a running millisecond count into a (sec, nsec) stamp.
std::pair<int32_t, uint32_t> stamp_from_ms(int64_t total_ms)
{
  return {
    static_cast<int32_t>(total_ms / 1000), static_cast<uint32_t>((total_ms % 1000) * 1'000'000)};
}

// Drives an out-and-back poke over the 47|48 boundary with an object ahead in 47 for up to
// max_cycles, returning true once a crossing onsets. Shared by the onset-positive tests.
bool run_until_crossing(
  Simulator & sim, const std::vector<lanelet::Id> & route,
  const std::vector<lanelet::BasicPoint2d> & trajectory, const lanelet::BasicPoint2d & ego,
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects, int max_cycles,
  int64_t & time_ms)
{
  for (int cycle = 0; cycle < max_cycles; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const uint8_t state = sim.step(
      test_maps::make_trajectory_input(
        route, ego, sec, nsec, trajectory, {ego}, TurnIndicatorsReport::DISABLE, objects));
    if (state == DS::INTENTIONAL_LANE_CROSSING) {
      return true;
    }
    time_ms += 100;
  }
  return false;
}
}  // namespace

// Onset then completion: ego in 47 going straight on-route (47 -> 1167), an object in 47 ahead, the
// trajectory pokes out toward off-route lane 48 and back around it -> INTENTIONAL_LANE_CROSSING;
// the footprint returning fully inside 47 completes the crossing.
TEST(LaneCrossingTest, onset_then_return_completes)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map)) << "failed to load " << TEST_MAP_PATH;

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_in_47 = crossing_trajectory.front();
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  ASSERT_TRUE(run_until_crossing(sim, route, crossing_trajectory, ego_in_47, objects, 10, time_ms));
  EXPECT_EQ(sim.reference_lane_id(), 47);  // reference held to the origin lane at onset

  // Return: the footprint sits fully back inside the reference lane 47.
  const auto point_in_47 = point_at_fraction(centerline_points(map, 47), 0.6);
  const auto footprint_47 = footprint_box(point_in_47);
  const auto return_trajectory = build_lane_trajectory(map, {47}, 0.5, 0.95, 20);

  bool completed = false;
  uint8_t state = DS::INTENTIONAL_LANE_CROSSING;
  for (int cycle = 0; cycle < 12; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, point_in_47, sec, nsec, return_trajectory, footprint_47,
        TurnIndicatorsReport::DISABLE, objects));
    if (state == DS::UNKNOWN) {
      completed = true;
      break;
    }
  }
  EXPECT_TRUE(completed) << "footprint back inside 47 for the settle window should complete";
}

// No object: the identical out-and-back poke with no obstacle is never a crossing (a plain drift or
// a chatter poke, with nothing to go around, is not an intentional crossing).
TEST(LaneCrossingTest, no_object_is_not_a_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_in_47 = crossing_trajectory.front();

  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 15; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const uint8_t state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, crossing_trajectory, {ego_in_47}));
    EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING) << "no obstacle, so no intentional crossing";
    time_ms += 100;
  }
}

// A monotonic sweep into the neighbour commits to it and never returns within the look-ahead: that
// is a lane change, not a crossing, so no closed departure forms and onset never fires even with an
// object present. The full-entry footprint escape is a backstop; this rejects it earlier, at the
// trajectory geometry.
TEST(LaneCrossingTest, monotonic_sweep_into_neighbour_is_not_a_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto sweep_trajectory = build_lane_trajectory(map, {47, 48}, 0.3, 0.9, 20);
  const auto ego_in_47 = sweep_trajectory.front();
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 15; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const uint8_t state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, sweep_trajectory, {ego_in_47}, TurnIndicatorsReport::DISABLE,
        objects));
    EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING)
      << "a path that commits to the neighbour is a lane change, not a crossing";
    time_ms += 100;
  }
}

// A moving object qualifies: the object need not be static. The ego pokes out and back around a
// slow-moving obstacle, so the closed departure brackets it -> INTENTIONAL_LANE_CROSSING. (A moving
// object the ego merely follows straight produces no departure, so it cannot onset on its own.)
TEST(LaneCrossingTest, moving_object_can_qualify)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_in_47 = crossing_trajectory.front();
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects = test_maps::make_objects(
    {test_maps::make_object(object_point.x(), object_point.y(), 2.0, 2.0, 5.0)});  // 5 m/s

  int64_t time_ms = 0;
  EXPECT_TRUE(run_until_crossing(sim, route, crossing_trajectory, ego_in_47, objects, 10, time_ms))
    << "a moving object the ego dodges around still qualifies";
}

// Two staggered objects, only one at the poke: the closed departure need only bracket one candidate
// (note: objects need not line up). A poke around the first object onsets even though the second is
// elsewhere in the sequence.
TEST(LaneCrossingTest, staggered_objects_one_bracketed_is_a_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_in_47 = crossing_trajectory.front();
  const auto centerline_47 = centerline_points(map, 47);
  const auto near_object = point_at_fraction(centerline_47, 0.5);  // at the poke
  const auto far_object = point_at_fraction(centerline_47, 0.95);  // beyond the re-enter
  const auto objects = test_maps::make_objects(
    {test_maps::make_object(near_object.x(), near_object.y()),
     test_maps::make_object(far_object.x(), far_object.y())});

  int64_t time_ms = 0;
  EXPECT_TRUE(run_until_crossing(sim, route, crossing_trajectory, ego_in_47, objects, 10, time_ms))
    << "bracketing one of several staggered objects is enough";
}

// Full-entry escape: once crossing, a footprint fully inside off-route lane 48 means the move is a
// lane change, so the crossing classifier drops out (leaving the lane-change classifier to own it).
TEST(LaneCrossingTest, full_entry_into_adjacent_lane_escapes)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_in_47 = crossing_trajectory.front();
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  ASSERT_TRUE(run_until_crossing(sim, route, crossing_trajectory, ego_in_47, objects, 10, time_ms));

  // Footprint fully inside off-route lane 48, ego now past the object (object behind): with nothing
  // left to cross for, a full crossover is a lane change, not a crossing.
  const auto point_in_48 = point_at_fraction(centerline_points(map, 48), 0.8);
  const auto footprint_48 = footprint_box(point_in_48);
  const auto through_48_trajectory = build_lane_trajectory(map, {48}, 0.7, 0.95, 20);
  time_ms += 100;
  const auto [sec, nsec] = stamp_from_ms(time_ms);
  const uint8_t state = sim.step(
    test_maps::make_trajectory_input(
      route, point_in_48, sec, nsec, through_48_trajectory, footprint_48,
      TurnIndicatorsReport::DISABLE, objects));
  EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING)
    << "full entry into 48 past the object is a lane change, not a crossing";
}

// Candidate-object memory: the object is perceived while the ego approaches (latching the memory),
// then perception drops it entirely exactly as the ego dodges past. The remembered candidate must
// still let the bracket onset fire (mirrors the real perception dropout at the crossing moment).
TEST(LaneCrossingTest, candidate_memory_bridges_perception_dropout)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  const auto straight_trajectory = build_lane_trajectory(map, {47}, 0.3, 0.6, 20);
  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_in_47 = straight_trajectory.front();

  int64_t time_ms = 0;
  // Phase 1: going straight, the object is perceived and latches the candidate memory. No crossing.
  for (int cycle = 0; cycle < 4; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, {ego_in_47},
        TurnIndicatorsReport::DISABLE, objects));
    time_ms += 100;
  }
  // Phase 2: the ego dodges while perception drops the object entirely. The remembered candidate
  // must still let onset fire.
  bool became_crossing = false;
  for (int cycle = 0; cycle < 6; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const uint8_t state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, crossing_trajectory, {ego_in_47},
        TurnIndicatorsReport::DISABLE, nullptr));
    if (state == DS::INTENTIONAL_LANE_CROSSING) {
      became_crossing = true;
      break;
    }
    time_ms += 100;
  }
  EXPECT_TRUE(became_crossing)
    << "a candidate seen a moment ago should bridge a transient perception dropout";
}

// Physical (footprint) source: the shallow-dodge case. The planned trajectory stays straight
// in-lane (no centerline departure at all), but the ego body pokes well past the 47|48 boundary
// while an object is ahead in 47. The physical footprint source must onset even though the
// trajectory bracket finds nothing - this is the field case the centerline-only detector missed.
TEST(LaneCrossingTest, footprint_over_boundary_is_a_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto straight_trajectory = build_lane_trajectory(map, {47}, 0.3, 0.9, 20);
  const auto ego_in_47 = straight_trajectory.front();
  const auto footprint = build_straddle_footprint(map, 0.6, 1.0);  // 1.0 m into 48, > 0.5 threshold
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  uint8_t state = DS::UNKNOWN;
  int64_t time_ms = 0;
  bool became_crossing = false;
  for (int cycle = 0; cycle < 10; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, footprint, TurnIndicatorsReport::DISABLE,
        objects));
    if (state == DS::INTENTIONAL_LANE_CROSSING) {
      became_crossing = true;
      break;
    }
    time_ms += 100;
  }
  EXPECT_TRUE(became_crossing)
    << "the body poking past the boundary is a crossing even on an in-lane path";
}

// Cornering guard: a body that only slightly overhangs the boundary (below
// footprint_boundary_overshoot_m) with a straight in-lane path is a cornering graze, not a
// crossing.
TEST(LaneCrossingTest, slight_cornering_overhang_is_not_a_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto straight_trajectory = build_lane_trajectory(map, {47}, 0.3, 0.9, 20);
  const auto ego_in_47 = straight_trajectory.front();
  const auto footprint = build_straddle_footprint(map, 0.6, 0.1);  // 0.1 m into 48, < 0.5 threshold
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 15; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const uint8_t state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, footprint, TurnIndicatorsReport::DISABLE,
        objects));
    EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING)
      << "a slight cornering overhang is not a crossing";
    time_ms += 100;
  }
}

// No time cap: a slow, long dodge (the body sits over the boundary far longer than the old 10 s
// backstop) must stay INTENTIONAL_LANE_CROSSING until the ego actually returns, not force-complete
// mid-excursion. Force-completing mid-dodge is what let the reference migrate to the off-route
// neighbour and read the return as a lane change.
TEST(LaneCrossingTest, long_dodge_does_not_time_out)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto straight_trajectory = build_lane_trajectory(map, {47}, 0.3, 0.9, 20);
  const auto ego_in_47 = straight_trajectory.front();
  const auto straddle_footprint =
    build_straddle_footprint(map, 0.6, 1.0);  // body over the boundary
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  uint8_t state = DS::UNKNOWN;
  int64_t time_ms = 0;
  bool became_crossing = false;
  for (int cycle = 0; cycle < 10; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, straddle_footprint,
        TurnIndicatorsReport::DISABLE, objects));
    if (state == DS::INTENTIONAL_LANE_CROSSING) {
      became_crossing = true;
      break;
    }
    time_ms += 100;
  }
  ASSERT_TRUE(became_crossing);

  // Hold the body over the boundary for 20 s - well past the old 10 s backstop. It must stay a
  // crossing the whole time (no time-based completion).
  for (int cycle = 0; cycle < 200; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, straddle_footprint,
        TurnIndicatorsReport::DISABLE, objects));
    ASSERT_EQ(state, DS::INTENTIONAL_LANE_CROSSING)
      << "a long dodge must not time out at cycle " << cycle;
  }

  // The ego finally returns fully inside 47: now it completes.
  const auto point_in_47 = point_at_fraction(centerline_points(map, 47), 0.6);
  const auto footprint_47 = footprint_box(point_in_47);
  const auto return_trajectory = build_lane_trajectory(map, {47}, 0.5, 0.95, 20);
  bool completed = false;
  for (int cycle = 0; cycle < 12; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, point_in_47, sec, nsec, return_trajectory, footprint_47,
        TurnIndicatorsReport::DISABLE, objects));
    if (state == DS::UNKNOWN) {
      completed = true;
      break;
    }
  }
  EXPECT_TRUE(completed) << "returning fully inside 47 completes the crossing";
}

// Ending requires the ego to be fully inside one lane. A straddling ego (not fully in the route,
// not fully in a neighbour) holds the crossing even after the object is gone - it does NOT end mid-
// straddle. The ego is expected to settle fully into a lane (by manual override or reposition).
TEST(LaneCrossingTest, object_gone_while_straddling_stays_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto straight_trajectory = build_lane_trajectory(map, {47}, 0.3, 0.9, 20);
  const auto ego_in_47 = straight_trajectory.front();
  const auto straddle_footprint = build_straddle_footprint(map, 0.6, 1.0);
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  uint8_t state = DS::UNKNOWN;
  int64_t time_ms = 0;
  bool became_crossing = false;
  for (int cycle = 0; cycle < 10; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, straddle_footprint,
        TurnIndicatorsReport::DISABLE, objects));
    if (state == DS::INTENTIONAL_LANE_CROSSING) {
      became_crossing = true;
      break;
    }
    time_ms += 100;
  }
  ASSERT_TRUE(became_crossing);

  // The object is gone but the ego keeps straddling (never fully in 47, never fully in 48). The
  // crossing must hold - it is not fully inside any lane, so there is nothing to end into.
  for (int cycle = 0; cycle < 60; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, straddle_footprint,
        TurnIndicatorsReport::DISABLE, nullptr));
    ASSERT_EQ(state, DS::INTENTIONAL_LANE_CROSSING)
      << "a straddling ego does not end the crossing at cycle " << cycle;
  }
}

// Full entry ends the crossing even with an object still ahead: once the whole footprint is fully
// inside the neighbour lane 48, the ego has committed, so the crossing ends and the move is handed
// to the lane-change classifier - regardless of any object still ahead in the route lane.
TEST(LaneCrossingTest, full_crossover_ends_even_with_object_ahead)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_in_47 = crossing_trajectory.front();
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.6);  // stays ahead
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  ASSERT_TRUE(run_until_crossing(sim, route, crossing_trajectory, ego_in_47, objects, 10, time_ms));

  // Whole footprint fully inside 48 at 0.35, with the object still ahead at 0.6 in the route lane.
  const auto point_in_48 = point_at_fraction(centerline_points(map, 48), 0.35);
  const auto footprint_48 = footprint_box(point_in_48);
  const auto through_48_trajectory = build_lane_trajectory(map, {48}, 0.3, 0.8, 20);
  time_ms += 100;
  const auto [sec, nsec] = stamp_from_ms(time_ms);
  const uint8_t state = sim.step(
    test_maps::make_trajectory_input(
      route, point_in_48, sec, nsec, through_48_trajectory, footprint_48,
      TurnIndicatorsReport::DISABLE, objects));
  EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING)
    << "a full crossover into the neighbour ends the crossing (now a lane change)";
}

// Mutual exclusion with lane change: from off-route lane 48 a move toward route primitive 47 is a
// lane change, not a crossing. The on-route-straight condition (reference must be an on-route
// primitive with an on-route straight successor) fails for 48, so no crossing fires even with an
// object present.
TEST(LaneCrossingTest, lane_change_regime_is_not_a_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 48, 47, 0.3, 0.9, 25);
  const auto ego_in_48 = crossing_trajectory.front();
  const auto object_point = point_at_fraction(centerline_points(map, 48), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 15; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const uint8_t state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_48, sec, nsec, crossing_trajectory, {ego_in_48},
        TurnIndicatorsReport::DISABLE, objects));
    EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING)
      << "crossing from an off-route lane is a lane change";
    time_ms += 100;
  }
}

}  // namespace autoware::lane_event_classifier
