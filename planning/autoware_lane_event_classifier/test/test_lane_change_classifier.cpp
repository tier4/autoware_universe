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

// Trajectory-driven lane-change tests (see docs/lane_change.md) on the real test map
// (test/map/lanelet2_map.osm). The primitive sequence 47 -> 1167 -> 51 -> 55 runs in the left lane
// of the three-lane bundle 47|48|50 (left->right). Trajectories are built from the map's
// centerlines so the tests carry no hard-coded coordinates; the ego is driven through cycles by
// advancing the message stamp.

#include "synthetic_lanelet_maps.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lane_event_classifier/lane_change/classifier.hpp>
#include <autoware_lane_event_classifier/lane_tracker.hpp>
#include <autoware_lane_event_classifier/msg/driving_state.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <array>
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
// (in order) while advancing forward along the first lane. The forward extent is [start, end] of
// that lane's centerline arc.
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

// Drives one cycle exactly as the node does: update the tracker, run the classifier, then hold or
// release the reference lane based on whether an event is active.
class Simulator
{
public:
  Simulator(lanelet::LaneletMapPtr map, LaneChangeConfig config)
  : classifier_{true, config, tracker_}
  {
    const auto result = tracker_.set_lanelet_map(map);
    EXPECT_TRUE(result.has_value());
  }

  uint8_t step(const LaneEventInput & input)
  {
    tracker_.update(input);
    classifier_.update(input);
    const uint8_t state = classifier_.get_state();
    const bool is_active = state == DS::LANE_CHANGING || state == DS::ABORTING_LANE_CHANGE;
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
  LaneChangeClassifier classifier_;
};

LaneChangeConfig make_config()
{
  LaneChangeConfig config;
  config.enable_classifier = true;
  config.crossing_persist_duration_s = 0.3;
  config.settle_confirm_duration_s = 0.5;
  config.crossing_look_ahead_m = 50.0;
  config.crossing_position_tolerance_m = 2.0;
  config.confidence_factor = 0.5;
  return config;
}

// Splits a running millisecond count into a (sec, nsec) stamp.
std::pair<int32_t, uint32_t> stamp_from_ms(int64_t total_ms)
{
  return {
    static_cast<int32_t>(total_ms / 1000), static_cast<uint32_t>((total_ms % 1000) * 1'000'000)};
}
}  // namespace

// Single left change 48 -> 47: the trajectory crosses toward route primitive 47; onset confirms,
// then the footprint settling fully inside 47 completes the change.
TEST(LaneChangeTest, single_left_change_onset_then_settle)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map)) << "failed to load " << TEST_MAP_PATH;

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_lane_trajectory(map, {48, 47}, 0.3, 0.9, 20);
  const auto ego_in_48 = crossing_trajectory.front();

  // Onset: hold the crossing until LANE_CHANGING is confirmed.
  uint8_t state = DS::UNKNOWN;
  int64_t time_ms = 0;
  bool became_changing = false;
  for (int cycle = 0; cycle < 10; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_48, sec, nsec, crossing_trajectory, {ego_in_48}));
    if (state == DS::LANE_CHANGING) {
      became_changing = true;
      break;
    }
    time_ms += 100;
  }
  ASSERT_TRUE(became_changing);
  EXPECT_EQ(sim.reference_lane_id(), 48);  // reference held to the origin lane at onset

  // Settle: the footprint sits fully inside route primitive 47.
  const auto point_in_47 = point_at_fraction(centerline_points(map, 47), 0.6);
  const auto footprint_47 = footprint_box(point_in_47);
  const auto settle_trajectory = build_lane_trajectory(map, {47}, 0.5, 0.95, 20);

  bool settled = false;
  for (int cycle = 0; cycle < 12; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, point_in_47, sec, nsec, settle_trajectory, footprint_47));
    if (state == DS::UNKNOWN) {
      settled = true;
      break;
    }
  }
  EXPECT_TRUE(settled)
    << "footprint fully inside 47 for the settle window should complete the change";
}

// Double left change 50 -> 48 -> 47: recognized from the start because the trajectory reaches route
// primitive 47 within the look-ahead; stays LANE_CHANGING through the non-primitive intermediate
// lane 48 and settles only at 47.
TEST(LaneChangeTest, double_left_change_settles_only_at_route_primitive)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_lane_trajectory(map, {50, 48, 47}, 0.3, 0.9, 30);
  const auto ego_in_50 = crossing_trajectory.front();

  uint8_t state = DS::UNKNOWN;
  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 10 && state != DS::LANE_CHANGING; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_50, sec, nsec, crossing_trajectory, {ego_in_50}));
    time_ms += 100;
  }
  ASSERT_EQ(state, DS::LANE_CHANGING);

  // Footprint fully inside the intermediate off-route lane 48 must NOT settle.
  const auto point_in_48 = point_at_fraction(centerline_points(map, 48), 0.6);
  const auto footprint_48 = footprint_box(point_in_48);
  const auto through_48_trajectory = build_lane_trajectory(map, {48, 47}, 0.5, 0.95, 20);
  for (int cycle = 0; cycle < 8; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, point_in_48, sec, nsec, through_48_trajectory, footprint_48));
    EXPECT_EQ(state, DS::LANE_CHANGING) << "must not settle in intermediate lane 48";
  }

  // Footprint fully inside route primitive 47 settles.
  const auto point_in_47 = point_at_fraction(centerline_points(map, 47), 0.6);
  const auto footprint_47 = footprint_box(point_in_47);
  const auto settle_trajectory = build_lane_trajectory(map, {47}, 0.5, 0.95, 20);
  bool settled = false;
  for (int cycle = 0; cycle < 12; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, point_in_47, sec, nsec, settle_trajectory, footprint_47));
    if (state == DS::UNKNOWN) {
      settled = true;
      break;
    }
  }
  EXPECT_TRUE(settled);
}

// A crossing from an on-route primitive (47) toward an off-route lane (48) is never a lane change:
// going straight (47 -> 1167) already keeps the ego on-route (the straight-on-route skip case).
TEST(LaneChangeTest, on_route_primitive_crossing_off_route_is_not_lane_change)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_lane_trajectory(map, {47, 48}, 0.3, 0.9, 20);
  const auto ego_in_47 = crossing_trajectory.front();

  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 15; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const uint8_t state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, crossing_trajectory, {ego_in_47}));
    EXPECT_NE(state, DS::LANE_CHANGING) << "crossing off-route from an on-route primitive";
    time_ms += 100;
  }
}

// Abort: after onset, the trajectory swings back into the reference lane (48) -> ABORTING; the
// footprint fully back inside 48 completes the abort (geometric, no dwell).
TEST(LaneChangeTest, abort_when_trajectory_returns_then_geometric_completion)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_lane_trajectory(map, {48, 47}, 0.3, 0.9, 20);
  const auto ego_in_48 = crossing_trajectory.front();

  uint8_t state = DS::UNKNOWN;
  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 10 && state != DS::LANE_CHANGING; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_48, sec, nsec, crossing_trajectory, {ego_in_48}));
    time_ms += 100;
  }
  ASSERT_EQ(state, DS::LANE_CHANGING);

  // Trajectory now stays inside the reference lane 48 (planner gives up).
  const auto return_trajectory = build_lane_trajectory(map, {48}, 0.3, 0.9, 20);
  bool aborting = false;
  for (int cycle = 0; cycle < 10; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_48, sec, nsec, return_trajectory, {ego_in_48}));
    if (state == DS::ABORTING_LANE_CHANGE) {
      aborting = true;
      break;
    }
  }
  ASSERT_TRUE(aborting);

  // Footprint fully back inside the reference lane 48 completes the abort.
  const auto point_in_48 = point_at_fraction(centerline_points(map, 48), 0.6);
  const auto footprint_48 = footprint_box(point_in_48);
  time_ms += 100;
  const auto [sec, nsec] = stamp_from_ms(time_ms);
  state = sim.step(
    test_maps::make_trajectory_input(
      route, point_in_48, sec, nsec, return_trajectory, footprint_48));
  EXPECT_EQ(state, DS::UNKNOWN) << "abort completes geometrically with no dwell";
}

// From ABORTING, the trajectory swinging back toward the target lane returns directly to
// LANE_CHANGING (no trip through lane following).
TEST(LaneChangeTest, aborting_recommits_to_changing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_lane_trajectory(map, {48, 47}, 0.3, 0.9, 20);
  const auto return_trajectory = build_lane_trajectory(map, {48}, 0.3, 0.9, 20);
  const auto ego_in_48 = crossing_trajectory.front();

  uint8_t state = DS::UNKNOWN;
  int64_t time_ms = 0;
  auto run = [&](const std::vector<lanelet::BasicPoint2d> & trajectory, int cycles, uint8_t until) {
    for (int cycle = 0; cycle < cycles; ++cycle) {
      const auto [sec, nsec] = stamp_from_ms(time_ms);
      state = sim.step(
        test_maps::make_trajectory_input(route, ego_in_48, sec, nsec, trajectory, {ego_in_48}));
      time_ms += 100;
      if (state == until) {
        return true;
      }
    }
    return false;
  };

  ASSERT_TRUE(run(crossing_trajectory, 10, DS::LANE_CHANGING));
  ASSERT_TRUE(run(return_trajectory, 10, DS::ABORTING_LANE_CHANGE));
  EXPECT_TRUE(run(crossing_trajectory, 10, DS::LANE_CHANGING)) << "re-commit from ABORTING";
}

// Blinker confidence signal: with the footprint straddling 48/47 (so the footprint-off-route signal
// is inactive), the blinker toward the target shortens the onset window versus no blinker.
TEST(LaneChangeTest, blinker_confidence_signal_shortens_onset_window)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};
  const auto crossing_trajectory = build_lane_trajectory(map, {48, 47}, 0.3, 0.9, 20);
  const auto ego_in_48 = crossing_trajectory.front();

  // Footprint with one corner in 48 and one in 47 -> not fully off the route primitives, so the
  // footprint-off-route signal is inactive and the blinker is the only confidence signal.
  const auto point_in_48 = point_at_fraction(centerline_points(map, 48), 0.4);
  const auto point_in_47 = nearest_point_on(centerline_points(map, 47), point_in_48);
  const std::vector<lanelet::BasicPoint2d> straddling_footprint{point_in_48, point_in_47};

  auto onset_cycle = [&](uint8_t turn_indicator) {
    Simulator sim{map, make_config()};
    int64_t time_ms = 0;
    for (int cycle = 0; cycle < 20; ++cycle) {
      const auto [sec, nsec] = stamp_from_ms(time_ms);
      const uint8_t state = sim.step(
        test_maps::make_trajectory_input(
          route, ego_in_48, sec, nsec, crossing_trajectory, straddling_footprint, turn_indicator));
      if (state == DS::LANE_CHANGING) {
        return cycle;
      }
      time_ms += 100;
    }
    return 999;
  };

  const int cycles_no_blinker = onset_cycle(TurnIndicatorsReport::DISABLE);
  const int cycles_with_blinker = onset_cycle(TurnIndicatorsReport::ENABLE_LEFT);
  EXPECT_LT(cycles_with_blinker, cycles_no_blinker)
    << "blinker toward the target should shorten the onset window";
}

}  // namespace autoware::lane_event_classifier
