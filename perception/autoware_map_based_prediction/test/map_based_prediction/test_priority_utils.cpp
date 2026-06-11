// Copyright 2026 TIER IV, inc.
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

#include "map_based_prediction/lanelet_util.hpp"
#include "map_based_prediction/priority_utils.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/LaneletPath.h>

#include <optional>
#include <string>
#include <vector>

namespace autoware::map_based_prediction::priority
{
namespace
{
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;

lanelet::Point3d makePoint(const lanelet::Id id, const double x, const double y)
{
  return lanelet::Point3d(id, x, y, 0.0);
}

// Build a trivial straight lanelet [ (x,0)-(x,1) wide, length 1 ] with a running id.
lanelet::Lanelet makeLanelet(lanelet::Id & next_id, const double x_offset)
{
  const lanelet::LineString3d left(
    next_id++, {makePoint(next_id++, x_offset, 1.0), makePoint(next_id++, x_offset + 1.0, 1.0)});
  const lanelet::LineString3d right(
    next_id++, {makePoint(next_id++, x_offset, 0.0), makePoint(next_id++, x_offset + 1.0, 0.0)});
  return lanelet::Lanelet(next_id++, left, right);
}

TrafficLightElement makeElement(const uint8_t color, const uint8_t shape)
{
  TrafficLightElement element;
  element.color = color;
  element.shape = shape;
  return element;
}

// A vertical stop line at x == x_pos, spanning y in [-1, 1].
lanelet::LineString3d makeStopLine(lanelet::Id & next_id, const double x_pos)
{
  return lanelet::LineString3d(
    next_id++, {makePoint(next_id++, x_pos, -1.0), makePoint(next_id++, x_pos, 1.0)});
}

// Attach a TrafficLight regulatory element (with an optional stop line) to a lanelet.
void attachTrafficLight(
  lanelet::Lanelet & lanelet, lanelet::Id & next_id,
  const std::optional<lanelet::LineString3d> & stop_line)
{
  const lanelet::LineString3d bulb(
    next_id++, {makePoint(next_id++, 0.0, 2.0), makePoint(next_id++, 0.5, 2.0)});
  const auto traffic_light = stop_line
                               ? lanelet::TrafficLight::make(next_id++, {}, {bulb}, *stop_line)
                               : lanelet::TrafficLight::make(next_id++, {}, {bulb});
  lanelet.addRegulatoryElement(traffic_light);
}

// A straight reference pose path along +x, [0, length] at 1 m spacing.
PosePath makeRefPath(const double length)
{
  PosePath ref_path;
  for (double x = 0.0; x <= length + 1e-6; x += 1.0) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = 0.0;
    ref_path.push_back(pose);
  }
  return ref_path;
}

// ---- findTrafficLightLaneletOnPath --------------------------------------------

TEST(PriorityUtils, FindsTrafficLightLaneletOnPath)
{
  lanelet::Id id = 4000;
  const auto approach = makeLanelet(id, 0.0);
  auto junction = makeLanelet(id, 5.0);
  const auto stop_line = makeStopLine(id, 6.0);
  attachTrafficLight(junction, id, stop_line);
  const lanelet::routing::LaneletPath path(lanelet::ConstLanelets{approach, junction});
  lanelet::ConstLanelet signal_lanelet;
  ASSERT_TRUE(findTrafficLightLaneletOnPath(path, signal_lanelet));
  EXPECT_EQ(signal_lanelet.id(), junction.id());
  const auto got = lanelet_util::getStopLineOrEntryEdge(signal_lanelet);
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->id(), stop_line.id());
}

TEST(PriorityUtils, NoTrafficLightLaneletOnPath)
{
  lanelet::Id id = 4100;
  const auto a = makeLanelet(id, 0.0);
  const auto b = makeLanelet(id, 5.0);
  const lanelet::routing::LaneletPath path(lanelet::ConstLanelets{a, b});
  lanelet::ConstLanelet signal_lanelet;
  EXPECT_FALSE(findTrafficLightLaneletOnPath(path, signal_lanelet));
}

TEST(PriorityUtils, FindsFirstTrafficLightLaneletOnPath)
{
  // With two signalized lanelets ahead, the first one entered wins.
  lanelet::Id id = 4200;
  const auto approach = makeLanelet(id, 0.0);
  auto first = makeLanelet(id, 5.0);
  auto second = makeLanelet(id, 10.0);
  attachTrafficLight(first, id, makeStopLine(id, 6.0));
  attachTrafficLight(second, id, makeStopLine(id, 11.0));
  const lanelet::routing::LaneletPath path(lanelet::ConstLanelets{approach, first, second});
  lanelet::ConstLanelet signal_lanelet;
  ASSERT_TRUE(findTrafficLightLaneletOnPath(path, signal_lanelet));
  EXPECT_EQ(signal_lanelet.id(), first.id());
}

TEST(PriorityUtils, GetStopLineOrEntryEdgeFallsBackToEntryEdge)
{
  // A signalized lanelet with no tagged stop line still gets a stop target: the
  // entry edge of its lane bounds.
  lanelet::Id id = 4300;
  auto junction = makeLanelet(id, 0.0);
  attachTrafficLight(junction, id, std::nullopt);
  EXPECT_TRUE(lanelet_util::getStopLineOrEntryEdge(junction).has_value());
}

// ---- arcLengthToStopLine (intersection-based) ------------------------------

TEST(PriorityUtils, ArcLengthToStopLineUsesIntersectionNotCentroid)
{
  // Oblique stop line from (10,0) to (12,4): it crosses the straight path (y=0) at
  // x=10, while its centroid (11,2) would mislead a nearest-vertex method to x=11.
  lanelet::Id id = 5000;
  const PosePath ref_path = makeRefPath(50.0);
  const lanelet::LineString3d stop_line(
    id++, {makePoint(id++, 10.0, 0.0), makePoint(id++, 12.0, 4.0)});
  const auto arc = arcLengthToStopLine(ref_path, stop_line);
  ASSERT_TRUE(arc.has_value());
  EXPECT_NEAR(*arc, 10.0, 0.2);
}

TEST(PriorityUtils, ArcLengthToStopLineFallsBackWhenNoCrossing)
{
  // A stop line beyond the path end never crosses it: fall back to the nearest
  // path vertex (the path end at 50 m).
  lanelet::Id id = 5100;
  const PosePath ref_path = makeRefPath(50.0);
  const lanelet::LineString3d stop_line(
    id++, {makePoint(id++, 60.0, -1.0), makePoint(id++, 60.0, 1.0)});
  const auto arc = arcLengthToStopLine(ref_path, stop_line);
  ASSERT_TRUE(arc.has_value());
  EXPECT_NEAR(*arc, 50.0, 0.6);
}

TEST(PriorityUtils, ArcLengthToStopLineStraightPath)
{
  lanelet::Id id = 5200;
  const auto ref_path = makeRefPath(20.0);
  const auto stop_line = makeStopLine(id, 10.0);
  const auto distance = arcLengthToStopLine(ref_path, stop_line);
  ASSERT_TRUE(distance.has_value());
  EXPECT_NEAR(*distance, 10.0, 1e-6);
}

TEST(PriorityUtils, ArcLengthToStopLineNoneForShortPath)
{
  lanelet::Id id = 5300;
  const PosePath ref_path;  // empty
  const auto stop_line = makeStopLine(id, 10.0);
  EXPECT_FALSE(arcLengthToStopLine(ref_path, stop_line).has_value());
}

TEST(PriorityUtils, ArcLengthToStopLineNoneWhenBehindPathStart)
{
  // A stop line behind the path start never crosses it and is nearest to the
  // FIRST vertex: it is not on this path, so no distance must be fabricated.
  lanelet::Id id = 5400;
  const PosePath ref_path = makeRefPath(50.0);
  const auto stop_line = makeStopLine(id, -10.0);
  EXPECT_FALSE(arcLengthToStopLine(ref_path, stop_line).has_value());
}

TEST(PriorityUtils, HasStopLineAheadByObjectPosition)
{
  // The same path / stop line answers differently depending on where the
  // object currently is: ahead of the line -> false, behind it -> true.
  lanelet::Id id = 5500;
  const PosePath ref_path = makeRefPath(50.0);
  const auto stop_line = makeStopLine(id, 10.0);

  geometry_msgs::msg::Point position;
  position.y = 0.0;

  position.x = 5.0;  // before the line
  EXPECT_TRUE(hasStopLineAhead(position, ref_path, stop_line));

  position.x = -5.0;  // behind the path start, still before the line
  EXPECT_TRUE(hasStopLineAhead(position, ref_path, stop_line));

  position.x = 15.0;  // already past the line
  EXPECT_FALSE(hasStopLineAhead(position, ref_path, stop_line));

  // A stop line that is not on the path at all is never "ahead".
  const auto behind_line = makeStopLine(id, -10.0);
  position.x = 5.0;
  EXPECT_FALSE(hasStopLineAhead(position, ref_path, behind_line));
}

// ---- evaluateSignalStopRequirement ------------------------------------------------

TEST(PriorityUtils, SignalStopNotRequiredWithoutObservation)
{
  lanelet::Id id = 5000;
  const auto lane = makeLanelet(id, 0.0);
  EXPECT_FALSE(evaluateSignalStopRequirement(lane, std::nullopt));
}

TEST(PriorityUtils, SignalStopNotRequiredOnGreenCircle)
{
  lanelet::Id id = 5100;
  const auto lane = makeLanelet(id, 0.0);
  TrafficLightGroup signal;
  signal.elements.push_back(makeElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE));
  EXPECT_FALSE(evaluateSignalStopRequirement(lane, signal));
}

TEST(PriorityUtils, SignalStopRequiredOnAmber)
{
  // Amber is treated as a stop, the same as red.
  lanelet::Id id = 5200;
  const auto lane = makeLanelet(id, 0.0);
  TrafficLightGroup signal;
  signal.elements.push_back(makeElement(TrafficLightElement::AMBER, TrafficLightElement::CIRCLE));
  EXPECT_TRUE(evaluateSignalStopRequirement(lane, signal));
}

TEST(PriorityUtils, SignalStopRequiredOnRed)
{
  lanelet::Id id = 5300;
  const auto lane = makeLanelet(id, 0.0);
  TrafficLightGroup signal;
  signal.elements.push_back(makeElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE));
  EXPECT_TRUE(evaluateSignalStopRequirement(lane, signal));
}

TEST(PriorityUtils, SignalStopNotRequiredForMatchingArrow)
{
  // A green UP arrow is a protected straight movement: a "straight" lane may go,
  // while a "right" lane (no matching arrow) must stop.
  TrafficLightGroup signal;
  signal.elements.push_back(makeElement(TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW));

  lanelet::Id id = 5500;
  auto straight_lane = makeLanelet(id, 0.0);
  straight_lane.setAttribute("turn_direction", "straight");
  EXPECT_FALSE(evaluateSignalStopRequirement(straight_lane, signal));

  auto right_lane = makeLanelet(id, 0.0);
  right_lane.setAttribute("turn_direction", "right");
  EXPECT_TRUE(evaluateSignalStopRequirement(right_lane, signal));
}

// ---- helpers ---------------------------------------------------------------

TEST(PriorityUtils, GetStopLineFromTrafficLight)
{
  lanelet::Id id = 6100;
  auto junction = makeLanelet(id, 0.0);
  const auto stop_line = makeStopLine(id, 10.0);
  attachTrafficLight(junction, id, stop_line);
  const auto got = lanelet_util::getStopLine(junction);
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->id(), stop_line.id());
}

TEST(PriorityUtils, GetStopLineNoneWhenUntagged)
{
  lanelet::Id id = 6200;
  const auto plain_lanelet = makeLanelet(id, 0.0);
  EXPECT_FALSE(lanelet_util::getStopLine(plain_lanelet).has_value());
}

// ---- calibrateStopDecision -----------------------------------------------------

TEST(PriorityUtils, ShouldAddStopHypothesisOnlyOnRedWithStopLineAhead)
{
  PriorityCalibrationParams params;
  EXPECT_TRUE(shouldAddStopHypothesis(true, true, params));
  EXPECT_FALSE(shouldAddStopHypothesis(false, true, params));  // signal does not demand a stop
  EXPECT_FALSE(shouldAddStopHypothesis(true, false, params));  // no stop line ahead
}

TEST(PriorityUtils, ShouldAddStopHypothesisRespectsDisableFlag)
{
  PriorityCalibrationParams params;
  params.use_signal_priority = false;
  EXPECT_FALSE(shouldAddStopHypothesis(true, true, params));
}

TEST(PriorityUtils, StopHypothesisConfidenceCenterIsStrongest)
{
  // Among stop hypotheses, the lane-follow (center) copy must be the strongest.
  const double weight = 0.35;
  const double center = weakenConfidenceInLaneChange(Maneuver::LANE_FOLLOW, weight);
  EXPECT_DOUBLE_EQ(center, weight);
  EXPECT_LT(weakenConfidenceInLaneChange(Maneuver::LEFT_LANE_CHANGE, weight), center);
  EXPECT_LT(weakenConfidenceInLaneChange(Maneuver::RIGHT_LANE_CHANGE, weight), center);
}

}  // namespace
}  // namespace autoware::map_based_prediction::priority
