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

#include "autoware/ml_planner/preprocessing/items/map.hpp"

#include <gtest/gtest.h>

namespace autoware::ml_planner::preprocess
{
namespace
{
using Element = autoware_perception_msgs::msg::TrafficLightElement;

Element light(uint8_t color, uint8_t shape, float confidence = 0.8F)
{
  Element result;
  result.color = color;
  result.shape = shape;
  result.status = Element::SOLID_ON;
  result.confidence = confidence;
  return result;
}

TEST(TrafficLightSelection, CombinedArrowsOverrideCircleOnlyForIncludedDirections)
{
  for (const auto shape : {Element::UP_LEFT_ARROW, Element::UP_RIGHT_ARROW}) {
    const auto turn = shape == Element::UP_LEFT_ARROW ? LaneSegment::TURN_DIRECTION_LEFT
                                                      : LaneSegment::TURN_DIRECTION_RIGHT;
    const auto opposite = shape == Element::UP_LEFT_ARROW ? LaneSegment::TURN_DIRECTION_RIGHT
                                                          : LaneSegment::TURN_DIRECTION_LEFT;
    const std::vector<Element> elements{
      light(Element::RED, Element::CIRCLE, 1.0F), light(Element::GREEN, shape, 0.5F)};
    for (const auto direction : {LaneSegment::TURN_DIRECTION_STRAIGHT, turn}) {
      const auto result = identify_current_light_status(direction, elements);
      EXPECT_EQ(result.color, Element::GREEN);
      EXPECT_TRUE(result.is_arrow);
    }
    const auto result = identify_current_light_status(opposite, elements);
    EXPECT_EQ(result.color, Element::RED);
    EXPECT_FALSE(result.is_arrow);
  }
}

TEST(TrafficLightSelection, CombinedArrowsAreMarkedWhenSelectedAlone)
{
  for (const auto shape : {Element::UP_LEFT_ARROW, Element::UP_RIGHT_ARROW}) {
    const auto result = identify_current_light_status(
      LaneSegment::TURN_DIRECTION_STRAIGHT, {light(Element::GREEN, shape)});
    EXPECT_EQ(result.color, Element::GREEN);
    EXPECT_TRUE(result.is_arrow);
  }
}

TEST(TrafficLightSelection, MatchingArrowsStillUseConfidenceAndStableTies)
{
  const auto combined = light(Element::GREEN, Element::UP_LEFT_ARROW, 0.9F);
  const auto single = light(Element::RED, Element::LEFT_ARROW, 0.5F);
  EXPECT_EQ(
    identify_current_light_status(LaneSegment::TURN_DIRECTION_LEFT, {single, combined}).color,
    Element::GREEN);
  auto stronger = single;
  stronger.confidence = 1.0F;
  EXPECT_EQ(
    identify_current_light_status(LaneSegment::TURN_DIRECTION_LEFT, {combined, stronger}).color,
    Element::RED);
  stronger.confidence = combined.confidence;
  EXPECT_EQ(
    identify_current_light_status(LaneSegment::TURN_DIRECTION_LEFT, {stronger, combined}).color,
    Element::RED);
}

TEST(TrafficLightSelection, ExistingSingleElementAndStatusRulesAreUnchanged)
{
  auto arrow = light(Element::GREEN, Element::RIGHT_ARROW);
  arrow.status = Element::SOLID_OFF;
  const auto result = identify_current_light_status(LaneSegment::TURN_DIRECTION_LEFT, {arrow});
  EXPECT_EQ(result.color, Element::GREEN);
  EXPECT_TRUE(result.is_arrow);
  EXPECT_EQ(
    identify_current_light_status(
      LaneSegment::TURN_DIRECTION_LEFT, {light(Element::UNKNOWN, Element::LEFT_ARROW)})
      .color,
    Element::UNKNOWN);
}
}  // namespace
}  // namespace autoware::ml_planner::preprocess
