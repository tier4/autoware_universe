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

#include "autoware/ml_planner/preprocessing/route_signal_release.hpp"

#include "autoware/ml_planner/preprocessing/items/map.hpp"

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace autoware::ml_planner::preprocess
{
namespace
{

using autoware_perception_msgs::msg::TrafficLightElement;

TrafficLightElement bulb(const uint8_t color, const uint8_t shape)
{
  TrafficLightElement element;
  element.color = color;
  element.shape = shape;
  element.status = TrafficLightElement::SOLID_ON;
  element.confidence = 1.0F;
  return element;
}

TEST(RouteSignalReleaseTest, FirstColorArmsTheMemoryWithoutReleasing)
{
  RouteSignalRelease release;
  EXPECT_FALSE(release.observe(true, TrafficLightElement::GREEN));
  EXPECT_FALSE(release.observe(true, TrafficLightElement::GREEN));
}

TEST(RouteSignalReleaseTest, ColorChangeInEitherDirectionReleasesOnce)
{
  RouteSignalRelease release;
  EXPECT_FALSE(release.observe(true, TrafficLightElement::GREEN));
  EXPECT_TRUE(release.observe(true, TrafficLightElement::AMBER));
  EXPECT_FALSE(release.observe(true, TrafficLightElement::AMBER));
  EXPECT_TRUE(release.observe(true, TrafficLightElement::RED));
  EXPECT_TRUE(release.observe(true, TrafficLightElement::GREEN));
  EXPECT_TRUE(release.observe(true, TrafficLightElement::WHITE));
}

TEST(RouteSignalReleaseTest, UnknownAndNoRouteLightDoNotEraseTheLastColor)
{
  RouteSignalRelease release;
  EXPECT_FALSE(release.observe(true, TrafficLightElement::GREEN));
  EXPECT_FALSE(release.observe(true, TrafficLightElement::UNKNOWN));
  EXPECT_FALSE(release.observe(false, TrafficLightElement::RED));
  EXPECT_FALSE(release.observe(true, TrafficLightElement::GREEN));
  EXPECT_TRUE(release.observe(true, TrafficLightElement::RED));
}

TEST(RouteSignalReleaseTest, StraightRouteIgnoresAnArrowThatDoesNotGovernIt)
{
  const std::vector<TrafficLightElement> green_circle_red_arrow = {
    bulb(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE),
    bulb(TrafficLightElement::RED, TrafficLightElement::LEFT_ARROW)};
  EXPECT_EQ(
    identify_current_light_status(LaneSegment::TURN_DIRECTION_STRAIGHT, green_circle_red_arrow)
      .color,
    TrafficLightElement::GREEN);

  const std::vector<TrafficLightElement> green_circle_green_arrow = {
    bulb(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE),
    bulb(TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW)};
  EXPECT_EQ(
    identify_current_light_status(LaneSegment::TURN_DIRECTION_STRAIGHT, green_circle_green_arrow)
      .color,
    TrafficLightElement::GREEN);

  const std::vector<TrafficLightElement> amber_circle_green_arrow = {
    bulb(TrafficLightElement::AMBER, TrafficLightElement::CIRCLE),
    bulb(TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW)};
  EXPECT_EQ(
    identify_current_light_status(LaneSegment::TURN_DIRECTION_STRAIGHT, amber_circle_green_arrow)
      .color,
    TrafficLightElement::AMBER);
}

TEST(RouteSignalReleaseTest, TurningRouteFollowsItsOwnArrow)
{
  const std::vector<TrafficLightElement> green_circle_red_arrow = {
    bulb(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE),
    bulb(TrafficLightElement::RED, TrafficLightElement::LEFT_ARROW)};
  EXPECT_EQ(
    identify_current_light_status(LaneSegment::TURN_DIRECTION_LEFT, green_circle_red_arrow).color,
    TrafficLightElement::RED);

  const std::vector<TrafficLightElement> green_circle_green_arrow = {
    bulb(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE),
    bulb(TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW)};
  EXPECT_EQ(
    identify_current_light_status(LaneSegment::TURN_DIRECTION_LEFT, green_circle_green_arrow).color,
    TrafficLightElement::GREEN);
}

}  // namespace
}  // namespace autoware::ml_planner::preprocess
