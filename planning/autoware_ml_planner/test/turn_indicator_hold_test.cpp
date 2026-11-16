// Copyright 2025 TIER IV, Inc.
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


#include "autoware/ml_planner/postprocessing/postprocessing_utils.hpp"

#include <rclcpp/time.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <gtest/gtest.h>

namespace autoware::ml_planner::test
{

using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

constexpr uint8_t OFF = TurnIndicatorsCommand::DISABLE;
constexpr uint8_t LEFT = TurnIndicatorsCommand::ENABLE_LEFT;
constexpr uint8_t RIGHT = TurnIndicatorsCommand::ENABLE_RIGHT;
constexpr double HOLD_SEC = 3.0;

rclcpp::Time at(const double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

TEST(TurnIndicatorHoldTest, OffStaysOffWithoutHold)
{
  postprocess::TurnIndicatorHold hold;
  EXPECT_EQ(hold.apply(OFF, at(0.0), HOLD_SEC), OFF);
  EXPECT_EQ(hold.apply(OFF, at(0.1), HOLD_SEC), OFF);
}

TEST(TurnIndicatorHoldTest, OffIsHeldUntilTheDurationFromTurningOn)
{
  postprocess::TurnIndicatorHold hold;
  EXPECT_EQ(hold.apply(RIGHT, at(10.0), HOLD_SEC), RIGHT);
  EXPECT_EQ(hold.apply(OFF, at(10.1), HOLD_SEC), RIGHT);
  EXPECT_EQ(hold.apply(OFF, at(12.9), HOLD_SEC), RIGHT);
  EXPECT_EQ(hold.apply(OFF, at(13.0), HOLD_SEC), OFF);
  EXPECT_EQ(hold.apply(OFF, at(13.1), HOLD_SEC), OFF);
}

TEST(TurnIndicatorHoldTest, StayingOnDoesNotExtendTheHold)
{
  postprocess::TurnIndicatorHold hold;
  EXPECT_EQ(hold.apply(LEFT, at(0.0), HOLD_SEC), LEFT);
  EXPECT_EQ(hold.apply(LEFT, at(5.0), HOLD_SEC), LEFT);
  // Turned on at 0 s: off is let through after 3 s.
  EXPECT_EQ(hold.apply(OFF, at(5.1), HOLD_SEC), OFF);
}

TEST(TurnIndicatorHoldTest, SwitchingSidesIsAllowedAndStartsANewHold)
{
  postprocess::TurnIndicatorHold hold;
  EXPECT_EQ(hold.apply(LEFT, at(0.0), HOLD_SEC), LEFT);
  EXPECT_EQ(hold.apply(RIGHT, at(1.0), HOLD_SEC), RIGHT);
  EXPECT_EQ(hold.apply(OFF, at(3.5), HOLD_SEC), RIGHT);
  EXPECT_EQ(hold.apply(OFF, at(4.0), HOLD_SEC), OFF);
}

TEST(TurnIndicatorHoldTest, OnAgainAfterTheHoldStartsANewHold)
{
  postprocess::TurnIndicatorHold hold;
  EXPECT_EQ(hold.apply(RIGHT, at(0.0), HOLD_SEC), RIGHT);
  EXPECT_EQ(hold.apply(OFF, at(4.0), HOLD_SEC), OFF);
  EXPECT_EQ(hold.apply(RIGHT, at(4.1), HOLD_SEC), RIGHT);
  EXPECT_EQ(hold.apply(OFF, at(6.0), HOLD_SEC), RIGHT);
}

TEST(TurnIndicatorHoldTest, ZeroDurationHoldsNothing)
{
  postprocess::TurnIndicatorHold hold;
  EXPECT_EQ(hold.apply(RIGHT, at(0.0), 0.0), RIGHT);
  EXPECT_EQ(hold.apply(OFF, at(0.1), 0.0), OFF);
}

TEST(TurnIndicatorHoldTest, TimeGoingBackEndsTheHold)
{
  postprocess::TurnIndicatorHold hold;
  EXPECT_EQ(hold.apply(RIGHT, at(10.0), HOLD_SEC), RIGHT);
  EXPECT_EQ(hold.apply(OFF, at(5.0), HOLD_SEC), OFF);
}

}  // namespace autoware::ml_planner::test
