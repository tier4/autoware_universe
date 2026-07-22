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

#include "autoware/diffusion_planner/postprocessing/turn_indicator_manager.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::test
{
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using postprocess::TurnIndicatorManager;

namespace
{
std::vector<float> logits_for(const uint8_t command)
{
  std::vector<float> logits(TURN_INDICATOR_OUTPUT_DIM, -10.0F);
  logits.at(command) = 10.0F;
  return logits;
}

TEST(TurnIndicatorManagerTest, MapsDenseThreeClassOutputWithoutKeepClass)
{
  const auto stamp = rclcpp::Time(1, 0);
  for (const auto & [dense_class, command] :
       {std::pair<std::size_t, uint8_t>{0U, TurnIndicatorsCommand::DISABLE},
        std::pair<std::size_t, uint8_t>{1U, TurnIndicatorsCommand::ENABLE_LEFT},
        std::pair<std::size_t, uint8_t>{2U, TurnIndicatorsCommand::ENABLE_RIGHT}}) {
    TurnIndicatorManager manager(rclcpp::Duration::from_seconds(0.0));
    EXPECT_EQ(manager.evaluate(logits_for(dense_class), stamp).command, command);
  }
}

TEST(TurnIndicatorManagerTest, KeepsLastAcceptedCommandForHoldDuration)
{
  TurnIndicatorManager manager(rclcpp::Duration::from_seconds(1.0));

  EXPECT_EQ(
    manager.evaluate(logits_for(1U), rclcpp::Time(1, 0)).command,
    TurnIndicatorsCommand::ENABLE_LEFT);
  EXPECT_EQ(
    manager.evaluate(logits_for(0U), rclcpp::Time(1, 500000000)).command,
    TurnIndicatorsCommand::ENABLE_LEFT);
  // Preserve the legacy 5-class node's inclusive hold boundary: a command
  // received exactly at last_command_stamp + hold_duration is still held.
  EXPECT_EQ(
    manager.evaluate(logits_for(0U), rclcpp::Time(2, 0)).command,
    TurnIndicatorsCommand::ENABLE_LEFT);
  EXPECT_EQ(
    manager.evaluate(logits_for(0U), rclcpp::Time(2, 1)).command, TurnIndicatorsCommand::DISABLE);
  EXPECT_EQ(
    manager.evaluate(logits_for(2U), rclcpp::Time(2, 500000000)).command,
    TurnIndicatorsCommand::DISABLE);
  EXPECT_EQ(
    manager.evaluate(logits_for(2U), rclcpp::Time(3, 1)).command, TurnIndicatorsCommand::DISABLE);
  EXPECT_EQ(
    manager.evaluate(logits_for(2U), rclcpp::Time(3, 2)).command,
    TurnIndicatorsCommand::ENABLE_RIGHT);
}

TEST(TurnIndicatorManagerTest, InvalidLogitShapeDisablesStaleCommand)
{
  TurnIndicatorManager manager(rclcpp::Duration::from_seconds(0.0));
  EXPECT_EQ(
    manager.evaluate(logits_for(2U), rclcpp::Time(1, 0)).command,
    TurnIndicatorsCommand::ENABLE_RIGHT);
  EXPECT_EQ(manager.evaluate({}, rclcpp::Time(2, 0)).command, TurnIndicatorsCommand::DISABLE);
}
}  // namespace
}  // namespace autoware::diffusion_planner::test
