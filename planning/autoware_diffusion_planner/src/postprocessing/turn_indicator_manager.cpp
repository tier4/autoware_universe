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

#include "autoware/diffusion_planner/postprocessing/turn_indicator_manager.hpp"

#include <algorithm>
#include <vector>

namespace autoware::diffusion_planner::postprocess
{
namespace
{
uint8_t raw_state_to_command(const std::size_t raw_state)
{
  switch (raw_state) {
    case TURN_INDICATOR_OUTPUT_DISABLE:
      return TurnIndicatorsCommand::DISABLE;
    case TURN_INDICATOR_OUTPUT_ENABLE_LEFT:
      return TurnIndicatorsCommand::ENABLE_LEFT;
    case TURN_INDICATOR_OUTPUT_ENABLE_RIGHT:
      return TurnIndicatorsCommand::ENABLE_RIGHT;
    default:
      return TurnIndicatorsCommand::DISABLE;
  }
}
}  // namespace

TurnIndicatorManager::TurnIndicatorManager(const rclcpp::Duration & hold_duration)
: hold_duration_(hold_duration)
{
}

void TurnIndicatorManager::set_hold_duration(const rclcpp::Duration & hold_duration)
{
  hold_duration_ = hold_duration;
}

TurnIndicatorsCommand TurnIndicatorManager::evaluate(
  const std::vector<float> & turn_indicator_logit, const rclcpp::Time & stamp)
{
  TurnIndicatorsCommand command_msg;
  command_msg.stamp = stamp;

  if (turn_indicator_logit.size() != static_cast<std::size_t>(TURN_INDICATOR_OUTPUT_DIM)) {
    // A missing or stale-shape auxiliary output must never leave the previous
    // command latched indefinitely.
    stable_command_ = TurnIndicatorsCommand::DISABLE;
    last_command_stamp_ = rclcpp::Time{};
    command_msg.command = TurnIndicatorsCommand::DISABLE;
    return command_msg;
  }

  if (last_command_stamp_.nanoseconds() > 0) {
    const auto expiration = last_command_stamp_ + hold_duration_;
    if (stamp <= expiration) {
      command_msg.command = stable_command_;
      return command_msg;
    }
  }

  const auto max_it = std::max_element(turn_indicator_logit.begin(), turn_indicator_logit.end());
  const auto predicted_command = raw_state_to_command(
    static_cast<std::size_t>(std::distance(turn_indicator_logit.begin(), max_it)));
  stable_command_ = predicted_command;
  last_command_stamp_ = stamp;

  command_msg.command = stable_command_;
  return command_msg;
}

}  // namespace autoware::diffusion_planner::postprocess
