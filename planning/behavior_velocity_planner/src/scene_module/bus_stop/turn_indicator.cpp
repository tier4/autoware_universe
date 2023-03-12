// Copyright 2022 TIER IV, Inc.
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

#include "scene_module/bus_stop/turn_indicator.hpp"

namespace behavior_velocity_planner
{
TurnIndicator::TurnIndicator(rclcpp::Node & node) : node_(node)
{
  pub_turn_indicator_ =
    node.create_publisher<TurnIndicatorsCommand>("/planning/turn_indicators_cmd", 1);
}

void TurnIndicator::setTurnSignal(const TurnIndicatorsCommand & turn_signal)
{
  turn_signal_ = turn_signal;
}

void TurnIndicator::setTurnSignal(const uint8_t turn_signal, const rclcpp::Time & time)
{
  turn_signal_.stamp = time;
  switch (turn_signal) {
    case 0:
      turn_signal_.command = TurnIndicatorsCommand::NO_COMMAND;
      break;

    case 1:
      turn_signal_.command = TurnIndicatorsCommand::DISABLE;
      break;

    case 2:
      turn_signal_.command = TurnIndicatorsCommand::ENABLE_LEFT;
      break;

    case 3:
      turn_signal_.command = TurnIndicatorsCommand::ENABLE_RIGHT;
      break;

    default:
      turn_signal_.command = TurnIndicatorsCommand::NO_COMMAND;
      break;
  }
}

void TurnIndicator::publish() { pub_turn_indicator_->publish(turn_signal_); }

}  // namespace behavior_velocity_planner
