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
#ifndef SCENE_MODULE__BUS_STOP_TURN_INDICATOR_HPP_
#define SCENE_MODULE__BUS_STOP_TURN_INDICATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp"

namespace behavior_velocity_planner
{
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;

class TurnIndicator
{
public:
  explicit TurnIndicator(rclcpp::Node & node);
  ~TurnIndicator() {}

  void setTurnSignal(const TurnIndicatorsCommand & turn_signal);
  void setTurnSignal(const uint8_t turn_signal, const rclcpp::Time & time);
  void publish();

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr pub_turn_indicator_;
  TurnIndicatorsCommand turn_signal_;
};

}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__BUS_STOP_TURN_INDICATOR_HPP_
