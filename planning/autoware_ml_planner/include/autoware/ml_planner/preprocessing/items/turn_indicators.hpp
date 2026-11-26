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

#ifndef AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__TURN_INDICATORS_HPP_
#define AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__TURN_INDICATORS_HPP_

#include "autoware/ml_planner/preprocessing/message_view.hpp"

#include <rclcpp/time.hpp>
#include <xtensor/xarray.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>

#include <cstdint>
#include <deque>

namespace autoware::ml_planner::preprocess
{
xt::xarray<float> create_turn_indicators(
  const MessageView<autoware_vehicle_msgs::msg::TurnIndicatorsReport> & turn_indicators_msgs,
  const rclcpp::Time & current_time, int64_t num_timesteps, double time_step_s);
}  // namespace autoware::ml_planner::preprocess

#endif  // AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__TURN_INDICATORS_HPP_
