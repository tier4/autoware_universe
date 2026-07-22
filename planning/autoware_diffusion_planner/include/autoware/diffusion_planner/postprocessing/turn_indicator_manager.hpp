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

#ifndef AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__TURN_INDICATOR_MANAGER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__TURN_INDICATOR_MANAGER_HPP_

#include "autoware/diffusion_planner/dimensions.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <cstdint>
#include <vector>

namespace autoware::diffusion_planner::postprocess
{
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

/**
 * @brief Holds the last accepted raw turn-indicator state for a duration.
 *
 * The model emits three dense state classes (DISABLE, ENABLE_LEFT,
 * ENABLE_RIGHT).  The old node held the last non-KEEP command for the
 * configured duration.  The same deployment-side hold behavior is retained;
 * KEEP is not a model class.
 */
class TurnIndicatorManager
{
public:
  /**
   * @brief Constructs a manager that holds the last accepted command.
   *
   * @param hold_duration Duration to hold the last accepted command before allowing changes.
   */
  explicit TurnIndicatorManager(const rclcpp::Duration & hold_duration);

  /**
   * @brief Evaluates three-class logits into a held command.
   *
   * @param turn_indicator_logit Logits in Python order: disable, left, right.
   * @param stamp Timestamp for the command message.
   * @return TurnIndicatorsCommand with the selected command and stamp.
   */
  TurnIndicatorsCommand evaluate(
    const std::vector<float> & turn_indicator_logit, const rclcpp::Time & stamp);

  /**
   * @brief Updates the hold duration for the last accepted command.
   *
   * @param hold_duration New hold duration.
   */
  void set_hold_duration(const rclcpp::Duration & hold_duration);

private:
  rclcpp::Duration hold_duration_;
  uint8_t stable_command_{TurnIndicatorsCommand::DISABLE};
  rclcpp::Time last_command_stamp_{};
};

}  // namespace autoware::diffusion_planner::postprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__TURN_INDICATOR_MANAGER_HPP_
