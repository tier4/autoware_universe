//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef AUTOWARE_COMMAND_MODE_DECIDER__STATUS_HPP_
#define AUTOWARE_COMMAND_MODE_DECIDER__STATUS_HPP_

#include <autoware_command_mode_types/types/command_mode_status.hpp>

#include <tier4_system_msgs/msg/command_mode_availability.hpp>

#include <unordered_map>
#include <vector>

namespace autoware::command_mode_decider
{

using autoware::command_mode_types::CommandModeStatusItem;
using tier4_system_msgs::msg::CommandModeAvailabilityItem;

class CommandModeStatusTable
{
public:
  void init(const std::vector<uint16_t> & modes);
  void set(const CommandModeStatusItem & item, const rclcpp::Time & stamp);
  void set(const CommandModeAvailabilityItem & item, const rclcpp::Time & stamp);
  void check_timeout(const rclcpp::Time & stamp);
  bool ready() const;
  bool available(uint16_t mode, bool is_manual) const;
  const CommandModeStatusItem & get(uint16_t mode) const;

  auto begin() const { return command_mode_status_.begin(); }
  auto end() const { return command_mode_status_.end(); }

private:
  CommandModeStatusItem empty_item_;
  std::unordered_map<uint16_t, CommandModeStatusItem> command_mode_status_;
  std::unordered_map<uint16_t, rclcpp::Time> command_mode_stamps_;
};

}  // namespace autoware::command_mode_decider

#endif  // AUTOWARE_COMMAND_MODE_DECIDER__STATUS_HPP_
