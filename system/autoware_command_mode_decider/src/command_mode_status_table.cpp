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

#include "autoware_command_mode_decider/command_mode_status_table.hpp"

#include <autoware_command_mode_types/constants/modes.hpp>

#include <string>
#include <vector>

namespace autoware::command_mode_decider
{

void CommandModeStatusTable::init(const std::vector<uint16_t> & modes)
{
  for (const auto & mode : modes) {
    const auto item = CommandModeStatusItem(autoware::command_mode_types::modes::unknown);
    command_mode_status_[mode] = item;
  }
}

bool CommandModeStatusTable::ready() const
{
  for (const auto & [mode, item] : command_mode_status_) {
    if (item.mode == autoware::command_mode_types::modes::unknown) return false;
  }
  return true;
}

void CommandModeStatusTable::set(const CommandModeStatusItem & item, const rclcpp::Time & stamp)
{
  const auto iter = command_mode_status_.find(item.mode);
  if (iter != command_mode_status_.end()) {
    iter->second = item;
    command_mode_stamps_[item.mode] = stamp;
  }
}

void CommandModeStatusTable::check_timeout(const rclcpp::Time & stamp)
{
  for (const auto & [mode, message_stamp] : command_mode_stamps_) {
    const auto duration = (stamp - message_stamp).seconds();
    if (1.0 < duration) {
      command_mode_status_[mode] = CommandModeStatusItem(mode);
    }
  }
}

const CommandModeStatusItem & CommandModeStatusTable::get(uint16_t mode) const
{
  const auto iter = command_mode_status_.find(mode);
  if (iter != command_mode_status_.end()) {
    return iter->second;
  }
  return empty_item_;
}

}  // namespace autoware::command_mode_decider
