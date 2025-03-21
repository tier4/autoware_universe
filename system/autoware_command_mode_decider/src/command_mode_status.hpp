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

#ifndef COMMAND_MODE_STATUS_HPP_
#define COMMAND_MODE_STATUS_HPP_

#include <tier4_system_msgs/msg/command_mode_status.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::command_mode_decider
{

using tier4_system_msgs::msg::CommandModeStatusItem;

class CommandModeStatusWrapper
{
public:
  void init(const std::vector<std::string> & modes);
  bool ready() const;
  const CommandModeStatusItem & get(const std::string & mode) const;
  void set(const CommandModeStatusItem & item);

private:
  CommandModeStatusItem empty_item_;
  std::unordered_map<std::string, CommandModeStatusItem> command_mode_status_;
};

}  // namespace autoware::command_mode_decider

#endif  // COMMAND_MODE_STATUS_HPP_
