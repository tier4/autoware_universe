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

#ifndef COMMAND_MODE_DECIDER_HPP_
#define COMMAND_MODE_DECIDER_HPP_

#include "command_mode_decider_base.hpp"

#include <string>

namespace autoware::command_mode_decider
{

class CommandModeDecider : public CommandModeDeciderBase
{
public:
  explicit CommandModeDecider(const rclcpp::NodeOptions & options);

protected:
  std::string decide_command_mode() override;

private:
  bool use_pull_over_;
  bool use_comfortable_stop_;
};

}  // namespace autoware::command_mode_decider

#endif  // COMMAND_MODE_DECIDER_HPP_
