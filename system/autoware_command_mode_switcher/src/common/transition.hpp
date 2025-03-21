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

#ifndef COMMON__TRANSITION_HPP_
#define COMMON__TRANSITION_HPP_

#include <tier4_system_msgs/msg/command_mode_status_item.hpp>
#include <tier4_system_msgs/msg/command_source_status.hpp>

#include <string>

namespace autoware::command_mode_switcher
{

using tier4_system_msgs::msg::CommandModeStatusItem;
using SwitcherState = CommandModeStatusItem::_state_type;

enum class SourceStatus {
  Disabled,
  Transition,
  Enabled,
  Neutral,  // TODO(Takagi, Isamu): Use neutral. This means no source state.
};

struct TransitionContext
{
  SwitcherState sequence_target;
  SourceStatus source_state;
  bool is_source_exclusive;
  bool is_source_selected;
  bool is_control_selected;
};

struct TransitionResult
{
  const SwitcherState state;
  const std::string error;
};

}  // namespace autoware::command_mode_switcher

namespace autoware::command_mode_switcher::transition
{

TransitionResult next(SwitcherState state, const TransitionContext & context);
SwitcherState disable(SwitcherState state);
SwitcherState request_enabled(SwitcherState state);
SwitcherState request_standby(SwitcherState state);

}  // namespace autoware::command_mode_switcher::transition

#endif  // COMMON__TRANSITION_HPP_
