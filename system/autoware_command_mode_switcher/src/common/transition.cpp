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

#include "transition.hpp"

namespace autoware::command_mode_switcher::transition
{

TransitionResult wait_command_mode_ready(const TransitionContext & context)
{
  // TODO(Takagi, Isamu): Subscribe and use command mode availability.
  (void)context;
  return {CommandModeStatusItem::WAIT_SOURCE_READY, ""};
}

TransitionResult wait_source_ready(const TransitionContext & context)
{
  if (context.is_source_ready) {
    return {CommandModeStatusItem::WAIT_SOURCE_EXCLUSIVE, ""};
  } else {
    return {CommandModeStatusItem::WAIT_SOURCE_READY, ""};
  }
}

TransitionResult wait_source_exclusive(const TransitionContext & context)
{
  if (context.is_source_exclusive) {
    return {CommandModeStatusItem::WAIT_SOURCE_SELECTED, ""};
  } else {
    return {CommandModeStatusItem::WAIT_SOURCE_EXCLUSIVE, ""};
  }
}

TransitionResult wait_source_selected(const TransitionContext & context)
{
  if (!context.is_source_selected) {
    return {CommandModeStatusItem::WAIT_SOURCE_SELECTED, ""};
  }
  if (context.target_state == CommandModeStatusItem::ENABLED) {
    return {CommandModeStatusItem::WAIT_CONTROL_READY, ""};
  }
  if (context.target_state == CommandModeStatusItem::STANDBY) {
    return {CommandModeStatusItem::STANDBY, ""};
  }
  return {CommandModeStatusItem::WAIT_SOURCE_SELECTED, "invalid target state"};
}

TransitionResult wait_control_ready(const TransitionContext & context)
{
  if (context.is_control_selected) {
    return {CommandModeStatusItem::WAIT_CONTROL_SELECTED, ""};
  }
  return {CommandModeStatusItem::WAIT_CONTROL_READY, ""};
}

TransitionResult next(SwitcherState state, const TransitionContext & context)
{
  using State = tier4_system_msgs::msg::CommandModeStatusItem;

  // clang-format off
    switch (state) {
      case State::WAIT_COMMAND_MODE_READY: return wait_command_mode_ready(context);  // NOLINT
      case State::WAIT_SOURCE_READY:       return wait_source_ready(context);        // NOLINT
      case State::WAIT_SOURCE_EXCLUSIVE:   return wait_source_exclusive(context);    // NOLINT
      case State::WAIT_SOURCE_SELECTED:    return wait_source_selected(context);     // NOLINT
    }
  // clang-format on

  return {state, ""};
}

}  // namespace autoware::command_mode_switcher::transition
