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

TransitionResult wait_source_acquire(const TransitionContext & context)
{
  if (context.source_state == SourceStatus::Enabled) {
    return {CommandModeStatusItem::WAIT_SOURCE_EXCLUSIVE, ""};
  } else {
    return {CommandModeStatusItem::WAIT_SOURCE_READY, ""};
  }
}

TransitionResult wait_source_release(const TransitionContext & context)
{
  if (context.source_state == SourceStatus::Disabled) {
    return {CommandModeStatusItem::DISABLED, ""};
  } else {
    return {CommandModeStatusItem::CLEANUP, ""};
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
  if (context.sequence_target == CommandModeStatusItem::ENABLED) {
    return {CommandModeStatusItem::WAIT_CONTROL_READY, ""};
  }
  if (context.sequence_target == CommandModeStatusItem::STANDBY) {
    return {CommandModeStatusItem::STANDBY, ""};
  }
  return {CommandModeStatusItem::WAIT_SOURCE_SELECTED, "invalid target state"};
}

TransitionResult wait_control_ready(const TransitionContext & context)
{
  // TODO(Takagi, Isamu): Subscribe and use transition availability.
  (void)context;
  return {CommandModeStatusItem::WAIT_CONTROL_SELECTED, ""};
}

TransitionResult wait_control_selected(const TransitionContext & context)
{
  if (context.is_control_selected) {
    return {CommandModeStatusItem::WAIT_CONTROL_SELECTED, ""};
  } else {
    return {CommandModeStatusItem::WAIT_COMMAND_MODE_STABLE, ""};
  }
}

TransitionResult wait_command_mode_stable(const TransitionContext & context)
{
  // TODO(Takagi, Isamu): Subscribe and use transition completed.
  (void)context;
  return {CommandModeStatusItem::ENABLED, ""};
}

TransitionResult next(SwitcherState state, const TransitionContext & context)
{
  using State = tier4_system_msgs::msg::CommandModeStatusItem;

  // clang-format off
  switch (state) {
    case State::WAIT_COMMAND_MODE_READY:  return wait_command_mode_ready(context);
    case State::WAIT_SOURCE_READY:        return wait_source_acquire(context);
    case State::WAIT_SOURCE_EXCLUSIVE:    return wait_source_exclusive(context);
    case State::WAIT_SOURCE_SELECTED:     return wait_source_selected(context);
    case State::WAIT_CONTROL_READY:       return wait_control_ready(context);
    case State::WAIT_CONTROL_SELECTED:    return wait_control_selected(context);
    case State::WAIT_COMMAND_MODE_STABLE: return wait_command_mode_stable(context);
    case State::DISABLED:                 return {state, ""};
    case State::STANDBY:                  return {state, ""};
    case State::ENABLED:                  return {state, ""};
    case State::CLEANUP:                  return wait_source_release(context);
  }
  // clang-format on

  return {state, "unknown transition"};
}

SwitcherState disable(SwitcherState state)
{
  using State = tier4_system_msgs::msg::CommandModeStatusItem;
  return state == State::DISABLED ? State::DISABLED : State::CLEANUP;
}

}  // namespace autoware::command_mode_switcher::transition
