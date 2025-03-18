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

TransitionResult wait_command_mode_ready(SwitcherState state, const TransitionContext & context)
{
  (void)context;
  return {state, ""};
}

TransitionResult next(SwitcherState state, const TransitionContext & context)
{
  using State = tier4_system_msgs::msg::CommandModeStatusItem;

  // clang-format off
    switch (state) {
      case State::WAIT_COMMAND_MODE_READY: return transition::wait_command_mode_ready(state, context);  // NOLINT
    }
  // clang-format on

  return {state, ""};
}

}  // namespace autoware::command_mode_switcher::transition
