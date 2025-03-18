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

#include "plugin.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::command_mode_switcher
{

void SwitcherPlugin::construct(rclcpp::Node * node)
{
  node_ = node;

  status_.mode = mode_name();
  status_.state = CommandModeStatusItem::DISABLED;
  status_.target = CommandModeStatusItem::DISABLED;
  status_.mrm = CommandModeStatusItem::NONE;
}

void SwitcherPlugin::request(SwitcherState target)
{
  switch (target) {
    case CommandModeStatusItem::ENABLED:
    case CommandModeStatusItem::STANDBY:
      status_.state = CommandModeStatusItem::WAIT_COMMAND_MODE_READY;
      break;
    case CommandModeStatusItem::CLEANUP:
      status_.state = CommandModeStatusItem::DISABLED;
      break;
    case CommandModeStatusItem::ABORTED:
      // Keep the state but release the command source.
      break;
    default:
      RCLCPP_ERROR_STREAM(node_->get_logger(), "invalid target state: " << target);
      break;
  }
  status_.target = target;
}

void SwitcherPlugin::update_state(const TransitionContext & context)
{
  const auto logger = node_->get_logger();
  const auto update = [logger](SwitcherState state, const TransitionContext & context) {
    constexpr int loop_limit = 20;
    std::vector<SwitcherState> history;
    history.push_back(state);

    for (int i = 0; i < loop_limit; ++i) {
      const auto result = transition::next(state, context);
      if (!result.error.empty()) {
        RCLCPP_ERROR_STREAM(logger, result.error);
      }
      if (state == result.state) {
        break;
      }
      history.push_back(state);
    }

    if (loop_limit <= history.size()) {
      RCLCPP_ERROR_STREAM(logger, "exeeded");
    }
    if (1 < history.size()) {
      std::string log;
      for (const auto & h : history) {
        log += std::to_string(h) + " ";
      }
      RCLCPP_WARN_STREAM(logger, log);
    }

    return state;
  };

  status_.state = update(status_.state, context);
}

/*
void SwitcherPlugin::request(bool activate)
{
  if (activate) {
    context_->select_source(source());
  }
}

void SwitcherPlugin::on_source_status(const CommandSourceStatus & msg)
{
  status_.activation = msg.source == source();
  status_.transition = msg.transition;
}
*/

}  // namespace autoware::command_mode_switcher
