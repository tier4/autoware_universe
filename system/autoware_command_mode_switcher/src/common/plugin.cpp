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
  source_status_ = SourceStatus::Disabled;

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

void SwitcherPlugin::override()
{
  if (status_.target != CommandModeStatusItem::ENABLED) {
    return;
  }
  switch (status_.state) {
    case CommandModeStatusItem::WAIT_CONTROL_READY:
    case CommandModeStatusItem::WAIT_CONTROL_SELECTED:
    case CommandModeStatusItem::WAIT_COMMAND_MODE_STABLE:
    case CommandModeStatusItem::ENABLED:
      status_.state = CommandModeStatusItem::STANDBY;
      break;
  }
  status_.target = CommandModeStatusItem::STANDBY;
}

void SwitcherPlugin::update_source_status()
{
  switch (status_.target) {
    case CommandModeStatusItem::ENABLED:
    case CommandModeStatusItem::STANDBY:
      source_status_ = SourceStatus::Enabled;
      break;
    case CommandModeStatusItem::CLEANUP:
    case CommandModeStatusItem::ABORTED:
      source_status_ = SourceStatus::Disabled;
      break;
  }
}

void SwitcherPlugin::update_status(const TransitionContext & context)
{
  const auto logger = [this](const std::string & message) {
    RCLCPP_WARN_STREAM(node_->get_logger(), mode_name() << " mode: " << message);
  };

  const auto update = [logger](SwitcherState state, const TransitionContext & context) {
    constexpr int loop_limit = 20;
    std::vector<SwitcherState> history;
    history.push_back(state);

    for (int i = 0; i < loop_limit; ++i) {
      const auto result = transition::next(state, context);
      if (!result.error.empty()) {
        logger("transition error: " + result.error);
      }
      if (state == result.state) {
        break;
      }
      state = result.state;
      history.push_back(state);
    }

    if (loop_limit <= history.size()) {
      logger("loop limit exceeded");
    }
    if (1 < history.size()) {
      std::string log;
      for (const auto & h : history) {
        log += " " + std::to_string(h);
      }
      logger("update" + log);
    }

    return state;
  };

  // TODO(Takagi, Isamu): update source ready
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
