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

#include "command_mode_switcher.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::command_mode_switcher
{

CommandModeSwitcher::CommandModeSwitcher(const rclcpp::NodeOptions & options)
: Node("command_mode_switcher", options),
  selector_interface_(*this),
  loader_("autoware_command_mode_switcher", "autoware::command_mode_switcher::SwitcherPlugin")
{
  pub_status_ =
    create_publisher<CommandModeStatus>("~/command_mode/status", rclcpp::QoS(1).transient_local());
  sub_request_ = create_subscription<CommandModeRequest>(
    "~/command_mode/request", rclcpp::QoS(1),
    std::bind(&CommandModeSwitcher::on_request, this, std::placeholders::_1));

  // Init manual switcher
  manual_switcher_ = std::make_shared<ManualSwitcher>();
  manual_switcher_->construct(this);

  // Init source switchers
  {
    const auto plugins = declare_parameter<std::vector<std::string>>("plugins");

    for (const auto & plugin : plugins) {
      if (!loader_.isClassAvailable(plugin)) {
        RCLCPP_WARN_STREAM(get_logger(), "ignore unknown plugin: " << plugin);
        continue;
      }
      const auto switcher = loader_.createSharedInstance(plugin);
      if (switchers_.count(switcher->mode_name())) {
        RCLCPP_WARN_STREAM(get_logger(), "ignore duplicate plugin: " << plugin);
        continue;
      }
      switcher->construct(this);
      switchers_[switcher->mode_name()] = switcher;
    }
  }

  const auto period = rclcpp::Rate(declare_parameter<double>("update_rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void CommandModeSwitcher::on_timer()
{
  const auto is_source_exclusive = [this](std::shared_ptr<SwitcherPlugin> target) {
    for (const auto & [mode, switcher] : switchers_) {
      if (switcher == target) continue;
      if (switcher->source_name() != target->source_name()) continue;
      if (switcher->source_status() == SourceStatus::Disabled) continue;
      return false;
    }
    return true;
  };

  // TODO(Takagi, Isamu): Detect override.

  // NOTE: Update the source status first since the transition context depends on.
  for (const auto & [mode, switcher] : switchers_) {
    switcher->update_source_status();
  }

  for (const auto & [mode, switcher] : switchers_) {
    // TODO(Takagi, Isamu): move to utility function.
    TransitionContext context;
    context.target_state = switcher->target_state();
    context.is_source_ready = switcher->source_status() == SourceStatus::Enabled;
    context.is_source_exclusive = is_source_exclusive(switcher);
    context.is_source_selected = switcher->source_name() == selector_interface_.source_name();
    context.is_control_selected =
      switcher->autoware_control() == selector_interface_.autoware_control();
    switcher->update_status(context);
  }

  // Sync command source.
  if (source_transition_) {
    if (source_transition_->sequence_state() == CommandModeStatusItem::WAIT_SOURCE_SELECTED) {
      bool req = selector_interface_.select_source(source_transition_->source_name());
      if (req) {
        RCLCPP_WARN_STREAM(get_logger(), "source select request");
      }
    }
  }

  // Sync control mode.
  if (manual_transition_) {
    if (manual_transition_->sequence_state() == CommandModeStatusItem::WAIT_CONTROL_SELECTED) {
      bool req = selector_interface_.select_control(manual_transition_->autoware_control());
      if (req) {
        RCLCPP_WARN_STREAM(get_logger(), "control select request");
      }
    }
  }

  publish_command_mode_status();
}

void CommandModeSwitcher::on_request(const CommandModeRequest & msg)
{
  const auto iter = switchers_.find(msg.mode);
  if (iter == switchers_.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "invalid mode: " << msg.ctrl << " " << msg.mode);
    return;
  }

  const auto source_switcher = iter->second;
  if (msg.ctrl) {
    manual_transition_ = source_switcher;  // Set source_switcher to disable manual control.
    source_transition_ = source_switcher;
    source_transition_->request(CommandModeStatusItem::ENABLED);
  } else {
    manual_transition_ = manual_switcher_;  // Set manual_switcher to enable manual control.
    source_transition_ = source_switcher;
    source_transition_->request(CommandModeStatusItem::STANDBY);
    manual_transition_->request(CommandModeStatusItem::ENABLED);
  }

  for (const auto & [mode, switcher] : switchers_) {
    if (switcher == source_transition_) continue;
    if (switcher == manual_transition_) continue;
    switcher->handover();
  }
}

void CommandModeSwitcher::on_selector_updated(/* selector status */)
{
  // update(elector_status);
}

void CommandModeSwitcher::publish_command_mode_status()
{
  CommandModeStatus msg;
  msg.stamp = now();
  for (const auto & [mode, switcher] : switchers_) {
    msg.items.push_back(switcher->status());
  }
  pub_status_->publish(msg);
}

}  // namespace autoware::command_mode_switcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::command_mode_switcher::CommandModeSwitcher)
