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
  selector_interface_(*this, [this]() { update_status(); }),
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
  switchers_.push_back(manual_switcher_);

  // Init source switchers
  {
    const auto plugins = declare_parameter<std::vector<std::string>>("plugins");

    for (const auto & plugin : plugins) {
      if (!loader_.isClassAvailable(plugin)) {
        RCLCPP_WARN_STREAM(get_logger(), "ignore unknown plugin: " << plugin);
        continue;
      }
      const auto switcher = loader_.createSharedInstance(plugin);
      if (autoware_switchers_.count(switcher->mode_name())) {
        RCLCPP_WARN_STREAM(get_logger(), "ignore duplicate plugin: " << plugin);
        continue;
      }
      switcher->construct(this);
      autoware_switchers_[switcher->mode_name()] = switcher;
      switchers_.push_back(switcher);
    }
  }

  const auto period = rclcpp::Rate(declare_parameter<double>("update_rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { update_status(); });
}

void CommandModeSwitcher::on_request(const CommandModeRequest & msg)
{
  const auto iter = autoware_switchers_.find(msg.mode);
  if (iter == autoware_switchers_.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "invalid mode: " << msg.ctrl << " " << msg.mode);
    return;
  }

  const auto source_switcher = iter->second;
  if (msg.ctrl) {
    foreground_transition_ = source_switcher;  // Set source_switcher to disable manual control.
    background_transition_ = source_switcher;
    background_transition_->request_enabled();
  } else {
    foreground_transition_ = manual_switcher_;  // Set manual_switcher to enable manual control.
    background_transition_ = source_switcher;
    foreground_transition_->request_enabled();
    background_transition_->request_standby();
  }

  for (const auto & switcher : switchers_) {
    if (switcher == background_transition_) continue;
    if (switcher == foreground_transition_) continue;
    switcher->handover();
  }

  update_status();  // Reflect immediately.
}

void CommandModeSwitcher::update_status()
{
  const auto is_source_exclusive = [this](std::shared_ptr<SwitcherPlugin> target) {
    if (target->source_name().empty()) {
      return true;
    }
    for (const auto & switcher : switchers_) {
      if (switcher == target) continue;
      if (switcher->source_name() != target->source_name()) continue;
      if (switcher->source_status() == SourceStatus::Disabled) continue;
      return false;
    }
    return true;
  };

  // Check if the foreground transition is complete.
  if (foreground_transition_) {
    if (foreground_transition_->sequence_state() == CommandModeStatusItem::ENABLED) {
      for (const auto & switcher : switchers_) {
        if (switcher == background_transition_) continue;
        if (switcher == foreground_transition_) continue;
        // TODO(Takagi, Isamu): override
        switcher->disable();
      }
      // TODO(Takagi, Isamu): override if manual mode is enabled.
      const auto is_same = foreground_transition_ == background_transition_;
      foreground_transition_ = nullptr;
      background_transition_ = is_same ? nullptr : background_transition_;
    }
  }

  // Check if the background transition is complete.
  if (background_transition_) {
    if (background_transition_->sequence_state() == CommandModeStatusItem::STANDBY) {
      for (const auto & switcher : switchers_) {
        if (switcher == background_transition_) continue;
        if (switcher == foreground_transition_) continue;
        switcher->disable();
      }
      background_transition_ = nullptr;
    }
  }

  // NOTE: Update the source status first since the transition context depends on.
  for (const auto & switcher : switchers_) {
    switcher->update_source_status();
  }
  for (const auto & switcher : switchers_) {
    // TODO(Takagi, Isamu): move to utility function.
    TransitionContext context;
    context.sequence_target = switcher->sequence_target();
    context.source_state = switcher->source_status();
    context.is_source_exclusive = is_source_exclusive(switcher);
    context.is_source_selected = switcher->source_name() == selector_interface_.source_name();
    context.is_control_selected =
      switcher->autoware_control() == selector_interface_.autoware_control();
    switcher->update_status(context);
  }

  // Sync command source.
  if (background_transition_) {
    if (background_transition_->sequence_state() == CommandModeStatusItem::WAIT_SOURCE_SELECTED) {
      bool req = selector_interface_.select_source(background_transition_->source_name());
      if (req) {
        RCLCPP_WARN_STREAM(get_logger(), "source select request");
      }
    }
  }

  // Sync control mode.
  if (foreground_transition_) {
    if (foreground_transition_->sequence_state() == CommandModeStatusItem::WAIT_CONTROL_SELECTED) {
      bool req = selector_interface_.select_control(foreground_transition_->autoware_control());
      if (req) {
        RCLCPP_WARN_STREAM(get_logger(), "control select request");
      }
    }
  }

  publish_command_mode_status();
}

void CommandModeSwitcher::publish_command_mode_status()
{
  CommandModeStatus msg;
  msg.stamp = now();
  for (const auto & switcher : switchers_) {
    msg.items.push_back(switcher->status());
  }
  pub_status_->publish(msg);
}

}  // namespace autoware::command_mode_switcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::command_mode_switcher::CommandModeSwitcher)
