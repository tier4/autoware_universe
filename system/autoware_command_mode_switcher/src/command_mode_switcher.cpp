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
  loader_("autoware_command_mode_switcher", "autoware::command_mode_switcher::SwitcherPlugin"),
  control_gate_interface_(*this, [this]() { update_status(); }),
  vehicle_gate_interface_(*this, [this]() { update_status(); })
{
  // Create vehicle gate switcher.
  manual_switcher_ = std::make_shared<ManualSwitcher>();
  switchers_.push_back(manual_switcher_);

  // Create control gate switcher.
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
      autoware_switchers_[switcher->mode_name()] = switcher;
      switchers_.push_back(switcher);
    }
  }

  // Initialize all switchers. Call "construct" first, which acts as the base class constructor.
  for (const auto & switcher : switchers_) {
    switcher->construct(this);
    switcher->initialize();
  }

  pub_status_ =
    create_publisher<CommandModeStatus>("~/command_mode/status", rclcpp::QoS(1).transient_local());
  sub_request_ = create_subscription<CommandModeRequest>(
    "~/command_mode/request", rclcpp::QoS(1),
    std::bind(&CommandModeSwitcher::on_request, this, std::placeholders::_1));
  sub_availability_ = create_subscription<CommandModeAvailability>(
    "~/command_mode/availability", rclcpp::QoS(1),
    std::bind(&CommandModeSwitcher::on_availability, this, std::placeholders::_1));

  const auto period = rclcpp::Rate(declare_parameter<double>("update_rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { update_status(); });
}

void CommandModeSwitcher::on_availability(const CommandModeAvailability & msg)
{
  for (const auto & item : msg.items) {
    const auto iter = autoware_switchers_.find(item.mode);
    if (iter != autoware_switchers_.end()) {
      iter->second->set_mode_continuable(item.available);
      iter->second->set_mode_available(item.available);
      // TODO(Takagi, Isamu): Replace with the method using diagnostics.
      // iter->second->set_ctrl_available(item.???);
      // iter->second->set_transition_completed(item.???);
    }
  }
  is_ready_ = true;
  update_status();  // Reflect immediately.
}

void CommandModeSwitcher::on_request(const CommandModeRequest & msg)
{
  const auto iter = autoware_switchers_.find(msg.mode);
  if (iter == autoware_switchers_.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "invalid mode: " << msg.ctrl << " " << msg.mode);
    return;
  }

  const auto source_switcher = iter->second;
  const auto control_target = msg.ctrl ? source_switcher : source_switcher;
  const auto vehicle_target = msg.ctrl ? source_switcher : manual_switcher_;

  const auto control_status = control_target->status();
  const auto vehicle_status = vehicle_target->status();

  if (!control_status.mode_available || !vehicle_status.ctrl_available) {
    RCLCPP_WARN_STREAM(get_logger(), "request ignored: " << msg.ctrl << " " << msg.mode);
    return;
  }

  control_gate_target_ = control_target;
  vehicle_gate_target_ = vehicle_target;
  update_status();  // Reflect immediately.

  /*
  if (msg.ctrl) {
    vehicle_gate_switcher_->enable_autoware_control();
    control_gate_switcher_ = source_switcher;
    control_gate_switcher_->request_enabled();
  } else {
    vehicle_gate_switcher_->disable_autoware_control();
    control_gate_switcher_ = source_switcher;
    control_gate_switcher_->request_standby();
  }

  for (const auto & switcher : switchers_) {
    if (switcher == vehicle_gate_switcher_) continue;
    if (switcher == control_gate_switcher_) continue;
    switcher->handover();
  }
  */
}

void CommandModeSwitcher::handle_autoware_transition()
{
  const auto control_selected = control_gate_interface_.is_selected(*control_gate_target_);
  const auto vehicle_selected = vehicle_gate_interface_.is_selected(*vehicle_gate_target_);

  // TODO(Takagi, Isamu): Wait status update delay.
  if (!vehicle_selected) {
    vehicle_gate_interface_.request(*vehicle_gate_target_);
    return;
  }

  // TODO(Takagi, Isamu): Wait status update delay.
  if (!control_selected) {
    control_gate_interface_.request(*control_gate_target_, true);
    return;
  }

  // Note: (vehicle_selected && control_selected) is true here.
  if (!vehicle_gate_target_->status().transition_completed) {
    return;
  }

  control_gate_interface_.request(*control_gate_target_, false);
}

void CommandModeSwitcher::handle_manual_transition()
{
  const auto control_selected = control_gate_interface_.is_selected(*control_gate_target_);
  const auto vehicle_selected = vehicle_gate_interface_.is_selected(*vehicle_gate_target_);

  // TODO(Takagi, Isamu): Wait status update delay.
  if (!control_selected) {
    control_gate_interface_.request(*control_gate_target_, false);
    return;
  }

  // TODO(Takagi, Isamu): Wait status update delay.
  if (!vehicle_selected) {
    vehicle_gate_interface_.request(*vehicle_gate_target_);
  }

  /*
  const auto command_selected = vehicle_selected && (control_selected || request_manual_control);
  if (command_selected) {
    if (vehicle_gate_target_->status().transition_completed) {

    }
    if (request_autoware_control) {
      control_gate_interface_.request(*control_gate_target_, false);
    }
  }
  */
}

void CommandModeSwitcher::update_status()
{
  // TODO(Takagi, Isamu): Check call rate.
  if (!is_ready_) return;

  // NOTE: Update the source status first since the transition context depends on.
  for (const auto & switcher : switchers_) {
    switcher->update_source_status();
  }

  // TODO(Takagi, Isamu): Handle aborted transition (control, source, source group).
  for (const auto & switcher : switchers_) {
    switcher->update_status(create_transition_context(*switcher));
  }

  if (control_gate_target_ && vehicle_gate_target_) {
    if (control_gate_target_ == vehicle_gate_target_) {
      handle_autoware_transition();
    } else {
      handle_manual_transition();
      handle_background_transition();
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

TransitionContext CommandModeSwitcher::create_transition_context(const SwitcherPlugin & target)
{
  const auto is_source_exclusive = [this](const SwitcherPlugin & target) {
    if (target.source_name().empty()) {
      return true;
    }
    for (const auto & switcher : switchers_) {
      if (switcher->mode_name() == target.mode_name()) continue;
      if (switcher->source_name() != target.source_name()) continue;
      if (switcher->source_status() == SourceStatus::Disabled) continue;
      return false;
    }
    return true;
  };

  TransitionContext context;
  context.sequence_target = target.sequence_target();
  context.source_state = target.source_status();
  context.is_source_exclusive = is_source_exclusive(target);
  context.is_source_selected = control_gate_interface_.is_selected(target);
  context.is_control_selected = vehicle_gate_interface_.is_selected(target);
  context.is_mode_continuable = target.status().mode_continuable;
  context.is_mode_available = target.status().mode_available;
  context.is_ctrl_available = target.status().ctrl_available;
  context.is_transition_completed = target.status().transition_completed;
  return context;
}

}  // namespace autoware::command_mode_switcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::command_mode_switcher::CommandModeSwitcher)
