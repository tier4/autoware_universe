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
  loader_("autoware_command_mode_switcher", "autoware::command_mode_switcher::CommandPlugin"),
  control_gate_interface_(*this, [this]() { update(); }),
  vehicle_gate_interface_(*this, [this]() { update(); })
{
  // Create vehicle gate switcher.
  {
    const auto command = std::make_shared<Command>(std::make_shared<ManualCommand>());
    manual_command_ = command;
    commands_.push_back(command);
  }

  // Create control gate switcher.
  {
    const auto plugins = declare_parameter<std::vector<std::string>>("plugins");

    for (const auto & plugin : plugins) {
      if (!loader_.isClassAvailable(plugin)) {
        RCLCPP_WARN_STREAM(get_logger(), "ignore unknown plugin: " << plugin);
        continue;
      }
      const auto instance = loader_.createSharedInstance(plugin);
      if (autoware_commands_.count(instance->mode_name())) {
        RCLCPP_WARN_STREAM(get_logger(), "ignore duplicate plugin: " << plugin);
        continue;
      }

      const auto command = std::make_shared<Command>(instance);
      autoware_commands_[instance->mode_name()] = command;
      commands_.push_back(command);
    }
  }

  // Initialize all switchers. Call "construct" first, which acts as the base class constructor.
  for (const auto & command : commands_) {
    command->plugin->construct(this);
    command->plugin->initialize();
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
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { update(); });
}

void CommandModeSwitcher::on_availability(const CommandModeAvailability & msg)
{
  for (const auto & item : msg.items) {
    const auto iter = autoware_commands_.find(item.mode);
    if (iter != autoware_commands_.end()) {
      iter->second->status.mode_continuable = item.available;
      iter->second->status.mode_available = item.available;
      // TODO(Takagi, Isamu): Replace with the method using diagnostics.
      // iter->second->status.control_gate_ready = item.???;
      // iter->second->status.vehicle_gate_ready = item.???;
      // iter->second->status.transition_completed = item.???;
    }
  }
  is_ready_ = true;
  update();  // Reflect immediately.
}

void CommandModeSwitcher::on_request(const CommandModeRequest & msg)
{
  const auto iter = autoware_commands_.find(msg.mode);
  if (iter == autoware_commands_.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "invalid mode: " << msg.ctrl << " " << msg.mode);
    return;
  }

  const auto target_command = iter->second;
  const auto control_target = msg.ctrl ? target_command : target_command;
  const auto vehicle_target = msg.ctrl ? target_command : manual_command_;

  // Check transition conditions.
  if (!control_target->status.mode_available || !vehicle_target->status.vehicle_gate_ready) {
    RCLCPP_WARN_STREAM(get_logger(), "request rejected: " << msg.ctrl << " " << msg.mode);
    return;
  }

  // Update request status.
  for (const auto & command : commands_) {
    command->status.control_gate_request = false;
    command->status.vehicle_gate_request = false;
  }
  control_target->status.control_gate_request = true;
  vehicle_target->status.vehicle_gate_request = true;

  // Update command target.
  const auto control_target_changed = control_gate_target_ != control_target;
  const auto vehicle_target_changed = vehicle_gate_target_ != vehicle_target;
  control_gate_target_ = control_target;
  vehicle_gate_target_ = vehicle_target;

  // Update if the command target is changed.
  if (control_target_changed || vehicle_target_changed) {
    RCLCPP_INFO_STREAM(get_logger(), "request updated: " << msg.ctrl << " " << msg.mode);
    update();
  }
}

void CommandModeSwitcher::update()
{
  // TODO(Takagi, Isamu): Check call rate.
  if (!is_ready_) return;

  // NOTE: Update the source status first since the transition context depends on.
  for (const auto & command : commands_) {
    command->status.source_state = command->plugin->update_source_state();
  }

  // TODO(Takagi, Isamu): Handle aborted transition (control, source, source group).
  for (const auto & command : commands_) {
    command->update_status();
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
  const auto convert = [](const Command & command) {
    CommandModeStatusItem item;
    item.mode = command.plugin->mode_name();
    // state
    // mrm
    item.mode_continuable = command.status.mode_continuable;
    item.mode_available = command.status.mode_available;
    item.control_gate_request = command.status.control_gate_request;
    item.vehicle_gate_request = command.status.vehicle_gate_request;
    item.control_gate_ready = command.status.control_gate_ready;
    item.vehicle_gate_ready = command.status.vehicle_gate_ready;
    item.transition_completed = command.status.transition_completed;
    item.debug = convert_debug_string(command.status);
    return item;
  };

  CommandModeStatus msg;
  msg.stamp = now();
  for (const auto & command : commands_) {
    msg.items.push_back(convert(*command));
  }
  pub_status_->publish(msg);
}

void CommandModeSwitcher::handle_autoware_transition()
{
  const auto control_selected =
    control_gate_target_->status.control_gate_state == ControlGateState::Selected;
  const auto vehicle_selected =
    vehicle_gate_target_->status.vehicle_gate_state == VehicleGateState::Selected;

  // TODO(Takagi, Isamu): Wait status update delay.
  if (!vehicle_selected) {
    vehicle_gate_interface_.request(*vehicle_gate_target_->plugin);
    return;
  }

  // TODO(Takagi, Isamu): Wait status update delay.
  if (!control_selected) {
    control_gate_interface_.request(*control_gate_target_->plugin, true);
    return;
  }

  // Note: (vehicle_selected && control_selected) is true here.
  if (!vehicle_gate_target_->status.transition_completed) {
    return;
  }

  control_gate_interface_.request(*control_gate_target_->plugin, false);
}

void CommandModeSwitcher::handle_manual_transition()
{
  const auto control_selected =
    control_gate_target_->status.control_gate_state == ControlGateState::Selected;
  const auto vehicle_selected =
    vehicle_gate_target_->status.vehicle_gate_state == VehicleGateState::Selected;

  // TODO(Takagi, Isamu): Wait status update delay.
  if (!control_selected) {
    control_gate_interface_.request(*control_gate_target_->plugin, false);
    return;
  }

  // TODO(Takagi, Isamu): Wait status update delay.
  if (!vehicle_selected) {
    vehicle_gate_interface_.request(*vehicle_gate_target_->plugin);
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

}  // namespace autoware::command_mode_switcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::command_mode_switcher::CommandModeSwitcher)
