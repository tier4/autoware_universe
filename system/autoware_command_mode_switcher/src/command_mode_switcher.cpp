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
#include <unordered_map>
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
      // TODO(Takagi, Isamu): Use data from diagnostics for transition conditions.
      iter->second->plugin->set_mode_continuable(item.available);
      iter->second->plugin->set_mode_available(item.available);
    }
  }
  is_ready_ = true;
  update();  // Reflect immediately.
}

void CommandModeSwitcher::on_request(const CommandModeRequest & msg)
{
  struct RequestTargets
  {
    bool success;
    std::shared_ptr<Command> control;
    std::shared_ptr<Command> vehicle;
  };

  const auto get_request_targets = [this](const CommandModeRequest & msg) {
    if (msg.type == CommandModeRequest::MANUAL) {
      return RequestTargets{true, nullptr, manual_command_};
    }

    const auto iter = autoware_commands_.find(msg.mode);
    if (iter == autoware_commands_.end()) {
      RCLCPP_ERROR_STREAM(get_logger(), "invalid mode: " << msg.mode);
      return RequestTargets{false, nullptr, nullptr};
    }
    const auto target = iter->second;
    const auto status = iter->second->status;

    // Check transition conditions.
    const auto transition_available = status.transition_available || !msg.ctrl;
    if (!status.mode_available || !transition_available) {
      RCLCPP_WARN_STREAM(get_logger(), "request rejected: " << msg.type << " " << msg.mode);
      return RequestTargets{false, nullptr, nullptr};
    }

    if (msg.type == CommandModeRequest::FOREGROUND) {
      return RequestTargets{true, target, target};
    }
    if (msg.type == CommandModeRequest::BACKGROUND) {
      return RequestTargets{true, target, nullptr};
    }

    RCLCPP_WARN_STREAM(get_logger(), "request rejected: " << msg.type << " " << msg.mode);
    return RequestTargets{false, nullptr, nullptr};
  };

  const auto targets = get_request_targets(msg);
  if (!targets.success) {
    return;
  }

  // Update command target.
  {
    const auto control_target = targets.control ? targets.control : control_gate_target_;
    const auto vehicle_target = targets.vehicle ? targets.vehicle : vehicle_gate_target_;
    const auto control_target_changed = control_target != control_gate_target_;
    const auto vehicle_target_changed = vehicle_target != vehicle_gate_target_;
    control_gate_target_ = control_target;
    vehicle_gate_target_ = vehicle_target;

    if (!control_target_changed && !vehicle_target_changed) {
      RCLCPP_INFO_STREAM(get_logger(), "request ignored: " << msg.type << " " << msg.mode);
      return;
    }
  }

  // Update request status.
  for (const auto & command : commands_) {
    command->status.control_gate_request = false;
    command->status.vehicle_gate_request = false;
  }
  if (control_gate_target_) {
    control_gate_target_->status.control_gate_request = true;
  }
  if (vehicle_gate_target_) {
    vehicle_gate_target_->status.vehicle_gate_request = true;
  }

  RCLCPP_INFO_STREAM(get_logger(), "request updated: " << msg.type << " " << msg.mode);
  update();
}

void CommandModeSwitcher::update()
{
  // TODO(Takagi, Isamu): Check call rate.
  if (!is_ready_) return;

  // NOTE: Update local states first since global states depend on them.
  for (const auto & command : commands_) {
    auto & status = command->status;
    auto & plugin = command->plugin;
    status.source_state = plugin->update_source_state(status.control_gate_request);
    status.control_gate_state = control_gate_interface_.get_state(*plugin);
    status.network_gate_state = NetworkGateState::Selected;
    status.vehicle_gate_state = vehicle_gate_interface_.get_state(*plugin);
    status.mode_continuable = plugin->get_mode_continuable();
    status.mode_available = plugin->get_mode_available();
    status.transition_available = plugin->get_transition_available();
    status.transition_completed = plugin->get_transition_completed();
    status.mrm = plugin->update_mrm_state();
  }

  std::unordered_map<std::string, int> source_group_count;
  for (const auto & command : commands_) {
    const auto uses = command->status.source_state != SourceState::Disabled;
    source_group_count[command->plugin->source_name()] += uses ? 1 : 0;
  }

  for (const auto & command : commands_) {
    auto & status = command->status;
    auto & plugin = command->plugin;
    const auto srouce_count = source_group_count[plugin->source_name()];
    status.source_group = srouce_count <= 1 ? SourceGroup::Exclusive : SourceGroup::Shared;
    status.state = update_main_state(status);
  }

  switch (request_.type) {
    case SwitcherRequestType::FOREGROUND:
      handle_all_gate_transition();
      break;
    case SwitcherRequestType::BACKGROUND:
      handle_control_gate_transition();
      break;
    case SwitcherRequestType::MANUAL:
      handle_vehicle_gate_transition();
      break;
  }

  publish_command_mode_status();
}

void CommandModeSwitcher::publish_command_mode_status()
{
  const auto convert = [](const Command & command) {
    CommandModeStatusItem item;
    item.mode = command.plugin->mode_name();
    item.state = convert_main_state(command.status.state);
    item.mrm = convert_mrm_state(command.status.mrm);
    item.mode_continuable = command.status.mode_continuable;
    item.mode_available = command.status.mode_available;
    item.control_gate_request = command.status.control_gate_request;
    item.vehicle_gate_request = command.status.vehicle_gate_request;
    item.transition_available = command.status.transition_available;
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

void CommandModeSwitcher::handle_all_gate_transition()
{
  if (!request_.command) {
    return;
  }

  if (!request_.command->is_control_gate_selected()) {
    control_gate_interface_.request(*request_.command->plugin, true);
    return;
  }

  if (!request_.command->is_vehicle_gate_selected()) {
    vehicle_gate_interface_.request(*request_.command->plugin);
    return;
  }

  // When both gate is selected, check the transition completion condition.
  if (!request_.command->status.transition_completed) {
    return;
  }

  if (control_gate_interface_.is_in_transition()) {
    control_gate_interface_.request(*request_.command->plugin, false);
  }
}

void CommandModeSwitcher::handle_vehicle_gate_transition()
{
  // Wait control gate transition.

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
