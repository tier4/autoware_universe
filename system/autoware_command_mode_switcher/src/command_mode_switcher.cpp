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

#include <autoware_command_mode_types/adapters/command_mode_status.hpp>

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
    bool is_main_ecu = declare_parameter<bool>("is_main_ecu", true);
    if (is_main_ecu) {
      platform_commands_[command->plugin->mode_name()] = command;
      commands_.push_back(command);
    }
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
      autoware_commands_[command->plugin->mode_name()] = command;
      commands_.push_back(command);
    }
  }

  // Initialize all switchers. Call "construct" first, which acts as the base class constructor.
  for (const auto & command : commands_) {
    command->plugin->construct(this);
    command->plugin->initialize();
  }

  pub_status_ = create_publisher<CommandModeStatusAdapter>(
    "~/command_mode/status", rclcpp::QoS(1).transient_local());
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
  const auto get_command = [this](const std::string & mode, bool background) {
    // Platform commands are only available in foreground.
    if (!background) {
      const auto iter = platform_commands_.find(mode);
      if (iter != platform_commands_.end()) {
        return iter->second;
      }
    }
    // Autoware commands are available in both foreground and background.
    const auto iter = autoware_commands_.find(mode);
    if (iter == autoware_commands_.end()) {
      RCLCPP_ERROR_STREAM(get_logger(), "invalid mode: " << mode << " " << background);
      return std::shared_ptr<Command>(nullptr);
    }
    // Check transition conditions.
    const auto status = iter->second->status;
    const auto available = status.mode_available && (status.transition_available || background);
    if (!available) {
      RCLCPP_ERROR_STREAM(get_logger(), "unavailable mode: " << mode << " " << background);
      return std::shared_ptr<Command>(nullptr);
    }
    return iter->second;
  };

  // Update command target.
  const auto foreground = msg.foreground.empty() ? nullptr : get_command(msg.foreground, false);
  const auto background = msg.background.empty() ? nullptr : get_command(msg.background, true);
  {
    const auto foreground_changed = foreground != foreground_;
    const auto background_changed = background != background_;
    foreground_ = foreground;
    background_ = background;
    if (!foreground_changed && !background_changed) return;
  }

  // Update request status.
  for (const auto & command : commands_) {
    command->status.request_phase = GateType::NotSelected;
    command->status.transition_state = TriState::Disabled;
  }
  if (foreground_) {
    foreground_->status.request_phase = GateType::VehicleGate;
    foreground_->status.transition_state = TriState::Transition;
  }
  if (background_) {
    background_->status.request_phase = GateType::ControlGate;
    background_->status.transition_state = TriState::Transition;
  }
  RCLCPP_INFO_STREAM(get_logger(), "request updated: " << msg.foreground << " " << msg.background);
  update();
}

void CommandModeSwitcher::update()
{
  // TODO(Takagi, Isamu): Check call rate.
  if (!is_ready_) return;

  detect_override();
  update_status();
  handle_foreground_transition();
  handle_background_transition();
  publish_command_mode_status();
}

void CommandModeSwitcher::detect_override()
{
  const auto curr_autoware_control = vehicle_gate_interface_.is_autoware_control();
  const auto is_changed_to_manual = (is_autoware_control_ && !curr_autoware_control);
  is_autoware_control_ = curr_autoware_control;

  if (!is_changed_to_manual) return;
  if (foreground_ == manual_command_) return;
  RCLCPP_WARN_STREAM(get_logger(), "override detected");

  // Move foreground to background.
  if (foreground_) {
    background_ = foreground_;
    background_->status.request_phase = GateType::ControlGate;
    background_->status.transition_state = TriState::Transition;
  }
  foreground_ = manual_command_;
  foreground_->status.request_phase = GateType::VehicleGate;
  foreground_->status.transition_state = TriState::Transition;
}

void CommandModeSwitcher::update_status()
{
  // NOTE: Update the source state first since the source group state depends on it.
  const auto to_tri_state = [](bool state) {
    return state ? TriState::Enabled : TriState::Disabled;
  };
  for (const auto & command : commands_) {
    auto & status = command->status;
    auto & plugin = command->plugin;
    status.mrm = plugin->update_mrm_state();
    status.source_state =
      plugin->update_source_state(status.request_phase != GateType::NotSelected);
    status.control_gate_state = to_tri_state(control_gate_interface_.is_selected(*plugin));
    status.vehicle_gate_state = to_tri_state(vehicle_gate_interface_.is_selected(*plugin));
    status.mode_continuable = plugin->get_mode_continuable();
    status.mode_available = plugin->get_mode_available();
    status.transition_available = plugin->get_transition_available();
    status.transition_completed = plugin->get_transition_completed();
  }

  // Within the source group, all sources except for the active must be disabled.
  std::unordered_map<std::string, int> source_group_count;
  for (const auto & command : commands_) {
    const auto uses = command->status.source_state != TriState::Disabled;
    source_group_count[command->plugin->source_name()] += uses ? 1 : 0;
  }
  for (const auto & command : commands_) {
    auto & status = command->status;
    auto & plugin = command->plugin;
    const auto source_count = source_group_count[plugin->source_name()];
    status.source_group = source_count <= 1 ? TriState::Enabled : TriState::Disabled;
    status.current_phase = update_current_phase(status);  // Depends on the source group.
    status.gate_state = update_gate_state(status);        // Depends on the source group.
    status.mode_state = update_mode_state(status);        // Depends on the gate state.
  }
}

void CommandModeSwitcher::publish_command_mode_status()
{
  CommandModeStatus msg;
  msg.stamp = now();
  for (const auto & command : commands_) {
    msg.items.push_back(command->status);
  }
  pub_status_->publish(msg);
}

void CommandModeSwitcher::handle_foreground_transition()
{
  if (!foreground_) {
    return;
  }

  // Disable control gate transition filter when the transition is completed.
  if (foreground_->status.transition_state == TriState::Enabled) {
    if (control_gate_interface_.is_in_transition()) {
      control_gate_interface_.request(*foreground_->plugin, false);
    }
    return;
  }

  // Select control gate with transition filter.
  if (foreground_->status.control_gate_state != TriState::Enabled) {
    control_gate_interface_.request(*foreground_->plugin, true);
    return;
  }

  // Select vehicle gate when the control gate is selected.
  if (foreground_->status.vehicle_gate_state != TriState::Enabled) {
    vehicle_gate_interface_.request(*foreground_->plugin);
    return;
  }

  // When both gate is selected, check the transition completion condition.
  // NOTE(Takagi, Isamu): Use transition_state to commonize background transition?
  if (!foreground_->status.transition_completed) {
    return;
  }

  // Complete transition.
  foreground_->status.transition_state = TriState::Enabled;
}

void CommandModeSwitcher::handle_background_transition()
{
  if (!background_ || !foreground_) {
    return;
  }

  if (background_->status.transition_state == TriState::Enabled) {
    return;
  }

  // Wait for the foreground transition so that the command does not affect the vehicle behavior.
  if (foreground_->status.transition_state != TriState::Enabled) {
    return;
  }

  // Select control gate without transition filter.
  if (background_->status.control_gate_state != TriState::Enabled) {
    control_gate_interface_.request(*background_->plugin, false);
    return;
  }

  // Complete transition, No need to check the transition completion condition.
  background_->status.transition_state = TriState::Enabled;
}

}  // namespace autoware::command_mode_switcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::command_mode_switcher::CommandModeSwitcher)
