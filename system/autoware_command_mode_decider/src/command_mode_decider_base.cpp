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

#include "command_mode_decider_base.hpp"

#include <autoware_command_mode_types/constants/modes.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::command_mode_decider
{

void logging_mode_change(
  const rclcpp::Logger & logger, const std::string & name, uint16_t prev, uint16_t next)
{
  if (prev != next) {
    const auto prev_text = std::to_string(prev);
    const auto next_text = std::to_string(next);
    RCLCPP_INFO_STREAM(logger, name << " mode changed: " << prev_text << " -> " << next_text);
  }
}

std::string text(const std::vector<uint16_t> & modes)
{
  std::string result;
  for (size_t i = 0; i < modes.size(); ++i) {
    result += (i ? ", " : "") + std::to_string(modes.at(i));
  }
  return "[" + result + "]";
}

CommandModeDeciderBase::CommandModeDeciderBase(const rclcpp::NodeOptions & options)
: Node("command_mode_decider", options),
  diagnostics_(this),
  loader_("autoware_command_mode_decider", "autoware::command_mode_decider::DeciderPlugin")
{
  diagnostics_.setHardwareID("none");
  diagnostics_.add("ready", this, &CommandModeDeciderBase::on_diagnostics);

  const auto plugin_name = declare_parameter<std::string>("plugin_name");
  if (!loader_.isClassAvailable(plugin_name)) {
    throw std::invalid_argument("unknown plugin: " + plugin_name);
  }
  plugin_ = loader_.createSharedInstance(plugin_name);
  plugin_->construct(this);

  transition_timeout_ = declare_parameter<double>("transition_timeout");
  request_timeout_ = declare_parameter<double>("request_timeout");

  // Note: declare_parameter does not support std::vector<uint16_t>
  std::vector<uint16_t> command_modes;
  for (const auto & mode : declare_parameter<std::vector<int64_t>>("command_modes")) {
    command_modes.push_back(static_cast<uint16_t>(mode));
  }
  is_modes_ready_ = false;
  command_mode_status_.init(command_modes);
  system_request_.operation_mode = declare_parameter<uint16_t>("initial_operation_mode");
  system_request_.autoware_control = declare_parameter<bool>("initial_autoware_control");
  command_mode_ = autoware::command_mode_types::modes::unknown;
  request_mode_ = autoware::command_mode_types::modes::unknown;
  request_stamp_ = std::nullopt;

  curr_autoware_control_ = false;
  last_autoware_control_ = false;
  curr_operation_mode_ = autoware::command_mode_types::modes::unknown;
  last_operation_mode_ = autoware::command_mode_types::modes::unknown;

  using std::placeholders::_1;
  using std::placeholders::_2;

  // Interface with switcher nodes.
  pub_command_mode_request_ =
    create_publisher<CommandModeRequest>("~/command_mode/request", rclcpp::QoS(1));
  sub_command_mode_status_ = create_subscription<CommandModeStatusAdapter>(
    "~/command_mode/status", rclcpp::QoS(50).transient_local(),
    std::bind(&CommandModeDeciderBase::on_status, this, std::placeholders::_1));

  // Interface for API.
  pub_operation_mode_ = create_publisher<OperationModeState>(
    "~/operation_mode/state", rclcpp::QoS(1).transient_local());
  srv_operation_mode_ = create_service<ChangeOperationMode>(
    "~/operation_mode/change_operation_mode",
    std::bind(&CommandModeDeciderBase::on_change_operation_mode, this, _1, _2));
  srv_autoware_control_ = create_service<ChangeAutowareControl>(
    "~/operation_mode/change_autoware_control",
    std::bind(&CommandModeDeciderBase::on_change_autoware_control, this, _1, _2));

  pub_mrm_state_ = create_publisher<MrmState>("~/mrm/state", rclcpp::QoS(1));

  const auto period = rclcpp::Rate(declare_parameter<double>("update_rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

bool CommandModeDeciderBase::is_in_transition() const
{
  if (system_request_.autoware_control) {
    return last_operation_mode_ != system_request_.operation_mode;
  } else {
    return last_autoware_control_ != system_request_.autoware_control;
  }
}

void CommandModeDeciderBase::on_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  std::vector<uint16_t> waiting_modes;
  for (const auto & [mode, item] : command_mode_status_) {
    if (item.mode == autoware::command_mode_types::modes::unknown) {
      waiting_modes.push_back(mode);
    }
  }

  if (waiting_modes.empty()) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
    return;
  }

  std::string message = "Waiting for mode status:";
  for (const auto & mode : waiting_modes) {
    message += " " + std::to_string(mode);
  }
  status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
}

void CommandModeDeciderBase::on_timer()
{
  if (!is_modes_ready_) {
    return;
  }
  command_mode_status_.check_timeout(now());
  update();
}

void CommandModeDeciderBase::on_status(const CommandModeStatus & msg)
{
  // Update command mode status.
  for (const auto & item : msg.items) {
    command_mode_status_.set(item, msg.stamp);
  }

  // Check if all command mode status items are ready.
  is_modes_ready_ = is_modes_ready_ ? true : command_mode_status_.ready();
  if (!is_modes_ready_) {
    return;
  }
  update();
}

void CommandModeDeciderBase::update()
{
  // Note: is_modes_ready_ should be checked in the function that called this.
  // TODO(Takagi, Isamu): Check call rate.
  detect_override();
  detect_operation_mode_timeout();
  update_request_mode();
  update_current_mode();
  sync_command_mode();
  publish_operation_mode_state();
  publish_mrm_state();
}

void CommandModeDeciderBase::detect_override()
{
  // if (foreground_request_ == autoware::command_mode_types::modes::manual) return;

  const auto status = command_mode_status_.get(autoware::command_mode_types::modes::manual);
  if (status.request_phase == GateType::VehicleGate) {
    RCLCPP_WARN_STREAM(get_logger(), "override detected");
    // foreground_request_ = autoware::command_mode_types::modes::manual;
    system_request_.autoware_control = false;
  }
}

void CommandModeDeciderBase::detect_operation_mode_timeout()
{
  if (!is_in_transition()) {
    transition_stamp_ = std::nullopt;
    return;
  }
  if (!transition_stamp_) {
    transition_stamp_ = now();
  }

  const auto duration = (now() - *transition_stamp_).seconds();
  if (duration < transition_timeout_) {
    return;
  }

  // Rollback to the last operation mode.
  if (last_autoware_control_) {
    system_request_.operation_mode = last_operation_mode_;
    system_request_.autoware_control = true;
  } else {
    // Keep the operation mode.
    system_request_.autoware_control = false;
  }
  RCLCPP_INFO_STREAM(get_logger(), "Mode transition is canceled due to timeout.");
}

void CommandModeDeciderBase::update_request_mode()
{
  // Decide command mode with system-dependent logic.
  {
    const auto modes = plugin_->decide(system_request_, command_mode_status_);
    if (modes.size() == 0) {
      RCLCPP_WARN_STREAM(get_logger(), "No command mode is decided.");
    }
    if (request_modes_ != modes) {
      const auto prev = text(request_modes_);
      const auto next = text(modes);
      RCLCPP_INFO_STREAM(get_logger(), "request mode changed: " << prev << " -> " << next);
    }
    request_modes_ = modes;
    request_autoware_control_ = system_request_.autoware_control;
  }
}

void CommandModeDeciderBase::update_current_mode()
{
  // Update current command mode.
  for (const auto & [mode, status] : command_mode_status_) {
    if (status.check_gate_ready(GateType::VehicleGate)) {
      logging_mode_change(get_logger(), "command", command_mode_, mode);
      command_mode_ = mode;
      break;
    }
  }

  // Update current operation mode
  for (const auto & [mode, status] : command_mode_status_) {
    if (plugin_->to_operation_mode(mode) == OperationModeState::UNKNOWN) {
      continue;
    }
    if (status.check_gate_ready(GateType::ControlGate)) {
      curr_operation_mode_ = mode;
      break;
    }
  }

  // Update current manual control.
  {
    const auto status = command_mode_status_.get(autoware::command_mode_types::modes::manual);
    curr_autoware_control_ = !status.check_gate_ready(GateType::VehicleGate);
  }

  // Update last operator (manual control or autoware operation mode).
  if (is_in_transition()) {
    if (system_request_.autoware_control) {
      const auto status = command_mode_status_.get(system_request_.operation_mode);
      if (status.check_mode_ready()) {
        last_autoware_control_ = true;
        last_operation_mode_ = system_request_.operation_mode;
        RCLCPP_INFO_STREAM(get_logger(), "last mode: " << last_operation_mode_);
      }
    } else {
      const auto status = command_mode_status_.get(autoware::command_mode_types::modes::manual);
      if (status.check_mode_ready()) {
        last_autoware_control_ = false;
        last_operation_mode_ = autoware::command_mode_types::modes::unknown;
        RCLCPP_INFO_STREAM(get_logger(), "last mode: manual");
      }
    }
  }
}

void CommandModeDeciderBase::sync_command_mode()
{
  // Skip the request if mode is already requested.
  bool is_requested = true;
  for (const auto & mode : request_modes_) {
    const auto status = command_mode_status_.get(mode);
    is_requested = is_requested && status.request;
  }
  if (is_requested) {
    request_stamp_ = std::nullopt;
    return;
  }

  // Skip the request if mode is now requesting.
  if (request_stamp_) {
    const auto duration = (now() - *request_stamp_).seconds();
    if (duration < request_timeout_) {
      return;
    }
    request_stamp_ = std::nullopt;
    RCLCPP_WARN_STREAM(get_logger(), "request mode timeout");
  }

  // Request stamp is used to check timeout and requesting.
  const auto stamp = now();
  request_stamp_ = stamp;

  // Request command mode to switcher nodes.
  CommandModeRequest msg;
  msg.stamp = stamp;
  for (const auto & mode : request_modes_) {
    using Item = CommandModeRequestItem;
    Item item;
    item.type = Item::COMMAND_MODE_CHANGE;
    item.command = mode;
    item.vehicle = request_autoware_control_ ? Item::AUTOWARE : Item::MANUAL;
    msg.items.push_back(item);
  }
  pub_command_mode_request_->publish(msg);
}

void CommandModeDeciderBase::publish_operation_mode_state()
{
  const auto is_transition_available = [this](const auto & mode) {
    const auto status = command_mode_status_.get(mode);
    return status.mode_available && status.transition_available;
  };
  namespace modes = autoware::command_mode_types::modes;
  OperationModeState state;
  state.stamp = now();
  state.mode = plugin_->to_operation_mode(curr_operation_mode_);
  state.is_autoware_control_enabled = curr_autoware_control_;
  state.is_in_transition = is_in_transition();
  state.is_stop_mode_available = is_transition_available(modes::stop);
  state.is_autonomous_mode_available = is_transition_available(modes::autonomous);
  state.is_local_mode_available = is_transition_available(modes::local);
  state.is_remote_mode_available = is_transition_available(modes::remote);
  pub_operation_mode_->publish(state);
}

void CommandModeDeciderBase::publish_mrm_state()
{
  using CommandModeMrmState = autoware::command_mode_types::MrmState;
  const auto convert = [](const CommandModeMrmState state) {
    // clang-format off
    switch (state) {
      case CommandModeMrmState::Normal:    return MrmState::NORMAL;
      case CommandModeMrmState::Operating: return MrmState::MRM_OPERATING;
      case CommandModeMrmState::Succeeded: return MrmState::MRM_SUCCEEDED;
      case CommandModeMrmState::Failed:    return MrmState::MRM_FAILED;
      default:                             return MrmState::UNKNOWN;
    }
    // clang-format on
  };
  const auto status = command_mode_status_.get(command_mode_);
  MrmState state;
  state.stamp = now();
  state.state = convert(status.mrm);
  state.behavior = plugin_->to_mrm_behavior(status.mode);
  pub_mrm_state_->publish(state);
}

ResponseStatus make_response(bool success, const std::string & message = "")
{
  ResponseStatus res;
  res.success = success;
  res.message = message;
  return res;
};

ResponseStatus CommandModeDeciderBase::check_mode_exists(uint16_t mode)
{
  if (!is_modes_ready_) {
    return make_response(false, "Mode management is not ready.");
  }
  if (command_mode_status_.get(mode).mode == autoware::command_mode_types::modes::unknown) {
    return make_response(false, "Invalid mode name: " + std::to_string(mode));
  }
  return make_response(true);
}

ResponseStatus CommandModeDeciderBase::check_mode_request(uint16_t mode, bool background)
{
  const auto result = check_mode_exists(mode);
  if (!result.success) {
    return result;
  }
  const auto status = command_mode_status_.get(mode);
  const auto available = status.mode_available && (status.transition_available || background);
  if (!available) {
    make_response(false, "Mode is not available: " + std::to_string(mode));
  }
  return make_response(true);
}

void CommandModeDeciderBase::on_change_autoware_control(
  ChangeAutowareControl::Request::SharedPtr req, ChangeAutowareControl::Response::SharedPtr res)
{
  // Assume the driver is always ready.
  if (req->autoware_control) {
    res->status = check_mode_request(system_request_.operation_mode, false);
    if (!res->status.success) {
      RCLCPP_WARN_STREAM(get_logger(), res->status.message);
      return;
    }
  }
  res->status = make_response(true);  // For autoware_control is false.
  system_request_.autoware_control = req->autoware_control;
  update();
}

void CommandModeDeciderBase::on_change_operation_mode(
  ChangeOperationMode::Request::SharedPtr req, ChangeOperationMode::Response::SharedPtr res)
{
  const auto mode = plugin_->from_operation_mode(req->mode);
  res->status = check_mode_request(mode, !system_request_.autoware_control);
  if (!res->status.success) {
    RCLCPP_WARN_STREAM(get_logger(), res->status.message);
    return;
  }
  system_request_.operation_mode = mode;
  update();
}

}  // namespace autoware::command_mode_decider

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::command_mode_decider::CommandModeDeciderBase)
