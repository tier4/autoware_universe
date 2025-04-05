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

#include "command_mode_conversion.hpp"

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::command_mode_decider
{

CommandModeDeciderBase::CommandModeDeciderBase(const rclcpp::NodeOptions & options)
: Node("command_mode_decider", options)
{
  request_timeout_ = declare_parameter<double>("request_timeout");
  manual_mode_name_ = "manual";

  is_modes_ready_ = false;
  command_mode_status_.init(declare_parameter<std::vector<std::string>>("command_modes"));

  const auto initial_operation_mode = declare_parameter<std::string>("initial_operation_mode");
  system_request_.autoware_control = true;
  system_request_.operation_mode = initial_operation_mode;
  system_request_.mrm = "";
  foreground_request_ = "";
  background_request_ = "";
  request_mode_ = "";
  current_mode_ = "";
  request_stamp_ = std::nullopt;

  using std::placeholders::_1;
  using std::placeholders::_2;

  // Interface with switcher nodes.
  pub_command_mode_request_ =
    create_publisher<CommandModeRequest>("~/command_mode/request", rclcpp::QoS(1));
  sub_command_mode_status_ = create_subscription<CommandModeStatus>(
    "~/command_mode/status", rclcpp::QoS(1).transient_local(),
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
  srv_mrm_request_ = create_service<RequestMrm>(
    "~/mrm/request", std::bind(&CommandModeDeciderBase::on_request_mrm, this, _1, _2));

  const auto period = rclcpp::Rate(declare_parameter<double>("update_rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void CommandModeDeciderBase::on_status(const CommandModeStatus & msg)
{
  // Update command mode status.
  for (const auto & item : msg.items) {
    command_mode_status_.set(item);
  }

  // Check if all command mode status items are ready.
  is_modes_ready_ = is_modes_ready_ ? true : command_mode_status_.ready();
  if (!is_modes_ready_) {
    return;
  }
  update();
}

void CommandModeDeciderBase::on_timer()
{
  if (!is_modes_ready_) {
    return;
  }

  if (request_stamp_) {
    const auto duration = (now() - *request_stamp_).seconds();
    if (request_timeout_ < duration) {
      request_stamp_ = std::nullopt;
      RCLCPP_WARN_STREAM(get_logger(), "command mode request timeout");
    }
  }

  update();
}

void CommandModeDeciderBase::update()
{
  // Note: is_modes_ready_ should be checked in the function that called this.
  // TODO(Takagi, Isamu): Check call rate.
  update_command_mode();
  sync_command_mode();
  publish_operation_mode_state();
  publish_mrm_state();
}

void CommandModeDeciderBase::update_command_mode()
{
  // Decide command mode with system-dependent logic.
  {
    const auto prev_mode = request_mode_;
    const auto next_mode = decide_command_mode();
    request_mode_ = next_mode;

    if (prev_mode != next_mode) {
      const auto prev_text = "'" + prev_mode + "'";
      const auto next_text = "'" + next_mode + "'";
      RCLCPP_INFO_STREAM(get_logger(), "mode changed: " << prev_text << " -> " << next_text);
    }
  }

  // Convert the request into internal structure.
  if (system_request_.autoware_control) {
    foreground_request_ = request_mode_;
    background_request_ = "";
  } else {
    foreground_request_ = manual_mode_name_;
    background_request_ = request_mode_;
  }
}

void CommandModeDeciderBase::sync_command_mode()
{
  bool foreground_request_reflected = foreground_request_.empty();
  bool background_request_reflected = background_request_.empty();

  if (!foreground_request_reflected) {
    const auto status = command_mode_status_.get(foreground_request_);
    foreground_request_reflected = status.control_gate_request && status.vehicle_gate_request;
  }
  if (!background_request_reflected) {
    const auto status = command_mode_status_.get(background_request_);
    background_request_reflected = status.control_gate_request;
  }

  // Skip the request if mode is already requested or now requesting.
  if (foreground_request_reflected && background_request_reflected) {
    request_stamp_ = std::nullopt;
    return;
  }
  if (request_stamp_) {
    return;
  }

  // Request stamp is used for timeout check and request flag.
  const auto stamp = now();
  request_stamp_ = stamp;

  // Request command mode to switcher nodes.
  CommandModeRequest msg;
  msg.stamp = stamp;
  msg.foreground = foreground_request_;
  msg.background = background_request_;
  pub_command_mode_request_->publish(msg);
}

bool operator!=(const OperatorStatus & s1, const OperatorStatus & s2)
{
  return s1.operation_mode != s2.operation_mode || s1.autoware_control != s2.autoware_control;
}

void CommandModeDeciderBase::publish_operation_mode_state()
{
  const auto is_transition_available = [this](const auto & mode) {
    const auto status = command_mode_status_.get(mode);
    return status.mode_available && status.transition_available;
  };
  OperationModeState state;
  state.stamp = now();
  state.mode = command_to_operation_mode(temporary_operator_.operation_mode);
  state.is_autoware_control_enabled = temporary_operator_.autoware_control;
  state.is_in_transition = (temporary_operator_ != confirmed_operator_);
  state.is_stop_mode_available = is_transition_available("stop");
  state.is_autonomous_mode_available = is_transition_available("autonomous");
  state.is_local_mode_available = is_transition_available("local");
  state.is_remote_mode_available = is_transition_available("remote");
  pub_operation_mode_->publish(state);
}

void CommandModeDeciderBase::publish_mrm_state()
{
  const auto convert = [](uint32_t item) {
    // clang-format off
    switch (item) {
      case CommandModeStatusItem::NORMAL:    return MrmState::NORMAL;
      case CommandModeStatusItem::OPERATING: return MrmState::MRM_OPERATING;
      case CommandModeStatusItem::SUCCEEDED: return MrmState::MRM_SUCCEEDED;
      case CommandModeStatusItem::FAILED:    return MrmState::MRM_FAILED;
      default:                               return MrmState::UNKNOWN;
    }
    // clang-format on
  };
  const auto status = command_mode_status_.get(current_mode_);
  MrmState state;
  state.stamp = now();
  state.state = convert(status.mrm);
  state.behavior = command_to_mrm_behavior(status.mode);
  pub_mrm_state_->publish(state);
}

ResponseStatus make_response(bool success, const std::string & message = "")
{
  ResponseStatus res;
  res.success = success;
  res.message = message;
  return res;
};

ResponseStatus CommandModeDeciderBase::check_mode_exists(const std::string & mode)
{
  if (!is_modes_ready_) {
    return make_response(false, "Mode management is not ready.");
  }
  if (command_mode_status_.get(mode).mode.empty()) {
    return make_response(false, "Invalid mode name: " + mode);
  }
  return make_response(true);
}

ResponseStatus CommandModeDeciderBase::check_mode_request(const std::string & mode, bool background)
{
  const auto result = check_mode_exists(mode);
  if (!result.success) {
    return result;
  }
  const auto status = command_mode_status_.get(mode);
  const auto available = status.mode_available && (status.transition_available || background);
  return available ? make_response(true) : make_response(false, "Mode is not available: " + mode);
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
  system_request_.autoware_control = req->autoware_control;
  update();
}

void CommandModeDeciderBase::on_change_operation_mode(
  ChangeOperationMode::Request::SharedPtr req, ChangeOperationMode::Response::SharedPtr res)
{
  const auto mode = operation_mode_to_command(req->mode);
  res->status = check_mode_request(mode, !system_request_.autoware_control);
  if (!res->status.success) {
    RCLCPP_WARN_STREAM(get_logger(), res->status.message);
    return;
  }
  system_request_.operation_mode = mode;
  update();
}

void CommandModeDeciderBase::on_request_mrm(
  RequestMrm::Request::SharedPtr req, RequestMrm::Response::SharedPtr res)
{
  res->status = check_mode_exists(req->name);
  if (!res->status.success) {
    RCLCPP_WARN_STREAM(get_logger(), res->status.message);
    return;
  }
  system_request_.mrm = req->name;
  update();
}

}  // namespace autoware::command_mode_decider
