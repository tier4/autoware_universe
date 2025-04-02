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

  is_modes_ready_ = false;
  command_mode_status_.init(declare_parameter<std::vector<std::string>>("command_modes"));
  command_mode_request_stamp_ = std::nullopt;

  const auto initial_operation_mode = declare_parameter<std::string>("initial_operation_mode");
  request_.autoware_control = true;
  request_.operation_mode = initial_operation_mode;
  request_.mrm = "";
  request_.command_mode = "";

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
  update_command_mode();
}

void CommandModeDeciderBase::on_timer()
{
  if (!is_modes_ready_) {
    return;
  }

  if (command_mode_request_stamp_) {
    const auto duration = (now() - *command_mode_request_stamp_).seconds();
    if (request_timeout_ < duration) {
      command_mode_request_stamp_ = std::nullopt;
      RCLCPP_WARN_STREAM(get_logger(), "command mode request timeout");
    }
  }

  update_command_mode();
}

void CommandModeDeciderBase::update_command_mode()
{
  // Note: is_modes_ready_ should be checked in the function that called this.
  // TODO(Takagi, Isamu): Check call rate.
  {
    const auto prev_mode = request_.command_mode;
    const auto next_mode = decide_command_mode();
    request_.command_mode = next_mode;

    if (prev_mode != next_mode) {
      const auto prev_text = "'" + prev_mode + "'";
      const auto next_text = "'" + next_mode + "'";
      RCLCPP_INFO_STREAM(get_logger(), "mode changed: " << prev_text << " -> " << next_text);
    }
  }

  sync_command_mode();

  publish_operation_mode_state();
  publish_mrm_state();
}

void CommandModeDeciderBase::sync_command_mode()
{
  const auto control_status = command_mode_status_.get("manual");
  const auto command_status = command_mode_status_.get(request_.command_mode);
  bool control_synced = false;
  bool command_synced = false;

  if (request_.autoware_control) {
    control_synced = control_status.target == CommandModeStatusItem::DISABLED;
    command_synced = command_status.target == CommandModeStatusItem::ENABLED;
  } else {
    control_synced = control_status.target == CommandModeStatusItem::ENABLED;
    command_synced = command_status.target == CommandModeStatusItem::STANDBY;
  }

  // Skip the request if mode is synced or is requested.
  if (control_synced && command_synced) {
    command_mode_request_stamp_ = std::nullopt;
    return;
  }
  if (command_mode_request_stamp_) {
    return;
  }

  // Request stamp is used for timeout check and request flag.
  const auto stamp = now();
  command_mode_request_stamp_ = stamp;

  // Request command mode to switcher nodes.
  CommandModeRequest msg;
  msg.stamp = stamp;
  msg.ctrl = request_.autoware_control;
  msg.mode = request_.command_mode;
  pub_command_mode_request_->publish(msg);
}

void CommandModeDeciderBase::publish_operation_mode_state()
{
  const auto is_transition_available = [this](const auto & mode) {
    const auto status = command_mode_status_.get(mode);
    return status.mode_available && status.ctrl_available;
  };
  OperationModeState state;
  state.stamp = now();
  state.mode = command_to_operation_mode(request_.operation_mode);  // TODO(Takagi, Isamu): check
  state.is_autoware_control_enabled = request_.autoware_control;    // TODO(Takagi, Isamu): check
  state.is_in_transition = command_mode_request_stamp_.has_value();
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
      case CommandModeStatusItem::NONE:      return MrmState::NONE;
      case CommandModeStatusItem::OPERATING: return MrmState::MRM_OPERATING;
      case CommandModeStatusItem::SUCCEEDED: return MrmState::MRM_SUCCEEDED;
      case CommandModeStatusItem::FAILED:    return MrmState::MRM_FAILED;
      default:                               return MrmState::UNKNOWN;
    }
    // clang-format on
  };
  const auto status = command_mode_status_.get(request_.command_mode);
  MrmState state;
  state.stamp = now();
  state.state = convert(status.mrm);
  state.behavior = command_to_mrm_behavior(request_.command_mode);
  pub_mrm_state_->publish(state);
}

ResponseStatus CommandModeDeciderBase::check_request(
  const std::string & mode, bool check_mode_ready, bool check_ctrl_ready)
{
  const auto response = [](bool success, const std::string & message) {
    ResponseStatus res;
    res.success = success;
    res.message = message;
    return res;
  };

  if (!is_modes_ready_) {
    return response(false, "Mode management is not ready.");
  }

  const auto item = command_mode_status_.get(mode);
  if (item.mode.empty()) {
    return response(false, "Invalid mode name: " + mode);
  }

  const auto mode_available = item.mode_available || (!check_mode_ready);
  const auto ctrl_available = item.ctrl_available || (!check_ctrl_ready);
  if (!mode_available || !ctrl_available) {
    return response(false, "Mode is not available: " + mode);
  }

  return response(true, "");
}

void CommandModeDeciderBase::on_change_autoware_control(
  ChangeAutowareControl::Request::SharedPtr req, ChangeAutowareControl::Response::SharedPtr res)
{
  const auto mode = req->autoware_control ? request_.operation_mode : "manual";

  res->status = check_request(mode, true, true);
  if (!res->status.success) {
    RCLCPP_WARN_STREAM(get_logger(), res->status.message);
    return;
  }
  request_.autoware_control = req->autoware_control;
  update_command_mode();
}

void CommandModeDeciderBase::on_change_operation_mode(
  ChangeOperationMode::Request::SharedPtr req, ChangeOperationMode::Response::SharedPtr res)
{
  const auto mode = operation_mode_to_command(req->mode);

  res->status = check_request(mode, true, request_.autoware_control);
  if (!res->status.success) {
    RCLCPP_WARN_STREAM(get_logger(), res->status.message);
    return;
  }
  request_.operation_mode = mode;
  update_command_mode();
}

void CommandModeDeciderBase::on_request_mrm(
  RequestMrm::Request::SharedPtr req, RequestMrm::Response::SharedPtr res)
{
  const auto mode = req->name;

  res->status = check_request(mode, false, false);
  if (!res->status.success) {
    RCLCPP_WARN_STREAM(get_logger(), res->status.message);
    return;
  }
  request_.mrm = mode;
  update_command_mode();
}

}  // namespace autoware::command_mode_decider
