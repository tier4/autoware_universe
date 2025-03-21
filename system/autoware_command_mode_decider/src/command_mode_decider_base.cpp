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
  is_modes_ready_ = false;
  command_mode_status_.init(declare_parameter<std::vector<std::string>>("command_modes"));
  command_mode_request_stamp_ = std::nullopt;

  request_.autoware_control = true;
  request_.operation_mode = declare_parameter<std::string>("initial_operation_mode");
  request_.mrm = std::string();

  decided_.autoware_control = true;
  decided_.command_mode = std::string();

  current_.autoware_control = true;
  current_.command_mode = std::string();

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

  /*
  const auto & status = command_mode_status_.at(curr_command_mode_);
  (void)status;
  */

  /*
  if (mode.status.activation) {
    command_mode_request_stamp_ = std::nullopt;
    return;
  }
  */

  if (!command_mode_request_stamp_) {
    return;
  }

  /*
  const auto duration = (now() - *command_mode_request_stamp_).seconds();
  RCLCPP_INFO_STREAM(get_logger(), "time: " << duration);
  */
}

void CommandModeDeciderBase::update_command_mode()
{
  // Note: is_modes_ready_ should be checked in the function that called this.

  const auto previous = decided_;
  {
    const auto next_mode = decide_command_mode();
    const auto curr_mode = decided_.command_mode;
    if (curr_mode != next_mode) {
      const auto curr_text = "'" + curr_mode + "'";
      const auto next_text = "'" + next_mode + "'";
      RCLCPP_INFO_STREAM(get_logger(), "mode changed: " << curr_text << " -> " << next_text);
    }
    decided_.command_mode = next_mode;
    decided_.autoware_control = request_.autoware_control;
  }

  const auto stamp = now();

  // Request command mode to switcher nodes.
  if (previous != decided_) {
    CommandModeRequest msg;
    msg.stamp = stamp;
    msg.ctrl = decided_.autoware_control;
    msg.mode = decided_.command_mode;
    pub_command_mode_request_->publish(msg);

    command_mode_request_stamp_ = stamp;
  }

  // Update operation mode status.
  const auto is_available = [this](const auto & mode) {
    return command_mode_status_.get(mode).available;
  };
  OperationModeState state;
  state.stamp = stamp;
  state.mode = text_to_mode(decided_.command_mode);
  state.is_autoware_control_enabled = true;  // TODO(Takagi, Isamu): subscribe
  state.is_in_transition = false;            // TODO(Takagi, Isamu): check status is enabled
  state.is_stop_mode_available = is_available("stop");
  state.is_autonomous_mode_available = is_available("autonomous");
  state.is_local_mode_available = is_available("local");
  state.is_remote_mode_available = is_available("remote");
  pub_operation_mode_->publish(state);
}

template <class T>
bool on_change_mode(T & res, const CommandModeStatusWrapper & wrapper, const std::string & mode)
{
  const auto item = wrapper.get(mode);
  if (item.mode.empty()) {
    res->status.success = false;
    res->status.message = "invalid mode name: " + mode;
    return false;
  }
  if (!item.available) {
    res->status.success = false;
    res->status.message = "mode is not available: " + mode;
    return false;
  }
  res->status.success = true;
  return true;
}

void CommandModeDeciderBase::on_change_autoware_control(
  ChangeAutowareControl::Request::SharedPtr req, ChangeAutowareControl::Response::SharedPtr res)
{
  if (!is_modes_ready_) {
    res->status.success = false;
    res->status.message = "Mode management is not ready.";
    return;
  }

  const auto mode = req->autoware_control ? request_.operation_mode : "manual";
  if (!on_change_mode(res, command_mode_status_, mode)) {
    RCLCPP_WARN_STREAM(get_logger(), res->status.message);
    return;
  }

  request_.autoware_control = req->autoware_control;
  update_command_mode();
}

void CommandModeDeciderBase::on_change_operation_mode(
  ChangeOperationMode::Request::SharedPtr req, ChangeOperationMode::Response::SharedPtr res)
{
  if (!is_modes_ready_) {
    res->status.success = false;
    res->status.message = "Mode management is not ready.";
    return;
  }

  const auto mode = mode_to_text(req->mode);
  if (!on_change_mode(res, command_mode_status_, mode)) {
    RCLCPP_WARN_STREAM(get_logger(), res->status.message);
    return;
  }

  request_.operation_mode = mode;
  update_command_mode();
}

void CommandModeDeciderBase::on_request_mrm(
  RequestMrm::Request::SharedPtr req, RequestMrm::Response::SharedPtr res)
{
  if (!is_modes_ready_) {
    res->status.success = false;
    res->status.message = "Mode management is not ready.";
    return;
  }

  const auto mode = req->name;
  if (!on_change_mode(res, command_mode_status_, mode)) {
    RCLCPP_WARN_STREAM(get_logger(), res->status.message);
    return;
  }

  request_.mrm = mode;
  update_command_mode();
}

}  // namespace autoware::command_mode_decider
