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
  target_autoware_control_ = true;
  target_operation_mode_ = declare_parameter<std::string>("initial_operation_mode");
  target_mrm_ = std::string();
  curr_command_mode_ = std::string();
  command_mode_request_stamp_ = std::nullopt;

  const auto command_modes = declare_parameter<std::vector<std::string>>("command_modes");
  for (const auto & mode : command_modes) {
    // NOTE: The mode field will be used to check topic reception.
    command_mode_status_[mode] = CommandModeStatusItem();
  }

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
  srv_request_mrm_ = create_service<RequestMrm>(
    "~/mrm/request", std::bind(&CommandModeDeciderBase::on_request_mrm, this, _1, _2));

  const auto period = rclcpp::Rate(declare_parameter<double>("update_rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void CommandModeDeciderBase::on_status(const CommandModeStatus & msg)
{
  // Update command mode status.
  for (const auto & item : msg.items) {
    const auto iter = command_mode_status_.find(item.mode);
    if (iter == command_mode_status_.end()) continue;
    iter->second = item;
  }

  // Check if all command mode status items are ready.
  const auto check_ready = [this]() {
    for (const auto & [mode, status] : command_mode_status_) {
      if (status.mode.empty()) return false;
    }
    return true;
  };
  is_modes_ready_ = is_modes_ready_ ? true : check_ready();
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
  if (!is_modes_ready_) {
    return;
  }

  const auto stamp = now();

  // Decide target command mode.
  bool is_command_mode_changed = false;
  {
    const auto next_command_mode = decide_command_mode();
    is_command_mode_changed = curr_command_mode_ != next_command_mode;
    if (is_command_mode_changed) {
      const auto curr_text = "'" + curr_command_mode_ + "'";
      const auto next_text = "'" + next_command_mode + "'";
      RCLCPP_INFO_STREAM(
        get_logger(), "command mode changed: " << curr_text << " -> " << next_text);
    }
    curr_command_mode_ = next_command_mode;
  }

  // Request command mode to switcher nodes.
  if (is_command_mode_changed) {
    CommandModeRequest msg;
    msg.stamp = stamp;
    msg.mode = curr_command_mode_;
    msg.ctrl = target_autoware_control_;  // TODO(Takagi, Isamu): use current autoware control.
    pub_command_mode_request_->publish(msg);

    command_mode_request_stamp_ = stamp;
  }

  // Update operation mode status.
  const auto is_available = [this](const auto & mode) {
    const auto iter = command_mode_status_.find(mode);
    return iter == command_mode_status_.end() ? false : iter->second.available;
  };
  OperationModeState state;
  state.stamp = stamp;
  state.mode = text_to_mode(target_operation_mode_);
  state.is_autoware_control_enabled = true;  // TODO(Takagi, Isamu): subscribe
  state.is_in_transition = false;            // TODO(Takagi, Isamu): check status is enabled
  state.is_stop_mode_available = is_available("stop");
  state.is_autonomous_mode_available = is_available("autonomous");
  state.is_local_mode_available = is_available("local");
  state.is_remote_mode_available = is_available("remote");
  pub_operation_mode_->publish(state);
}

void CommandModeDeciderBase::on_change_operation_mode(
  ChangeOperationMode::Request::SharedPtr req, ChangeOperationMode::Response::SharedPtr res)
{
  // TODO(Takagi, Isamu): Commonize on_change_operation_mode and on_request_mrm.
  // TODO(Takagi, Isamu): Check is_modes_ready_.

  const auto mode = mode_to_text(req->mode);
  const auto iter = command_mode_status_.find(mode);
  if (iter == command_mode_status_.end()) {
    RCLCPP_WARN_STREAM(get_logger(), "invalid mode name: " << mode);
    res->status.success = false;
    res->status.message = "invalid mode name: " + mode;
    return;
  }

  const auto status = iter->second;
  if (!status.available) {
    RCLCPP_WARN_STREAM(get_logger(), "mode is not available: " << mode);
    res->status.success = false;
    res->status.message = "mode is not available: " + mode;
    return;
  }

  target_operation_mode_ = mode;
  res->status.success = true;

  update_command_mode();
}

void CommandModeDeciderBase::on_request_mrm(
  RequestMrm::Request::SharedPtr req, RequestMrm::Response::SharedPtr res)
{
  // TODO(Takagi, Isamu): Commonize on_change_operation_mode and on_request_mrm.
  // TODO(Takagi, Isamu): Check is_modes_ready_.

  const auto mode = req->name;
  const auto iter = command_mode_status_.find(mode);
  if (iter == command_mode_status_.end()) {
    RCLCPP_WARN_STREAM(get_logger(), "invalid mode name: " << mode);
    res->status.success = false;
    res->status.message = "invalid mode name: " + mode;
    return;
  }

  const auto status = iter->second;
  if (!status.available) {
    RCLCPP_WARN_STREAM(get_logger(), "mode is not available: " << mode);
    res->status.success = false;
    res->status.message = "mode is not available: " + mode;
    return;
  }

  target_mrm_ = mode;
  res->status.success = true;

  update_command_mode();
}

}  // namespace autoware::command_mode_decider
