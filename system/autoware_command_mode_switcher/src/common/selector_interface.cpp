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

#include "selector_interface.hpp"

#include <memory>
#include <string>
#include <utility>

namespace autoware::command_mode_switcher
{

SelectorInterface::SelectorInterface(rclcpp::Node & node, Callback callback) : node_(node)
{
  group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  notification_callback_ = callback;

  cli_source_select_ = node.create_client<SelectCommandSource>(
    "~/source/select", rmw_qos_profile_services_default, group_);

  sub_source_status_ = node.create_subscription<CommandSourceStatus>(
    "~/source/status", rclcpp::QoS(1).transient_local(),
    std::bind(&SelectorInterface::on_source_status, this, std::placeholders::_1));

  cli_control_mode_ = node.create_client<ControlModeCommand>(
    "~/control_mode/request", rmw_qos_profile_services_default, group_);

  sub_control_mode_ = node.create_subscription<ControlModeReport>(
    "~/control_mode/report", rclcpp::QoS(1),
    std::bind(&SelectorInterface::on_control_mode, this, std::placeholders::_1));
}

std::optional<bool> SelectorInterface::autoware_control() const
{
  if (control_mode_.mode == ControlModeReport::AUTONOMOUS) {
    return true;
  }
  if (control_mode_.mode == ControlModeReport::MANUAL) {
    return false;
  }
  return std::nullopt;
}

template <class T>
bool equals_except_stamp(const T & msg1, const T & msg2)
{
  T t1 = msg1;
  T t2 = msg2;
  t1.stamp = t2.stamp = rclcpp::Time();
  return t1 == t2;
}

void SelectorInterface::on_source_status(const CommandSourceStatus & msg)
{
  const auto equals = equals_except_stamp(source_status_, msg);
  source_status_ = msg;
  if (!equals) notification_callback_();
}

void SelectorInterface::on_control_mode(const ControlModeReport & msg)
{
  const auto equals = equals_except_stamp(control_mode_, msg);
  control_mode_ = msg;
  if (!equals) notification_callback_();
}

bool SelectorInterface::select_source(const std::string & source)
{
  if (this->source_name() == source) {
    return false;
  }
  if (waiting_source_select_) {
    return false;
  }
  if (!cli_source_select_->service_is_ready()) {
    return false;
  }

  using SharedFuture = rclcpp::Client<SelectCommandSource>::SharedFuture;
  auto request = std::make_shared<SelectCommandSource::Request>();
  request->source = source;

  waiting_source_select_ = true;
  cli_source_select_->async_send_request(
    request, [this](SharedFuture) { waiting_source_select_ = false; });
  return true;
}

bool SelectorInterface::select_control(const bool autoware_control)
{
  if (this->autoware_control() == autoware_control) {
    return false;
  }
  if (waiting_control_mode_) {
    return false;
  }
  if (!cli_control_mode_->service_is_ready()) {
    return false;
  }

  using SharedFuture = rclcpp::Client<ControlModeCommand>::SharedFuture;
  auto request = std::make_shared<ControlModeCommand::Request>();
  if (autoware_control) {
    request->mode = ControlModeCommand::Request::AUTONOMOUS;
  } else {
    request->mode = ControlModeCommand::Request::MANUAL;
  }

  waiting_control_mode_ = true;
  cli_control_mode_->async_send_request(
    request, [this](SharedFuture) { waiting_control_mode_ = false; });
  return true;
}

}  // namespace autoware::command_mode_switcher
