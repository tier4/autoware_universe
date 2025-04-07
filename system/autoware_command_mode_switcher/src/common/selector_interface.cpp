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

ControlGateInterface::ControlGateInterface(rclcpp::Node & node, Callback callback) : node_(node)
{
  notification_callback_ = callback;
  status_.source = "";

  cli_source_select_ = node.create_client<SelectCommandSource>("~/source/select");
  sub_source_status_ = node.create_subscription<CommandSourceStatus>(
    "~/source/status", rclcpp::QoS(1).transient_local(),
    std::bind(&ControlGateInterface::on_source_status, this, std::placeholders::_1));
}

VehicleGateInterface::VehicleGateInterface(rclcpp::Node & node, Callback callback) : node_(node)
{
  notification_callback_ = callback;
  status_.mode = ControlModeReport::NO_COMMAND;

  cli_control_mode_ = node.create_client<ControlModeCommand>("~/control_mode/request");
  sub_control_mode_ = node.create_subscription<ControlModeReport>(
    "~/control_mode/report", rclcpp::QoS(1),
    std::bind(&VehicleGateInterface::on_control_mode, this, std::placeholders::_1));
}

template <class T>
bool equals_except_stamp(const T & msg1, const T & msg2)
{
  T t1 = msg1;
  T t2 = msg2;
  t1.stamp = t2.stamp = rclcpp::Time();
  return t1 == t2;
}

void ControlGateInterface::on_source_status(const CommandSourceStatus & msg)
{
  const auto equals = equals_except_stamp(status_, msg);
  status_ = msg;
  if (!equals) notification_callback_();
}

void VehicleGateInterface::on_control_mode(const ControlModeReport & msg)
{
  const auto equals = equals_except_stamp(status_, msg);
  status_ = msg;
  if (!equals) notification_callback_();
}

bool ControlGateInterface::is_in_transition() const
{
  return status_.transition;
}

bool VehicleGateInterface::is_autoware_control() const
{
  return status_.mode == ControlModeReport::AUTONOMOUS;
}

bool ControlGateInterface::is_selected(const CommandPlugin & plugin) const
{
  if (plugin.source_name().empty()) {
    return true;
  }
  return plugin.source_name() == status_.source;
}

bool VehicleGateInterface::is_selected(const CommandPlugin & plugin) const
{
  if (plugin.autoware_control()) {
    return status_.mode == ControlModeReport::AUTONOMOUS;
  } else {
    return status_.mode == ControlModeReport::MANUAL;
  }
}

bool ControlGateInterface::request(const CommandPlugin & plugin, bool transition)
{
  if (requesting_) {
    return false;
  }
  if (!cli_source_select_->service_is_ready()) {
    RCLCPP_WARN_STREAM(node_.get_logger(), "control gate service is not ready");
    return false;
  }

  using SharedFuture = rclcpp::Client<SelectCommandSource>::SharedFuture;
  auto request = std::make_shared<SelectCommandSource::Request>();
  request->source = plugin.source_name();
  request->transition = transition;

  RCLCPP_INFO_STREAM(node_.get_logger(), "control gate request");
  requesting_ = true;
  cli_source_select_->async_send_request(request, [this](SharedFuture) { requesting_ = false; });
  return true;
}

bool VehicleGateInterface::request(const CommandPlugin & plugin)
{
  if (requesting_) {
    return false;
  }
  if (!cli_control_mode_->service_is_ready()) {
    RCLCPP_WARN_STREAM(node_.get_logger(), "vehicle gate service is not ready");
    return false;
  }

  using SharedFuture = rclcpp::Client<ControlModeCommand>::SharedFuture;
  auto request = std::make_shared<ControlModeCommand::Request>();
  if (plugin.autoware_control()) {
    request->mode = ControlModeCommand::Request::AUTONOMOUS;
  } else {
    request->mode = ControlModeCommand::Request::MANUAL;
  }

  RCLCPP_INFO_STREAM(node_.get_logger(), "vehicle gate request");
  requesting_ = true;
  cli_control_mode_->async_send_request(request, [this](SharedFuture) { requesting_ = false; });
  return true;
}

}  // namespace autoware::command_mode_switcher
