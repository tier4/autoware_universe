// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware_state.hpp"

#include <string>
#include <vector>

namespace autoware::default_adapi
{

AutowareStateNode::AutowareStateNode(const rclcpp::NodeOptions & options)
: Node("autoware_state", options)
{
  const std::vector<std::string> module_names = {
    "sensing", "perception", "map", "localization", "planning", "control", "vehicle", "system",
  };

  for (size_t i = 0; i < module_names.size(); ++i) {
    const auto name = "/system/component_state_monitor/component/launch/" + module_names[i];
    const auto qos = rclcpp::QoS(1).transient_local();
    const auto callback = [this, i](const ModeChangeAvailable::ConstSharedPtr msg) {
      component_states_[i] = msg->available;
    };
    sub_component_states_.push_back(create_subscription<ModeChangeAvailable>(name, qos, callback));
  }

  pub_autoware_state_ = create_publisher<AutowareState>("/autoware/state", 1);
  srv_autoware_shutdown_ = create_service<std_srvs::srv::Trigger>(
    "/autoware/shutdown",
    std::bind(&AutowareStateNode::on_shutdown, this, std::placeholders::_1, std::placeholders::_2));

  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  adaptor.init_sub(sub_localization_, this, &AutowareStateNode::on_localization);
  adaptor.init_sub(sub_routing_, this, &AutowareStateNode::on_routing);
  adaptor.init_sub(sub_operation_mode_, this, &AutowareStateNode::on_operation_mode);

  const auto rate = rclcpp::Rate(declare_parameter<double>("update_rate"));
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });

  component_states_.resize(module_names.size());
  launch_state_ = LaunchState::Initializing;
  localization_state_.state = LocalizationState::UNKNOWN;
  routing_state_.state = RoutingState::UNKNOWN;
  operation_mode_state_.mode = OperationModeState::UNKNOWN;
  last_log_time_ = now();
}

void AutowareStateNode::on_localization(const LocalizationState::ConstSharedPtr msg)
{
  localization_state_ = *msg;
}
void AutowareStateNode::on_routing(const RoutingState::ConstSharedPtr msg)
{
  routing_state_ = *msg;
}
void AutowareStateNode::on_operation_mode(const OperationModeState::ConstSharedPtr msg)
{
  operation_mode_state_ = *msg;
}

void AutowareStateNode::on_shutdown(
  const Trigger::Request::SharedPtr, const Trigger::Response::SharedPtr res)
{
  launch_state_ = LaunchState::Finalizing;
  res->success = true;
  res->message = "Shutdown Autoware.";
}

void AutowareStateNode::on_timer()
{
  const auto convert_state = [this]() {
    const auto current_time = now();
    const auto time_since_last_log = (current_time - last_log_time_).seconds();
    const bool should_log = time_since_last_log >= 1.0;
    
    if (launch_state_ == LaunchState::Finalizing) {
      if (should_log) {
        RCLCPP_INFO(get_logger(), "State transition: FINALIZING (launch_state_ == LaunchState::Finalizing)");
        last_log_time_ = current_time;
      }
      return AutowareState::FINALIZING;
    }
    if (launch_state_ == LaunchState::Initializing) {
      if (should_log) {
        RCLCPP_INFO(get_logger(), "State transition: INITIALIZING (launch_state_ == LaunchState::Initializing)");
        last_log_time_ = current_time;
      }
      return AutowareState::INITIALIZING;
    }
    if (localization_state_.state == LocalizationState::UNKNOWN) {
      if (should_log) {
        RCLCPP_INFO(get_logger(), "State transition: INITIALIZING (localization_state_.state == LocalizationState::UNKNOWN)");
        last_log_time_ = current_time;
      }
      return AutowareState::INITIALIZING;
    }
    if (routing_state_.state == RoutingState::UNKNOWN) {
      if (should_log) {
        RCLCPP_INFO(get_logger(), "State transition: INITIALIZING (routing_state_.state == RoutingState::UNKNOWN)");
        last_log_time_ = current_time;
      }
      return AutowareState::INITIALIZING;
    }
    if (operation_mode_state_.mode == OperationModeState::UNKNOWN) {
      if (should_log) {
        RCLCPP_INFO(get_logger(), "State transition: INITIALIZING (operation_mode_state_.mode == OperationModeState::UNKNOWN)");
        last_log_time_ = current_time;
      }
      return AutowareState::INITIALIZING;
    }
    if (localization_state_.state != LocalizationState::INITIALIZED) {
      if (should_log) {
        RCLCPP_INFO(get_logger(), "State transition: INITIALIZING (localization_state_.state != LocalizationState::INITIALIZED)");
        last_log_time_ = current_time;
      }
      return AutowareState::INITIALIZING;
    }
    if (routing_state_.state == RoutingState::UNSET) {
      if (should_log) {
        RCLCPP_INFO(get_logger(), "State transition: WAITING_FOR_ROUTE (routing_state_.state == RoutingState::UNSET)");
        last_log_time_ = current_time;
      }
      return AutowareState::WAITING_FOR_ROUTE;
    }
    if (routing_state_.state == RoutingState::ARRIVED) {
      if (should_log) {
        RCLCPP_INFO(get_logger(), "State transition: ARRIVED_GOAL (routing_state_.state == RoutingState::ARRIVED)");
        last_log_time_ = current_time;
      }
      return AutowareState::ARRIVED_GOAL;
    }
    if (operation_mode_state_.mode != OperationModeState::STOP) {
      if (operation_mode_state_.is_autoware_control_enabled) {
        if (should_log) {
          RCLCPP_INFO(get_logger(), "State transition: DRIVING (operation_mode_state_.mode != STOP && is_autoware_control_enabled)");
          last_log_time_ = current_time;
        }
        return AutowareState::DRIVING;
      }
    }
    if (operation_mode_state_.is_autonomous_mode_available) {
      if (should_log) {
        RCLCPP_INFO(get_logger(), "State transition: WAITING_FOR_ENGAGE (operation_mode_state_.is_autonomous_mode_available)");
        last_log_time_ = current_time;
      }
      return AutowareState::WAITING_FOR_ENGAGE;
    }
    if (should_log) {
      RCLCPP_INFO(get_logger(), "State transition: PLANNING (default case)");
      last_log_time_ = current_time;
    }
    return AutowareState::PLANNING;
  };

  // Update launch state.
  if (launch_state_ == LaunchState::Initializing) {
    bool is_initialized = true;
    for (const auto & state : component_states_) {
      is_initialized &= state;
    }
    if (is_initialized) {
      launch_state_ = LaunchState::Running;
    }
  }

  // Update routing state to reproduce old logic.
  if (routing_state_.state == RoutingState::ARRIVED) {
    const auto duration = now() - rclcpp::Time(routing_state_.stamp);
    if (2.0 < duration.seconds()) {
      routing_state_.state = RoutingState::UNSET;
    }
  }

  AutowareState msg;
  msg.stamp = now();
  msg.state = convert_state();
  pub_autoware_state_->publish(msg);
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::AutowareStateNode)
