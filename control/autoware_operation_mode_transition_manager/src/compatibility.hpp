// Copyright 2022 Autoware Foundation
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

#ifndef COMPATIBILITY_HPP_
#define COMPATIBILITY_HPP_

#include "data.hpp"

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>

#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <tier4_control_msgs/msg/external_command_selector_mode.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_control_msgs/srv/external_command_select.hpp>

namespace autoware::operation_mode_transition_manager
{

class Compatibility
{
public:
  explicit Compatibility(autoware::agnocast_wrapper::Node * node);
  void set_mode(const OperationMode mode);
  std::optional<OperationMode> get_mode() const;

private:
  using AutowareEngage = autoware_vehicle_msgs::msg::Engage;
  using GateMode = tier4_control_msgs::msg::GateMode;
  using SelectorModeMsg = tier4_control_msgs::msg::ExternalCommandSelectorMode;
  using SelectorModeSrv = tier4_control_msgs::srv::ExternalCommandSelect;
  AUTOWARE_SUBSCRIPTION_PTR(AutowareEngage) sub_autoware_engage_;
  AUTOWARE_SUBSCRIPTION_PTR(GateMode) sub_gate_mode_;
  AUTOWARE_SUBSCRIPTION_PTR(SelectorModeMsg) sub_selector_mode_;
  AUTOWARE_PUBLISHER_PTR(AutowareEngage) pub_autoware_engage_;
  AUTOWARE_PUBLISHER_PTR(GateMode) pub_gate_mode_;
  AUTOWARE_CLIENT_PTR(SelectorModeSrv) cli_selector_mode_;
  void on_autoware_engage(const AutowareEngage::ConstSharedPtr msg);
  void on_gate_mode(const GateMode::ConstSharedPtr msg);
  void on_selector_mode(const SelectorModeMsg::ConstSharedPtr msg);

  bool is_calling_service_ = false;
  autoware::agnocast_wrapper::Node * node_;
  AutowareEngage::ConstSharedPtr autoware_engage_;
  GateMode::ConstSharedPtr gate_mode_;
  SelectorModeMsg::ConstSharedPtr selector_mode_;
};

}  // namespace autoware::operation_mode_transition_manager

#endif  // COMPATIBILITY_HPP_
