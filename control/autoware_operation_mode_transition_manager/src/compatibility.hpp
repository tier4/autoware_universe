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

#include <agnocast/agnocast.hpp>

#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <tier4_control_msgs/msg/external_command_selector_mode.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_control_msgs/srv/external_command_select.hpp>

namespace autoware::operation_mode_transition_manager
{

class Compatibility
{
public:
  explicit Compatibility(agnocast::Node * node);
  void set_mode(const OperationMode mode);
  std::optional<OperationMode> get_mode() const;

private:
  using AutowareEngage = autoware_vehicle_msgs::msg::Engage;
  using GateMode = tier4_control_msgs::msg::GateMode;
  using SelectorModeMsg = tier4_control_msgs::msg::ExternalCommandSelectorMode;
  using SelectorModeSrv = tier4_control_msgs::srv::ExternalCommandSelect;
  agnocast::Subscription<AutowareEngage>::SharedPtr sub_autoware_engage_;
  agnocast::Subscription<GateMode>::SharedPtr sub_gate_mode_;
  agnocast::Subscription<SelectorModeMsg>::SharedPtr sub_selector_mode_;
  agnocast::Publisher<AutowareEngage>::SharedPtr pub_autoware_engage_;
  agnocast::Publisher<GateMode>::SharedPtr pub_gate_mode_;
  agnocast::Client<SelectorModeSrv>::SharedPtr cli_selector_mode_;
  void on_autoware_engage(const agnocast::ipc_shared_ptr<const AutowareEngage> & msg);
  void on_gate_mode(const agnocast::ipc_shared_ptr<const GateMode> & msg);
  void on_selector_mode(const agnocast::ipc_shared_ptr<const SelectorModeMsg> & msg);

  bool is_calling_service_ = false;
  agnocast::Node * node_;
  agnocast::ipc_shared_ptr<const AutowareEngage> autoware_engage_;
  agnocast::ipc_shared_ptr<const GateMode> gate_mode_;
  agnocast::ipc_shared_ptr<const SelectorModeMsg> selector_mode_;
};

}  // namespace autoware::operation_mode_transition_manager

#endif  // COMPATIBILITY_HPP_
