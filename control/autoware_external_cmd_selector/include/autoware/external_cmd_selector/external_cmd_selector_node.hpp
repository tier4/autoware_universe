// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_
#define AUTOWARE__EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_

#include <agnocast/node/agnocast_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/manual_operator_heartbeat.hpp>
#include <autoware_adapi_v1_msgs/msg/pedals_command.hpp>
#include <autoware_adapi_v1_msgs/msg/steering_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_control_msgs/msg/external_command_selector_mode.hpp>
#include <tier4_control_msgs/srv/external_command_select.hpp>

#include <memory>

namespace autoware::external_cmd_selector
{
class ExternalCmdSelector : public agnocast::Node
{
public:
  explicit ExternalCmdSelector(const rclcpp::NodeOptions & node_options);

private:
  using CommandSourceSelect = tier4_control_msgs::srv::ExternalCommandSelect;
  using CommandSourceMode = tier4_control_msgs::msg::ExternalCommandSelectorMode;

  using PedalsCommand = autoware_adapi_v1_msgs::msg::PedalsCommand;
  using SteeringCommand = autoware_adapi_v1_msgs::msg::SteeringCommand;
  using OperatorHeartbeat = autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat;

  using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
  using TurnIndicatorsCommand = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
  using HazardLightsCommand = autoware_vehicle_msgs::msg::HazardLightsCommand;

  // Publisher
  agnocast::Publisher<CommandSourceMode>::SharedPtr pub_current_selector_mode_;
  agnocast::Publisher<PedalsCommand>::SharedPtr pub_pedals_cmd_;
  agnocast::Publisher<SteeringCommand>::SharedPtr pub_steering_cmd_;
  agnocast::Publisher<OperatorHeartbeat>::SharedPtr pub_heartbeat_;
  agnocast::Publisher<GearCommand>::SharedPtr pub_gear_cmd_;
  agnocast::Publisher<TurnIndicatorsCommand>::SharedPtr pub_turn_indicators_cmd_;
  agnocast::Publisher<HazardLightsCommand>::SharedPtr pub_hazard_lights_cmd_;

  // Subscriber
  agnocast::Subscription<PedalsCommand>::SharedPtr sub_local_pedals_cmd_;
  agnocast::Subscription<SteeringCommand>::SharedPtr sub_local_steering_cmd_;
  agnocast::Subscription<OperatorHeartbeat>::SharedPtr sub_local_heartbeat_;
  agnocast::Subscription<GearCommand>::SharedPtr sub_local_gear_cmd_;
  agnocast::Subscription<TurnIndicatorsCommand>::SharedPtr sub_local_turn_indicators_cmd_;
  agnocast::Subscription<HazardLightsCommand>::SharedPtr sub_local_hazard_lights_cmd_;

  agnocast::Subscription<PedalsCommand>::SharedPtr sub_remote_pedals_cmd_;
  agnocast::Subscription<SteeringCommand>::SharedPtr sub_remote_steering_cmd_;
  agnocast::Subscription<OperatorHeartbeat>::SharedPtr sub_remote_heartbeat_;
  agnocast::Subscription<GearCommand>::SharedPtr sub_remote_gear_cmd_;
  agnocast::Subscription<TurnIndicatorsCommand>::SharedPtr sub_remote_turn_indicators_cmd_;
  agnocast::Subscription<HazardLightsCommand>::SharedPtr sub_remote_hazard_lights_cmd_;

  template <class T, class F>
  std::function<void(const agnocast::ipc_shared_ptr<T> &)> bind_on_cmd(F && func, uint8_t mode)
  {
    return [this, func, mode](const agnocast::ipc_shared_ptr<T> & msg) {
      (this->*func)(*msg, mode);
    };
  }
  void on_pedals_cmd(const PedalsCommand & msg, uint8_t mode);
  void on_steering_cmd(const SteeringCommand & msg, uint8_t mode);
  void on_heartbeat(const OperatorHeartbeat & msg, uint8_t mode);
  void on_gear_cmd(const GearCommand & msg, uint8_t mode);
  void on_turn_indicators_cmd(const TurnIndicatorsCommand & msg, uint8_t mode);
  void on_hazard_lights_cmd(const HazardLightsCommand & msg, uint8_t mode);

  // Service
  agnocast::Service<CommandSourceSelect>::SharedPtr srv_select_external_command_;
  CommandSourceMode current_selector_mode_;
  void on_select_external_command(
    const agnocast::ipc_shared_ptr<agnocast::Service<CommandSourceSelect>::RequestT> & req,
    agnocast::ipc_shared_ptr<agnocast::Service<CommandSourceSelect>::ResponseT> & res);

  // Timer
  agnocast::TimerBase::SharedPtr timer_;
  void on_timer();
};
}  // namespace autoware::external_cmd_selector
#endif  // AUTOWARE__EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_
