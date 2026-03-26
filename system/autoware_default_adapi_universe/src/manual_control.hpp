// Copyright 2025 The Autoware Contributors
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

#ifndef MANUAL_CONTROL_HPP_
#define MANUAL_CONTROL_HPP_

#include <agnocast/agnocast.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/acceleration_command.hpp>
#include <autoware_adapi_v1_msgs/msg/gear_command.hpp>
#include <autoware_adapi_v1_msgs/msg/hazard_lights_command.hpp>
#include <autoware_adapi_v1_msgs/msg/manual_control_mode.hpp>
#include <autoware_adapi_v1_msgs/msg/manual_control_mode_status.hpp>
#include <autoware_adapi_v1_msgs/msg/manual_operator_heartbeat.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/pedals_command.hpp>
#include <autoware_adapi_v1_msgs/msg/steering_command.hpp>
#include <autoware_adapi_v1_msgs/msg/turn_indicators_command.hpp>
#include <autoware_adapi_v1_msgs/msg/velocity_command.hpp>
#include <autoware_adapi_v1_msgs/srv/list_manual_control_mode.hpp>
#include <autoware_adapi_v1_msgs/srv/select_manual_control_mode.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <string>

namespace autoware::default_adapi
{

class ManualControlNode : public agnocast::Node
{
public:
  explicit ManualControlNode(const rclcpp::NodeOptions & options);

private:
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  agnocast::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;
  std::shared_ptr<const OperationModeState> latest_operation_mode_;

  using ManualControlModeStatus = autoware_adapi_v1_msgs::msg::ManualControlModeStatus;
  using ManualControlMode = autoware_adapi_v1_msgs::msg::ManualControlMode;
  using ListMode = autoware_adapi_v1_msgs::srv::ListManualControlMode;
  using SelectMode = autoware_adapi_v1_msgs::srv::SelectManualControlMode;
  void update_mode_status(uint8_t mode);
  void on_list_mode(
    const agnocast::ipc_shared_ptr<typename agnocast::Service<ListMode>::RequestT> & req,
    agnocast::ipc_shared_ptr<typename agnocast::Service<ListMode>::ResponseT> & res);
  void on_select_mode(
    const agnocast::ipc_shared_ptr<typename agnocast::Service<SelectMode>::RequestT> & req,
    agnocast::ipc_shared_ptr<typename agnocast::Service<SelectMode>::ResponseT> & res);
  agnocast::Service<ListMode>::SharedPtr srv_list_mode_;
  agnocast::Service<SelectMode>::SharedPtr srv_select_mode_;
  agnocast::Publisher<ManualControlModeStatus>::SharedPtr pub_mode_status_;

  using PedalsCommand = autoware_adapi_v1_msgs::msg::PedalsCommand;
  using AccelerationCommand = autoware_adapi_v1_msgs::msg::AccelerationCommand;
  using VelocityCommand = autoware_adapi_v1_msgs::msg::VelocityCommand;
  using SteeringCommand = autoware_adapi_v1_msgs::msg::SteeringCommand;
  using GearCommand = autoware_adapi_v1_msgs::msg::GearCommand;
  using HazardLightsCommand = autoware_adapi_v1_msgs::msg::HazardLightsCommand;
  using TurnIndicatorsCommand = autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand;
  using OperatorHeartbeat = autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat;
  void disable_all_commands();
  void enable_common_commands();
  void enable_pedals_commands();
  void enable_acceleration_commands();
  void enable_velocity_commands();
  agnocast::Subscription<OperatorHeartbeat>::SharedPtr sub_heartbeat_;
  agnocast::Subscription<PedalsCommand>::SharedPtr sub_pedals_;
  agnocast::Subscription<AccelerationCommand>::SharedPtr sub_acceleration_;
  agnocast::Subscription<VelocityCommand>::SharedPtr sub_velocity_;
  agnocast::Subscription<SteeringCommand>::SharedPtr sub_steering_;
  agnocast::Subscription<GearCommand>::SharedPtr sub_gear_;
  agnocast::Subscription<TurnIndicatorsCommand>::SharedPtr sub_turn_indicators_;
  agnocast::Subscription<HazardLightsCommand>::SharedPtr sub_hazard_lights_;

  using InternalGear = autoware_vehicle_msgs::msg::GearCommand;
  using InternalTurnIndicators = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
  using InternalHazardLights = autoware_vehicle_msgs::msg::HazardLightsCommand;
  agnocast::Publisher<OperatorHeartbeat>::SharedPtr pub_heartbeat_;
  agnocast::Publisher<PedalsCommand>::SharedPtr pub_pedals_;
  agnocast::Publisher<SteeringCommand>::SharedPtr pub_steering_;
  agnocast::Publisher<InternalGear>::SharedPtr pub_gear_;
  agnocast::Publisher<InternalTurnIndicators>::SharedPtr pub_turn_indicators_;
  agnocast::Publisher<InternalHazardLights>::SharedPtr pub_hazard_lights_;

  uint8_t target_operation_mode_;
  std::string ns_;
  ManualControlMode current_mode_;
};

}  // namespace autoware::default_adapi

#endif  // MANUAL_CONTROL_HPP_
