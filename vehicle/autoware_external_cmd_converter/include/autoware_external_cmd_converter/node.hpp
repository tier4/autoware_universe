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

#ifndef AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_
#define AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_

#include <agnocast/node/agnocast_node.hpp>
#include <autoware_raw_vehicle_cmd_converter/accel_map.hpp>
#include <autoware_raw_vehicle_cmd_converter/brake_map.hpp>

#include <autoware_adapi_v1_msgs/msg/manual_operator_heartbeat.hpp>
#include <autoware_adapi_v1_msgs/msg/pedals_command.hpp>
#include <autoware_adapi_v1_msgs/msg/steering_command.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>

#include <memory>
#include <string>

namespace autoware::external_cmd_converter
{

using autoware::raw_vehicle_cmd_converter::AccelMap;
using autoware::raw_vehicle_cmd_converter::BrakeMap;
using autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat;
using autoware_adapi_v1_msgs::msg::PedalsCommand;
using autoware_adapi_v1_msgs::msg::SteeringCommand;
using autoware_control_msgs::msg::Control;
using autoware_vehicle_msgs::msg::GearCommand;
using nav_msgs::msg::Odometry;
using tier4_control_msgs::msg::GateMode;

class ExternalCmdConverterNode : public agnocast::Node
{
public:
  explicit ExternalCmdConverterNode(const rclcpp::NodeOptions & node_options);

private:
  // Publisher
  agnocast::Publisher<Control>::SharedPtr cmd_pub_;

  // Subscriber
  agnocast::Subscription<PedalsCommand>::SharedPtr pedals_cmd_sub_;
  agnocast::Subscription<ManualOperatorHeartbeat>::SharedPtr heartbeat_sub_;

  // Polling Subscriber
  agnocast::PollingSubscriber<SteeringCommand>::SharedPtr steering_cmd_sub_;
  agnocast::PollingSubscriber<Odometry>::SharedPtr velocity_sub_;
  agnocast::PollingSubscriber<GearCommand>::SharedPtr gear_cmd_sub_;
  agnocast::PollingSubscriber<GateMode>::SharedPtr gate_mode_sub_;

  void on_pedals_cmd(const agnocast::ipc_shared_ptr<PedalsCommand> & cmd_ptr);
  void on_heartbeat(const agnocast::ipc_shared_ptr<ManualOperatorHeartbeat> & msg);

  agnocast::ipc_shared_ptr<const Odometry> current_velocity_ptr_;  // [m/s]
  agnocast::ipc_shared_ptr<const GearCommand> current_gear_cmd_;
  agnocast::ipc_shared_ptr<const GateMode> current_gate_mode_;

  std::shared_ptr<rclcpp::Time> latest_heartbeat_received_time_;
  std::shared_ptr<rclcpp::Time> latest_cmd_received_time_;

  // Timer
  void on_timer();
  agnocast::TimerBase::SharedPtr rate_check_timer_;

  // Parameter
  double ref_vel_gain_;  // reference velocity = current velocity + desired acceleration * gain
  bool wait_for_first_topic_;
  double control_command_timeout_;
  double emergency_stop_timeout_;

  // Algorithm
  AccelMap accel_map_;
  BrakeMap brake_map_;
  bool acc_map_initialized_;

  double calculate_acc(const PedalsCommand & cmd, const double vel);
  double get_gear_velocity_sign(const GearCommand & cmd);
};

}  // namespace autoware::external_cmd_converter

#endif  // AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_
