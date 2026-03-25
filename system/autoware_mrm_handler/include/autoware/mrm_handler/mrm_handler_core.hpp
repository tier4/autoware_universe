// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__MRM_HANDLER__MRM_HANDLER_CORE_HPP_
#define AUTOWARE__MRM_HANDLER__MRM_HANDLER_CORE_HPP_

// Core
#include <memory>
#include <optional>
#include <string>
#include <variant>

// Autoware
#include <agnocast/agnocast.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_system_msgs/msg/emergency_holding_state.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_system_msgs/msg/operation_mode_availability.hpp>
#include <tier4_system_msgs/srv/operate_mrm.hpp>

// ROS 2 core
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::mrm_handler
{

struct HazardLampPolicy
{
  bool emergency;
};

struct TurnIndicatorPolicy
{
  bool emergency;
};

struct Param
{
  int update_rate;
  double timeout_operation_mode_availability;
  double timeout_call_mrm_behavior;
  double timeout_cancel_mrm_behavior;
  bool use_emergency_holding;
  double timeout_emergency_recovery;
  bool use_parking_after_stopped;
  bool use_pull_over;
  bool use_comfortable_stop;
  HazardLampPolicy turning_hazard_on{};
  TurnIndicatorPolicy turning_indicator_on{};
};

class MrmHandler : public agnocast::Node
{
public:
  explicit MrmHandler(const rclcpp::NodeOptions & options);

private:
  // type
  enum RequestType { CALL, CANCEL };

  // Subscribers with callback
  agnocast::Subscription<tier4_system_msgs::msg::OperationModeAvailability>::SharedPtr
    sub_operation_mode_availability_;
  // Subscribers without callback
  agnocast::PollingSubscriber<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  agnocast::PollingSubscriber<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    sub_control_mode_;
  agnocast::PollingSubscriber<tier4_system_msgs::msg::MrmBehaviorStatus>::SharedPtr
    sub_mrm_pull_over_status_;
  agnocast::PollingSubscriber<tier4_system_msgs::msg::MrmBehaviorStatus>::SharedPtr
    sub_mrm_comfortable_stop_status_;
  agnocast::PollingSubscriber<tier4_system_msgs::msg::MrmBehaviorStatus>::SharedPtr
    sub_mrm_emergency_stop_status_;
  agnocast::PollingSubscriber<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    sub_operation_mode_state_;
  agnocast::PollingSubscriber<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr sub_gear_cmd_;

  std::optional<tier4_system_msgs::msg::OperationModeAvailability> operation_mode_availability_;

  void onOperationModeAvailability(
    const agnocast::ipc_shared_ptr<const tier4_system_msgs::msg::OperationModeAvailability> & msg);

  // Publisher

  agnocast::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    pub_turn_indicator_cmd_;
  agnocast::Publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr pub_hazard_cmd_;
  agnocast::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr pub_gear_cmd_;

  void publishTurnIndicatorCmd();
  void publishHazardCmd();
  void publishGearCmd();

  agnocast::Publisher<autoware_adapi_v1_msgs::msg::MrmState>::SharedPtr pub_mrm_state_;

  autoware_adapi_v1_msgs::msg::MrmState mrm_state_;
  void publishMrmState();

  agnocast::Publisher<tier4_system_msgs::msg::EmergencyHoldingState>::SharedPtr
    pub_emergency_holding_;
  void publishEmergencyHolding();

  // Clients
  rclcpp::CallbackGroup::SharedPtr client_mrm_pull_over_group_;
  agnocast::Client<tier4_system_msgs::srv::OperateMrm>::SharedPtr client_mrm_pull_over_;
  rclcpp::CallbackGroup::SharedPtr client_mrm_comfortable_stop_group_;
  agnocast::Client<tier4_system_msgs::srv::OperateMrm>::SharedPtr client_mrm_comfortable_stop_;
  rclcpp::CallbackGroup::SharedPtr client_mrm_emergency_stop_group_;
  agnocast::Client<tier4_system_msgs::srv::OperateMrm>::SharedPtr client_mrm_emergency_stop_;

  bool requestMrmBehavior(
    const autoware_adapi_v1_msgs::msg::MrmState::_behavior_type & mrm_behavior,
    RequestType request_type) const;
  void logMrmCallingResult(
    const tier4_system_msgs::srv::OperateMrm::Response & result, const std::string & behavior,
    bool is_call) const;

  // Timer
  agnocast::TimerBase::SharedPtr timer_;

  // Parameters
  Param param_;

  bool isDataReady();
  void onTimer();

  // Heartbeat
  rclcpp::Time stamp_operation_mode_availability_;
  std::optional<rclcpp::Time> stamp_autonomous_become_unavailable_ = std::nullopt;
  bool is_operation_mode_availability_timeout;
  void checkOperationModeAvailabilityTimeout();

  // Algorithm
  bool is_emergency_holding_ = false;
  uint8_t last_gear_command_{autoware_vehicle_msgs::msg::GearCommand::DRIVE};
  void transitionTo(const int new_state);
  void updateMrmState();
  void operateMrm();
  void handleFailedRequest();
  autoware_adapi_v1_msgs::msg::MrmState::_behavior_type getCurrentMrmBehavior();
  bool isStopped();
  bool isEmergency() const;
  bool isControlModeAutonomous();
  bool isOperationModeAutonomous();
  bool isPullOverStatusAvailable();
  bool isComfortableStopStatusAvailable();
  bool isEmergencyStopStatusAvailable();
  bool isArrivedAtGoal();
};

}  // namespace autoware::mrm_handler

#endif  // AUTOWARE__MRM_HANDLER__MRM_HANDLER_CORE_HPP_
