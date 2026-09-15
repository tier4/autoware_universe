// Copyright 2015-2019 Autoware Foundation
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

#ifndef VEHICLE_CMD_GATE_HPP_
#define VEHICLE_CMD_GATE_HPP_

#include "adapi_pause_interface.hpp"
#include "autoware_utils/ros/logger_level_configure.hpp"
#include "moderate_stop_interface.hpp"
#include "vehicle_cmd_filter.hpp"

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/diagnostic_updater.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_vehicle_cmd_gate/msg/is_filter_activated.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/manual_operator_heartbeat.hpp>
#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_internal_debug_msgs/msg/bool_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/msg/emergency.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <tier4_external_api_msgs/srv/set_emergency.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::vehicle_cmd_gate
{

using autoware_adapi_v1_msgs::msg::MrmState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_control_msgs::msg::Control;
using autoware_control_msgs::msg::Longitudinal;
using autoware_internal_debug_msgs::msg::BoolStamped;
using autoware_vehicle_cmd_gate::msg::IsFilterActivated;
using autoware_vehicle_msgs::msg::GearCommand;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using std_srvs::srv::Trigger;
using tier4_control_msgs::msg::GateMode;
using tier4_external_api_msgs::msg::Emergency;
using tier4_external_api_msgs::srv::SetEmergency;
using tier4_system_msgs::msg::MrmBehaviorStatus;
using tier4_vehicle_msgs::msg::VehicleEmergencyStamped;
using visualization_msgs::msg::MarkerArray;

using diagnostic_msgs::msg::DiagnosticStatus;
using nav_msgs::msg::Odometry;

using Heartbeat = autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat;
using EngageMsg = autoware_vehicle_msgs::msg::Engage;
using EngageSrv = tier4_external_api_msgs::srv::Engage;

using autoware::motion_utils::VehicleStopCheckerBase;
struct Commands
{
  Control control;
  TurnIndicatorsCommand turn_indicator;
  HazardLightsCommand hazard_light;
  GearCommand gear;
  explicit Commands(const uint8_t & default_gear = GearCommand::PARK)
  {
    gear.command = default_gear;
  }
};

class VehicleCmdGate : public autoware::agnocast_wrapper::Node
{
public:
  explicit VehicleCmdGate(const rclcpp::NodeOptions & node_options);

private:
  using NodeT = autoware::agnocast_wrapper::Node;
  template <class T>
  using PollingSubscriber = autoware::agnocast_wrapper::polling::PollingSubscriber<T>;

  // Publisher
  AUTOWARE_PUBLISHER_PTR(VehicleEmergencyStamped) vehicle_cmd_emergency_pub_;
  AUTOWARE_PUBLISHER_PTR(Control) control_cmd_pub_;
  AUTOWARE_PUBLISHER_PTR(GearCommand) gear_cmd_pub_;
  AUTOWARE_PUBLISHER_PTR(TurnIndicatorsCommand) turn_indicator_cmd_pub_;
  AUTOWARE_PUBLISHER_PTR(HazardLightsCommand) hazard_light_cmd_pub_;
  AUTOWARE_PUBLISHER_PTR(GateMode) gate_mode_pub_;
  AUTOWARE_PUBLISHER_PTR(EngageMsg) engage_pub_;
  AUTOWARE_PUBLISHER_PTR(OperationModeState) operation_mode_pub_;
  AUTOWARE_PUBLISHER_PTR(IsFilterActivated) is_filter_activated_pub_;
  AUTOWARE_PUBLISHER_PTR(MarkerArray) filter_activated_marker_pub_;
  AUTOWARE_PUBLISHER_PTR(MarkerArray) filter_activated_marker_raw_pub_;
  AUTOWARE_PUBLISHER_PTR(BoolStamped) filter_activated_flag_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float64Stamped) processing_time_pub_;
  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);
  // Subscription
  AUTOWARE_SUBSCRIPTION_PTR(Heartbeat) external_emergency_stop_heartbeat_sub_;
  AUTOWARE_SUBSCRIPTION_PTR(GateMode) gate_mode_sub_;
  AUTOWARE_SUBSCRIPTION_PTR(OperationModeState) operation_mode_sub_;
  AUTOWARE_SUBSCRIPTION_PTR(MrmState) mrm_state_sub_;
  AUTOWARE_SUBSCRIPTION_PTR(Odometry) kinematics_sub_;             // for filter and stop checker
  AUTOWARE_SUBSCRIPTION_PTR(AccelWithCovarianceStamped) acc_sub_;  // for filter
  AUTOWARE_SUBSCRIPTION_PTR(SteeringReport) steer_sub_;            // for filter

  void onGateMode(GateMode::ConstSharedPtr msg);
  void onExternalEmergencyStopHeartbeat(Heartbeat::ConstSharedPtr msg);
  void onMrmState(MrmState::ConstSharedPtr msg);

  bool is_engaged_;
  bool is_system_emergency_ = false;
  bool is_external_emergency_stop_ = false;
  double current_steer_ = 0;
  GateMode current_gate_mode_;
  MrmState current_mrm_state_;
  Odometry current_kinematics_;
  double current_acceleration_ = 0.0;
  int filter_activated_count_ = 0;

  // Heartbeat
  std::shared_ptr<rclcpp::Time> emergency_state_heartbeat_received_time_;
  bool is_emergency_state_heartbeat_timeout_ = false;
  std::shared_ptr<rclcpp::Time> external_emergency_stop_heartbeat_received_time_;
  bool is_external_emergency_stop_heartbeat_timeout_ = false;
  bool isHeartbeatTimeout(
    const std::shared_ptr<rclcpp::Time> & heartbeat_received_time, const double timeout);

  // Check initialization
  bool isDataReady();

  // Subscriber for auto
  Commands auto_commands_;
  AUTOWARE_SUBSCRIPTION_PTR(Control) auto_control_cmd_sub_;
  PollingSubscriber<TurnIndicatorsCommand>::SharedPtr auto_turn_indicator_cmd_sub_;
  PollingSubscriber<HazardLightsCommand>::SharedPtr auto_hazard_light_cmd_sub_;
  PollingSubscriber<GearCommand>::SharedPtr auto_gear_cmd_sub_;
  void onAutoCtrlCmd(Control::ConstSharedPtr msg);

  // Subscription for external
  Commands remote_commands_;
  AUTOWARE_SUBSCRIPTION_PTR(Control) remote_control_cmd_sub_;
  PollingSubscriber<TurnIndicatorsCommand>::SharedPtr remote_turn_indicator_cmd_sub_;
  PollingSubscriber<HazardLightsCommand>::SharedPtr remote_hazard_light_cmd_sub_;
  PollingSubscriber<GearCommand>::SharedPtr remote_gear_cmd_sub_;
  void onRemoteCtrlCmd(Control::ConstSharedPtr msg);

  // Subscription for emergency
  Commands emergency_commands_;
  AUTOWARE_SUBSCRIPTION_PTR(Control) emergency_control_cmd_sub_;
  PollingSubscriber<TurnIndicatorsCommand>::SharedPtr emergency_turn_indicator_cmd_sub_;
  PollingSubscriber<HazardLightsCommand>::SharedPtr emergency_hazard_light_cmd_sub_;
  PollingSubscriber<GearCommand>::SharedPtr emergency_gear_cmd_sub_;
  void onEmergencyCtrlCmd(Control::ConstSharedPtr msg);

  // Previous Turn Indicators, Hazard Lights and Gear
  TurnIndicatorsCommand::SharedPtr prev_turn_indicator_;
  HazardLightsCommand::SharedPtr prev_hazard_light_;
  GearCommand::SharedPtr prev_gear_;

  // Parameter
  bool use_emergency_handling_;
  bool check_external_emergency_heartbeat_;
  double system_emergency_heartbeat_timeout_;
  double external_emergency_stop_heartbeat_timeout_;
  double stop_hold_acceleration_;
  double emergency_acceleration_;
  double moderate_stop_service_acceleration_;
  bool enable_cmd_limit_filter_;
  int filter_activated_count_threshold_;
  double filter_activated_velocity_threshold_;

  // Service
  AUTOWARE_SERVICE_PTR(EngageSrv) srv_engage_;
  AUTOWARE_SERVICE_PTR(SetEmergency) srv_external_emergency_;
  AUTOWARE_PUBLISHER_PTR(Emergency) pub_external_emergency_;
  void onEngageService(
    const EngageSrv::Request::SharedPtr request, const EngageSrv::Response::SharedPtr response);
  void onExternalEmergencyStopService(
    const SetEmergency::Request::SharedPtr request,
    const SetEmergency::Response::SharedPtr response);

  // TODO(Takagi, Isamu): deprecated
  AUTOWARE_SUBSCRIPTION_PTR(EngageMsg) engage_sub_;
  AUTOWARE_SERVICE_PTR(Trigger) srv_external_emergency_stop_;
  AUTOWARE_SERVICE_PTR(Trigger) srv_clear_external_emergency_stop_;
  void onEngage(EngageMsg::ConstSharedPtr msg);
  bool onSetExternalEmergencyStopService(
    const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res);
  bool onClearExternalEmergencyStopService(
    const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res);

  // Timer / Event
  AUTOWARE_TIMER_PTR timer_;
  AUTOWARE_TIMER_PTR timer_pub_status_;

  void onTimer();
  void publishControlCommands(const Commands & commands);
  void publishEmergencyStopControlCommands();
  void publishStatus();

  // Diagnostics Updater
  autoware::agnocast_wrapper::diagnostic_updater::Updater updater_;

  void checkExternalEmergencyStop(diagnostic_updater::DiagnosticStatusWrapper & stat);

  template <typename T>
  T getContinuousTopic(
    const std::shared_ptr<T> & prev_topic, const T & current_topic,
    const std::string & topic_name = "");

  // Algorithm
  Control prev_control_cmd_;
  Control createStopControlCmd() const;
  Longitudinal createLongitudinalStopControlCmd() const;
  Control createEmergencyStopControlCmd() const;

  std::shared_ptr<rclcpp::Time> prev_time_;
  double getDt();
  Control getActualStatusAsCommand();

  VehicleCmdFilter filter_;
  Control filterControlCommand(const Control & in);

  // filtering on transition
  OperationModeState current_operation_mode_;
  VehicleCmdFilter filter_on_transition_;

  // Pause interface for API
  std::unique_ptr<AdapiPauseInterface> adapi_pause_;
  std::unique_ptr<ModerateStopInterface> moderate_stop_interface_;

  // stop checker. VehicleStopChecker owns an rclcpp subscription, so only its node-agnostic base
  // is used and the odometry is fed from kinematics_sub_, which covers the same topic and QoS.
  std::unique_ptr<VehicleStopCheckerBase> vehicle_stop_checker_;
  double stop_check_duration_;

  // debug
  MarkerArray createMarkerArray(const IsFilterActivated & filter_activated);
  void publishMarkers(const IsFilterActivated & filter_activated);

  std::unique_ptr<autoware_utils::BasicLoggerLevelConfigure<NodeT>> logger_configure_;

  std::unique_ptr<autoware_utils::BasicPublishedTimePublisher<NodeT>> published_time_publisher_;
};

}  // namespace autoware::vehicle_cmd_gate
#endif  // VEHICLE_CMD_GATE_HPP_
