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

#ifndef COMMAND_MODE_DECIDER_BASE_HPP_
#define COMMAND_MODE_DECIDER_BASE_HPP_

#include "autoware_command_mode_decider/plugin.hpp"
#include "autoware_command_mode_decider/status.hpp"

#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <autoware_command_mode_types/adapters/command_mode_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <tier4_system_msgs/msg/command_mode_request.hpp>
#include <tier4_system_msgs/msg/command_mode_status.hpp>
#include <tier4_system_msgs/srv/change_autoware_control.hpp>
#include <tier4_system_msgs/srv/change_operation_mode.hpp>
#include <tier4_system_msgs/srv/request_mrm.hpp>

#include <memory>
#include <vector>

namespace autoware::command_mode_decider
{

using autoware::command_mode_types::CommandModeStatus;
using autoware::command_mode_types::CommandModeStatusAdapter;
using autoware::command_mode_types::GateType;
using autoware::command_mode_types::TriState;
using autoware_adapi_v1_msgs::msg::MrmState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_common_msgs::msg::ResponseStatus;
using tier4_system_msgs::msg::CommandModeRequest;
using tier4_system_msgs::msg::CommandModeRequestItem;
using tier4_system_msgs::srv::ChangeAutowareControl;
using tier4_system_msgs::srv::ChangeOperationMode;
using tier4_system_msgs::srv::RequestMrm;

class CommandModeDeciderBase : public rclcpp::Node
{
public:
  explicit CommandModeDeciderBase(const rclcpp::NodeOptions & options);

private:
  bool is_in_transition() const;
  void update();
  void detect_override();
  void detect_operation_mode_timeout();
  void update_request_mode();
  void update_current_mode();
  void sync_command_mode();
  void publish_operation_mode_state();
  void publish_mrm_state();

  void on_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);
  void on_timer();
  void on_status(const CommandModeStatus & msg);
  void on_change_operation_mode(
    ChangeOperationMode::Request::SharedPtr req, ChangeOperationMode::Response::SharedPtr res);
  void on_change_autoware_control(
    ChangeAutowareControl::Request::SharedPtr req, ChangeAutowareControl::Response::SharedPtr res);

  ResponseStatus check_mode_exists(uint16_t mode);
  ResponseStatus check_mode_request(uint16_t mode, bool background);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<CommandModeRequest>::SharedPtr pub_command_mode_request_;
  rclcpp::Subscription<CommandModeStatusAdapter>::SharedPtr sub_command_mode_status_;

  rclcpp::Service<ChangeAutowareControl>::SharedPtr srv_autoware_control_;
  rclcpp::Service<ChangeOperationMode>::SharedPtr srv_operation_mode_;
  rclcpp::Publisher<OperationModeState>::SharedPtr pub_operation_mode_;
  rclcpp::Publisher<MrmState>::SharedPtr pub_mrm_state_;

  diagnostic_updater::Updater diagnostics_;
  pluginlib::ClassLoader<DeciderPlugin> loader_;
  std::shared_ptr<DeciderPlugin> plugin_;

  // parameters
  double transition_timeout_;
  double request_timeout_;

  // status
  bool is_modes_ready_;
  CommandModeStatusTable command_mode_status_;
  RequestModeStatus system_request_;
  std::vector<uint16_t> request_modes_;

  bool curr_autoware_control_;
  uint16_t curr_operation_mode_;
  uint16_t current_mode_;

  std::optional<rclcpp::Time> request_stamp_;
  std::optional<rclcpp::Time> transition_stamp_;
  std::optional<uint16_t> last_mode_;  // nullopt means manual control
};

}  // namespace autoware::command_mode_decider

#endif  // COMMAND_MODE_DECIDER_BASE_HPP_
