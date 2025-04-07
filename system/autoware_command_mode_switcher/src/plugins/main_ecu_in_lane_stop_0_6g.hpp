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

#ifndef SWITCHERS__MAIN_ECU_IN_LANE_STOP_0_6G_HPP_
#define SWITCHERS__MAIN_ECU_IN_LANE_STOP_0_6G_HPP_

#include "common/command_plugin.hpp"

#include <jerk_constant_deceleration_controller_msgs/msg/jerk_constant_deceleration_trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>

namespace autoware::command_mode_switcher
{
using jerk_constant_deceleration_controller_msgs::msg::JerkConstantDecelerationTrigger;

class MainEcuInLaneStop06GSwitcher : public ControlCommandPlugin
{
public:
  std::string mode_name() const override { return "main_ecu_in_lane_stop_0_6g"; }
  std::string source_name() const override { return "in_lane_stop"; }
  bool autoware_control() const override { return true; }
  void initialize() override;

  TriState update_source_state(bool request) override;
  MrmState update_mrm_state() override;

  bool get_transition_available() override { return true; }
  bool get_transition_completed() override { return true; }
private:
  void publish_jerk_constant_deceleration_trigger(bool turn_on);
  bool is_stopped();

  rclcpp::Publisher<JerkConstantDecelerationTrigger>::SharedPtr 
    pub_trigger_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  std::optional<nav_msgs::msg::Odometry> odom_;
  MrmState mrm_state_;
};

}  // namespace autoware::command_mode_switcher

#endif  //SWITCHERS__MAIN_ECU_IN_LANE_STOP_0_6G_HPP_
