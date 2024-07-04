// Copyright 2024 The Autoware Contributors
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

#ifndef COMMON__CONVERTER__AVAILABILITY_CONVERTER_HPP_
#define COMMON__CONVERTER__AVAILABILITY_CONVERTER_HPP_

#include "udp_sender.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <tier4_system_msgs/msg/operation_mode_availability.hpp>

#include <memory>
#include <string>

namespace leader_election_converter
{

struct Availability
{
  autoware_auto_vehicle_msgs::msg::ControlModeReport::_mode_type mode;
  tier4_system_msgs::msg::OperationModeAvailability::_stop_type stop;
  tier4_system_msgs::msg::OperationModeAvailability::_autonomous_type autonomous;
  tier4_system_msgs::msg::OperationModeAvailability::_local_type local;
  tier4_system_msgs::msg::OperationModeAvailability::_remote_type remote;
  tier4_system_msgs::msg::OperationModeAvailability::_emergency_stop_type emergency_stop;
  tier4_system_msgs::msg::OperationModeAvailability::_comfortable_stop_type comfortable_stop;
  tier4_system_msgs::msg::OperationModeAvailability::_pull_over_type pull_over;
  // tier4_system_msgs::msg::OperationModeAvailability::_stop_next_bus_stop_type stop_next_bus_stop;
};

class AvailabilityConverter
{
public:
  explicit AvailabilityConverter(rclcpp::Node * node);

  void setUdpSender(const std::string & dest_ip, const std::string & dest_port);
  void setSubscriber();
  void convertToUdp(
    const tier4_system_msgs::msg::OperationModeAvailability::ConstSharedPtr availability_msg);

private:
  rclcpp::Node * node_;
  std::unique_ptr<UdpSender<Availability>> udp_availability_sender_;
  rclcpp::CallbackGroup::SharedPtr availability_callback_group_;
  rclcpp::CallbackGroup::SharedPtr control_mode_callback_group_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    sub_control_mode_;
  rclcpp::Subscription<tier4_system_msgs::msg::OperationModeAvailability>::SharedPtr
    sub_operation_mode_availability_;
};

}  // namespace leader_election_converter

#endif  // COMMON__CONVERTER__AVAILABILITY_CONVERTER_HPP_
