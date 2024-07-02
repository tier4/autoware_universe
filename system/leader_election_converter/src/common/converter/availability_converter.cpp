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

#include "availability_converter.hpp"

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>

namespace leader_election_converter
{

AvailabilityConverter::AvailabilityConverter(rclcpp::Node * node) : node_(node)
{
}

void AvailabilityConverter::setUdpSender(const std::string & dest_ip, const std::string & dest_port)
{
  udp_availability_sender_ = std::make_unique<UdpSender<Availability>>(dest_ip, dest_port);
}

void AvailabilityConverter::setSubscriber()
{
  const auto qos = rclcpp::QoS(1);
  availability_callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions availability_options = rclcpp::SubscriptionOptions();
  availability_options.callback_group = availability_callback_group_;

  control_mode_callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions control_mode_options = rclcpp::SubscriptionOptions();
  control_mode_options.callback_group = control_mode_callback_group_;
  auto not_executed_callback = []([[maybe_unused]] const typename autoware_auto_vehicle_msgs::msg::
                                    ControlModeReport::ConstSharedPtr msg) {};

  sub_operation_mode_availability_ =
    node_->create_subscription<tier4_system_msgs::msg::OperationModeAvailability>(
      "~/input/operation_mode_availability", qos,
      std::bind(&AvailabilityConverter::convertToUdp, this, std::placeholders::_1),
      availability_options);

  sub_control_mode_ =
    node_->create_subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
      "~/input/control_mode", qos, not_executed_callback, control_mode_options);
}

void AvailabilityConverter::convertToUdp(
  const tier4_system_msgs::msg::OperationModeAvailability::ConstSharedPtr availability_msg)
{
  auto control_mode_report = std::make_shared<autoware_auto_vehicle_msgs::msg::ControlModeReport>();
  rclcpp::MessageInfo message_info;
  const bool success = sub_control_mode_->take(*control_mode_report, message_info);
  if (success) {
    Availability availability;
    availability.mode = control_mode_report->mode;
    availability.stop = availability_msg->stop;
    availability.autonomous = availability_msg->autonomous;
    availability.local = availability_msg->local;
    availability.remote = availability_msg->remote;
    availability.emergency_stop = availability_msg->emergency_stop;
    availability.comfortable_stop = availability_msg->comfortable_stop;
    availability.pull_over = availability_msg->pull_over;
    udp_availability_sender_->send(availability);
  } else {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "Failed to take control mode report");
  }
}

}  // namespace leader_election_converter
