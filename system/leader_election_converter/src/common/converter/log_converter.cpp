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

#include "log_converter.hpp"

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>

namespace leader_election_converter
{

LogConverter::LogConverter(rclcpp::Node * node)
: node_(node), is_election_communication_running_(true), is_election_status_running_(true)
{
}

LogConverter::~LogConverter()
{
  is_election_communication_running_ = false;
  udp_election_communication_receiver_->~UdpReceiver();
  is_election_status_running_ = false;
  udp_election_status_receiver_->~UdpReceiver();
  if (udp_election_communication_thread_.joinable()) {
    udp_election_communication_thread_.join();
  }
  if (udp_election_status_thread_.joinable()) {
    udp_election_status_thread_.join();
  }
}

void LogConverter::setUdpElectionCommunicationReceiver(
  const std::string & src_ip, const std::string & src_port)
{
  udp_election_communication_thread_ =
    std::thread(&LogConverter::startUdpElectionCommunicationReceiver, this, src_ip, src_port);
}

void LogConverter::startUdpElectionCommunicationReceiver(
  const std::string & src_ip, const std::string & src_port)
{
  try {
    udp_election_communication_receiver_ = std::make_unique<UdpReceiver<ElectionCommunication>>(
      src_ip, src_port,
      std::bind(&LogConverter::convertElectionCommunicationToTopic, this, std::placeholders::_1));
    while (is_election_communication_running_) {
      udp_election_communication_receiver_->receive();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Error in UDP receiver thread: %s", e.what());
  }
}

void LogConverter::setUdpElectionStatusReceiver(
  const std::string & src_ip, const std::string & src_port)
{
  udp_election_status_thread_ =
    std::thread(&LogConverter::startUdpElectionStatusReceiver, this, src_ip, src_port);
}

void LogConverter::startUdpElectionStatusReceiver(
  const std::string & src_ip, const std::string & src_port)
{
  try {
    udp_election_status_receiver_ = std::make_unique<UdpReceiver<ElectionStatus>>(
      src_ip, src_port,
      std::bind(&LogConverter::convertElectionStatusToTopic, this, std::placeholders::_1));
    while (is_election_status_running_) {
      udp_election_status_receiver_->receive();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Error in UDP receiver thread: %s", e.what());
  }
}

void LogConverter::setPublisher()
{
  pub_election_communication_ =
    node_->create_publisher<tier4_system_msgs::msg::ElectionCommunication>(
      "~/output/election_communication", rclcpp::QoS{1});
  pub_election_status_ = node_->create_publisher<tier4_system_msgs::msg::ElectionStatus>(
    "~/output/election_status", rclcpp::QoS{1});
  pub_over_all_mrm_state_ = node_->create_publisher<autoware_adapi_v1_msgs::msg::MrmState>(
    "~/output/over_all_mrm_state", rclcpp::QoS{1});
}

void LogConverter::convertElectionCommunicationToTopic(const ElectionCommunication & node_msg)
{
  tier4_system_msgs::msg::ElectionCommunication msg;
  msg.stamp = node_->now();
  msg.node_id = (node_msg.msg >> 8) & 0xFF;
  msg.type = node_msg.msg & 0xFF;
  msg.term = (node_msg.msg >> 16) & 0xFF;
  msg.link = (node_msg.msg >> 24) & 0xFF;
  msg.heartbeat = (node_msg.msg >> 56) & 0x0F;
  msg.checksum = (node_msg.msg >> 60) & 0x0F;
  pub_election_communication_->publish(msg);
}

void LogConverter::convertElectionStatusToTopic(const ElectionStatus & status)
{
  tier4_system_msgs::msg::ElectionStatus election_status;
  autoware_adapi_v1_msgs::msg::MrmState mrm_status;

  election_status.stamp = node_->now();
  election_status.leader_id = status.leader_id;
  election_status.path_info = status.path_info;
  election_status.mrm_state.state = status.state;
  election_status.mrm_state.behavior.type = status.behavior;
  election_status.election_start_count = status.election_start_count;
  election_status.in_election = status.in_election;
  election_status.has_received_availability = status.has_received_availability;
  election_status.has_received_mrm_state = status.has_received_mrm_state;
  election_status.is_autoware_emergency = status.is_autoware_emergency;
  election_status.is_main_ecu_connected = status.is_main_ecu_connected;
  election_status.is_sub_ecu_connected = status.is_sub_ecu_connected;
  election_status.is_main_vcu_connected = status.is_main_vcu_connected;
  election_status.is_sub_vcu_connected = status.is_sub_vcu_connected;
  pub_election_status_->publish(election_status);

  mrm_status.stamp = node_->now();
  mrm_status.state = status.state;
  mrm_status.behavior = status.behavior;
  pub_over_all_mrm_state_->publish(mrm_status);
}

}  // namespace leader_election_converter
