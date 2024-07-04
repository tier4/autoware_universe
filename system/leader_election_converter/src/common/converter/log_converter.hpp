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

#ifndef COMMON__CONVERTER__LOG_CONVERTER_HPP_
#define COMMON__CONVERTER__LOG_CONVERTER_HPP_

#include "udp_receiver.hpp"
#include "udp_sender.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <tier4_system_msgs/msg/election_communication.hpp>
#include <tier4_system_msgs/msg/election_status.hpp>
#include <tier4_system_msgs/msg/mrm_state.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace leader_election_converter
{

typedef struct ElectionCommunication
{
  uint64_t msg;
} ElectionCommunication;

typedef struct ElectionStatus
{
  tier4_system_msgs::msg::ElectionStatus::_leader_id_type leader_id;
  tier4_system_msgs::msg::ElectionStatus::_path_info_type path_info;
  tier4_system_msgs::msg::MrmState::_state_type state;
  tier4_system_msgs::msg::MrmBehavior::_type_type behavior;
  tier4_system_msgs::msg::ElectionStatus::_election_start_count_type election_start_count;
  tier4_system_msgs::msg::ElectionStatus::_in_election_type in_election;
  tier4_system_msgs::msg::ElectionStatus::_has_received_availability_type has_received_availability;
  tier4_system_msgs::msg::ElectionStatus::_has_received_mrm_state_type has_received_mrm_state;
  tier4_system_msgs::msg::ElectionStatus::_is_autoware_emergency_type is_autoware_emergency;
  tier4_system_msgs::msg::ElectionStatus::_is_main_ecu_connected_type is_main_ecu_connected;
  tier4_system_msgs::msg::ElectionStatus::_is_sub_ecu_connected_type is_sub_ecu_connected;
  tier4_system_msgs::msg::ElectionStatus::_is_main_vcu_connected_type is_main_vcu_connected;
  tier4_system_msgs::msg::ElectionStatus::_is_sub_vcu_connected_type is_sub_vcu_connected;
} ElectionStatus;

class LogConverter
{
public:
  explicit LogConverter(rclcpp::Node * node);
  ~LogConverter();

  void setUdpElectionCommunicationReceiver(
    const std::string & src_ip, const std::string & src_port);
  void setUdpElectionStatusReceiver(const std::string & src_ip, const std::string & src_port);
  void setPublisher();

private:
  void startUdpElectionCommunicationReceiver(
    const std::string & src_ip, const std::string & src_port);
  void startUdpElectionStatusReceiver(const std::string & src_ip, const std::string & src_port);
  void convertElectionCommunicationToTopic(const ElectionCommunication & node_msg);
  void convertElectionStatusToTopic(const ElectionStatus & status);

  rclcpp::Node * node_;
  std::unique_ptr<UdpReceiver<ElectionCommunication>> udp_election_communication_receiver_;
  std::unique_ptr<UdpReceiver<ElectionStatus>> udp_election_status_receiver_;
  rclcpp::Publisher<tier4_system_msgs::msg::ElectionCommunication>::SharedPtr
    pub_election_communication_;
  rclcpp::Publisher<tier4_system_msgs::msg::ElectionStatus>::SharedPtr pub_election_status_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::MrmState>::SharedPtr pub_over_all_mrm_state_;

  std::thread udp_election_communication_thread_;
  std::thread udp_election_status_thread_;
  std::atomic<bool> is_election_communication_running_;
  std::atomic<bool> is_election_status_running_;
};

}  // namespace leader_election_converter

#endif  // COMMON__CONVERTER__LOG_CONVERTER_HPP_
