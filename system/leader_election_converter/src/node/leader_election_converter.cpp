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

#include "leader_election_converter.hpp"

#include <string>

namespace leader_election_converter
{

LeaderElectionConverter::LeaderElectionConverter(const rclcpp::NodeOptions & node_options)
: Node("leader_election_converter", node_options),
  availability_converter_(this),
  mrm_converter_(this),
  log_converter_(this)
{
  availability_dest_ip_ = declare_parameter<std::string>("availability_dest_ip");
  availability_dest_port_ = declare_parameter<std::string>("availability_dest_port");
  mrm_state_dest_ip_ = declare_parameter<std::string>("mrm_state_dest_ip");
  mrm_state_dest_port_ = declare_parameter<std::string>("mrm_state_dest_port");
  mrm_request_src_ip_ = declare_parameter<std::string>("mrm_request_src_ip");
  mrm_request_src_port_ = declare_parameter<std::string>("mrm_request_src_port");
  election_communication_src_ip_ = declare_parameter<std::string>("election_communication_src_ip");
  election_communication_src_port_ =
    declare_parameter<std::string>("election_communication_src_port");
  election_status_src_ip_ = declare_parameter<std::string>("election_status_src_ip");
  election_status_src_port_ = declare_parameter<std::string>("election_status_src_port");

  // convert udp packets of availability to topics
  availability_converter_.setUdpSender(availability_dest_ip_, availability_dest_port_);
  availability_converter_.setSubscriber();

  // convert topics of mrm state to udp packets
  mrm_converter_.setUdpSender(mrm_state_dest_ip_, mrm_state_dest_port_);
  mrm_converter_.setSubscriber();

  // convert udp packets of mrm request to topics
  mrm_converter_.setPublisher();
  mrm_converter_.setUdpReceiver(mrm_request_src_ip_, mrm_request_src_port_);

  // convert udp packets of election info to topics
  log_converter_.setPublisher();
  log_converter_.setUdpElectionCommunicationReceiver(
    election_communication_src_ip_, election_communication_src_port_);
  log_converter_.setUdpElectionStatusReceiver(election_status_src_ip_, election_status_src_port_);
}

}  // namespace leader_election_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(leader_election_converter::LeaderElectionConverter)
