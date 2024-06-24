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

#ifndef COMMON__CONVERTER__MRM_CONVERTER_HPP_
#define COMMON__CONVERTER__MRM_CONVERTER_HPP_

#include "udp_receiver.hpp"
#include "udp_sender.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/mrm_behavior.hpp>
#include <tier4_system_msgs/msg/mrm_state.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace leader_election_converter
{

typedef struct MrmState
{
  tier4_system_msgs::msg::MrmState::_state_type state;
  tier4_system_msgs::msg::MrmBehavior::_type_type behavior;
} MrmState;

typedef struct MrmRequest
{
  tier4_system_msgs::msg::MrmBehavior::_type_type behavior;
} MrmRequest;

class MrmConverter
{
public:
  explicit MrmConverter(rclcpp::Node * node);
  ~MrmConverter();

  void setUdpSender(const std::string & dest_ip, const std::string & dest_port);
  void setUdpReceiver(const std::string & src_ip, const std::string & src_port);
  void setSubscriber();
  void setPublisher();

private:
  void startUdpReceiver(const std::string & src_ip, const std::string & src_port);
  void convertToUdp(const tier4_system_msgs::msg::MrmState::ConstSharedPtr mrm_state_msg);
  void convertToTopic(const MrmRequest & mrm_request);

  rclcpp::Node * node_;
  std::unique_ptr<UdpSender<MrmState>> udp_mrm_state_sender_;
  std::unique_ptr<UdpReceiver<MrmRequest>> udp_mrm_request_receiver_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<tier4_system_msgs::msg::MrmState>::SharedPtr sub_mrm_state_;
  rclcpp::Publisher<tier4_system_msgs::msg::MrmBehavior>::SharedPtr pub_mrm_request_;

  std::thread udp_receiver_thread_;
  std::atomic<bool> is_udp_receiver_running_;
};

}  // namespace leader_election_converter

#endif  // COMMON__CONVERTER__MRM_CONVERTER_HPP_
