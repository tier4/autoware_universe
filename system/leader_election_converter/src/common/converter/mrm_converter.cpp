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

#include "mrm_converter.hpp"

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>

namespace leader_election_converter
{

MrmConverter::MrmConverter(rclcpp::Node * node) : node_(node), is_udp_receiver_running_(true)
{
}

MrmConverter::~MrmConverter()
{
  is_udp_receiver_running_ = false;
  udp_mrm_request_receiver_->~UdpReceiver();
  if (udp_receiver_thread_.joinable()) {
    udp_receiver_thread_.join();
  }
}

void MrmConverter::setUdpSender(const std::string & dest_ip, const std::string & dest_port)
{
  udp_mrm_state_sender_ = std::make_unique<UdpSender<MrmState>>(dest_ip, dest_port);
}

void MrmConverter::setUdpReceiver(const std::string & src_ip, const std::string & src_port)
{
  udp_receiver_thread_ = std::thread(&MrmConverter::startUdpReceiver, this, src_ip, src_port);
}

void MrmConverter::startUdpReceiver(const std::string & src_ip, const std::string & src_port)
{
  try {
    udp_mrm_request_receiver_ = std::make_unique<UdpReceiver<MrmRequest>>(
      src_ip, src_port, std::bind(&MrmConverter::convertToTopic, this, std::placeholders::_1));
    while (is_udp_receiver_running_) {
      udp_mrm_request_receiver_->receive();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Error in UDP receiver thread: %s", e.what());
  }
}

void MrmConverter::setSubscriber()
{
  const auto qos = rclcpp::QoS(1);
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  sub_mrm_state_ = node_->create_subscription<tier4_system_msgs::msg::MrmState>(
    "~/input/mrm_state", qos, std::bind(&MrmConverter::convertToUdp, this, std::placeholders::_1),
    options);
}

void MrmConverter::setPublisher()
{
  pub_mrm_request_ = node_->create_publisher<tier4_system_msgs::msg::MrmBehavior>(
    "~/output/mrm_request", rclcpp::QoS{1});
}

void MrmConverter::convertToUdp(
  const tier4_system_msgs::msg::MrmState::ConstSharedPtr mrm_state_msg)
{
  MrmState mrm_state;
  mrm_state.state = mrm_state_msg->state;
  mrm_state.behavior = mrm_state_msg->behavior.type;

  udp_mrm_state_sender_->send(mrm_state);
}

void MrmConverter::convertToTopic(const MrmRequest & mrm_request)
{
  tier4_system_msgs::msg::MrmBehavior mrm_request_msg;
  mrm_request_msg.stamp = node_->now();
  mrm_request_msg.type = mrm_request.behavior;

  pub_mrm_request_->publish(mrm_request_msg);
}

}  // namespace leader_election_converter
