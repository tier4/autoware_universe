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

#include "reset_converter.hpp"

#include "rclcpp/rclcpp.hpp"

#include <autoware_adapi_v1_msgs/msg/response_status.hpp>

#include <memory>
#include <string>

namespace redundancy_switcher_interface
{

ResetConverter::ResetConverter(rclcpp::Node * node) : node_(node)
{
}

void ResetConverter::setUdpSender(const std::string & dest_ip, const std::string & dest_port)
{
  udp_reset_request_sender_ = std::make_unique<UdpSender<ResetRequest>>(dest_ip, dest_port);
}

void ResetConverter::setUdpReceiver(const std::string & src_ip, const std::string & src_port)
{
  udp_reset_response_receiver_ =
    std::make_unique<UdpReceiver<ResetResponse>>(src_ip, src_port, false);
}

void ResetConverter::setService()
{
  srv_reset_ = node_->create_service<autoware_adapi_v1_msgs::srv::RedundancySwitcherReset>(
    "~/service/reset",
    std::bind(&ResetConverter::onResetRequest, this, std::placeholders::_1, std::placeholders::_2));
}

void ResetConverter::onResetRequest(
  const autoware_adapi_v1_msgs::srv::RedundancySwitcherReset::Request::SharedPtr,
  const autoware_adapi_v1_msgs::srv::RedundancySwitcherReset::Response::SharedPtr response)
{
  auto async_task = std::async(std::launch::async, [this, response]() {
    ResetRequest reset_request;
    reset_request.request = true;
    udp_reset_request_sender_->send(reset_request);
    RCLCPP_INFO(node_->get_logger(), "Reset request sent.");

    ResetResponse udp_response;
    try {
      bool result = udp_reset_response_receiver_->receive(udp_response, 30);
      ;
      if (!result) {
        response->status.success = false;
        response->status.code = autoware_adapi_v1_msgs::msg::ResponseStatus::SERVICE_TIMEOUT;
        response->status.message = "Timed out waiting for response.";
        RCLCPP_WARN(node_->get_logger(), "Timed out waiting for response.");
      } else if (!udp_response.response) {
        response->status.success = false;
        response->status.code = autoware_adapi_v1_msgs::msg::ResponseStatus::NO_EFFECT;
        response->status.message = "Failed to reset in redundancy switcher.";
        RCLCPP_WARN(node_->get_logger(), "Failed to reset in redundancy switcher.");
      } else {
        response->status.success = true;
        response->status.message = "Reset successfully.";
        RCLCPP_INFO(node_->get_logger(), "Reset successfully.");
      }
    } catch (const std::exception & e) {
      response->status.success = false;
      response->status.code = autoware_adapi_v1_msgs::msg::ResponseStatus::TRANSFORM_ERROR;
      response->status.message = "Failed to receive UDP response.";
      RCLCPP_WARN(node_->get_logger(), "Failed to receive UDP response: %s", e.what());
    }
  });
}

}  // namespace redundancy_switcher_interface
