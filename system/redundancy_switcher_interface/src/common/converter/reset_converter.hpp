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

#ifndef RESET__CONVERTER__RESET_CONVERTER_HPP_
#define RESET__CONVERTER__RESET_CONVERTER_HPP_

#include "udp_sender.hpp"
#include "udp_receiver.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/srv/redundancy_switcher_reset.hpp>

#include <memory>
#include <string>

namespace redundancy_switcher_interface
{

struct ResetRequest
{
  bool request;
};

struct ResetResponse
{
  bool response;
};

class ResetConverter
{
public:
  explicit ResetConverter(rclcpp::Node * node);

  void setUdpSender(const std::string & dest_ip, const std::string & dest_port);
  void setUdpReceiver(const std::string & src_ip, const std::string & src_port);
  void setService();

private:
  rclcpp::Node * node_;
  std::unique_ptr<UdpSender<ResetRequest>> udp_reset_request_sender_;
  std::unique_ptr<UdpReceiver<ResetResponse>> udp_reset_response_receiver_;
  rclcpp::Service<autoware_adapi_v1_msgs::srv::RedundancySwitcherReset>::SharedPtr srv_reset_;

  void onResetRequest(
    const autoware_adapi_v1_msgs::srv::RedundancySwitcherReset::Request::SharedPtr,
    const autoware_adapi_v1_msgs::srv::RedundancySwitcherReset::Response::SharedPtr response);
};

}  // namespace redundancy_switcher_interface

#endif  // RESET__CONVERTER__RESET_CONVERTER_HPP_
