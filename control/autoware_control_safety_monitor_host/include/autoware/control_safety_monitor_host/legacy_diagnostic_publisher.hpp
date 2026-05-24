// Copyright 2025 Autoware Foundation
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

#ifndef AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__LEGACY_DIAGNOSTIC_PUBLISHER_HPP_
#define AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__LEGACY_DIAGNOSTIC_PUBLISHER_HPP_

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autoware::control_safety_monitor_host
{

/// Publishes diagnostics with exact legacy status names (no host node prefix).
class LegacyDiagnosticPublisher
{
public:
  explicit LegacyDiagnosticPublisher(rclcpp::Node * node)
  : publisher_(node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(1)))
  {
  }

  void publish(rclcpp::Node * node, const diagnostic_msgs::msg::DiagnosticStatus & status)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = node->now();
    array.status.push_back(status);
    publisher_->publish(array);
  }

private:
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
};

}  // namespace autoware::control_safety_monitor_host

#endif  // AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__LEGACY_DIAGNOSTIC_PUBLISHER_HPP_
