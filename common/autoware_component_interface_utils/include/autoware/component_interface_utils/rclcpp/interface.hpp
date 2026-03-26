// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_

#include <agnocast/agnocast.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/service_log.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <variant>

namespace autoware::component_interface_utils
{

struct NodeInterface
{
  using SharedPtr = std::shared_ptr<NodeInterface>;
  using ServiceLog = tier4_system_msgs::msg::ServiceLog;

  explicit NodeInterface(rclcpp::Node * node)
  : node_variant(node), rclcpp_logger(node->get_logger())
  {
    this->node = node;
    this->logger = agnocast::create_publisher<ServiceLog>(node, "/service_log", rclcpp::QoS(10));

    node_name = node->get_namespace();
    if (node_name.empty() || node_name.back() != '/') {
      node_name += "/";
    }
    node_name += node->get_name();
  }

  explicit NodeInterface(agnocast::Node * node)
  : node_variant(node), rclcpp_logger(node->get_logger())
  {
    this->node = nullptr;
    this->logger = agnocast::create_publisher<ServiceLog>(node, "/service_log", rclcpp::QoS(10));

    node_name = node->get_namespace();
    if (node_name.empty() || node_name.back() != '/') {
      node_name += "/";
    }
    node_name += node->get_name();
  }

  void log(ServiceLog::_type_type type, const std::string & name, const std::string & yaml = "")
  {
    static const auto type_text = std::unordered_map<ServiceLog::_type_type, std::string>(
      {{ServiceLog::CLIENT_REQUEST, "client call"},
       {ServiceLog::SERVER_REQUEST, "server call"},
       {ServiceLog::SERVER_RESPONSE, "server exit"},
       {ServiceLog::CLIENT_RESPONSE, "client exit"},
       {ServiceLog::ERROR_UNREADY, "client unready"},
       {ServiceLog::ERROR_TIMEOUT, "client timeout"}});
    RCLCPP_DEBUG_STREAM(rclcpp_logger, type_text.at(type) << ": " << name);

    auto loaned_msg = logger->borrow_loaned_message();
    loaned_msg->stamp = get_now();
    loaned_msg->type = type;
    loaned_msg->name = name;
    loaned_msg->node = node_name;
    loaned_msg->yaml = yaml;
    logger->publish(std::move(loaned_msg));
  }

  rclcpp::Time get_now() const
  {
    return std::visit(
      [](auto * n) -> rclcpp::Time { return n->now(); }, node_variant);
  }

  rclcpp::Node * node;  // kept for backward compatibility with code that uses interface_->node
  std::variant<rclcpp::Node *, agnocast::Node *> node_variant;
  rclcpp::Logger rclcpp_logger;
  agnocast::Publisher<ServiceLog>::SharedPtr logger;
  std::string node_name;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_
