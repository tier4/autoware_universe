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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_

#include <autoware/component_interface_utils/rclcpp/interface.hpp>
#include <autoware/component_interface_utils/rclcpp/service_client.hpp>
#include <autoware/component_interface_utils/rclcpp/service_server.hpp>
#include <autoware/component_interface_utils/rclcpp/topic_publisher.hpp>
#include <autoware/component_interface_utils/rclcpp/topic_subscription.hpp>
#include <autoware/component_interface_utils/specs.hpp>
#include <rclcpp/rclcpp.hpp>

#include <utility>

namespace autoware::component_interface_utils
{

/// Create a client wrapper for logging. This is a private implementation.
template <class SpecT>
typename Client<SpecT>::SharedPtr create_client_impl(
  NodeInterface::SharedPtr interface, rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<Client<SpecT>>(interface, group);
}

/// Create a service wrapper for logging. This is a private implementation.
template <class SpecT, class CallbackT>
typename Service<SpecT>::SharedPtr create_service_impl(
  NodeInterface::SharedPtr interface, CallbackT && callback,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<Service<SpecT>>(interface, std::forward<CallbackT>(callback), group);
}

/// Create a publisher using traits like services. This is a private implementation.
template <class SpecT, class NodeT>
typename Publisher<SpecT>::SharedPtr create_publisher_impl(NodeT * node)
{
  auto publisher =
    agnocast::create_publisher<typename SpecT::Message>(node, SpecT::name, get_qos<SpecT>());
  return Publisher<SpecT>::make_shared(publisher);
}

/// Create a subscription using traits like services. This is a private implementation.
template <class SpecT, class NodeT, class CallbackT>
typename Subscription<SpecT>::SharedPtr create_subscription_impl(
  NodeT * node, CallbackT && callback)
{
  auto subscription = agnocast::create_subscription<typename SpecT::Message>(
    node, SpecT::name, get_qos<SpecT>(), std::forward<CallbackT>(callback));
  return Subscription<SpecT>::make_shared(subscription);
}

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_
