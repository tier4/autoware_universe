// Copyright 2021 Tier IV, Inc.
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

#ifndef TIER4_API_UTILS__RCLCPP__PROXY_HPP_
#define TIER4_API_UTILS__RCLCPP__PROXY_HPP_

#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tier4_api_utils/rclcpp/client.hpp"
#include "tier4_api_utils/rclcpp/service.hpp"

#include <string>
#include <type_traits>
#include <utility>

namespace tier4_api_utils
{

namespace detail
{
/// Return a pointer to the agnocast-compatible base type (rclcpp::Node* or agnocast::Node*).
template <typename NodeT>
auto * to_agnocast_node(NodeT * node)
{
  if constexpr (std::is_base_of_v<agnocast::Node, NodeT>) {
    return static_cast<agnocast::Node *>(node);
  } else {
    return static_cast<rclcpp::Node *>(node);
  }
}
}  // namespace detail

template <class NodeT>
class ServiceProxyNodeInterface
{
public:
  // Use a raw pointer because shared_from_this cannot be used in constructor.
  explicit ServiceProxyNodeInterface(NodeT * node) { node_ = node; }

  template <typename ServiceT, typename CallbackT>
  typename Service<ServiceT>::SharedPtr create_service(
    const std::string & service_name, CallbackT && callback,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    auto wrapped_callback = Service<ServiceT>::template wrap<CallbackT>(
      std::forward<CallbackT>(callback), node_->get_logger());
    (void)qos_profile;
    auto * base_node = detail::to_agnocast_node(node_);
    return Service<ServiceT>::make_shared(std::make_shared<agnocast::Service<ServiceT>>(
      base_node, service_name, std::move(wrapped_callback), rclcpp::ServicesQoS(), group));
  }

  template <typename ServiceT>
  typename Client<ServiceT>::SharedPtr create_client(
    const std::string & service_name,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    (void)qos_profile;
    auto * base_node = detail::to_agnocast_node(node_);
    return Client<ServiceT>::make_shared(
      std::make_shared<agnocast::Client<ServiceT>>(
        base_node, service_name, rclcpp::ServicesQoS(), group),
      node_->get_logger());
  }

private:
  NodeT * node_;
};

}  // namespace tier4_api_utils

#endif  // TIER4_API_UTILS__RCLCPP__PROXY_HPP_
