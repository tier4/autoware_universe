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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_

#include <agnocast/agnocast.hpp>
#include <autoware/component_interface_utils/rclcpp/exceptions.hpp>
#include <autoware/component_interface_utils/rclcpp/interface.hpp>
#include <rclcpp/node.hpp>

#include <tier4_system_msgs/msg/service_log.hpp>

#include <optional>
#include <string>
#include <utility>

namespace autoware::component_interface_utils
{

/// The wrapper class of agnocast::Client for logging.
template <class SpecT>
class Client
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Client)
  using SpecType = SpecT;
  using WrapType = agnocast::Client<typename SpecT::Service>;
  using RequestT = typename WrapType::RequestT;
  using ResponseT = typename WrapType::ResponseT;
  using SharedResponse = agnocast::ipc_shared_ptr<ResponseT>;
  using SharedRequest = std::shared_ptr<typename SpecT::Service::Request>;
  using ServiceLog = tier4_system_msgs::msg::ServiceLog;

  /// Constructor.
  Client(NodeInterface::SharedPtr interface, rclcpp::CallbackGroup::SharedPtr group)
  : interface_(interface)
  {
    std::visit(
      [this, &group](auto * n) {
        client_ = std::make_shared<WrapType>(n, SpecT::name, rclcpp::ServicesQoS(), group);
      },
      interface->node_variant);
  }

  /// Send request (synchronous). Takes a standard ROS request shared_ptr for compatibility.
  SharedResponse call(
    const SharedRequest request, std::optional<double> timeout = std::nullopt)
  {
    if (!client_->service_is_ready()) {
      interface_->log(ServiceLog::ERROR_UNREADY, SpecType::name);
      throw ServiceUnready(SpecT::name);
    }

#ifdef ROS_DISTRO_GALACTIC
    using rosidl_generator_traits::to_yaml;
#endif
    interface_->log(ServiceLog::CLIENT_REQUEST, SpecType::name, to_yaml(*request));

    auto loaned_req = client_->borrow_loaned_request();
    static_cast<typename SpecT::Service::Request &>(*loaned_req) = *request;
    auto future_and_id = client_->async_send_request(std::move(loaned_req));

    if (timeout) {
      const auto duration = std::chrono::duration<double, std::ratio<1>>(timeout.value());
      if (future_and_id.future.wait_for(duration) != std::future_status::ready) {
        interface_->log(ServiceLog::ERROR_TIMEOUT, SpecType::name);
        throw ServiceTimeout(SpecT::name);
      }
    }
    auto response = future_and_id.future.get();
    interface_->log(ServiceLog::CLIENT_RESPONSE, SpecType::name, to_yaml(*response));
    return response;
  }

  /// Send request (async, no callback).
  typename WrapType::Future async_send_request(SharedRequest request)
  {
#ifdef ROS_DISTRO_GALACTIC
    using rosidl_generator_traits::to_yaml;
#endif
    interface_->log(ServiceLog::CLIENT_REQUEST, SpecType::name, to_yaml(*request));

    auto loaned_req = client_->borrow_loaned_request();
    static_cast<typename SpecT::Service::Request &>(*loaned_req) = *request;
    auto future_and_id = client_->async_send_request(std::move(loaned_req));
    return std::move(future_and_id.future);
  }

  /// Send request (async, with callback).
  template <class CallbackT>
  typename WrapType::SharedFuture async_send_request(
    SharedRequest request, CallbackT && callback)
  {
#ifdef ROS_DISTRO_GALACTIC
    using rosidl_generator_traits::to_yaml;
#endif
    interface_->log(ServiceLog::CLIENT_REQUEST, SpecType::name, to_yaml(*request));

    const auto wrapped = [this, callback](typename WrapType::SharedFuture future) {
      interface_->log(ServiceLog::CLIENT_RESPONSE, SpecType::name, to_yaml(*future.get()));
      callback(future);
    };

    auto loaned_req = client_->borrow_loaned_request();
    static_cast<typename SpecT::Service::Request &>(*loaned_req) = *request;
    auto future_and_id = client_->async_send_request(std::move(loaned_req), wrapped);
    return std::move(future_and_id.future);
  }

  /// Check if the service is ready.
  bool service_is_ready() const { return client_->service_is_ready(); }

private:
  RCLCPP_DISABLE_COPY(Client)
  typename WrapType::SharedPtr client_;
  NodeInterface::SharedPtr interface_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_
