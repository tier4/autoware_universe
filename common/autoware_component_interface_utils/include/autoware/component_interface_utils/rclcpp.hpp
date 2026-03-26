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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_

#include <autoware/component_interface_utils/rclcpp/create_interface.hpp>
#include <autoware/component_interface_utils/rclcpp/interface.hpp>
#include <autoware/component_interface_utils/rclcpp/service_client.hpp>
#include <autoware/component_interface_utils/rclcpp/service_server.hpp>
#include <autoware/component_interface_utils/rclcpp/topic_publisher.hpp>
#include <autoware/component_interface_utils/rclcpp/topic_subscription.hpp>

#include <memory>
#include <optional>
#include <utility>

namespace autoware::component_interface_utils
{

class NodeAdaptor
{
private:
  using CallbackGroup = rclcpp::CallbackGroup::SharedPtr;

  template <class SharedPtrT, class InstanceT>
  using MessagePtrCallback = void (InstanceT::*)(
    const agnocast::ipc_shared_ptr<const typename SharedPtrT::element_type::SpecType::Message> &);
  template <class SharedPtrT, class InstanceT>
  using MessageRefCallback =
    void (InstanceT::*)(const typename SharedPtrT::element_type::SpecType::Message &);

  template <class SharedPtrT, class InstanceT>
  using ServiceCallback = void (InstanceT::*)(
    const agnocast::ipc_shared_ptr<typename SharedPtrT::element_type::RequestT> &,
    agnocast::ipc_shared_ptr<typename SharedPtrT::element_type::ResponseT> &);

  // Overloads for ROS-style callback signatures (std::shared_ptr<const Message>).
  template <class SharedPtrT, class InstanceT>
  using MessageConstSharedPtrCallback = void (InstanceT::*)(
    const std::shared_ptr<const typename SharedPtrT::element_type::SpecType::Message>);
  template <class SharedPtrT, class InstanceT>
  using RosServiceCallback = void (InstanceT::*)(
    const typename SharedPtrT::element_type::SpecType::Service::Request::SharedPtr,
    const typename SharedPtrT::element_type::SpecType::Service::Response::SharedPtr);

public:
  /// Constructor.
  explicit NodeAdaptor(rclcpp::Node * node) { interface_ = std::make_shared<NodeInterface>(node); }
  explicit NodeAdaptor(agnocast::Node * node) { interface_ = std::make_shared<NodeInterface>(node); }

  /// Create a client wrapper for logging.
  template <class SharedPtrT>
  void init_cli(SharedPtrT & cli, CallbackGroup group = nullptr) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    cli = create_client_impl<SpecT>(interface_, group);
  }

  /// Create a service wrapper for logging.
  template <class SharedPtrT, class CallbackT>
  void init_srv(SharedPtrT & srv, CallbackT && callback, CallbackGroup group = nullptr) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    srv = create_service_impl<SpecT>(interface_, std::forward<CallbackT>(callback), group);
  }

  /// Create a publisher using traits like services.
  template <class SharedPtrT>
  void init_pub(SharedPtrT & pub) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    std::visit(
      [&pub](auto * n) { pub = create_publisher_impl<SpecT>(n); }, interface_->node_variant);
  }

  /// Create a subscription using traits like services.
  template <class SharedPtrT, class CallbackT>
  void init_sub(SharedPtrT & sub, CallbackT && callback) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    std::visit(
      [&sub, &callback](auto * n) {
        sub = create_subscription_impl<SpecT>(n, std::forward<CallbackT>(callback));
      },
      interface_->node_variant);
  }

  /// Relay message.
  template <class P, class S>
  void relay_message(P & pub, S & sub) const
  {
    using Message = typename P::element_type::SpecType::Message;
    init_pub(pub);
    init_sub(
      sub, [pub](const agnocast::ipc_shared_ptr<const Message> & msg) { pub->publish(*msg); });
  }

  /// Relay service.
  template <class C, class S>
  void relay_service(
    C & cli, S & srv, CallbackGroup group, std::optional<double> timeout = std::nullopt) const
  {
    using ServiceType = typename C::element_type::SpecType::Service;
    init_cli(cli);
    init_srv(
      srv,
      [cli, timeout](const auto & req, auto & res) {
        auto ros_req = std::make_shared<typename ServiceType::Request>(
          static_cast<const typename ServiceType::Request &>(*req));
        auto response = cli->call(ros_req, timeout);
        static_cast<typename ServiceType::Response &>(*res) =
          static_cast<const typename ServiceType::Response &>(*response);
      },
      group);
  }

  /// Create a subscription wrapper for pointer callback.
  template <class SharedPtrT, class InstanceT>
  void init_sub(
    SharedPtrT & sub, InstanceT * instance,
    MessagePtrCallback<SharedPtrT, InstanceT> && callback) const
  {
    using std::placeholders::_1;
    init_sub(sub, std::bind(callback, instance, _1));
  }

  /// Create a subscription wrapper for reference callback.
  template <class SharedPtrT, class InstanceT>
  void init_sub(
    SharedPtrT & sub, InstanceT * instance,
    MessageRefCallback<SharedPtrT, InstanceT> && callback) const
  {
    using Message = typename SharedPtrT::element_type::SpecType::Message;
    auto bound = std::bind(callback, instance, std::placeholders::_1);
    init_sub(sub, [bound](const agnocast::ipc_shared_ptr<const Message> & msg) { bound(*msg); });
  }

  /// Create a service wrapper for logging.
  template <class SharedPtrT, class InstanceT>
  void init_srv(
    SharedPtrT & srv, InstanceT * instance, ServiceCallback<SharedPtrT, InstanceT> && callback,
    CallbackGroup group = nullptr) const
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    init_srv(srv, std::bind(callback, instance, _1, _2), group);
  }

  /// Create a subscription wrapper for ConstSharedPtr callback (compatibility with ROS-style
  /// signatures like void(Message::ConstSharedPtr)).
  template <class SharedPtrT, class InstanceT>
  void init_sub(
    SharedPtrT & sub, InstanceT * instance,
    MessageConstSharedPtrCallback<SharedPtrT, InstanceT> && callback) const
  {
    using Message = typename SharedPtrT::element_type::SpecType::Message;
    auto bound = std::bind(callback, instance, std::placeholders::_1);
    init_sub(sub, [bound](const agnocast::ipc_shared_ptr<const Message> & msg) {
      bound(std::make_shared<const Message>(*msg));
    });
  }

  /// Create a service wrapper for ROS-style callback (SharedPtr request/response).
  template <class SharedPtrT, class InstanceT>
  void init_srv(
    SharedPtrT & srv, InstanceT * instance,
    RosServiceCallback<SharedPtrT, InstanceT> && callback,
    CallbackGroup group = nullptr) const
  {
    using ServiceType = typename SharedPtrT::element_type::SpecType::Service;
    using RequestT = typename SharedPtrT::element_type::RequestT;
    using ResponseT = typename SharedPtrT::element_type::ResponseT;
    auto bound = std::bind(callback, instance, std::placeholders::_1, std::placeholders::_2);
    init_srv(
      srv,
      [bound](
        const agnocast::ipc_shared_ptr<RequestT> & req,
        agnocast::ipc_shared_ptr<ResponseT> & res) {
        auto ros_req =
          std::make_shared<typename ServiceType::Request>(
            static_cast<const typename ServiceType::Request &>(*req));
        auto ros_res =
          std::make_shared<typename ServiceType::Response>(
            static_cast<const typename ServiceType::Response &>(*res));
        bound(ros_req, ros_res);
        static_cast<typename ServiceType::Response &>(*res) = *ros_res;
      },
      group);
  }

private:
  // Use a node pointer because shared_from_this cannot be used in constructor.
  NodeInterface::SharedPtr interface_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_
