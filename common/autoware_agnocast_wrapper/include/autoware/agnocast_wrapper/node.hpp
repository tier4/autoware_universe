// Copyright 2025 TIER IV, Inc.
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

#pragma once

#include "autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp"

#include <agnocast/node/agnocast_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace autoware::agnocast_wrapper
{

/// @brief Check if agnocast mode is enabled via ENABLE_AGNOCAST environment variable
/// @return true if ENABLE_AGNOCAST=1, false otherwise
bool use_agnocast();

/// @brief Type-erased timer wrapper that holds either agnocast or rclcpp timer
class Timer
{
public:
  using SharedPtr = std::shared_ptr<Timer>;
  virtual ~Timer() = default;
  virtual void cancel() {}
  virtual void reset() {}
  virtual bool is_canceled() const { return false; }
  virtual void set_period(std::chrono::nanoseconds period) = 0;
  virtual std::chrono::nanoseconds time_until_trigger() const = 0;
};

class AgnocastTimer : public Timer
{
  agnocast::TimerBase::SharedPtr timer_;

public:
  explicit AgnocastTimer(agnocast::TimerBase::SharedPtr timer) : timer_(std::move(timer)) {}
  void cancel() override { timer_->cancel(); }
  void reset() override { timer_->reset(); }
  bool is_canceled() const override { return timer_->is_canceled(); }
  void set_period(std::chrono::nanoseconds /*period*/) override
  {
    // TODO(agnocast): agnocast TimerBase does not support dynamic period change yet
  }
  std::chrono::nanoseconds time_until_trigger() const override
  {
    // TODO(agnocast): agnocast TimerBase does not support time_until_trigger yet
    return std::chrono::nanoseconds(0);
  }
};

class RclcppTimer : public Timer
{
  rclcpp::TimerBase::SharedPtr timer_;

public:
  explicit RclcppTimer(rclcpp::TimerBase::SharedPtr timer) : timer_(std::move(timer)) {}
  void cancel() override { timer_->cancel(); }
  void reset() override { timer_->reset(); }
  bool is_canceled() const override { return timer_->is_canceled(); }
  void set_period(std::chrono::nanoseconds period) override
  {
    int64_t old_period = 0;
    rcl_timer_exchange_period(timer_->get_timer_handle().get(), period.count(), &old_period);
  }
  std::chrono::nanoseconds time_until_trigger() const override
  {
    return timer_->time_until_trigger();
  }
};

/// @brief Node wrapper class that can switch between rclcpp::Node and agnocast::Node at runtime
/// based on the ENABLE_AGNOCAST environment variable.
class Node
{
public:
  using SharedPtr = std::shared_ptr<Node>;

  /// @brief Constructor with node name (Component Node compatible)
  /// @param node_name The name of the node
  /// @param options Node options
  explicit Node(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Constructor with node name and namespace
  /// @param node_name The name of the node
  /// @param namespace_ The namespace of the node
  /// @param options Node options
  explicit Node(
    const std::string & node_name, const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~Node() = default;

  // ===== Basic information =====
  std::string get_name() const;
  std::string get_namespace() const;
  std::string get_fully_qualified_name() const;
  rclcpp::Logger get_logger() const;

  // ===== Time =====
  rclcpp::Clock::SharedPtr get_clock();
  rclcpp::Time now() const;

  // ===== Node interfaces =====
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface();
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface();

  // ===== Callback groups =====
  rclcpp::CallbackGroup::SharedPtr create_callback_group(
    rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node = true);

  // ===== Parameters (non-template) =====
  const rclcpp::ParameterValue & declare_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & descriptor =
      rcl_interfaces::msg::ParameterDescriptor{},
    bool ignore_override = false);

  const rclcpp::ParameterValue & declare_parameter(
    const std::string & name, rclcpp::ParameterType type,
    const rcl_interfaces::msg::ParameterDescriptor & descriptor =
      rcl_interfaces::msg::ParameterDescriptor{},
    bool ignore_override = false);

  bool has_parameter(const std::string & name) const;
  void undeclare_parameter(const std::string & name);
  rclcpp::Parameter get_parameter(const std::string & name) const;
  bool get_parameter(const std::string & name, rclcpp::Parameter & parameter) const;
  std::vector<rclcpp::Parameter> get_parameters(const std::vector<std::string> & names) const;

  rcl_interfaces::msg::SetParametersResult set_parameter(const rclcpp::Parameter & parameter);
  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);
  rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
    const std::vector<rclcpp::Parameter> & parameters);

  rcl_interfaces::msg::ParameterDescriptor describe_parameter(const std::string & name) const;
  std::vector<rcl_interfaces::msg::ParameterDescriptor> describe_parameters(
    const std::vector<std::string> & names) const;
  std::vector<uint8_t> get_parameter_types(const std::vector<std::string> & names) const;
  rcl_interfaces::msg::ListParametersResult list_parameters(
    const std::vector<std::string> & prefixes, uint64_t depth) const;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr add_on_set_parameters_callback(
    rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType callback);
  void remove_on_set_parameters_callback(
    const rclcpp::node_interfaces::OnSetParametersCallbackHandle * const handler);

  // ===== Parameters (template) =====
  template <typename ParameterT>
  ParameterT declare_parameter(
    const std::string & name, const ParameterT & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & descriptor =
      rcl_interfaces::msg::ParameterDescriptor{},
    bool ignore_override = false)
  {
    return declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor, ignore_override)
      .get<ParameterT>();
  }

  template <typename ParameterT>
  ParameterT declare_parameter(
    const std::string & name,
    const rcl_interfaces::msg::ParameterDescriptor & descriptor =
      rcl_interfaces::msg::ParameterDescriptor{},
    bool ignore_override = false)
  {
    rclcpp::ParameterValue value{ParameterT{}};
    return declare_parameter(name, value.get_type(), descriptor, ignore_override)
      .template get<ParameterT>();
  }

  template <typename ParameterT>
  bool get_parameter(const std::string & name, ParameterT & parameter) const
  {
    rclcpp::Parameter param;
    if (get_parameter(name, param)) {
      parameter = param.get_value<ParameterT>();
      return true;
    }
    return false;
  }

  template <typename ParameterT>
  bool get_parameters(const std::string & prefix, std::map<std::string, ParameterT> & values) const
  {
    std::map<std::string, rclcpp::Parameter> params;
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params_interface;
    if (use_agnocast_) {
      params_interface = agnocast_node_->get_node_parameters_interface();
    } else {
      params_interface = rclcpp_node_->get_node_parameters_interface();
    }
    bool result = params_interface->get_parameters_by_prefix(prefix, params);
    if (result) {
      for (const auto & param : params) {
        values[param.first] = param.second.get_value<ParameterT>();
      }
    }
    return result;
  }

#ifdef USE_AGNOCAST_ENABLED
  // ===== Timer =====
  template <typename DurationRepT, typename DurationT, typename CallbackT>
  Timer::SharedPtr create_timer(
    std::chrono::duration<DurationRepT, DurationT> period, CallbackT && callback)
  {
    if (use_agnocast_) {
      agnocast::TimerBase::SharedPtr timer =
        agnocast_node_->create_timer(period, std::forward<CallbackT>(callback));
      return std::make_shared<AgnocastTimer>(std::move(timer));
    } else {
      auto timer = rclcpp::create_timer(
        rclcpp_node_, rclcpp_node_->get_clock(), period, std::forward<CallbackT>(callback));
      return std::make_shared<RclcppTimer>(std::move(timer));
    }
  }

  /// Overload matching rclcpp::create_timer(node, clock, period, callback) signature.
  /// The node and clock arguments are accepted for API compatibility but ignored internally;
  /// the wrapper uses its own node and clock.
  template <typename DurationRepT, typename DurationT, typename CallbackT>
  Timer::SharedPtr create_timer(
    [[maybe_unused]] Node * node, [[maybe_unused]] rclcpp::Clock::SharedPtr clock,
    std::chrono::duration<DurationRepT, DurationT> period, CallbackT && callback)
  {
    return create_timer(period, std::forward<CallbackT>(callback));
  }

  // ===== AllPollingSubscriber =====
  template <typename MessageT>
  typename AllPollingSubscriber<MessageT>::SharedPtr create_all_polling_subscriber(
    const std::string & topic_name, const rclcpp::QoS & qos)
  {
    if (use_agnocast_) {
      return std::make_shared<AgnocastAllPollingSubscriber<MessageT>>(
        agnocast_node_.get(), topic_name, qos);
    } else {
      return std::make_shared<ROS2AllPollingSubscriber<MessageT>>(
        rclcpp_node_.get(), topic_name, qos);
    }
  }

  template <typename MessageT>
  typename AllPollingSubscriber<MessageT>::SharedPtr create_all_polling_subscriber(
    const std::string & topic_name, size_t qos_history_depth)
  {
    return create_all_polling_subscriber<MessageT>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
  }

  // ===== Publisher =====
  template <typename MessageT>
  typename Publisher<MessageT>::SharedPtr create_publisher(
    const std::string & topic_name, const rclcpp::QoS & qos,
    const agnocast::PublisherOptions & options = agnocast::PublisherOptions{})
  {
    if (use_agnocast_) {
      return std::make_shared<AgnocastPublisher<MessageT>>(
        agnocast_node_.get(), topic_name, qos, options);
    } else {
      return std::make_shared<ROS2Publisher<MessageT>>(rclcpp_node_.get(), topic_name, qos, options);
    }
  }

  template <typename MessageT>
  typename Publisher<MessageT>::SharedPtr create_publisher(
    const std::string & topic_name, size_t qos_history_depth)
  {
    return create_publisher<MessageT>(topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
  }

  // ===== Subscription =====
  template <typename MessageT, typename Func>
  typename Subscription<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    const agnocast::SubscriptionOptions & options = agnocast::SubscriptionOptions{})
  {
    if (use_agnocast_) {
      return std::make_shared<Subscription<MessageT>>(
        agnocast_node_.get(), topic_name, qos, std::forward<Func>(callback), options);
    } else {
      return std::make_shared<Subscription<MessageT>>(
        rclcpp_node_.get(), topic_name, qos, std::forward<Func>(callback), options);
    }
  }

  template <typename MessageT, typename Func>
  typename Subscription<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, size_t qos_history_depth, Func && callback,
    const agnocast::SubscriptionOptions & options = agnocast::SubscriptionOptions{})
  {
    return create_subscription<MessageT>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), std::forward<Func>(callback),
      options);
  }

  // ===== Polling Subscriber =====
  template <typename MessageT>
  typename PollingSubscriber<MessageT>::SharedPtr create_polling_subscriber(
    const std::string & topic_name, const rclcpp::QoS & qos)
  {
    if (use_agnocast_) {
      return std::make_shared<AgnocastPollingSubscriber<MessageT>>(
        agnocast_node_.get(), topic_name, qos);
    } else {
      return std::make_shared<ROS2PollingSubscriber<MessageT>>(rclcpp_node_.get(), topic_name, qos);
    }
  }

  template <typename MessageT>
  typename PollingSubscriber<MessageT>::SharedPtr create_polling_subscriber(
    const std::string & topic_name, size_t qos_history_depth)
  {
    return create_polling_subscriber<MessageT>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
  }
#else
  // ===== Publisher (rclcpp-only mode) =====
  template <typename MessageT>
  typename rclcpp::Publisher<MessageT>::SharedPtr create_publisher(
    const std::string & topic_name, const rclcpp::QoS & qos)
  {
    return rclcpp_node_->create_publisher<MessageT>(topic_name, qos);
  }

  template <typename MessageT>
  typename rclcpp::Publisher<MessageT>::SharedPtr create_publisher(
    const std::string & topic_name, size_t qos_history_depth)
  {
    return create_publisher<MessageT>(topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
  }

  // ===== Subscription (rclcpp-only mode) =====
  template <typename MessageT, typename Func>
  typename rclcpp::Subscription<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, const rclcpp::QoS & qos, Func && callback)
  {
    return rclcpp_node_->create_subscription<MessageT>(
      topic_name, qos, std::forward<Func>(callback));
  }

  template <typename MessageT, typename Func>
  typename rclcpp::Subscription<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, size_t qos_history_depth, Func && callback)
  {
    return create_subscription<MessageT>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), std::forward<Func>(callback));
  }

  // ===== Timer (rclcpp-only mode) =====
  template <typename DurationRepT, typename DurationT, typename CallbackT>
  Timer::SharedPtr create_timer(
    std::chrono::duration<DurationRepT, DurationT> period, CallbackT && callback)
  {
    auto timer = rclcpp::create_timer(
      rclcpp_node_, rclcpp_node_->get_clock(), period, std::forward<CallbackT>(callback));
    return std::make_shared<RclcppTimer>(std::move(timer));
  }

  template <typename DurationRepT, typename DurationT, typename CallbackT>
  Timer::SharedPtr create_timer(
    [[maybe_unused]] Node * node, [[maybe_unused]] rclcpp::Clock::SharedPtr clock,
    std::chrono::duration<DurationRepT, DurationT> period, CallbackT && callback)
  {
    return create_timer(period, std::forward<CallbackT>(callback));
  }
#endif

  // ===== Internal node access (for Executor) =====
  bool is_using_agnocast() const { return use_agnocast_; }
  std::shared_ptr<agnocast::Node> get_agnocast_node() const { return agnocast_node_; }
  std::shared_ptr<rclcpp::Node> get_rclcpp_node() const { return rclcpp_node_; }

private:
  void initialize(
    const std::string & node_name, const std::string & namespace_,
    const rclcpp::NodeOptions & options);

  bool use_agnocast_;
  std::shared_ptr<rclcpp::Node> rclcpp_node_;
  std::shared_ptr<agnocast::Node> agnocast_node_;
};

}  // namespace autoware::agnocast_wrapper
