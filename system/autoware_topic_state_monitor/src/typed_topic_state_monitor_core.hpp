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

#ifndef TYPED_TOPIC_STATE_MONITOR_CORE_HPP_
#define TYPED_TOPIC_STATE_MONITOR_CORE_HPP_

// ============================================================================
// WORKAROUND: agnocast does not yet have GenericSubscription support.
// ============================================================================
//
// The original TopicStateMonitorNode (rclcpp::Node) uses rclcpp::GenericSubscription
// to subscribe to arbitrary topic types determined at runtime from YAML config.
// GenericSubscription receives raw serialized bytes without compile-time type info.
//
// Problem: agnocast has no equivalent of GenericSubscription. agnocast subscriptions
// require compile-time message types (agnocast::Subscription<MsgT>).
//
// Key insight: The TopicStateMonitorNode callback IGNORES message content entirely.
// It only records that a message arrived (calls topic_state_monitor_->update()).
// This means we don't need runtime type flexibility — we just need to create
// a typed subscription that matches the topic's actual message type.
//
// Workaround: Compile-time type registry. At build time, we register all message types
// that topic_state_monitors need to handle (a finite set of ~15 types from YAML config).
// At runtime, we match the "topic_type" parameter string to the correct type and create
// a typed agnocast::Subscription. The subscription is stored as std::shared_ptr<void>
// (type-erased) since all callbacks are identical.
//
// This allows all topic_state_monitor instances to run as agnocast::Node (Stage 2)
// with AgnocastOnly executors, eliminating the rclcpp::GenericSubscription dependency.
//
// TODO(agnocast): When agnocast adds GenericSubscription support, this class can be
// removed and the original TopicStateMonitorNode can be used directly with agnocast::Node.
// ============================================================================

#include "topic_state_monitor.hpp"

#include <agnocast/agnocast.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::topic_state_monitor
{

class TypedTopicStateMonitorNode : public agnocast::Node
{
public:
  explicit TypedTopicStateMonitorNode(const rclcpp::NodeOptions & node_options);

private:
  // Parameter
  std::string topic_;
  std::string topic_type_;
  std::string diag_name_;
  Param param_;

  // Parameter Reconfigure
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  // Core
  std::unique_ptr<TopicStateMonitor> topic_state_monitor_;

  // Type-erased subscription holder.
  // The actual agnocast::Subscription<MsgT>::SharedPtr is stored as shared_ptr<void>,
  // which correctly destructs via the original deleter. This is safe because we only
  // need the subscription to stay alive — we never access it by type after creation.
  std::shared_ptr<void> sub_topic_;

  // Timer
  void onTimer();
  agnocast::TimerBase::SharedPtr timer_;

  // Creates a typed agnocast subscription for the given topic_type string.
  // Uses a compile-time registry of all known message types.
  // Returns nullptr if the type is not registered.
  std::shared_ptr<void> createTypedSubscription(
    const std::string & topic, const std::string & topic_type, const rclcpp::QoS & qos);
};

}  // namespace autoware::topic_state_monitor

#endif  // TYPED_TOPIC_STATE_MONITOR_CORE_HPP_
