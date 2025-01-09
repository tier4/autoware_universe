// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
// governing permissions and limitations under the License.

#include "topic_relay_controller_node.hpp"

namespace autoware::topic_relay_controller
{
TopicRelayController::TopicRelayController(const rclcpp::NodeOptions & options) : Node("topic_relay_controller", options)
{
  RCLCPP_INFO(get_logger(), "topic_relay_controller_node started.");
}
} // namespace autoware::topic_relay_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::topic_relay_controller::TopicRelayController)
