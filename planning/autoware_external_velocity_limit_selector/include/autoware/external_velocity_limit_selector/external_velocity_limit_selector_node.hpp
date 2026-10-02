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

#ifndef AUTOWARE__EXTERNAL_VELOCITY_LIMIT_SELECTOR__EXTERNAL_VELOCITY_LIMIT_SELECTOR_NODE_HPP_
#define AUTOWARE__EXTERNAL_VELOCITY_LIMIT_SELECTOR__EXTERNAL_VELOCITY_LIMIT_SELECTOR_NODE_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <external_velocity_limit_selector_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit_clear_command.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::external_velocity_limit_selector
{

using autoware_internal_debug_msgs::msg::StringStamped;
using autoware_internal_planning_msgs::msg::VelocityLimit;
using autoware_internal_planning_msgs::msg::VelocityLimitClearCommand;
using autoware_internal_planning_msgs::msg::VelocityLimitConstraints;

using VelocityLimitTable = std::unordered_map<std::string, VelocityLimit>;

class ExternalVelocityLimitSelectorNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit ExternalVelocityLimitSelectorNode(const rclcpp::NodeOptions & node_options);

  void onVelocityLimitFromAPI(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(VelocityLimit) & msg);
  void onVelocityLimitFromInternal(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(VelocityLimit) & msg);
  void onVelocityLimitClearCommand(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(VelocityLimitClearCommand) & msg);

private:
  AUTOWARE_SUBSCRIPTION_PTR(VelocityLimit) sub_external_velocity_limit_from_api_;
  AUTOWARE_SUBSCRIPTION_PTR(VelocityLimit) sub_external_velocity_limit_from_internal_;
  AUTOWARE_SUBSCRIPTION_PTR(VelocityLimitClearCommand) sub_velocity_limit_clear_command_;
  AUTOWARE_PUBLISHER_PTR(VelocityLimit) pub_external_velocity_limit_;
  AUTOWARE_PUBLISHER_PTR(StringStamped) pub_debug_string_;

  void publishVelocityLimit(const VelocityLimit & velocity_limit);
  void setVelocityLimitFromAPI(const VelocityLimit & velocity_limit);
  void setVelocityLimitFromInternal(const VelocityLimit & velocity_limit);
  void clearVelocityLimit(const std::string & sender);
  void updateVelocityLimit();
  void publishDebugString();
  VelocityLimit getCurrentVelocityLimit() { return hardest_limit_; }

  // Parameters
  std::shared_ptr<::external_velocity_limit_selector::ParamListener> param_listener_;
  VelocityLimit hardest_limit_{};
  VelocityLimitTable velocity_limit_table_;
};
}  // namespace autoware::external_velocity_limit_selector

#endif  // AUTOWARE__EXTERNAL_VELOCITY_LIMIT_SELECTOR__EXTERNAL_VELOCITY_LIMIT_SELECTOR_NODE_HPP_
