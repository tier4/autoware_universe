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

#include <agnocast/agnocast.hpp>
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

class ExternalVelocityLimitSelectorNode : public agnocast::Node
{
public:
  explicit ExternalVelocityLimitSelectorNode(const rclcpp::NodeOptions & node_options);

  void onVelocityLimitFromAPI(const agnocast::ipc_shared_ptr<const VelocityLimit> & msg);
  void onVelocityLimitFromInternal(const agnocast::ipc_shared_ptr<const VelocityLimit> & msg);
  void onVelocityLimitClearCommand(
    const agnocast::ipc_shared_ptr<const VelocityLimitClearCommand> & msg);

private:
  agnocast::Subscription<VelocityLimit>::SharedPtr sub_external_velocity_limit_from_api_;
  agnocast::Subscription<VelocityLimit>::SharedPtr sub_external_velocity_limit_from_internal_;
  agnocast::Subscription<VelocityLimitClearCommand>::SharedPtr sub_velocity_limit_clear_command_;
  agnocast::Publisher<VelocityLimit>::SharedPtr pub_external_velocity_limit_;
  agnocast::Publisher<StringStamped>::SharedPtr pub_debug_string_;

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
