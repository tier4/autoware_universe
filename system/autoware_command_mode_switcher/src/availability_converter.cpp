//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "availability_converter.hpp"

#include <autoware_command_mode_types/constants/modes.hpp>

namespace autoware::command_mode_switcher
{

CommandModeAvailabilityItem make_availability_item(uint16_t mode, bool available)
{
  CommandModeAvailabilityItem item;
  item.mode = mode;
  item.available = available;
  return item;
}

AvailabilityConverter::AvailabilityConverter(const rclcpp::NodeOptions & options)
: Node("availability_converter", options)
{
  pub_command_mode_ =
    this->create_publisher<CommandModeAvailability>("~/command_mode/availability", 1);
  sub_operation_mode_ = this->create_subscription<OperationModeAvailability>(
    "~/operation_mode/availability", rclcpp::QoS(1),
    [this](const OperationModeAvailability::SharedPtr in) {
      namespace modes = autoware::command_mode_types::modes;
      CommandModeAvailability out;
      out.stamp = in->stamp;
      out.items.push_back(make_availability_item(modes::stop, in->stop));
      out.items.push_back(make_availability_item(modes::autonomous, in->autonomous));
      out.items.push_back(make_availability_item(modes::local, in->local));
      out.items.push_back(make_availability_item(modes::remote, in->remote));
      out.items.push_back(make_availability_item(modes::emergency_stop, in->emergency_stop));
      out.items.push_back(make_availability_item(modes::comfortable_stop, in->comfortable_stop));
      out.items.push_back(make_availability_item(modes::pull_over, in->pull_over));
      pub_command_mode_->publish(out);
    });
}

}  // namespace autoware::command_mode_switcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::command_mode_switcher::AvailabilityConverter)
