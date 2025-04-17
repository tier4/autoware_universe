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

#include "autoware_command_mode_switcher/common/command_plugin.hpp"

namespace autoware::command_mode_switcher
{

TriState CommandPlugin::update_source_state(bool request)
{
  return request ? TriState::Enabled : TriState::Disabled;
}

bool ControlCommandPlugin::get_mode_continuable()
{
  return mode_continuable_;
}
bool ControlCommandPlugin::get_mode_available()
{
  return mode_available_;
}
void ControlCommandPlugin::set_mode_continuable(bool continuable)
{
  mode_continuable_ = continuable;
}
void ControlCommandPlugin::set_mode_available(bool available)
{
  mode_available_ = available;
}

void VehicleCommandPlugin::set_mode_continuable(bool continuable)
{
  RCLCPP_ERROR_STREAM(node_->get_logger(), "vehicle command has no continuable: " << continuable);
}

void VehicleCommandPlugin::set_mode_available(bool available)
{
  RCLCPP_ERROR_STREAM(node_->get_logger(), "vehicle command has no available: " << available);
}

}  // namespace autoware::command_mode_switcher
