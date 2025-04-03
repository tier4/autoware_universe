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

#ifndef COMMON__TARGET_PLUGIN_HPP_
#define COMMON__TARGET_PLUGIN_HPP_

#include "common/target_status.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::command_mode_switcher
{

class TargetPlugin
{
public:
  virtual ~TargetPlugin() = default;
  virtual std::string mode_name() const { return ""; }
  virtual std::string source_name() const = 0;
  virtual bool autoware_control() const = 0;
  virtual void initialize() {}
  virtual SourceState update_source_state() { return SourceState::Disabled; }

  void construct(rclcpp::Node * node) { node_ = node; }

protected:
  rclcpp::Node * node_;
};

}  // namespace autoware::command_mode_switcher

#endif  // COMMON__TARGET_PLUGIN_HPP_
