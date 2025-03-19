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

#ifndef COMMON__PLUGIN_HPP_
#define COMMON__PLUGIN_HPP_

#include "common/transition.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autoware::command_mode_switcher
{

class SwitcherPlugin
{
public:
  void construct(rclcpp::Node * node);
  auto status() const { return status_; }
  auto sequence_state() const { return status_.state; }
  auto sequence_target() const { return status_.target; }
  auto source_status() const { return source_status_; }

  void request(SwitcherState target);
  void disable();
  void handover();
  void override();
  void update_status(const TransitionContext & context);

  virtual ~SwitcherPlugin() = default;
  virtual std::string mode_name() const = 0;
  virtual std::string source_name() const = 0;
  virtual bool autoware_control() const { return true; }
  virtual void update_source_status();

private:
  rclcpp::Node * node_;
  CommandModeStatusItem status_;
  SourceStatus source_status_;
};

}  // namespace autoware::command_mode_switcher

#endif  // COMMON__PLUGIN_HPP_
