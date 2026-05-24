// Copyright 2025 Autoware Foundation
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

#ifndef AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__SAFETY_MONITOR_PLUGIN_BASE_HPP_
#define AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__SAFETY_MONITOR_PLUGIN_BASE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace autoware::control_safety_monitor_host::plugin
{

class SafetyMonitorPluginBase
{
public:
  SafetyMonitorPluginBase() = default;
  virtual ~SafetyMonitorPluginBase() = default;

  virtual void initialize(const std::string & name, rclcpp::Node * node_ptr)
  {
    name_ = name;
    node_ptr_ = node_ptr;
    set_up_params();
    on_initialize();
  }

  virtual void set_up_params() {}
  virtual void on_initialize() {}
  virtual void on_parameter(const std::vector<rclcpp::Parameter> & /*parameters*/) {}

  /// Called on the host timer tick.
  virtual void update() = 0;

  virtual bool is_enabled() const { return enabled_; }

  virtual void set_enabled(const bool enabled) { enabled_ = enabled; }

  std::string get_name() const { return name_; }

protected:
  rclcpp::Node * get_node_ptr() const { return node_ptr_; }

private:
  std::string name_{"unnamed_plugin"};
  rclcpp::Node * node_ptr_{nullptr};
  bool enabled_{true};
};

}  // namespace autoware::control_safety_monitor_host::plugin

#endif  // AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__SAFETY_MONITOR_PLUGIN_BASE_HPP_
