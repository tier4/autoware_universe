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

#ifndef AUTOWARE__LOCALIZATION_MONITOR_HOST__LOCALIZATION_MONITOR_PLUGIN_BASE_HPP_
#define AUTOWARE__LOCALIZATION_MONITOR_HOST__LOCALIZATION_MONITOR_PLUGIN_BASE_HPP_

#include "monitor_data.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace autoware::localization_monitor_host::plugin
{

class LocalizationMonitorPluginBase
{
public:
  LocalizationMonitorPluginBase() = default;
  virtual ~LocalizationMonitorPluginBase() = default;

  virtual void initialize(const std::string & name, rclcpp::Node * node_ptr)
  {
    name_ = name;
    node_ptr_ = node_ptr;
    set_up_params();
  }

  virtual void set_up_params() {}

  virtual void on_parameter(const std::vector<rclcpp::Parameter> & /*parameters*/) {}

  virtual void evaluate(
    const MonitorData & data, diagnostic_updater::Updater & updater) = 0;

  std::string get_name() const { return name_; }

protected:
  rclcpp::Node * get_node_ptr() const { return node_ptr_; }

private:
  std::string name_{"unnamed_plugin"};
  rclcpp::Node * node_ptr_{nullptr};
};

}  // namespace autoware::localization_monitor_host::plugin

#endif  // AUTOWARE__LOCALIZATION_MONITOR_HOST__LOCALIZATION_MONITOR_PLUGIN_BASE_HPP_
