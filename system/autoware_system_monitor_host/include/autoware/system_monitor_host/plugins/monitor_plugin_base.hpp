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

#ifndef AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__MONITOR_PLUGIN_BASE_HPP_
#define AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__MONITOR_PLUGIN_BASE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::system_monitor_host::plugin
{

class MonitorPluginBase
{
public:
  MonitorPluginBase() = default;
  virtual ~MonitorPluginBase() = default;

  virtual void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<diagnostic_updater::Updater> & updater)
  {
    name_ = name;
    node_ptr_ = node_ptr;
    updater_ = updater;
    setup_params();
  }

  virtual void setup_params() = 0;

  virtual rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & /*parameters*/)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  virtual void evaluate() = 0;

  std::string get_name() const { return name_; }

protected:
  rclcpp::Node * get_node_ptr() const { return node_ptr_; }
  std::shared_ptr<diagnostic_updater::Updater> get_updater() const { return updater_; }

  rclcpp::Node * node_ptr_{nullptr};
  std::shared_ptr<diagnostic_updater::Updater> updater_;
  std::string name_{"unnamed_plugin"};
};

}  // namespace autoware::system_monitor_host::plugin

#endif  // AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__MONITOR_PLUGIN_BASE_HPP_
