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

#ifndef AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__VOLTAGE_MONITOR_PLUGIN_HPP_
#define AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__VOLTAGE_MONITOR_PLUGIN_HPP_

#include "autoware/system_monitor_host/plugins/monitor_plugin_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>

namespace autoware::system_monitor_host::plugin
{

class VoltageMonitorPlugin : public MonitorPluginBase
{
public:
  VoltageMonitorPlugin() = default;
  ~VoltageMonitorPlugin() override = default;

  void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<diagnostic_updater::Updater> & updater) override;
  void setup_params() override;
  void evaluate() override;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  void on_timer();
  void check_cmos_battery(diagnostic_updater::DiagnosticStatusWrapper & stat);

  static constexpr int HOST_NAME_MAX_LEN = 256;
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  char hostname_[HOST_NAME_MAX_LEN];
  rclcpp::TimerBase::SharedPtr timer_;

  std::string cmos_battery_label_;
  float cmos_battery_warn_{2.95};
  float cmos_battery_error_{2.75};
  float cmos_battery_voltage_{0.0};
  bool cmos_battery_ok_{true};
  bool cmos_battery_available_{false};

  mutable std::mutex mutex_;
};

}  // namespace autoware::system_monitor_host::plugin

#endif  // AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__VOLTAGE_MONITOR_PLUGIN_HPP_
