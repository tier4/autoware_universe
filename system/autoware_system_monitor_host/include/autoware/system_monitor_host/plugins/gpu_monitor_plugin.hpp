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

#ifndef AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__GPU_MONITOR_PLUGIN_HPP_
#define AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__GPU_MONITOR_PLUGIN_HPP_

#include "autoware/system_monitor_host/plugins/monitor_plugin_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_external_api_msgs/msg/gpu_status.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace autoware::system_monitor_host::plugin
{

class GpuMonitorPlugin : public MonitorPluginBase
{
public:
  GpuMonitorPlugin() = default;
  ~GpuMonitorPlugin() override = default;

  void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<diagnostic_updater::Updater> & updater) override;
  void setup_params() override;
  void evaluate() override;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  void on_timer();
  void check_temperature(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void check_usage(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void check_memory_usage(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void check_thermal_throttling(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void check_frequency(diagnostic_updater::DiagnosticStatusWrapper & stat);

  static constexpr int HOST_NAME_MAX_LEN = 256;
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  char hostname_[HOST_NAME_MAX_LEN];
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::GpuStatus>::SharedPtr pub_gpu_;

  float temp_warn_{90.0};
  float temp_error_{95.0};
  float gpu_usage_warn_{0.90};
  float gpu_usage_error_{1.00};
  float memory_usage_warn_{0.95};
  float memory_usage_error_{0.99};

  float gpu_temperature_{0.0};
  float gpu_usage_{0.0};
  float gpu_memory_usage_{0.0};
  bool gpu_thermal_throttling_{false};
  float gpu_frequency_mhz_{0.0};
  bool gpu_found_{false};

  mutable std::mutex mutex_;
};

}  // namespace autoware::system_monitor_host::plugin

#endif  // AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__GPU_MONITOR_PLUGIN_HPP_
