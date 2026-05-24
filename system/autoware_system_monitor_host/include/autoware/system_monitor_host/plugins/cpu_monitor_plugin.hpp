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

#ifndef AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__CPU_MONITOR_PLUGIN_HPP_
#define AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__CPU_MONITOR_PLUGIN_HPP_

#include "autoware/system_monitor_host/plugins/monitor_plugin_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_external_api_msgs/msg/cpu_temperature.hpp>
#include <tier4_external_api_msgs/msg/cpu_usage.hpp>

#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::system_monitor_host::plugin
{

class CpuMonitorPlugin : public MonitorPluginBase
{
public:
  CpuMonitorPlugin() = default;
  ~CpuMonitorPlugin() override = default;

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
  void check_load(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void check_frequency(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void check_thermal_throttling(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_temperature_data();
  void update_usage_data();
  void update_load_data();
  void update_frequency_data();
  void publish_status();

  static constexpr int HOST_NAME_MAX_LEN = 256;
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  char hostname_[HOST_NAME_MAX_LEN];
  int num_cores_{0};

  float usage_warn_{0.96};
  float usage_error_{0.96};
  int usage_warn_count_{1};
  int usage_error_count_{2};
  bool usage_average_{true};

  struct TempInfo
  {
    std::string label;
    std::string path;
  };
  std::vector<TempInfo> temp_files_;

  struct CoreFreq
  {
    std::string label;
    std::string path;
    float frequency_mhz{0.0};
  };
  std::vector<CoreFreq> core_frequencies_;

  struct UsageData
  {
    std::string label;
    float usage_pct{0.0};
  };
  std::vector<UsageData> usage_data_;
  std::vector<int> usage_warn_counts_;
  std::vector<int> usage_error_counts_;
  float load_avg_1min_{0.0};
  bool thermal_throttling_{false};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::CpuUsage>::SharedPtr pub_cpu_usage_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::CpuTemperature>::SharedPtr pub_cpu_temp_;

  mutable std::mutex mutex_;
};

}  // namespace autoware::system_monitor_host::plugin

#endif  // AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__CPU_MONITOR_PLUGIN_HPP_
