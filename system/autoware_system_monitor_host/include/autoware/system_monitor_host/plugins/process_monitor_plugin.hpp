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

#ifndef AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__PROCESS_MONITOR_PLUGIN_HPP_
#define AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__PROCESS_MONITOR_PLUGIN_HPP_

#include "autoware/system_monitor_host/plugins/monitor_plugin_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace autoware::system_monitor_host::plugin
{

class ProcessMonitorPlugin : public MonitorPluginBase
{
public:
  ProcessMonitorPlugin() = default;
  ~ProcessMonitorPlugin() override = default;

  void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<diagnostic_updater::Updater> & updater) override;
  void setup_params() override;
  void evaluate() override;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  void on_timer();
  void check_tasks_summary(diagnostic_updater::DiagnosticStatusWrapper & stat);

  struct ProcInfo
  {
    int pid;
    std::string command;
    float cpu_pct;
    float mem_pct;
  };

  static constexpr int HOST_NAME_MAX_LEN = 256;
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  char hostname_[HOST_NAME_MAX_LEN];
  rclcpp::TimerBase::SharedPtr timer_;

  int num_of_procs_{5};
  int total_tasks_{0};
  int running_tasks_{0};
  int sleeping_tasks_{0};
  int stopped_tasks_{0};
  int zombie_tasks_{0};
  std::vector<ProcInfo> high_cpu_procs_;
  std::vector<ProcInfo> high_mem_procs_;

  mutable std::mutex mutex_;
};

}  // namespace autoware::system_monitor_host::plugin

#endif  // AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__PROCESS_MONITOR_PLUGIN_HPP_
