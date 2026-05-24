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

#ifndef AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__HDD_MONITOR_PLUGIN_HPP_
#define AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__HDD_MONITOR_PLUGIN_HPP_

#include "autoware/system_monitor_host/plugins/monitor_plugin_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_external_api_msgs/msg/hdd_status.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace autoware::system_monitor_host::plugin
{

class HddMonitorPlugin : public MonitorPluginBase
{
public:
  HddMonitorPlugin() = default;
  ~HddMonitorPlugin() override = default;

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
  void check_read_data_rate(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void check_write_data_rate(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void check_read_iops(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void check_write_iops(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void check_connection(diagnostic_updater::DiagnosticStatusWrapper & stat);

  struct DiskInfo
  {
    std::string device;
    std::string mount_point;
    long total_mb{0};
    long used_mb{0};
    long available_mb{0};
    float usage_pct{0.0};
    float temperature{0.0};
    bool mounted{true};
  };

  static constexpr int HOST_NAME_MAX_LEN = 256;
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  char hostname_[HOST_NAME_MAX_LEN];
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::HddStatus>::SharedPtr pub_hdd_;

  std::vector<DiskInfo> disks_;
  float hdd_usage_warn_{0.95};
  float hdd_usage_error_{0.99};
  float hdd_temp_warn_{55.0};
  float hdd_temp_error_{60.0};

  mutable std::mutex mutex_;
};

}  // namespace autoware::system_monitor_host::plugin

#endif  // AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__HDD_MONITOR_PLUGIN_HPP_
