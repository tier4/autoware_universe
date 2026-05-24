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

#ifndef AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__PROCESSING_TIME_PLUGIN_HPP_
#define AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__PROCESSING_TIME_PLUGIN_HPP_

#include "autoware/system_monitor_host/plugins/monitor_plugin_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::system_monitor_host::plugin
{

class ProcessingTimePlugin : public MonitorPluginBase
{
public:
  ProcessingTimePlugin() = default;
  ~ProcessingTimePlugin() override;

  void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<diagnostic_updater::Updater> & updater) override;
  void setup_params() override;
  void evaluate() override;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  void on_timer();
  void on_processing_time(const autoware_internal_debug_msgs::msg::Float64Stamped & msg,
                          const std::string & module_name);
  std::string extract_module_name(const std::string & topic_name);

  std::vector<rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr>
    subs_;
  rclcpp::Publisher<tier4_metric_msgs::msg::MetricArray>::SharedPtr metrics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> topic_name_list_;
  std::map<std::string, std::string> module_name_map_;
  std::map<std::string, double> latest_values_;
  bool output_metrics_{false};

  mutable std::mutex mutex_;
};

}  // namespace autoware::system_monitor_host::plugin

#endif  // AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__PROCESSING_TIME_PLUGIN_HPP_
