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

#ifndef AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__PIPELINE_LATENCY_PLUGIN_HPP_
#define AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__PIPELINE_LATENCY_PLUGIN_HPP_

#include "autoware/system_monitor_host/plugins/monitor_plugin_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::system_monitor_host::plugin
{

class PipelineLatencyPlugin : public MonitorPluginBase
{
public:
  PipelineLatencyPlugin() = default;
  ~PipelineLatencyPlugin() override = default;

  void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<diagnostic_updater::Updater> & updater) override;
  void setup_params() override;
  void evaluate() override;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  void on_timer();
  void check_total_latency(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void on_step_msg(
    const autoware_internal_debug_msgs::msg::Float64Stamped & msg, const std::string & step_name);
  double calculate_total_latency();

  struct StepConfig
  {
    std::string name;
    std::string topic;
    double latency_multiplier{1.0};
  };

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr>
    subs_;

  std::vector<StepConfig> steps_;
  std::map<std::string, double> step_timestamps_;
  double latency_threshold_ms_{100.0};
  double total_latency_ms_{0.0};
  int window_size_{10};

  mutable std::mutex mutex_;
};

}  // namespace autoware::system_monitor_host::plugin

#endif  // AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__PIPELINE_LATENCY_PLUGIN_HPP_
