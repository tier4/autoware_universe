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

#include "autoware/system_monitor_host/plugins/pipeline_latency_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace autoware::system_monitor_host::plugin
{

void PipelineLatencyPlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<diagnostic_updater::Updater> & updater)
{
  MonitorPluginBase::initialize(name, node_ptr, updater);

  updater_->setHardwareID("pipeline_latency_monitor");
  updater_->add("Total Latency", this, &PipelineLatencyPlugin::check_total_latency);

  // Subscribe to each step topic
  for (const auto & step : steps_) {
    auto sub = node_ptr_->create_subscription<autoware_internal_debug_msgs::msg::Float64Stamped>(
      step.topic, 1,
      [this, step_name = step.name](
        const autoware_internal_debug_msgs::msg::Float64Stamped & msg) {
        on_step_msg(msg, step_name);
      });
    subs_.push_back(sub);
  }

  timer_ = rclcpp::create_timer(
    node_ptr_, node_ptr_->get_clock(), std::chrono::seconds(1),
    std::bind(&PipelineLatencyPlugin::on_timer, this));
}

void PipelineLatencyPlugin::setup_params()
{
  node_ptr_->declare_parameter("pipeline_latency_monitor.update_rate", 1.0);
  node_ptr_->declare_parameter("pipeline_latency_monitor.latency_threshold_ms", 100.0);
  node_ptr_->declare_parameter("pipeline_latency_monitor.window_size", 10);
  node_ptr_->declare_parameter("pipeline_latency_monitor.processing_steps.sequence",
                                std::vector<std::string>());

  latency_threshold_ms_ =
    node_ptr_->get_parameter("pipeline_latency_monitor.latency_threshold_ms").as_double();
  window_size_ = node_ptr_->get_parameter("pipeline_latency_monitor.window_size").as_int();
  auto step_names =
    node_ptr_->get_parameter("pipeline_latency_monitor.processing_steps.sequence")
      .as_string_array();

  // Reconstruct step configs from individual params
  steps_.clear();
  for (const auto & sname : step_names) {
    std::string param_prefix = "pipeline_latency_monitor.processing_steps." + sname;
    node_ptr_->declare_parameter(param_prefix + ".topic", "");
    node_ptr_->declare_parameter(param_prefix + ".latency_multiplier", 1.0);

    StepConfig cfg;
    cfg.name = sname;
    cfg.topic = node_ptr_->get_parameter(param_prefix + ".topic").as_string();
    cfg.latency_multiplier =
      node_ptr_->get_parameter(param_prefix + ".latency_multiplier").as_double();
    if (!cfg.topic.empty()) {
      steps_.push_back(cfg);
    }
  }
}

rcl_interfaces::msg::SetParametersResult PipelineLatencyPlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : parameters) {
    if (p.get_name() == "pipeline_latency_monitor.latency_threshold_ms") {
      latency_threshold_ms_ = p.as_double();
    }
    if (p.get_name() == "pipeline_latency_monitor.window_size") {
      window_size_ = p.as_int();
    }
  }
  return result;
}

void PipelineLatencyPlugin::evaluate()
{
  updater_->force_update();
}

void PipelineLatencyPlugin::on_step_msg(
  const autoware_internal_debug_msgs::msg::Float64Stamped & msg, const std::string & step_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  step_timestamps_[step_name] = msg.data;
}

double PipelineLatencyPlugin::calculate_total_latency()
{
  double total = 0.0;
  double prev = 0.0;
  for (const auto & step : steps_) {
    auto it = step_timestamps_.find(step.name);
    if (it == step_timestamps_.end()) continue;
    double ts = it->second;
    if (prev > 0 && ts > prev) {
      total += (ts - prev) * step.latency_multiplier;
    }
    prev = ts;
  }
  return total;
}

void PipelineLatencyPlugin::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  total_latency_ms_ = calculate_total_latency();
}

void PipelineLatencyPlugin::check_total_latency(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  stat.add("total_latency_ms", total_latency_ms_);
  if (total_latency_ms_ > latency_threshold_ms_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "latency exceeds threshold");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  }
}

}  // namespace autoware::system_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::system_monitor_host::plugin::PipelineLatencyPlugin,
  autoware::system_monitor_host::plugin::MonitorPluginBase)
