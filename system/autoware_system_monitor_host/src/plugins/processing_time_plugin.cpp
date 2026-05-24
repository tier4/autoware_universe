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

#include "autoware/system_monitor_host/plugins/processing_time_plugin.hpp"

#include <rclcpp/logging.hpp>

#include <string>
#include <vector>

namespace autoware::system_monitor_host::plugin
{

ProcessingTimePlugin::~ProcessingTimePlugin()
{
  // simplified destructor - no JSON output in interim implementation
}

void ProcessingTimePlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<diagnostic_updater::Updater> & updater)
{
  MonitorPluginBase::initialize(name, node_ptr, updater);

  metrics_pub_ = node_ptr_->create_publisher<tier4_metric_msgs::msg::MetricArray>("~/metrics", 1);

  // Create subscriptions for each processing time topic
  for (const auto & topic_name : topic_name_list_) {
    auto module_name = extract_module_name(topic_name);
    module_name_map_[topic_name] = module_name;

    auto sub = node_ptr_->create_subscription<autoware_internal_debug_msgs::msg::Float64Stamped>(
      topic_name, 1,
      [this, module_name](
        const autoware_internal_debug_msgs::msg::Float64Stamped & msg) {
        on_processing_time(msg, module_name);
      });
    subs_.push_back(sub);
  }

  if (!topic_name_list_.empty()) {
    timer_ = rclcpp::create_timer(
      node_ptr_, node_ptr_->get_clock(), std::chrono::milliseconds(500),
      std::bind(&ProcessingTimePlugin::on_timer, this));
  }
}

void ProcessingTimePlugin::setup_params()
{
  node_ptr_->declare_parameter("processing_time_checker.output_metrics", false);
  node_ptr_->declare_parameter("processing_time_checker.update_rate", 2.0);
  node_ptr_->declare_parameter("processing_time_checker.processing_time_topic_name_list",
                                std::vector<std::string>());

  output_metrics_ = node_ptr_->get_parameter("processing_time_checker.output_metrics").as_bool();
  topic_name_list_ =
    node_ptr_->get_parameter("processing_time_checker.processing_time_topic_name_list")
      .as_string_array();
}

rcl_interfaces::msg::SetParametersResult ProcessingTimePlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : parameters) {
    if (p.get_name() == "processing_time_checker.output_metrics") output_metrics_ = p.as_bool();
  }
  return result;
}

void ProcessingTimePlugin::evaluate()
{
  // no-op: publishing happens on separate timer
}

std::string ProcessingTimePlugin::extract_module_name(const std::string & topic_name)
{
  auto tmp = topic_name;
  for (size_t i = 0; i < 4; ++i) {
    auto pos = tmp.find_last_of("/");
    if (pos == std::string::npos) break;
    tmp = tmp.substr(0, pos);
    auto name = tmp.substr(tmp.find_last_of("/") + 1);
    if (name != "processing_time_ms" && name != "debug" && name != "total_time") {
      return name;
    }
  }
  return topic_name;
}

void ProcessingTimePlugin::on_processing_time(
  const autoware_internal_debug_msgs::msg::Float64Stamped & msg, const std::string & module_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_values_[module_name] = msg.data;
}

void ProcessingTimePlugin::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  tier4_metric_msgs::msg::MetricArray metric_array;
  metric_array.stamp = node_ptr_->now();

  for (const auto & [module, value] : latest_values_) {
    tier4_metric_msgs::msg::Metric metric;
    metric.name = "processing_time/" + module;
    metric.value = std::to_string(value);
    metric.unit = "millisecond";
    metric_array.metric_array.push_back(metric);
  }

  metrics_pub_->publish(metric_array);
}

}  // namespace autoware::system_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::system_monitor_host::plugin::ProcessingTimePlugin,
  autoware::system_monitor_host::plugin::MonitorPluginBase)
