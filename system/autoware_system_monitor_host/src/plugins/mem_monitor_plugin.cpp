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

#include "autoware/system_monitor_host/plugins/mem_monitor_plugin.hpp"

#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>

namespace autoware::system_monitor_host::plugin
{

void MemMonitorPlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<diagnostic_updater::Updater> & updater)
{
  node_ptr_ = node_ptr;
  updater_ = updater;
  name_ = name;

  gethostname(hostname_, sizeof(hostname_));

  pub_mem_ = node_ptr_->create_publisher<tier4_external_api_msgs::msg::MemoryStatus>(
    "~/memory_status", rclcpp::QoS(1).transient_local());

  setup_params();

  updater_->setHardwareID(hostname_);
  updater_->add("Memory Usage", this, &MemMonitorPlugin::check_memory_usage);

  timer_ = rclcpp::create_timer(
    node_ptr_, node_ptr_->get_clock(), std::chrono::seconds(1),
    std::bind(&MemMonitorPlugin::on_timer, this));
}

void MemMonitorPlugin::setup_params()
{
  node_ptr_->declare_parameter("mem_monitor.available_size", 1024);
  available_size_mb_ = node_ptr_->get_parameter("mem_monitor.available_size").as_int();
}

rcl_interfaces::msg::SetParametersResult MemMonitorPlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : parameters) {
    if (p.get_name() == "mem_monitor.available_size") available_size_mb_ = p.as_int();
  }
  return result;
}

void MemMonitorPlugin::evaluate()
{
  updater_->force_update();
}

void MemMonitorPlugin::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);

  FILE * fp = popen("free -tb 2>/dev/null", "r");
  if (!fp) return;

  char buf[512];
  long total = 0, available = 0;
  while (fgets(buf, sizeof(buf), fp)) {
    std::string line(buf);
    if (line.rfind("Mem:", 0) == 0) {
      std::istringstream iss(line);
      std::string label;
      long val;
      iss >> label >> total;
      for (int i = 0; i < 4; ++i) iss >> val;
      iss >> available;
    }
  }
  pclose(fp);

  if (total > 0) {
    mem_usage_ = static_cast<float>(total - available) / static_cast<float>(total);
  }

  tier4_external_api_msgs::msg::MemoryStatus status;
  status.stamp = node_ptr_->now();
  status.hostname = hostname_;
  status.usage = mem_usage_;
  pub_mem_->publish(status);
}

void MemMonitorPlugin::check_memory_usage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  int level = DiagStatus::OK;
  std::string msg = "OK";

  stat.add("usage", mem_usage_);
  stat.add("available_threshold_mb", available_size_mb_);

  if (mem_usage_ >= 0.99f) {
    level = DiagStatus::ERROR;
    msg = "very high memory usage";
  } else if (available_size_mb_ > 0 && mem_usage_ >= 0.95f) {
    level = DiagStatus::WARN;
    msg = "high memory usage";
  }
  stat.summary(level, msg);
}

}  // namespace autoware::system_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::system_monitor_host::plugin::MemMonitorPlugin,
  autoware::system_monitor_host::plugin::MonitorPluginBase)
