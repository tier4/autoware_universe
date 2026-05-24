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

#include "autoware/system_monitor_host/plugins/gpu_monitor_plugin.hpp"

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>

namespace autoware::system_monitor_host::plugin
{

void GpuMonitorPlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<diagnostic_updater::Updater> & updater)
{
  node_ptr_ = node_ptr;
  updater_ = updater;
  name_ = name;

  gethostname(hostname_, sizeof(hostname_));

  pub_gpu_ = node_ptr_->create_publisher<tier4_external_api_msgs::msg::GpuStatus>(
    "~/gpu_status", rclcpp::QoS(1).transient_local());

  setup_params();

  updater_->setHardwareID(hostname_);
  updater_->add("GPU Temperature", this, &GpuMonitorPlugin::check_temperature);
  updater_->add("GPU Usage", this, &GpuMonitorPlugin::check_usage);
  updater_->add("GPU Memory Usage", this, &GpuMonitorPlugin::check_memory_usage);
  updater_->add("GPU Thermal Throttling", this, &GpuMonitorPlugin::check_thermal_throttling);
  updater_->add("GPU Frequency", this, &GpuMonitorPlugin::check_frequency);

  timer_ = rclcpp::create_timer(
    node_ptr_, node_ptr_->get_clock(), std::chrono::seconds(1),
    std::bind(&GpuMonitorPlugin::on_timer, this));
}

void GpuMonitorPlugin::setup_params()
{
  node_ptr_->declare_parameter("gpu_monitor.temp_warn", 90.0);
  node_ptr_->declare_parameter("gpu_monitor.temp_error", 95.0);
  node_ptr_->declare_parameter("gpu_monitor.gpu_usage_warn", 0.90);
  node_ptr_->declare_parameter("gpu_monitor.gpu_usage_error", 1.00);
  node_ptr_->declare_parameter("gpu_monitor.memory_usage_warn", 0.95);
  node_ptr_->declare_parameter("gpu_monitor.memory_usage_error", 0.99);

  temp_warn_ = node_ptr_->get_parameter("gpu_monitor.temp_warn").as_double();
  temp_error_ = node_ptr_->get_parameter("gpu_monitor.temp_error").as_double();
  gpu_usage_warn_ = node_ptr_->get_parameter("gpu_monitor.gpu_usage_warn").as_double();
  gpu_usage_error_ = node_ptr_->get_parameter("gpu_monitor.gpu_usage_error").as_double();
  memory_usage_warn_ = node_ptr_->get_parameter("gpu_monitor.memory_usage_warn").as_double();
  memory_usage_error_ = node_ptr_->get_parameter("gpu_monitor.memory_usage_error").as_double();
}

rcl_interfaces::msg::SetParametersResult GpuMonitorPlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : parameters) {
    if (p.get_name() == "gpu_monitor.temp_warn") temp_warn_ = p.as_double();
    if (p.get_name() == "gpu_monitor.temp_error") temp_error_ = p.as_double();
    if (p.get_name() == "gpu_monitor.gpu_usage_warn") gpu_usage_warn_ = p.as_double();
    if (p.get_name() == "gpu_monitor.gpu_usage_error") gpu_usage_error_ = p.as_double();
    if (p.get_name() == "gpu_monitor.memory_usage_warn") memory_usage_warn_ = p.as_double();
    if (p.get_name() == "gpu_monitor.memory_usage_error") memory_usage_error_ = p.as_double();
  }
  return result;
}

void GpuMonitorPlugin::evaluate()
{
  updater_->force_update();
}

void GpuMonitorPlugin::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);

  FILE * fp = popen(
    "nvidia-smi --query-gpu=temperature.gpu,utilization.gpu,clocks.sm --format=csv,noheader,nounits 2>/dev/null",
    "r");
  if (fp) {
    char buf[256];
    if (fgets(buf, sizeof(buf), fp)) {
      gpu_found_ = true;
      std::istringstream iss(buf);
      std::string token;
      float temp = 0, util = 0, clock = 0;
      std::getline(iss, token, ','); temp = std::stof(token);
      std::getline(iss, token, ','); util = std::stof(token);
      std::getline(iss, token, ','); clock = std::stof(token);

      gpu_temperature_ = temp;
      gpu_usage_ = util / 100.0f;
      gpu_frequency_mhz_ = clock;
    }
    pclose(fp);
  } else {
    gpu_found_ = false;
  }

  // Check thermal throttling via nvidia-smi -q
  fp = popen("nvidia-smi -q 2>/dev/null | grep -i 'clocks_throttle' | head -1 || true", "r");
  if (fp) {
    char buf2[256];
    if (fgets(buf2, sizeof(buf2), fp)) {
      std::string s(buf2);
      gpu_thermal_throttling_ = (s.find("Active") != std::string::npos || s.find("Yes") != std::string::npos);
    }
    pclose(fp);
  }

  tier4_external_api_msgs::msg::GpuStatus status_msg;
  status_msg.stamp = node_ptr_->now();
  status_msg.hostname = hostname_;
  tier4_external_api_msgs::msg::GpuUnitStatus unit;
  unit.name = "GPU0";
  unit.usage = gpu_usage_ * 100.0f;
  unit.temperature = static_cast<uint32_t>(gpu_temperature_);
  unit.clock = static_cast<uint32_t>(gpu_frequency_mhz_);
  unit.thermal_throttling = gpu_thermal_throttling_
    ? tier4_external_api_msgs::msg::GpuUnitStatus::THERMAL_THROTTLING_ON
    : tier4_external_api_msgs::msg::GpuUnitStatus::THERMAL_THROTTLING_OFF;
  status_msg.gpus.push_back(unit);
  pub_gpu_->publish(status_msg);
}

void GpuMonitorPlugin::check_temperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!gpu_found_) {
    stat.summary(DiagStatus::OK, "GPU not found or nvidia-smi unavailable");
    return;
  }
  int level = DiagStatus::OK;
  std::string msg = "OK";
  if (gpu_temperature_ >= temp_error_) { level = DiagStatus::ERROR; msg = "hot"; }
  else if (gpu_temperature_ >= temp_warn_) { level = DiagStatus::WARN; msg = "warm"; }
  stat.add("temperature", gpu_temperature_);
  stat.summary(level, msg);
}

void GpuMonitorPlugin::check_usage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!gpu_found_) {
    stat.summary(DiagStatus::OK, "GPU not found");
    return;
  }
  int level = DiagStatus::OK;
  if (gpu_usage_ >= gpu_usage_error_) level = DiagStatus::ERROR;
  else if (gpu_usage_ >= gpu_usage_warn_) level = DiagStatus::WARN;
  stat.add("usage", gpu_usage_);
  stat.summary(level, level == DiagStatus::OK ? "OK" : "high usage");
}

void GpuMonitorPlugin::check_memory_usage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!gpu_found_) {
    stat.summary(DiagStatus::OK, "GPU not found");
    return;
  }
  int level = DiagStatus::OK;
  if (gpu_memory_usage_ >= memory_usage_error_) level = DiagStatus::ERROR;
  else if (gpu_memory_usage_ >= memory_usage_warn_) level = DiagStatus::WARN;
  stat.add("memory_usage", gpu_memory_usage_);
  stat.summary(level, level == DiagStatus::OK ? "OK" : "high memory usage");
}

void GpuMonitorPlugin::check_thermal_throttling(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!gpu_found_) {
    stat.summary(DiagStatus::OK, "GPU not found");
    return;
  }
  stat.summary(gpu_thermal_throttling_ ? DiagStatus::ERROR : DiagStatus::OK,
               gpu_thermal_throttling_ ? "throttling" : "OK");
}

void GpuMonitorPlugin::check_frequency(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!gpu_found_) {
    stat.summary(DiagStatus::OK, "GPU not found");
    return;
  }
  stat.add("frequency_mhz", gpu_frequency_mhz_);
  stat.summary(DiagStatus::OK, "OK");
}

}  // namespace autoware::system_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::system_monitor_host::plugin::GpuMonitorPlugin,
  autoware::system_monitor_host::plugin::MonitorPluginBase)
