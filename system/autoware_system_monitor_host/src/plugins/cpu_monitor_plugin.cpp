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

#include "autoware/system_monitor_host/plugins/cpu_monitor_plugin.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

namespace autoware::system_monitor_host::plugin
{

namespace
{
bool file_exists(const std::string & path)
{
  return access(path.c_str(), R_OK) == 0;
}

std::string read_file(const std::string & path)
{
  std::ifstream ifs(path);
  if (!ifs) return "";
  std::string content;
  ifs >> content;
  return content;
}

std::vector<std::string> glob(const std::string & pattern)
{
  std::vector<std::string> result;
  std::regex re("cpu[*]");
  if (!std::regex_search(pattern, re)) return result;

  std::string prefix = std::regex_replace(pattern, std::regex("cpu[*].*"), "");
  std::string suffix = std::regex_replace(pattern, std::regex(".*cpu[*]"), "");
  for (int i = 0; i < 256; ++i) {
    std::string path = prefix + "cpu" + std::to_string(i) + suffix;
    if (file_exists(path)) {
      result.push_back(path);
    } else if (i > 32) {
      break;
    }
  }
  return result;
}
}  // namespace

void CpuMonitorPlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<diagnostic_updater::Updater> & updater)
{
  node_ptr_ = node_ptr;
  updater_ = updater;
  name_ = name;

  gethostname(hostname_, sizeof(hostname_));

  pub_cpu_usage_ = node_ptr_->create_publisher<tier4_external_api_msgs::msg::CpuUsage>(
    "~/cpu_usage", rclcpp::QoS(1).transient_local());
  pub_cpu_temp_ = node_ptr_->create_publisher<tier4_external_api_msgs::msg::CpuTemperature>(
    "~/cpu_temperature", rclcpp::QoS(1).transient_local());

  setup_params();

  updater_->setHardwareID(hostname_);
  updater_->add("CPU Temperature", this, &CpuMonitorPlugin::check_temperature);
  updater_->add("CPU Usage", this, &CpuMonitorPlugin::check_usage);
  updater_->add("CPU Load Average", this, &CpuMonitorPlugin::check_load);
  updater_->add("CPU Frequency", this, &CpuMonitorPlugin::check_frequency);
  updater_->add("CPU Thermal Throttling", this, &CpuMonitorPlugin::check_thermal_throttling);

  timer_ = rclcpp::create_timer(
    node_ptr_, node_ptr_->get_clock(), std::chrono::seconds(1),
    std::bind(&CpuMonitorPlugin::on_timer, this));
}

void CpuMonitorPlugin::setup_params()
{
  node_ptr_->declare_parameter("cpu_monitor.usage_warn", 0.96);
  node_ptr_->declare_parameter("cpu_monitor.usage_error", 0.96);
  node_ptr_->declare_parameter("cpu_monitor.usage_warn_count", 1);
  node_ptr_->declare_parameter("cpu_monitor.usage_error_count", 2);
  node_ptr_->declare_parameter("cpu_monitor.usage_average", true);

  usage_warn_ = node_ptr_->get_parameter("cpu_monitor.usage_warn").as_double();
  usage_error_ = node_ptr_->get_parameter("cpu_monitor.usage_error").as_double();
  usage_warn_count_ = node_ptr_->get_parameter("cpu_monitor.usage_warn_count").as_int();
  usage_error_count_ = node_ptr_->get_parameter("cpu_monitor.usage_error_count").as_int();
  usage_average_ = node_ptr_->get_parameter("cpu_monitor.usage_average").as_bool();
}

rcl_interfaces::msg::SetParametersResult CpuMonitorPlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : parameters) {
    if (p.get_name() == "cpu_monitor.usage_warn") usage_warn_ = p.as_double();
    if (p.get_name() == "cpu_monitor.usage_error") usage_error_ = p.as_double();
    if (p.get_name() == "cpu_monitor.usage_warn_count") usage_warn_count_ = p.as_int();
    if (p.get_name() == "cpu_monitor.usage_error_count") usage_error_count_ = p.as_int();
    if (p.get_name() == "cpu_monitor.usage_average") usage_average_ = p.as_bool();
  }
  return result;
}

void CpuMonitorPlugin::evaluate()
{
  updater_->force_update();
}

void CpuMonitorPlugin::on_timer()
{
  update_temperature_data();
  update_usage_data();
  update_load_data();
  update_frequency_data();
  publish_status();
}

void CpuMonitorPlugin::check_temperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  int level = DiagStatus::OK;
  std::string msg = "OK";

  for (auto & temp_info : temp_files_) {
    auto raw = read_file(temp_info.path);
    if (!raw.empty()) {
      try {
        float temp = std::stof(raw) / 1000.0f;
        stat.add(temp_info.label, temp);
      } catch (...) {
      }
    }
  }
  stat.summary(level, msg);
}

void CpuMonitorPlugin::check_usage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  int level = DiagStatus::OK;
  std::string msg = "OK";
  float sum_usage = 0.0f;

  for (size_t i = 0; i < usage_data_.size(); ++i) {
    const auto & d = usage_data_[i];
    stat.add(d.label, d.usage_pct);
    sum_usage += d.usage_pct;

    int item_level = DiagStatus::OK;
    if (d.usage_pct >= usage_error_) {
      usage_error_counts_[i]++;
      if (usage_error_counts_[i] >= usage_error_count_) {
        item_level = DiagStatus::ERROR;
        msg = "very high load";
      }
    } else if (d.usage_pct >= usage_warn_) {
      usage_warn_counts_[i]++;
      if (usage_warn_counts_[i] >= usage_warn_count_) {
        item_level = DiagStatus::WARN;
        msg = "high load";
      }
    } else {
      usage_error_counts_[i] = 0;
      usage_warn_counts_[i] = 0;
    }
    if (item_level > level) level = item_level;
  }

  if (usage_average_ && !usage_data_.empty()) {
    float avg = sum_usage / usage_data_.size();
    stat.add("average", avg);
  }
  stat.summary(level, msg);
}

void CpuMonitorPlugin::check_load(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  stat.add("load_1min", load_avg_1min_);
  float pct = (num_cores_ > 0) ? (load_avg_1min_ / num_cores_ * 100.0f) : 0.0f;
  stat.add("load_1min_pct", pct);
  stat.summary(DiagStatus::OK, "OK");
}

void CpuMonitorPlugin::check_frequency(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & f : core_frequencies_) {
    stat.add(f.label, f.frequency_mhz);
  }
  stat.summary(DiagStatus::OK, "OK");
}

void CpuMonitorPlugin::check_thermal_throttling(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (thermal_throttling_) {
    stat.summary(DiagStatus::ERROR, "throttling");
  } else {
    stat.summary(DiagStatus::OK, "OK");
  }
}

void CpuMonitorPlugin::update_temperature_data()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (temp_files_.empty()) {
    auto paths = glob("/sys/devices/platform/coretemp.0/hwmon/hwmon*/temp*_input");
    for (size_t i = 0; i < paths.size(); ++i) {
      TempInfo info;
      info.path = paths[i];
      info.label = "core_" + std::to_string(i);
      temp_files_.push_back(info);
    }
    if (temp_files_.empty()) {
      paths = glob("/sys/class/hwmon/hwmon*/temp*_input");
      for (size_t i = 0; i < paths.size(); ++i) {
        TempInfo info;
        info.path = paths[i];
        info.label = "temp_" + std::to_string(i);
        temp_files_.push_back(info);
      }
    }
  }
}

void CpuMonitorPlugin::update_usage_data()
{
  std::lock_guard<std::mutex> lock(mutex_);
  static long prev_total = 0, prev_idle = 0;

  std::ifstream stat_file("/proc/stat");
  if (!stat_file.is_open()) return;
  std::string line;

  usage_data_.clear();
  while (std::getline(stat_file, line)) {
    if (line.rfind("cpu", 0) != 0) break;
    std::istringstream iss(line);
    std::string label;
    long user = 0, nice = 0, system = 0, idle = 0, iowait = 0, irq = 0, softirq = 0, steal = 0;
    iss >> label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
    long total = user + nice + system + idle + iowait + irq + softirq + steal;
    long total_delta = total - prev_total;
    long idle_delta = idle - prev_idle;
    if (prev_total > 0) {
      float usage_frac = (total_delta > 0) ? (1.0f - static_cast<float>(idle_delta) / total_delta) : 0.0f;
      UsageData d;
      d.label = "cpu" + label.substr(3);
      d.usage_pct = usage_frac;
      usage_data_.push_back(d);
    }
    prev_total = total;
    prev_idle = idle;
  }
  num_cores_ = std::max(1, static_cast<int>(usage_data_.size()) - 1);
  usage_warn_counts_.resize(usage_data_.size(), 0);
  usage_error_counts_.resize(usage_data_.size(), 0);
}

void CpuMonitorPlugin::update_load_data()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::ifstream load_file("/proc/loadavg");
  if (load_file.is_open()) {
    load_file >> load_avg_1min_;
  }
}

void CpuMonitorPlugin::update_frequency_data()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (core_frequencies_.empty()) {
    for (int i = 0; i < 256; ++i) {
      std::string path =
        "/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpufreq/scaling_cur_freq";
      if (file_exists(path)) {
        CoreFreq cf;
        cf.label = "core_" + std::to_string(i);
        cf.path = path;
        core_frequencies_.push_back(cf);
      }
    }
  }
  for (auto & cf : core_frequencies_) {
    auto raw = read_file(cf.path);
    if (!raw.empty()) {
      try {
        cf.frequency_mhz = std::stof(raw) / 1000.0f;
      } catch (...) {
      }
    }
  }
}

void CpuMonitorPlugin::publish_status()
{
  tier4_external_api_msgs::msg::CpuUsage usage_msg;
  usage_msg.stamp = node_ptr_->now();
  usage_msg.all.status = tier4_external_api_msgs::msg::CpuStatus::OK;
  usage_msg.all.total = 0.0f;
  for (const auto & d : usage_data_) {
    tier4_external_api_msgs::msg::CpuStatus cs;
    cs.status = tier4_external_api_msgs::msg::CpuStatus::OK;
    cs.total = d.usage_pct * 100.0f;
    usage_msg.cpus.push_back(cs);
    usage_msg.all.total += cs.total;
  }
  pub_cpu_usage_->publish(usage_msg);

  tier4_external_api_msgs::msg::CpuTemperature temp_msg;
  temp_msg.stamp = node_ptr_->now();
  temp_msg.hostname = hostname_;
  temp_msg.temperature = 0.0f;
  temp_msg.thermal_throttling = thermal_throttling_
    ? tier4_external_api_msgs::msg::CpuTemperature::THERMAL_THROTTLING_ON
    : tier4_external_api_msgs::msg::CpuTemperature::THERMAL_THROTTLING_OFF;
  pub_cpu_temp_->publish(temp_msg);
}

}  // namespace autoware::system_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::system_monitor_host::plugin::CpuMonitorPlugin,
  autoware::system_monitor_host::plugin::MonitorPluginBase)
