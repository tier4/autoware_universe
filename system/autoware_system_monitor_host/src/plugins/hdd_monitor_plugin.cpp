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

#include "autoware/system_monitor_host/plugins/hdd_monitor_plugin.hpp"

#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>

namespace autoware::system_monitor_host::plugin
{

void HddMonitorPlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<diagnostic_updater::Updater> & updater)
{
  node_ptr_ = node_ptr;
  updater_ = updater;
  name_ = name;

  gethostname(hostname_, sizeof(hostname_));

  pub_hdd_ = node_ptr_->create_publisher<tier4_external_api_msgs::msg::HddStatus>(
    "~/hdd_status", rclcpp::QoS(1).transient_local());

  setup_params();

  updater_->setHardwareID(hostname_);
  updater_->add("HDD Temperature", this, &HddMonitorPlugin::check_temperature);
  updater_->add("HDD Usage", this, &HddMonitorPlugin::check_usage);
  updater_->add("HDD ReadDataRate", this, &HddMonitorPlugin::check_read_data_rate);
  updater_->add("HDD WriteDataRate", this, &HddMonitorPlugin::check_write_data_rate);
  updater_->add("HDD ReadIOPS", this, &HddMonitorPlugin::check_read_iops);
  updater_->add("HDD WriteIOPS", this, &HddMonitorPlugin::check_write_iops);
  updater_->add("HDD Connection", this, &HddMonitorPlugin::check_connection);

  timer_ = rclcpp::create_timer(
    node_ptr_, node_ptr_->get_clock(), std::chrono::seconds(1),
    std::bind(&HddMonitorPlugin::on_timer, this));
}

void HddMonitorPlugin::setup_params()
{
  node_ptr_->declare_parameter("hdd_monitor.usage_warn", 0.95);
  node_ptr_->declare_parameter("hdd_monitor.usage_error", 0.99);
  node_ptr_->declare_parameter("hdd_monitor.temp_warn", 55.0);
  node_ptr_->declare_parameter("hdd_monitor.temp_error", 60.0);

  hdd_usage_warn_ = node_ptr_->get_parameter("hdd_monitor.usage_warn").as_double();
  hdd_usage_error_ = node_ptr_->get_parameter("hdd_monitor.usage_error").as_double();
  hdd_temp_warn_ = node_ptr_->get_parameter("hdd_monitor.temp_warn").as_double();
  hdd_temp_error_ = node_ptr_->get_parameter("hdd_monitor.temp_error").as_double();
}

rcl_interfaces::msg::SetParametersResult HddMonitorPlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : parameters) {
    if (p.get_name() == "hdd_monitor.usage_warn") hdd_usage_warn_ = p.as_double();
    if (p.get_name() == "hdd_monitor.usage_error") hdd_usage_error_ = p.as_double();
    if (p.get_name() == "hdd_monitor.temp_warn") hdd_temp_warn_ = p.as_double();
    if (p.get_name() == "hdd_monitor.temp_error") hdd_temp_error_ = p.as_double();
  }
  return result;
}

void HddMonitorPlugin::evaluate()
{
  updater_->force_update();
}

void HddMonitorPlugin::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);

  FILE * fp = popen("df -B1M -P -x tmpfs -x devtmpfs -x squashfs 2>/dev/null", "r");
  if (fp) {
    disks_.clear();
    char buf[512];
    if (fgets(buf, sizeof(buf), fp) == nullptr) {
      pclose(fp);
      return;
    }

    while (fgets(buf, sizeof(buf), fp)) {
      std::istringstream iss(buf);
      std::string fs, size_str, used_str, avail_str, pct, mount;
      iss >> fs >> size_str >> used_str >> avail_str >> pct >> mount;

      if (mount == "/" || mount.rfind("/", 0) == 0) {
        DiskInfo di;
        di.device = fs;
        di.mount_point = mount;
        try {
          di.total_mb = std::stol(size_str);
          di.used_mb = std::stol(used_str);
          di.available_mb = std::stol(avail_str);
          pct.pop_back();
          di.usage_pct = std::stof(pct) / 100.0f;
        } catch (...) {
          continue;
        }
        disks_.push_back(di);
      }
    }
    pclose(fp);
  }

  tier4_external_api_msgs::msg::HddStatus hdd_msg;
  hdd_msg.stamp = node_ptr_->now();
  hdd_msg.hostname = hostname_;
  for (const auto & di : disks_) {
    tier4_external_api_msgs::msg::HddPartitionStatus part;
    part.filesystem = di.device;
    part.mounted_on = di.mount_point;
    part.size = static_cast<int32_t>(di.total_mb);
    part.used = static_cast<int32_t>(di.used_mb);
    part.avail = static_cast<int32_t>(di.available_mb);
    part.capacity = static_cast<int32_t>(di.usage_pct * 100.0f);
    hdd_msg.partitions.push_back(part);
  }
  pub_hdd_->publish(hdd_msg);
}

void HddMonitorPlugin::check_temperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & di : disks_) {
    stat.add(di.mount_point, di.temperature);
  }
  stat.summary(DiagStatus::OK, "OK");
}

void HddMonitorPlugin::check_usage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  int level = DiagStatus::OK;
  std::string msg = "OK";
  for (const auto & di : disks_) {
    stat.add(di.mount_point, di.usage_pct * 100.0f);
    if (di.usage_pct >= hdd_usage_error_) {
      level = DiagStatus::ERROR;
      msg = "disk full";
    } else if (di.usage_pct >= hdd_usage_warn_ && level != DiagStatus::ERROR) {
      level = DiagStatus::WARN;
      msg = "disk usage high";
    }
  }
  stat.summary(level, msg);
}

void HddMonitorPlugin::check_read_data_rate(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.summary(DiagStatus::OK, "OK");
}

void HddMonitorPlugin::check_write_data_rate(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.summary(DiagStatus::OK, "OK");
}

void HddMonitorPlugin::check_read_iops(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.summary(DiagStatus::OK, "OK");
}

void HddMonitorPlugin::check_write_iops(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.summary(DiagStatus::OK, "OK");
}

void HddMonitorPlugin::check_connection(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  int level = DiagStatus::OK;
  std::string msg = "OK";
  for (const auto & di : disks_) {
    stat.add(di.device, di.mounted ? "connected" : "disconnected");
    if (!di.mounted) {
      level = DiagStatus::ERROR;
      msg = "disk disconnected";
    }
  }
  stat.summary(level, msg);
}

}  // namespace autoware::system_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::system_monitor_host::plugin::HddMonitorPlugin,
  autoware::system_monitor_host::plugin::MonitorPluginBase)
