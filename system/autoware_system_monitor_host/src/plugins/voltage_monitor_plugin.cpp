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

#include "autoware/system_monitor_host/plugins/voltage_monitor_plugin.hpp"

#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <regex>
#include <string>

namespace autoware::system_monitor_host::plugin
{

void VoltageMonitorPlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<diagnostic_updater::Updater> & updater)
{
  node_ptr_ = node_ptr;
  updater_ = updater;
  name_ = name;

  gethostname(hostname_, sizeof(hostname_));

  setup_params();

  updater_->setHardwareID(hostname_);
  updater_->add("CMOS Battery Status", this, &VoltageMonitorPlugin::check_cmos_battery);

  timer_ = rclcpp::create_timer(
    node_ptr_, node_ptr_->get_clock(), std::chrono::seconds(10),
    std::bind(&VoltageMonitorPlugin::on_timer, this));
}

void VoltageMonitorPlugin::setup_params()
{
  node_ptr_->declare_parameter("voltage_monitor.cmos_battery_label", "");
  node_ptr_->declare_parameter("voltage_monitor.cmos_battery_warn", 2.95);
  node_ptr_->declare_parameter("voltage_monitor.cmos_battery_error", 2.75);

  cmos_battery_label_ = node_ptr_->get_parameter("voltage_monitor.cmos_battery_label").as_string();
  cmos_battery_warn_ = node_ptr_->get_parameter("voltage_monitor.cmos_battery_warn").as_double();
  cmos_battery_error_ = node_ptr_->get_parameter("voltage_monitor.cmos_battery_error").as_double();
}

rcl_interfaces::msg::SetParametersResult VoltageMonitorPlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : parameters) {
    if (p.get_name() == "voltage_monitor.cmos_battery_label") cmos_battery_label_ = p.as_string();
    if (p.get_name() == "voltage_monitor.cmos_battery_warn") cmos_battery_warn_ = p.as_double();
    if (p.get_name() == "voltage_monitor.cmos_battery_error") cmos_battery_error_ = p.as_double();
  }
  return result;
}

void VoltageMonitorPlugin::evaluate()
{
  updater_->force_update();
}

void VoltageMonitorPlugin::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);

  cmos_battery_available_ = false;

  // Try sensors command
  FILE * fp = popen("sensors -u 2>/dev/null", "r");
  if (fp) {
    char buf[512];
    std::string line;
    std::string current_chip;
    while (fgets(buf, sizeof(buf), fp)) {
      line = buf;
      // Remove newline
      if (!line.empty() && line.back() == '\n') line.pop_back();

      // Parse chip label or voltage
      if (line.empty()) continue;
      if (line.back() == ':') {
        current_chip = line.substr(0, line.size() - 1);
      } else if (line.find("_input:") != std::string::npos) {
        std::regex re(R"(([\d.]+))");
        std::smatch match;
        if (std::regex_search(line, match, re)) {
          float voltage = std::stof(match[1].str());
          if (!cmos_battery_label_.empty()) {
            if (current_chip.find(cmos_battery_label_) != std::string::npos ||
                (cmos_battery_label_.empty())) {
              cmos_battery_voltage_ = voltage;
              cmos_battery_available_ = true;
            }
          } else {
            cmos_battery_voltage_ = voltage;
            cmos_battery_available_ = true;
          }
        }
      }
    }
    pclose(fp);
  }

  // Fallback: check /proc/driver/rtc for batt_status
  if (!cmos_battery_available_) {
    std::ifstream rtc("/proc/driver/rtc");
    if (rtc.is_open()) {
      std::string line;
      while (std::getline(rtc, line)) {
        if (line.find("batt_status") != std::string::npos) {
          cmos_battery_ok_ = (line.find("okay") != std::string::npos || line.find("OK") != std::string::npos);
          cmos_battery_available_ = true;
        }
      }
    }
  }
}

void VoltageMonitorPlugin::check_cmos_battery(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  int level = DiagStatus::OK;
  std::string msg = "OK";

  if (!cmos_battery_available_) {
    level = DiagStatus::OK;
    msg = "CMOS battery status not available";
  } else if (cmos_battery_voltage_ > 0) {
    stat.add("voltage", cmos_battery_voltage_);
    if (cmos_battery_voltage_ <= cmos_battery_error_) {
      level = DiagStatus::ERROR;
      msg = "CMOS battery voltage critically low";
    } else if (cmos_battery_voltage_ <= cmos_battery_warn_) {
      level = DiagStatus::WARN;
      msg = "CMOS battery voltage low";
    }
  } else {
    if (!cmos_battery_ok_) {
      level = DiagStatus::ERROR;
      msg = "CMOS battery not okay";
    }
  }
  stat.summary(level, msg);
}

}  // namespace autoware::system_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::system_monitor_host::plugin::VoltageMonitorPlugin,
  autoware::system_monitor_host::plugin::MonitorPluginBase)
