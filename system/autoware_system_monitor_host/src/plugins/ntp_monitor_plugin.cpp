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

#include "autoware/system_monitor_host/plugins/ntp_monitor_plugin.hpp"

#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <regex>
#include <sstream>
#include <string>

namespace autoware::system_monitor_host::plugin
{

void NtpMonitorPlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<diagnostic_updater::Updater> & updater)
{
  node_ptr_ = node_ptr;
  updater_ = updater;
  name_ = name;

  gethostname(hostname_, sizeof(hostname_));

  setup_params();

  updater_->setHardwareID(hostname_);
  updater_->add("NTP Offset", this, &NtpMonitorPlugin::check_ntp_offset);

  timer_ = rclcpp::create_timer(
    node_ptr_, node_ptr_->get_clock(), std::chrono::seconds(1),
    std::bind(&NtpMonitorPlugin::on_timer, this));
}

void NtpMonitorPlugin::setup_params()
{
  node_ptr_->declare_parameter("ntp_monitor.offset_warn", 0.1);
  node_ptr_->declare_parameter("ntp_monitor.offset_error", 5.0);

  offset_warn_ = node_ptr_->get_parameter("ntp_monitor.offset_warn").as_double();
  offset_error_ = node_ptr_->get_parameter("ntp_monitor.offset_error").as_double();
}

rcl_interfaces::msg::SetParametersResult NtpMonitorPlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : parameters) {
    if (p.get_name() == "ntp_monitor.offset_warn") offset_warn_ = p.as_double();
    if (p.get_name() == "ntp_monitor.offset_error") offset_error_ = p.as_double();
  }
  return result;
}

void NtpMonitorPlugin::evaluate()
{
  updater_->force_update();
}

void NtpMonitorPlugin::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);

  ntp_available_ = false;

  // Try chronyc tracking first
  FILE * fp = popen("chronyc tracking 2>/dev/null | grep 'System time'", "r");
  if (fp) {
    char buf[256];
    if (fgets(buf, sizeof(buf), fp)) {
      std::string line(buf);
      // Parse: "System time     : 0.000123456 seconds slow of NTP time"
      std::regex re(R"(([\d.]+)\s*seconds)");
      std::smatch match;
      if (std::regex_search(line, match, re)) {
        ntp_offset_ = std::stof(match[1].str());
        ntp_available_ = true;
      }
    }
    pclose(fp);
  }
}

void NtpMonitorPlugin::check_ntp_offset(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  int level = DiagStatus::OK;
  std::string msg = "OK";

  stat.add("offset", ntp_offset_);

  if (!ntp_available_) {
    level = DiagStatus::OK;
    msg = "NTP not available";
  } else {
    float abs_offset = std::abs(ntp_offset_);
    if (abs_offset >= offset_error_) {
      level = DiagStatus::ERROR;
      msg = "NTP offset too large";
    } else if (abs_offset >= offset_warn_) {
      level = DiagStatus::WARN;
      msg = "NTP offset high";
    }
  }
  stat.summary(level, msg);
}

}  // namespace autoware::system_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::system_monitor_host::plugin::NtpMonitorPlugin,
  autoware::system_monitor_host::plugin::MonitorPluginBase)
