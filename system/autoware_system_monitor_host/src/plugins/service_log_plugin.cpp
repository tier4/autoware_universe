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

#include "autoware/system_monitor_host/plugins/service_log_plugin.hpp"

#include <mutex>
#include <string>

namespace autoware::system_monitor_host::plugin
{

void ServiceLogPlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<diagnostic_updater::Updater> & updater)
{
  MonitorPluginBase::initialize(name, node_ptr, updater);

  sub_ = node_ptr_->create_subscription<tier4_system_msgs::msg::ServiceLog>(
    "/service_log", rclcpp::QoS(50),
    [this](const tier4_system_msgs::msg::ServiceLog & msg) { on_service_log(msg); });

  updater_->setHardwareID("service_log_checker");
  updater_->add("response_status", this, &ServiceLogPlugin::check_response_status);
}

void ServiceLogPlugin::setup_params()
{
}

void ServiceLogPlugin::evaluate()
{
  updater_->force_update();
}

void ServiceLogPlugin::on_service_log(const tier4_system_msgs::msg::ServiceLog & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Filter for client/server requests (errors from responses)
  if (msg.type == tier4_system_msgs::msg::ServiceLog::CLIENT_REQUEST ||
      msg.type == tier4_system_msgs::msg::ServiceLog::SERVER_REQUEST) {
    return;
  }

  // Check for error status codes
  if (msg.type == tier4_system_msgs::msg::ServiceLog::ERROR_UNREADY ||
      msg.type == tier4_system_msgs::msg::ServiceLog::ERROR_TIMEOUT) {
    std::string key = msg.name + " (" + msg.node + ")";
    ErrorInfo info;
    info.service_name = msg.name;
    info.node_name = msg.node;
    info.type = msg.type;
    errors_[key].push_back(info);
  }
}

void ServiceLogPlugin::check_response_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  using diagnostic_msgs::msg::DiagnosticStatus;

  if (errors_.empty()) {
    stat.summary(DiagnosticStatus::OK, "OK");
    return;
  }

  for (const auto & [key, infos] : errors_) {
    for (size_t i = 0; i < infos.size(); ++i) {
      stat.add("Failed Service", key);
    }
  }
  stat.summary(DiagnosticStatus::ERROR, "service failures detected");
}

}  // namespace autoware::system_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::system_monitor_host::plugin::ServiceLogPlugin,
  autoware::system_monitor_host::plugin::MonitorPluginBase)
