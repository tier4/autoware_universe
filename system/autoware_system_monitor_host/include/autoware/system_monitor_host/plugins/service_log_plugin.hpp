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

#ifndef AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__SERVICE_LOG_PLUGIN_HPP_
#define AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__SERVICE_LOG_PLUGIN_HPP_

#include "autoware/system_monitor_host/plugins/monitor_plugin_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/service_log.hpp>

#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::system_monitor_host::plugin
{

class ServiceLogPlugin : public MonitorPluginBase
{
public:
  ServiceLogPlugin() = default;
  ~ServiceLogPlugin() override = default;

  void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<diagnostic_updater::Updater> & updater) override;
  void setup_params() override;
  void evaluate() override;

private:
  void on_service_log(const tier4_system_msgs::msg::ServiceLog & msg);
  void check_response_status(diagnostic_updater::DiagnosticStatusWrapper & stat);

  rclcpp::Subscription<tier4_system_msgs::msg::ServiceLog>::SharedPtr sub_;

  struct ErrorInfo
  {
    std::string service_name;
    std::string node_name;
    uint8_t type;
    int code;
    std::string message;
  };
  std::map<std::string, std::vector<ErrorInfo>> errors_;
  mutable std::mutex mutex_;
};

}  // namespace autoware::system_monitor_host::plugin

#endif  // AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__SERVICE_LOG_PLUGIN_HPP_
