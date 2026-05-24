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

#ifndef AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__DUPLICATED_NODE_PLUGIN_HPP_
#define AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__DUPLICATED_NODE_PLUGIN_HPP_

#include "autoware/system_monitor_host/legacy_diagnostic_publisher.hpp"
#include "autoware/system_monitor_host/plugins/monitor_plugin_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::system_monitor_host::plugin
{

class DuplicatedNodePlugin : public MonitorPluginBase
{
public:
  DuplicatedNodePlugin() = default;
  ~DuplicatedNodePlugin() override = default;

  void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<diagnostic_updater::Updater> & updater) override;
  void setup_params() override;
  void evaluate() override;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  void check_duplicated(diagnostic_updater::DiagnosticStatusWrapper & stat);
  std::vector<std::string> find_identical_names(const std::vector<std::string> & node_names);

  std::unordered_set<std::string> nodes_to_ignore_;
  bool add_duplicated_node_names_to_msg_{false};

  std::vector<std::string> duplicated_names_;
  mutable std::mutex mutex_;
  std::unique_ptr<LegacyDiagnosticPublisher> legacy_diag_pub_;
  static constexpr const char * k_legacy_diag_name = "duplicated_node_checker: duplicated_node_checker";
};

}  // namespace autoware::system_monitor_host::plugin

#endif  // AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__DUPLICATED_NODE_PLUGIN_HPP_
