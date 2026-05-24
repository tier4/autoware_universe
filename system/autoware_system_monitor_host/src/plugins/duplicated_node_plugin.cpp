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

#include "autoware/system_monitor_host/plugins/duplicated_node_plugin.hpp"

#include <algorithm>
#include <set>
#include <string>
#include <vector>

namespace autoware::system_monitor_host::plugin
{

void DuplicatedNodePlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<diagnostic_updater::Updater> & updater)
{
  MonitorPluginBase::initialize(name, node_ptr, updater);
  legacy_diag_pub_ = std::make_unique<LegacyDiagnosticPublisher>(node_ptr);
}

void DuplicatedNodePlugin::setup_params()
{
  if (!node_ptr_->has_parameter("duplicated_node_checker.update_rate")) {
    node_ptr_->declare_parameter("duplicated_node_checker.update_rate", 10.0);
  }
  if (!node_ptr_->has_parameter("duplicated_node_checker.add_duplicated_node_names_to_msg")) {
    node_ptr_->declare_parameter("duplicated_node_checker.add_duplicated_node_names_to_msg", false);
  }
  if (!node_ptr_->has_parameter("duplicated_node_checker.nodes_to_ignore")) {
    node_ptr_->declare_parameter(
      "duplicated_node_checker.nodes_to_ignore", std::vector<std::string>());
  }

  add_duplicated_node_names_to_msg_ =
    node_ptr_->get_parameter("duplicated_node_checker.add_duplicated_node_names_to_msg").as_bool();
  auto ignore_list =
    node_ptr_->get_parameter("duplicated_node_checker.nodes_to_ignore").as_string_array();
  nodes_to_ignore_.insert(ignore_list.begin(), ignore_list.end());
}

rcl_interfaces::msg::SetParametersResult DuplicatedNodePlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : parameters) {
    if (p.get_name() == "duplicated_node_checker.add_duplicated_node_names_to_msg") {
      add_duplicated_node_names_to_msg_ = p.as_bool();
    }
    if (p.get_name() == "duplicated_node_checker.nodes_to_ignore") {
      nodes_to_ignore_.clear();
      auto ignore_list = p.as_string_array();
      nodes_to_ignore_.insert(ignore_list.begin(), ignore_list.end());
    }
  }
  return result;
}

void DuplicatedNodePlugin::evaluate()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    duplicated_names_ = find_identical_names(node_ptr_->get_node_names());
  }

  if (!legacy_diag_pub_) {
    return;
  }

  diagnostic_updater::DiagnosticStatusWrapper stat;
  stat.name = k_legacy_diag_name;
  stat.hardware_id = "duplicated_node_checker";
  check_duplicated(stat);
  legacy_diag_pub_->publish(
    node_ptr_, {static_cast<diagnostic_msgs::msg::DiagnosticStatus>(stat)});
}

std::vector<std::string> DuplicatedNodePlugin::find_identical_names(
  const std::vector<std::string> & node_names)
{
  std::vector<std::string> identical;
  std::set<std::string> seen;

  for (const auto & name : node_names) {
    if (nodes_to_ignore_.find(name) != nodes_to_ignore_.end()) continue;
    if (seen.find(name) != seen.end()) {
      identical.push_back(name);
    } else {
      seen.insert(name);
    }
  }
  return identical;
}

void DuplicatedNodePlugin::check_duplicated(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  using diagnostic_msgs::msg::DiagnosticStatus;

  int level = DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!duplicated_names_.empty()) {
    level = DiagnosticStatus::ERROR;
    msg = "Error: Duplicated nodes detected";
    if (add_duplicated_node_names_to_msg_) {
      std::set<std::string> unique(duplicated_names_.begin(), duplicated_names_.end());
      for (const auto & n : unique) {
        msg += "[" + n + "], ";
      }
    }
    for (const auto & n : duplicated_names_) {
      stat.add("Duplicated Node Name", n);
    }
  }
  stat.summary(level, msg);
}

}  // namespace autoware::system_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::system_monitor_host::plugin::DuplicatedNodePlugin,
  autoware::system_monitor_host::plugin::MonitorPluginBase)
