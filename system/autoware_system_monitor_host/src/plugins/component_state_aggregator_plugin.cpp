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

#include "autoware/system_monitor_host/plugins/component_state_aggregator_plugin.hpp"

#include <rclcpp/logging.hpp>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>
#include <vector>

namespace
{

bool mode_matches(const YAML::Node & mode_node, const std::string & run_mode)
{
  if (run_mode == "all") {
    return true;
  }
  if (!mode_node) {
    return true;
  }
  if (mode_node.IsScalar()) {
    return mode_node.as<std::string>() == run_mode;
  }
  if (mode_node.IsSequence()) {
    for (const auto & mode : mode_node) {
      if (mode.as<std::string>() == run_mode) {
        return true;
      }
    }
    return false;
  }
  return false;
}

std::string make_topic_monitor_diag_name(const std::string & module, const std::string & suffix)
{
  return "topic_state_monitor_" + suffix + ": " + module + "_topic_status";
}

}  // namespace

namespace autoware::system_monitor_host::plugin
{

void ComponentStateAggregatorPlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<diagnostic_updater::Updater> & updater)
{
  MonitorPluginBase::initialize(name, node_ptr, updater);

  sub_ = node_ptr_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(100),
    [this](const diagnostic_msgs::msg::DiagnosticArray & msg) { on_diag(msg); });

  // Create publishers for each type+module
  const auto component_state_qos = rclcpp::QoS(1).transient_local();
  auto pub_factory = [&](const std::string & type, const std::string & module) {
    std::string topic = "~/component/" + type + "/" + module;
    return node_ptr_->create_publisher<ModeAvailable>(topic, component_state_qos);
  };

  pub_launch_sensing_ = pub_factory("launch", "sensing");
  pub_launch_perception_ = pub_factory("launch", "perception");
  pub_launch_map_ = pub_factory("launch", "map");
  pub_launch_localization_ = pub_factory("launch", "localization");
  pub_launch_planning_ = pub_factory("launch", "planning");
  pub_launch_control_ = pub_factory("launch", "control");
  pub_launch_vehicle_ = pub_factory("launch", "vehicle");
  pub_launch_system_ = pub_factory("launch", "system");
  pub_autonomous_sensing_ = pub_factory("autonomous", "sensing");
  pub_autonomous_perception_ = pub_factory("autonomous", "perception");
  pub_autonomous_map_ = pub_factory("autonomous", "map");
  pub_autonomous_localization_ = pub_factory("autonomous", "localization");
  pub_autonomous_planning_ = pub_factory("autonomous", "planning");
  pub_autonomous_control_ = pub_factory("autonomous", "control");
  pub_autonomous_vehicle_ = pub_factory("autonomous", "vehicle");
  pub_autonomous_system_ = pub_factory("autonomous", "system");

  timer_ = rclcpp::create_timer(
    node_ptr_, node_ptr_->get_clock(), std::chrono::milliseconds(100),
    std::bind(&ComponentStateAggregatorPlugin::on_timer, this));
}

void ComponentStateAggregatorPlugin::setup_params()
{
  if (!node_ptr_->has_parameter("component_state_monitor.topic_monitor_names")) {
    node_ptr_->declare_parameter(
      "component_state_monitor.topic_monitor_names", std::vector<std::string>());
  }
  if (!node_ptr_->has_parameter("component_state_monitor.run_mode")) {
    node_ptr_->declare_parameter("component_state_monitor.run_mode", "online");
  }
  if (!node_ptr_->has_parameter("component_state_monitor.config_file")) {
    node_ptr_->declare_parameter("component_state_monitor.config_file", "");
  }

  const auto config_file =
    node_ptr_->get_parameter("component_state_monitor.config_file").as_string();
  const auto run_mode =
    node_ptr_->get_parameter("component_state_monitor.run_mode").as_string();

  watched_topics_.clear();
  module_states_.clear();

  const std::vector<std::string> modules = {
    "sensing", "perception", "map", "localization", "planning", "control", "vehicle", "system"};
  for (const auto & m : modules) {
    module_states_[m] = ModuleState{};
  }

  const auto add_watched_topic = [&](const std::string & diag_name, const std::string & module,
                                     const std::string & type) {
    if (diag_name.empty() || module.empty() || type.empty()) {
      return;
    }
    watched_topics_.push_back(TopicEntry{diag_name, module, type});
  };

  if (!config_file.empty() && std::filesystem::exists(config_file)) {
    try {
      const YAML::Node config = YAML::LoadFile(config_file);
      const auto topic_names =
        node_ptr_->get_parameter("component_state_monitor.topic_monitor_names")
          .as_string_array();

      if (!topic_names.empty()) {
        for (const auto & tn : topic_names) {
          add_watched_topic(tn, "", "launch");
        }
      } else if (config.IsSequence()) {
        for (const auto & row : config) {
          if (!mode_matches(row["mode"], run_mode)) {
            continue;
          }
          const auto args = row["args"];
          if (!args) {
            continue;
          }
          const std::string module = row["module"] ? row["module"].as<std::string>() : "";
          const std::string suffix =
            args["node_name_suffix"] ? args["node_name_suffix"].as<std::string>() : "";
          const std::string type = row["type"] ? row["type"].as<std::string>() : "";
          add_watched_topic(make_topic_monitor_diag_name(module, suffix), module, type);
        }
      } else {
        for (const auto & kv : config) {
          const YAML::Node entry = kv.second;
          if (!mode_matches(entry["mode"], run_mode)) {
            continue;
          }
          const std::string module = entry["module"] ? entry["module"].as<std::string>() : "";
          const std::string type = entry["type"] ? entry["type"].as<std::string>() : "";
          add_watched_topic("topic_state_monitor: " + kv.first.as<std::string>(), module, type);
        }
      }
      RCLCPP_INFO(
        node_ptr_->get_logger(), "ComponentStateAggregator: watching %zu topic diagnostics",
        watched_topics_.size());
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        node_ptr_->get_logger(),
        "ComponentStateAggregator: config parse error: " << e.what());
    }
  }
}

rcl_interfaces::msg::SetParametersResult ComponentStateAggregatorPlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & /*parameters*/)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void ComponentStateAggregatorPlugin::evaluate()
{
  // work happens on timer
}

void ComponentStateAggregatorPlugin::on_diag(const diagnostic_msgs::msg::DiagnosticArray & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & status : msg.status) {
    // Only track topic_state_monitor entries
    if (status.hardware_id != "topic_state_monitor") continue;

    // Map diagnostic name to level
    diag_levels_[status.name] = status.level;
  }
}

void ComponentStateAggregatorPlugin::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Reset states
  for (auto & [name, state] : module_states_) {
    state.launch = true;
    state.autonomous = true;
  }

  // Check each watched topic
  for (const auto & te : watched_topics_) {
    auto it = diag_levels_.find(te.name);
    bool ok = (it != diag_levels_.end() && it->second == diagnostic_msgs::msg::DiagnosticStatus::OK);

    auto state_it = module_states_.find(te.module);
    if (state_it != module_states_.end()) {
      if (te.type == "launch") {
        if (!ok) state_it->second.launch = false;
      }
      if (te.type == "autonomous") {
        if (!ok) state_it->second.autonomous = false;
      }
      // "autonomous" requires "launch" topics to also be OK
      if (state_it->second.autonomous && !state_it->second.launch) {
        state_it->second.autonomous = false;
      }
    }
  }

  // Publish changes
  auto publish_if_changed = [this](rclcpp::Publisher<ModeAvailable>::SharedPtr & pub,
                                   bool & prev, bool current) {
    if (prev != current) {
      ModeAvailable ma;
      ma.stamp = node_ptr_->now();
      ma.available = current;
      pub->publish(ma);
      prev = current;
    }
  };

  publish_if_changed(pub_launch_sensing_, module_states_["sensing"].launch_prev,
                     module_states_["sensing"].launch);
  publish_if_changed(pub_launch_perception_, module_states_["perception"].launch_prev,
                     module_states_["perception"].launch);
  publish_if_changed(pub_launch_map_, module_states_["map"].launch_prev,
                     module_states_["map"].launch);
  publish_if_changed(pub_launch_localization_, module_states_["localization"].launch_prev,
                     module_states_["localization"].launch);
  publish_if_changed(pub_launch_planning_, module_states_["planning"].launch_prev,
                     module_states_["planning"].launch);
  publish_if_changed(pub_launch_control_, module_states_["control"].launch_prev,
                     module_states_["control"].launch);
  publish_if_changed(pub_launch_vehicle_, module_states_["vehicle"].launch_prev,
                     module_states_["vehicle"].launch);
  publish_if_changed(pub_launch_system_, module_states_["system"].launch_prev,
                     module_states_["system"].launch);
  publish_if_changed(pub_autonomous_sensing_, module_states_["sensing"].autonomous_prev,
                     module_states_["sensing"].autonomous);
  publish_if_changed(pub_autonomous_perception_, module_states_["perception"].autonomous_prev,
                     module_states_["perception"].autonomous);
  publish_if_changed(pub_autonomous_map_, module_states_["map"].autonomous_prev,
                     module_states_["map"].autonomous);
  publish_if_changed(pub_autonomous_localization_, module_states_["localization"].autonomous_prev,
                     module_states_["localization"].autonomous);
  publish_if_changed(pub_autonomous_planning_, module_states_["planning"].autonomous_prev,
                     module_states_["planning"].autonomous);
  publish_if_changed(pub_autonomous_control_, module_states_["control"].autonomous_prev,
                     module_states_["control"].autonomous);
  publish_if_changed(pub_autonomous_vehicle_, module_states_["vehicle"].autonomous_prev,
                     module_states_["vehicle"].autonomous);
  publish_if_changed(pub_autonomous_system_, module_states_["system"].autonomous_prev,
                     module_states_["system"].autonomous);
}

}  // namespace autoware::system_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::system_monitor_host::plugin::ComponentStateAggregatorPlugin,
  autoware::system_monitor_host::plugin::MonitorPluginBase)
