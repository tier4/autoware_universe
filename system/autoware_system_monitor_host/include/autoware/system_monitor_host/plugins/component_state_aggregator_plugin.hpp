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

#ifndef AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__COMPONENT_STATE_AGGREGATOR_PLUGIN_HPP_
#define AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__COMPONENT_STATE_AGGREGATOR_PLUGIN_HPP_

#include "autoware/system_monitor_host/plugins/monitor_plugin_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tier4_system_msgs/msg/mode_change_available.hpp>

#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace autoware::system_monitor_host::plugin
{

class ComponentStateAggregatorPlugin : public MonitorPluginBase
{
public:
  ComponentStateAggregatorPlugin() = default;
  ~ComponentStateAggregatorPlugin() override = default;

  void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<diagnostic_updater::Updater> & updater) override;
  void setup_params() override;
  void evaluate() override;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  void on_diag(const diagnostic_msgs::msg::DiagnosticArray & msg);
  void on_timer();

  struct ModuleState
  {
    bool launch{false};
    bool autonomous{false};
    bool launch_prev{false};
    bool autonomous_prev{false};
  };

  struct TopicEntry
  {
    std::string name;
    std::string module;
    std::string type;  // "launch" or "autonomous"
  };

  using ModeAvailable = tier4_system_msgs::msg::ModeChangeAvailable;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::map<std::string, ModuleState> module_states_;
  std::vector<TopicEntry> watched_topics_;
  std::map<std::string, int> diag_levels_;  // topic_monitor_name -> level

  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_launch_sensing_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_launch_perception_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_launch_map_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_launch_localization_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_launch_planning_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_launch_control_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_launch_vehicle_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_launch_system_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_autonomous_sensing_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_autonomous_perception_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_autonomous_map_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_autonomous_localization_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_autonomous_planning_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_autonomous_control_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_autonomous_vehicle_;
  rclcpp::Publisher<ModeAvailable>::SharedPtr pub_autonomous_system_;

  mutable std::mutex mutex_;
};

}  // namespace autoware::system_monitor_host::plugin

#endif  // AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__COMPONENT_STATE_AGGREGATOR_PLUGIN_HPP_
