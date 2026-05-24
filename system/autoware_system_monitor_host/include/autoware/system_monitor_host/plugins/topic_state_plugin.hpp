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

#ifndef AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__TOPIC_STATE_PLUGIN_HPP_
#define AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__TOPIC_STATE_PLUGIN_HPP_

#include "autoware/system_monitor_host/legacy_diagnostic_publisher.hpp"
#include "autoware/system_monitor_host/plugins/monitor_plugin_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::system_monitor_host::plugin
{

class TopicStatePlugin : public MonitorPluginBase
{
public:
  TopicStatePlugin() = default;
  ~TopicStatePlugin() override = default;

  void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<diagnostic_updater::Updater> & updater) override;
  void setup_params() override;
  void evaluate() override;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  struct TopicEntry
  {
    std::string diag_name;
    std::string topic;
    std::string topic_type;
    std::string frame_id;
    std::string child_frame_id;
    bool is_transform{false};
    bool transient_local{false};
    bool best_effort{false};
    double warn_rate{0.5};
    double error_rate{0.1};
    double timeout{1.0};
    int window_size{10};
    std::string module;
    std::string type;
    std::string mode;

    rclcpp::GenericSubscription::SharedPtr sub;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub;
    rclcpp::Time last_message_time;
    std::deque<rclcpp::Time> time_buffer;
    bool ever_received{false};
  };

  void on_generic_message(const std::string & topic_name);
  void on_transform_message(const std::string & frame_id, const std::string & child_frame_id);
  void produce_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper & stat, TopicEntry & entry);

  std::vector<TopicEntry> entries_;
  mutable std::mutex mutex_;
  std::unique_ptr<LegacyDiagnosticPublisher> legacy_diag_pub_;
};

}  // namespace autoware::system_monitor_host::plugin

#endif  // AUTOWARE__SYSTEM_MONITOR_HOST__PLUGINS__TOPIC_STATE_PLUGIN_HPP_
