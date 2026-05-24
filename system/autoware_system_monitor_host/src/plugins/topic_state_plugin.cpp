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

#include "autoware/system_monitor_host/plugins/topic_state_plugin.hpp"

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

void TopicStatePlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<diagnostic_updater::Updater> & updater)
{
  MonitorPluginBase::initialize(name, node_ptr, updater);

  legacy_diag_pub_ = std::make_unique<LegacyDiagnosticPublisher>(node_ptr);
}

void TopicStatePlugin::setup_params()
{
  if (!node_ptr_->has_parameter("topic_state_monitor.config_file")) {
    node_ptr_->declare_parameter("topic_state_monitor.config_file", "");
  }
  if (!node_ptr_->has_parameter("topic_state_monitor.update_rate")) {
    node_ptr_->declare_parameter("topic_state_monitor.update_rate", 10.0);
  }
  if (!node_ptr_->has_parameter("topic_state_monitor.default_warn_rate")) {
    node_ptr_->declare_parameter("topic_state_monitor.default_warn_rate", 0.5);
  }
  if (!node_ptr_->has_parameter("topic_state_monitor.default_error_rate")) {
    node_ptr_->declare_parameter("topic_state_monitor.default_error_rate", 0.1);
  }
  if (!node_ptr_->has_parameter("topic_state_monitor.default_timeout")) {
    node_ptr_->declare_parameter("topic_state_monitor.default_timeout", 1.0);
  }
  if (!node_ptr_->has_parameter("topic_state_monitor.default_window_size")) {
    node_ptr_->declare_parameter("topic_state_monitor.default_window_size", 10);
  }
  if (!node_ptr_->has_parameter("topic_state_monitor.run_mode")) {
    node_ptr_->declare_parameter("topic_state_monitor.run_mode", "online");
  }

  auto config_file = node_ptr_->get_parameter("topic_state_monitor.config_file").as_string();
  auto run_mode = node_ptr_->get_parameter("topic_state_monitor.run_mode").as_string();
  auto default_warn = node_ptr_->get_parameter("topic_state_monitor.default_warn_rate").as_double();
  auto default_error = node_ptr_->get_parameter("topic_state_monitor.default_error_rate").as_double();
  auto default_timeout = node_ptr_->get_parameter("topic_state_monitor.default_timeout").as_double();
  auto default_window = node_ptr_->get_parameter("topic_state_monitor.default_window_size").as_int();

  entries_.clear();

  const auto register_entry = [this](TopicEntry te) {
    if (te.is_transform) {
      if (te.topic.empty() || te.frame_id.empty() || te.child_frame_id.empty()) {
        return;
      }
      rclcpp::QoS qos(rclcpp::KeepLast(10));
      if (te.best_effort) {
        qos.best_effort();
      }
      if (te.transient_local) {
        qos.transient_local();
      }
      te.tf_sub = node_ptr_->create_subscription<tf2_msgs::msg::TFMessage>(
        te.topic, qos,
        [this, frame_id = te.frame_id, child_frame_id = te.child_frame_id](
          tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
          for (const auto & transform : msg->transforms) {
            if (
              transform.header.frame_id == frame_id &&
              transform.child_frame_id == child_frame_id) {
              on_transform_message(frame_id, child_frame_id);
              break;
            }
          }
        });
    } else {
      if (te.topic.empty() || te.topic_type.empty()) {
        return;
      }
      rclcpp::QoS qos(rclcpp::KeepLast(10));
      if (te.best_effort) {
        qos.best_effort();
      }
      if (te.transient_local) {
        qos.transient_local();
      }
      te.sub = node_ptr_->create_generic_subscription(
        te.topic, te.topic_type, qos,
        [this, topic_name = te.topic](std::shared_ptr<rclcpp::SerializedMessage>) {
          on_generic_message(topic_name);
        });
    }

    entries_.push_back(te);
  };

  const auto load_args_entry = [&](const YAML::Node & row) {
    if (!mode_matches(row["mode"], run_mode)) {
      return;
    }

    const auto args = row["args"];
    if (!args) {
      return;
    }

    const std::string module = row["module"] ? row["module"].as<std::string>() : "";
    const std::string suffix =
      args["node_name_suffix"] ? args["node_name_suffix"].as<std::string>() : "";

    TopicEntry te;
    te.diag_name = make_topic_monitor_diag_name(module, suffix);
    te.topic = args["topic"] ? args["topic"].as<std::string>() : "";
    te.topic_type = args["topic_type"] ? args["topic_type"].as<std::string>() : "";
    te.frame_id = args["frame_id"] ? args["frame_id"].as<std::string>() : "";
    te.child_frame_id = args["child_frame_id"] ? args["child_frame_id"].as<std::string>() : "";
    te.is_transform = te.topic == "/tf" || te.topic == "/tf_static";
    te.best_effort = args["best_effort"] ? args["best_effort"].as<bool>() : false;
    te.transient_local = args["transient_local"] ? args["transient_local"].as<bool>() : false;
    te.warn_rate = args["warn_rate"] ? args["warn_rate"].as<double>() : default_warn;
    te.error_rate = args["error_rate"] ? args["error_rate"].as<double>() : default_error;
    te.timeout = args["timeout"] ? args["timeout"].as<double>() : default_timeout;
    te.window_size = args["window_size"] ? args["window_size"].as<int>() : default_window;
    te.module = module;
    te.type = row["type"] ? row["type"].as<std::string>() : "";
    register_entry(te);
  };

  if (!config_file.empty() && std::filesystem::exists(config_file)) {
    try {
      const YAML::Node config = YAML::LoadFile(config_file);
      if (config.IsSequence()) {
        for (const auto & row : config) {
          load_args_entry(row);
        }
      } else {
        for (const auto & kv : config) {
          const std::string key = kv.first.as<std::string>();
          const YAML::Node entry = kv.second;

          if (!mode_matches(entry["mode"], run_mode)) {
            continue;
          }

          TopicEntry te;
          te.diag_name = "topic_state_monitor: " + key;
          te.topic = entry["topic"] ? entry["topic"].as<std::string>() : "";
          te.topic_type = entry["topic_type"] ? entry["topic_type"].as<std::string>() : "";
          te.best_effort = entry["best_effort"] ? entry["best_effort"].as<bool>() : false;
          te.transient_local = entry["transient_local"] ? entry["transient_local"].as<bool>() : false;
          te.warn_rate = entry["warn_rate"] ? entry["warn_rate"].as<double>() : default_warn;
          te.error_rate = entry["error_rate"] ? entry["error_rate"].as<double>() : default_error;
          te.timeout = entry["timeout"] ? entry["timeout"].as<double>() : default_timeout;
          te.window_size = entry["window_size"] ? entry["window_size"].as<int>() : default_window;
          te.module = entry["module"] ? entry["module"].as<std::string>() : "";
          te.type = entry["type"] ? entry["type"].as<std::string>() : "";
          register_entry(te);
        }
      }
      RCLCPP_INFO(
        node_ptr_->get_logger(), "TopicStatePlugin: monitoring %zu topics", entries_.size());
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        node_ptr_->get_logger(), "TopicStatePlugin: failed to parse config: " << e.what());
    }
  }
}

rcl_interfaces::msg::SetParametersResult TopicStatePlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & /*parameters*/)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void TopicStatePlugin::evaluate()
{
  if (!legacy_diag_pub_) {
    return;
  }

  std::vector<diagnostic_msgs::msg::DiagnosticStatus> statuses;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    statuses.reserve(entries_.size());
    for (auto & entry : entries_) {
      diagnostic_updater::DiagnosticStatusWrapper stat;
      stat.name = entry.diag_name;
      stat.hardware_id = "topic_state_monitor";
      produce_diagnostics(stat, entry);
      statuses.push_back(static_cast<diagnostic_msgs::msg::DiagnosticStatus>(stat));
    }
  }
  legacy_diag_pub_->publish(node_ptr_, statuses);
}

void TopicStatePlugin::on_generic_message(const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = node_ptr_->now();

  for (auto & entry : entries_) {
    if (!entry.is_transform && entry.topic == topic_name) {
      entry.last_message_time = now;
      entry.time_buffer.push_back(now);
      entry.ever_received = true;

      while (static_cast<int>(entry.time_buffer.size()) > entry.window_size) {
        entry.time_buffer.pop_front();
      }
      return;
    }
  }
}

void TopicStatePlugin::on_transform_message(
  const std::string & frame_id, const std::string & child_frame_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = node_ptr_->now();

  for (auto & entry : entries_) {
    if (
      entry.is_transform && entry.frame_id == frame_id &&
      entry.child_frame_id == child_frame_id) {
      entry.last_message_time = now;
      entry.time_buffer.push_back(now);
      entry.ever_received = true;

      while (static_cast<int>(entry.time_buffer.size()) > entry.window_size) {
        entry.time_buffer.pop_front();
      }
      return;
    }
  }
}

void TopicStatePlugin::produce_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat, TopicEntry & entry)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  int level = DiagnosticStatus::OK;
  std::string msg = "OK";
  const auto now = node_ptr_->now();

  if (entry.is_transform) {
    stat.addf("topic", "%s (%s to %s)", entry.topic.c_str(), entry.frame_id.c_str(),
      entry.child_frame_id.c_str());
  } else {
    stat.add("topic", entry.topic);
  }

  if (!entry.ever_received) {
    level = DiagnosticStatus::ERROR;
    msg = "NotReceived";
  } else if (entry.timeout > 0.0) {
    const double time_diff = (now - entry.last_message_time).seconds();
    if (time_diff > entry.timeout) {
      level = DiagnosticStatus::ERROR;
      msg = "Timeout";
    }
  }

  if (level == DiagnosticStatus::OK && entry.ever_received && !entry.time_buffer.empty()) {
    const rclcpp::Duration span = now - entry.time_buffer.front();
    const double span_sec = span.seconds();
    if (span_sec > 0.0 && static_cast<int>(entry.time_buffer.size()) >= 2) {
      const double rate =
        static_cast<double>(entry.time_buffer.size() - 1) / span_sec;
      stat.add("measured_rate", rate);

      if (entry.error_rate > 0.0 && rate < entry.error_rate) {
        level = DiagnosticStatus::ERROR;
        msg = "ErrorRate";
      } else if (entry.warn_rate > 0.0 && rate < entry.warn_rate) {
        level = DiagnosticStatus::WARN;
        msg = "WarnRate";
      }
    }
  }

  stat.add("hardware_id", "topic_state_monitor");
  stat.summary(level, msg);
}

}  // namespace autoware::system_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::system_monitor_host::plugin::TopicStatePlugin,
  autoware::system_monitor_host::plugin::MonitorPluginBase)
