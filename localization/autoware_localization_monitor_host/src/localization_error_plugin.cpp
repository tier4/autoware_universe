// Copyright 2025 Autoware Foundation
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

#include "autoware/localization_monitor_host/localization_error_plugin.hpp"

#include "autoware/localization_monitor_host/monitor_data.hpp"

#include <autoware/localization_util/covariance_ellipse.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <memory>
#include <string>

namespace autoware::localization_monitor_host::plugin
{

void LocalizationErrorPlugin::initialize(const std::string & name, rclcpp::Node * node_ptr)
{
  LocalizationMonitorPluginBase::initialize(name, node_ptr);

  rclcpp::QoS durable_qos(1);
  durable_qos.transient_local();
  ellipse_marker_pub_ =
    node_ptr->create_publisher<visualization_msgs::msg::Marker>("debug/ellipse_marker", durable_qos);

  diagnostics_pub_ =
    node_ptr->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
}

void LocalizationErrorPlugin::set_up_params()
{
  auto * node = get_node_ptr();

  scale_ = node->declare_parameter<double>("localization_error.scale");
  error_ellipse_size_ = node->declare_parameter<double>("localization_error.error_ellipse_size");
  warn_ellipse_size_ = node->declare_parameter<double>("localization_error.warn_ellipse_size");
  error_ellipse_size_lateral_direction_ =
    node->declare_parameter<double>("localization_error.error_ellipse_size_lateral_direction");
  warn_ellipse_size_lateral_direction_ =
    node->declare_parameter<double>("localization_error.warn_ellipse_size_lateral_direction");
}

void LocalizationErrorPlugin::on_parameter(const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    if (param.get_name() == "localization_error.scale") {
      scale_ = param.as_double();
    } else if (param.get_name() == "localization_error.error_ellipse_size") {
      error_ellipse_size_ = param.as_double();
    } else if (param.get_name() == "localization_error.warn_ellipse_size") {
      warn_ellipse_size_ = param.as_double();
    } else if (param.get_name() == "localization_error.error_ellipse_size_lateral_direction") {
      error_ellipse_size_lateral_direction_ = param.as_double();
    } else if (param.get_name() == "localization_error.warn_ellipse_size_lateral_direction") {
      warn_ellipse_size_lateral_direction_ = param.as_double();
    }
  }
}

void LocalizationErrorPlugin::evaluate(
  const MonitorData & data, diagnostic_updater::Updater & /*updater*/)
{
  if (!data.latest_odom.has_value()) {
    return;
  }

  const auto & odom = data.latest_odom.value();
  const auto ellipse =
    autoware::localization_util::calculate_xy_ellipse(odom.pose, scale_);

  const auto ellipse_marker = autoware::localization_util::create_ellipse_marker(
    ellipse, odom.header, odom.pose);
  ellipse_marker_pub_->publish(ellipse_marker);

  const auto long_status = [&]() {
    diagnostic_msgs::msg::DiagnosticStatus stat;
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "OK";
    if (ellipse.long_radius >= warn_ellipse_size_) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      stat.message = "ellipse size is too large";
    }
    if (ellipse.long_radius >= error_ellipse_size_) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      stat.message = "ellipse size is over the expected range";
    }
    return stat;
  }();

  const auto lateral_status = [&]() {
    diagnostic_msgs::msg::DiagnosticStatus stat;
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "OK";
    if (ellipse.size_lateral_direction >= warn_ellipse_size_lateral_direction_) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      stat.message = "ellipse size along lateral direction is too large";
    }
    if (ellipse.size_lateral_direction >= error_ellipse_size_lateral_direction_) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      stat.message = "ellipse size along lateral direction is over the expected range";
    }
    return stat;
  }();

  // Publish diagnostics preserving original diagnostic identity for graph compatibility
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "localization_error_monitor: ellipse_error_status";
  status.hardware_id = "localization_error_monitor";

  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "localization_error_ellipse";
  kv.value = std::to_string(ellipse.long_radius);
  status.values.push_back(kv);
  kv.key = "localization_error_ellipse_lateral_direction";
  kv.value = std::to_string(ellipse.size_lateral_direction);
  status.values.push_back(kv);

  // Merge levels: use the worse of the two checks
  status.level = std::max(long_status.level, lateral_status.level);
  if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    status.message = "OK";
  } else if (status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    status.message = "ellipse size is too large";
  } else {
    status.message = "ellipse size is over the expected range";
  }

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = get_node_ptr()->now();
  diagnostics.status.emplace_back(status);
  diagnostics_pub_->publish(diagnostics);
}

}  // namespace autoware::localization_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::localization_monitor_host::plugin::LocalizationErrorPlugin,
  autoware::localization_monitor_host::plugin::LocalizationMonitorPluginBase)
