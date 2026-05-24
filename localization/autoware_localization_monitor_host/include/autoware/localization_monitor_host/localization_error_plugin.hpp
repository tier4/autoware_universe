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

#ifndef AUTOWARE__LOCALIZATION_MONITOR_HOST__LOCALIZATION_ERROR_PLUGIN_HPP_
#define AUTOWARE__LOCALIZATION_MONITOR_HOST__LOCALIZATION_ERROR_PLUGIN_HPP_

#include "localization_monitor_plugin_base.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::localization_monitor_host::plugin
{

class LocalizationErrorPlugin : public LocalizationMonitorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr) override;
  void set_up_params() override;
  void on_parameter(const std::vector<rclcpp::Parameter> & parameters) override;
  void evaluate(
    const MonitorData & data, diagnostic_updater::Updater & updater) override;

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ellipse_marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  double scale_{3.0};
  double error_ellipse_size_{1.0};
  double warn_ellipse_size_{0.8};
  double error_ellipse_size_lateral_direction_{0.35};
  double warn_ellipse_size_lateral_direction_{0.3};
};

}  // namespace autoware::localization_monitor_host::plugin

#endif  // AUTOWARE__LOCALIZATION_MONITOR_HOST__LOCALIZATION_ERROR_PLUGIN_HPP_
