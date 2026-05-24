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

#ifndef AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__SAFETY_MONITOR_HOST_HPP_
#define AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__SAFETY_MONITOR_HOST_HPP_

#include "autoware/control_safety_monitor_host/safety_monitor_plugin_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::control_safety_monitor_host
{

class SafetyMonitorHost : public rclcpp::Node
{
public:
  explicit SafetyMonitorHost(const rclcpp::NodeOptions & options);

private:
  void set_up_params();
  void load_plugins();
  void apply_enable_flags();
  void on_timer();

  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters);

  std::vector<std::shared_ptr<plugin::SafetyMonitorPluginBase>> plugins_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool use_collision_detector_{true};
  bool use_obstacle_collision_checker_{false};
  bool use_predicted_path_checker_{false};
  double timer_period_{0.1};

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
};

}  // namespace autoware::control_safety_monitor_host

#endif  // AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__SAFETY_MONITOR_HOST_HPP_
