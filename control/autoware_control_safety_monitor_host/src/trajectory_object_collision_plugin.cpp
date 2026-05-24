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

#include "autoware/control_safety_monitor_host/plugin/trajectory_object_collision_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace autoware::control_safety_monitor_host::plugin
{

void TrajectoryObjectCollisionPlugin::on_initialize()
{
  auto * node = get_node_ptr();
  core_ = std::make_unique<autoware::predicted_path_checker::PredictedPathCheckerCore>(
    node, "predicted_path_checker.", "input/objects", "input/reference_trajectory",
    "input/predicted_trajectory", "input/odometry", "input/current_accel");

  updater_ = std::make_unique<diagnostic_updater::Updater>(node);
  updater_->setHardwareID("predicted_path_checker");
  updater_->add(
    "predicted_path_checker: predicted_path_checker", this,
    &TrajectoryObjectCollisionPlugin::on_diagnostic);
}

void TrajectoryObjectCollisionPlugin::update()
{
  if (!is_enabled() || !core_ || !updater_) {
    return;
  }
  core_->on_timer_cycle();
  updater_->force_update();
}

void TrajectoryObjectCollisionPlugin::on_diagnostic(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (core_) {
    core_->check_vehicle_state(stat);
  }
}

}  // namespace autoware::control_safety_monitor_host::plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::control_safety_monitor_host::plugin::TrajectoryObjectCollisionPlugin,
  autoware::control_safety_monitor_host::plugin::SafetyMonitorPluginBase)
