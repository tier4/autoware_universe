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

#include "autoware/control_safety_monitor_host/plugin/surround_collision_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace autoware::control_safety_monitor_host::plugin
{

void SurroundCollisionPlugin::on_initialize()
{
  auto * node = get_node_ptr();
  core_ = std::make_unique<autoware::collision_detector::CollisionDetectorCore>(
    node, "collision_detector.", "input/odometry", "input/pointcloud", "input/objects");

  legacy_diag_pub_ = std::make_unique<LegacyDiagnosticPublisher>(node);
}

void SurroundCollisionPlugin::update()
{
  if (!is_enabled() || !legacy_diag_pub_) {
    return;
  }

  diagnostic_updater::DiagnosticStatusWrapper stat;
  stat.name = k_legacy_diag_name;
  stat.hardware_id = "collision_detector";
  check_collision(stat);
  legacy_diag_pub_->publish(
    get_node_ptr(), static_cast<diagnostic_msgs::msg::DiagnosticStatus>(stat));
}

void SurroundCollisionPlugin::check_collision(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (core_) {
    core_->update_diagnostics(stat);
  }
}

}  // namespace autoware::control_safety_monitor_host::plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::control_safety_monitor_host::plugin::SurroundCollisionPlugin,
  autoware::control_safety_monitor_host::plugin::SafetyMonitorPluginBase)
