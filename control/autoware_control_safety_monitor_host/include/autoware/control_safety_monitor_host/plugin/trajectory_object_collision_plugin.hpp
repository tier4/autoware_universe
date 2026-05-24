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

#ifndef AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__PLUGIN__TRAJECTORY_OBJECT_COLLISION_PLUGIN_HPP_
#define AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__PLUGIN__TRAJECTORY_OBJECT_COLLISION_PLUGIN_HPP_

#include "autoware/control_safety_monitor_host/safety_monitor_plugin_base.hpp"

#include <autoware/predicted_path_checker/predicted_path_checker_core.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <memory>

namespace autoware::control_safety_monitor_host::plugin
{

class TrajectoryObjectCollisionPlugin : public SafetyMonitorPluginBase
{
public:
  void on_initialize() override;
  void update() override;
private:
  void on_diagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);

  std::unique_ptr<autoware::predicted_path_checker::PredictedPathCheckerCore> core_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;
};

}  // namespace autoware::control_safety_monitor_host::plugin

#endif  // AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__PLUGIN__TRAJECTORY_OBJECT_COLLISION_PLUGIN_HPP_
