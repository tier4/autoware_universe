// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__MODEL_PLANNING_FACTOR_ID_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__MODEL_PLANNING_FACTOR_ID_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_utils/planning_factor_utils.hpp"

namespace autoware::trajectory_modifier::plugin
{

/**
 * @brief Inspection-only plugin that labels stop / slowdown implied by the planner trajectory.
 *
 * Learning-based planners (diffusion, tensorrt e2e, …) emit a pose sequence without an
 * explicit planning-factor. This plugin reads the *input* trajectory kinematics, does not
 * mutate points, and publishes `/planning/planning_factors/planning_model`
 * (launch remaps that topic to `/planning/planning_factors/neural_network_planner`).
 * The factor `module` (RViz red-wall reason) is `planning_model`; this plugin
 * only identifies that factor from the model trajectory.
 *
 * Load it first in `plugin_names` so labels describe the raw model output, not later
 * ObstacleStop / TrafficLightStop insertions.
 */
class ModelPlanningFactorID : public TrajectoryModifierPluginBase
{
public:
  ModelPlanningFactorID() = default;

  /**
   * @brief Detect stop/slowdown and queue planning factors. Never mutates @p traj_points.
   * @return Always false (inspection only). The host still publishes queued factors.
   */
  bool modify_trajectory(TrajectoryPoints & traj_points, const InputData & input) override;

  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, const InputData & input) override;

  void update_params(const TrajectoryModifierParams & params) override;

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;

private:
  void apply_params(const TrajectoryModifierParams & params);
  void add_detected_factors(const TrajectoryPoints & traj_points);

  TrajectoryModifierParams::ModelPlanningFactorId params_{};
  utils::PlanningFactorDetectionConfig detection_config_{};
};

}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__MODEL_PLANNING_FACTOR_ID_HPP_
