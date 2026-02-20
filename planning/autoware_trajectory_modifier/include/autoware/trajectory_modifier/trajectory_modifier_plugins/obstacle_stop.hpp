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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__OBSTACLE_STOP_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__OBSTACLE_STOP_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp"

#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier::plugin
{

class ObstacleStop : public TrajectoryModifierPluginBase
{
public:
  ObstacleStop() = default;

  void modify_trajectory(TrajectoryPoints & traj_points) override;

  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points) const override;

  void update_params(const TrajectoryModifierParams & params) override
  {
    params_ = params.obstacle_stop;
    enabled_ = params.use_obstacle_stop;
  }
  const TrajectoryModifierParams::ObstacleStop & get_params() const { return params_; }

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;

private:
  TrajectoryModifierParams::ObstacleStop params_;

  bool check_obstacles();
  bool check_predicted_objects();
  bool check_pointcloud();
};

}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__OBSTACLE_STOP_HPP_
