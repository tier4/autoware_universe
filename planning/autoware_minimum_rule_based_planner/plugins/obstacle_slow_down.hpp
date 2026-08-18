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

#ifndef PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__OBSTACLE_SLOW_DOWN_HPP_
#define PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__OBSTACLE_SLOW_DOWN_HPP_

#include "plugin_interface.hpp"
#include "utils/obstacle_slow_down_utils.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <memory>

namespace autoware::minimum_rule_based_planner::plugin
{
using obstacle_slow_down_utils::SlowDownObstacle;
using obstacle_slow_down_utils::SlowDownPlanner;

class ObstacleSlowDown : public PluginInterface
{
public:
  ObstacleSlowDown() = default;

  void run(TrajectoryPoints & traj_points, const ModifierData & modifier_data) override;

  void update_params(const MinimumRuleBasedPlannerParams & params) override;

protected:
  void on_initialize(const MinimumRuleBasedPlannerParams & params) override;

private:
  MinimumRuleBasedPlannerParams::ObstacleSlowDown params_;

  // フィルタ・減速計画のノード非依存ロジックとフレーム間状態(SlowDownPlanner に集約)
  std::unique_ptr<SlowDownPlanner> planner_;

  void add_planning_factor(
    const TrajectoryPoints & slow_down_traj_points, const geometry_msgs::msg::Pose & current_pose,
    const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & end_pose,
    const bool is_driving_forward, const double slow_down_vel, const SlowDownObstacle & obstacle);
};

}  // namespace autoware::minimum_rule_based_planner::plugin

#endif  // PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__OBSTACLE_SLOW_DOWN_HPP_
