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

#ifndef CONSTRAINT_GENERATOR__OBSTACLE_STOP_HPP_
#define CONSTRAINT_GENERATOR__OBSTACLE_STOP_HPP_

#include "constraint_generator_interface.hpp"

#include <autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp>

#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner::experimental
{

class ObstacleStopConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "obstacle_stop"; }
  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;

private:
  using CollisionPoint = trajectory_modifier::utils::obstacle_stop::CollisionPoint;

  void update_collision_points_buffer(
    const TrajectoryPoints & traj_points, const std::optional<CollisionPoint> & collision_point,
    const rclcpp::Time & now);
  std::optional<CollisionPoint> get_nearest_collision_point() const;

  std::vector<CollisionPoint> collision_points_buffer_;
};

}  // namespace autoware::safety_planner::experimental

#endif  // CONSTRAINT_GENERATOR__OBSTACLE_STOP_HPP_
