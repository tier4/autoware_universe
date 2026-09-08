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

#ifndef AUTOWARE__SAFETY_PLANNER__CONSTRAINT_GENERATOR__OBSTACLE_STOP_HPP_
#define AUTOWARE__SAFETY_PLANNER__CONSTRAINT_GENERATOR__OBSTACLE_STOP_HPP_

#include "constraint_generator_interface.hpp"

#include <rclcpp/time.hpp>

#include <string>
#include <vector>

namespace autoware::safety_planner
{

//! Emits one rigid-body KeepOut with certainty = DEFINITE per predicted object. The waypoints are
//! the most likely predicted path; an object without one becomes a single waypoint, i.e. a static
//! object.
//! Why not emit every predicted mode: binding the less likely modes as DEFINITE too makes the
//! nominal trajectory needlessly stiff. Low-confidence modes belong to a plugin that emits
//! POSSIBLE (run_out, ...).
class ObstacleStopConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "obstacle_stop"; }
  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;
};

//! PredictedObjects -> KeepOut constraints. t_plan is the planning reference time, i.e. the stamp
//! of the odometry; the offset to the perception stamp is folded into the t of the waypoints.
//! Objects with a degenerate shape or a non-finite pose are dropped and the rest is kept.
std::vector<Constraint> make_obstacle_keep_out_constraints(
  const PredictedObjects & objects, const rclcpp::Time & t_plan, const double margin_m);

//! One Gate (stop line) per slow object whose footprint overlaps the reference-path corridor
//! ahead of the ego, placed stop_distance_m before the object's nearest point along the path.
//! Objects faster than max_object_speed_mps get no stop line (the KeepOut handles them).
//! Why not a stop line for every object: stopping 6 m behind a moving lead vehicle is wrong;
//! following is the KeepOut's job.
std::vector<Constraint> make_obstacle_stop_line_constraints(
  const PredictedObjects & objects, const PathPointTrajectory & reference_path, const double s_ego,
  const double corridor_half_width_m, const double stop_distance_m,
  const double max_object_speed_mps);

}  // namespace autoware::safety_planner

#endif  // AUTOWARE__SAFETY_PLANNER__CONSTRAINT_GENERATOR__OBSTACLE_STOP_HPP_
