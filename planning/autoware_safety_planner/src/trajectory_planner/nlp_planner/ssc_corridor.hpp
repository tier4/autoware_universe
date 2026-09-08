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

#ifndef AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__SSC_CORRIDOR_HPP_
#define AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__SSC_CORRIDOR_HPP_

// Carving of the spatio-temporal semantic corridor (Algorithm 1 of the SSC paper): generate the
// seed, inflate the cubes, attach the constraints.
//
// The seed is (s(t), l(t)) of the rough plan. The homotopy, i.e. which side to pass on, is already
// decided by the seed; the cubes model as much of the free space around it as they can while
// preserving it, and the carving performs no non-convex search of its own.
//
// Coordinates are Frenet (s, l) on the reference_path plus the time t. A cube bounds the **rear
// axle**, and the shape of the vehicle is absorbed by inflating the obstacles by the footprint, SSC
// treating the ego as a point.
//
// The time is partitioned in advance, since making the length of a piece a variable would render
// the problem non-convex, and the pieces do not overlap: the upper t of a cube is the lower t of
// the next.

#include "../../context.hpp"
#include "../../utils/sl_view_utils.hpp"
#include "constraints_compiler.hpp"
#include "rough_planner.hpp"

#include <vector>

namespace autoware::safety_planner
{

//! A box in (s, l, t); one per piece of the trajectory
struct SemanticCube
{
  double t0{0.0};  //!< [s]
  double t1{0.0};
  double s0{0.0};  //!< [m] arc length on the reference_path, of the rear axle
  double s1{0.0};
  double l0{0.0};  //!< [m] lateral offset of the rear axle, positive to the left
  double l1{0.0};
};

//! Parameters of the carving (ROS namespace `trajectory_optimizer.ssc_qp.*`)
struct SscCorridorParams
{
  double cube_duration_s{1.0};                //!< [s] duration alpha of one cube
  double margin_m{0.1};                       //!< [m] safety margin against obstacles and bounds
  double inflation_step_m{0.2};               //!< [m] one step of the inflation
  double max_lateral_inflation_m{4.0};        //!< [m] how far a face may leave the seed, in l
  double max_longitudinal_inflation_m{20.0};  //!< [m] the same, in s
};

//! One point of the seed, i.e. of the rough plan, in (s, l, t)
struct CorridorSeedPoint
{
  double t{0.0};
  double s{0.0};
  double l{0.0};
};

//! Converts the rough plan into the Frenet frame of the reference_path. s comes from the plan
//! itself; only l is projected from world coordinates.
std::vector<CorridorSeedPoint> make_corridor_seed(
  const PlannerContext & context, const RoughPlan & rough_plan);

//! Carves the cubes that enclose the seed together with its homotopy.
//! - [0, T] is divided evenly into pieces of about cube_duration_s, T being the end of the seed
//! - each cube starts as the smallest box around the seed of its time span, and its four faces are
//!   inflated in turn
//! - a face stops where it meets a constraint (the forbidden side of a boundary, an occupancy, a
//!   stop line)
//!
//! **An empty result rejects the corridor when the seed itself violates a constraint.** SSC can
//! only carve the free space around a seed that is already collision free; shrinking a blocked seed
//! would break the homotopy, i.e. the decision taken upstream. Emitting a stop plan in a blocked
//! cycle is the rough planner's job.
std::vector<SemanticCube> generate_semantic_corridor(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const std::vector<CorridorSeedPoint> & seed, const SscCorridorParams & params);

//! Whether a box of the rear axle stays clear of the constraints in (s, l, t); this is the test
//! the inflation runs
bool is_cube_free(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const SemanticCube & cube, double margin_m);

//! Converts the cubes into debug markers, from (s, l) back to world coordinates
MarkerArray make_corridor_markers(
  const PlannerContext & context, const std::vector<SemanticCube> & cubes, double z_base);

}  // namespace autoware::safety_planner

#endif  // AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__SSC_CORRIDOR_HPP_
