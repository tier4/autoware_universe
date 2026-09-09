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

#ifndef TRAJECTORY_PLANNER__FRENET_SAMPLING_BASED_PLANNER__FRENET_SAMPLING_BASED_PLANNER_HPP_
#define TRAJECTORY_PLANNER__FRENET_SAMPLING_BASED_PLANNER__FRENET_SAMPLING_BASED_PLANNER_HPP_

// Sampling based trajectory planner plugin, following the Frenet path sampling of
// sampling_based_planner / autoware_path_sampler. Path and velocity are sampled separately:
// - path l(s): in Frenet coordinates on the reference_path, sampling a grid of terminal states
//   (arc length L, lateral position l_T) and joining them with a quintic polynomial l(s). The
//   initial slope l'(0) comes from the ego heading and the initial curvature l''(0) from the ego
//   steer angle, so the candidate starts with the heading and the curvature of the ego. The
//   heading of each point comes from the analytic Frenet expression and the curvature from its
//   difference over path_resolution_m
// - velocity s(t): sampling terminal states (duration T, longitudinal speed v_T) and joining them
//   with a quintic polynomial s(t)
// A candidate is one path combined with one velocity profile. The hard constraints are evaluated on
// the projected views of the IR (sl_view_utils) and the cheapest candidate that passes is taken. As
// stated in constraints_compiler.hpp, everything is measured on the reference_path of the current
// cycle; the Spline2D of autoware_frenet_planner is not used.
// Why not sample l(t) directly: starting from standstill both s and l would rise as t^3, which puts
// the initial heading off the ego heading and rejects every candidate in the kinematic check.

#include "../../utils/constraints_compiler.hpp"
#include "../../utils/sl_view_utils.hpp"
#include "../../utils/trajectory_conversion.hpp"
#include "../trajectory_planner_interface.hpp"

#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner::experiment
{

class FrenetSamplingBasedPlanner : public TrajectoryPlannerInterface
{
public:
  std::string get_name() const override { return "frenet_sampling_based_planner"; }

  TrajectoryPlannerResult plan(const TrajectoryPlannerInput & input) override;

private:
  //! Ego state in Frenet coordinates, the initial conditions of the polynomials
  struct InitialState
  {
    double s{0.0};
    double l{0.0};
    double dl_ds{0.0};    //!< [-] tan(ego heading - centerline tangent)
    double d2l_ds2{0.0};  //!< [1/m] curvature from the ego steer angle, less the centerline one
    double v{0.0};        //!< [m/s] longitudinal speed ds/dt
    double a{0.0};        //!< [m/s^2]
    double l_goal{0.0};   //!< [m] lateral position of goal_pose, evaluated at the end of the path
  };

  //! Path sampled in space, every path_resolution_m from s0 to the end of the reference_path
  struct PathCandidate
  {
    std::vector<double> s;
    std::vector<double> l;
    std::vector<double> yaw;    //!< [rad] heading in world coordinates
    std::vector<double> kappa;  //!< [1/m]
    std::string tag;
  };

  //! Longitudinal profile sampled in time (t_k = k*dt)
  struct VelocityProfile
  {
    std::vector<double> t;
    std::vector<double> s;
    std::vector<double> v;
    std::vector<double> a;
    std::string tag;
  };

  //! A trajectory candidate: one path combined with one velocity profile
  struct Candidate
  {
    std::vector<double> s;  //!< [m] s(t_k)
    std::vector<double> l;  //!< [m] l(s(t_k))
    std::vector<OptimizedTrajectoryPoint> points;
    double cost{0.0};
    bool valid{true};
    std::string tag;
  };

  std::optional<Trajectory> plan_one_side(
    const PlannerContext & context, const std::vector<Constraint> & constraints,
    TrajectoryPlannerDebug & debug) const;

  InitialState compute_initial_state(const PlannerContext & context) const;

  //! Samples the quintic l(s) for one terminal state (arc length length, lateral position
  //! l_target)
  PathCandidate sample_path(
    const PlannerContext & context, const InitialState & initial_state, const double length,
    const double l_target) const;

  std::vector<PathCandidate> generate_paths(
    const PlannerContext & context, const InitialState & initial_state) const;

  std::vector<VelocityProfile> generate_velocity_profiles(
    const PlannerContext & context, const InitialState & initial_state,
    const CompiledConstraints & compiled_constraints) const;

  //! Last resort when no candidate is valid: hold the current lateral position and stop at the
  //! hardest deceleration
  VelocityProfile make_stop_profile(
    const InitialState & initial_state, const KinematicLimits & limits) const;

  //! Interpolates l, yaw and the curvature at s(t_k) along the path, in world coordinates
  Candidate combine(
    const PlannerContext & context, const PathCandidate & path,
    const VelocityProfile & profile) const;

  //! Evaluates the hard constraints and accumulates the soft cost, writing valid and cost
  void evaluate(
    const PlannerContext & context, const CompiledConstraints & compiled_constraints,
    const double l_goal, Candidate & candidate) const;

  Trajectory to_trajectory_msg(const PlannerContext & context, const Candidate & candidate) const;

  void append_debug_markers(
    const PlannerContext & context, const std::vector<Candidate> & candidates,
    MarkerArray & debug_markers) const;

  //! The lateral bounds of the projected views, drawn at a constant spacing along the
  //! reference_path as thin lines from the centerline to each boundary along the normal
  MarkerArray make_lateral_bounds_markers(
    const PlannerContext & context, const CompiledConstraints & compiled_constraints) const;
};

}  // namespace autoware::safety_planner::experiment

#endif  // TRAJECTORY_PLANNER__FRENET_SAMPLING_BASED_PLANNER__FRENET_SAMPLING_BASED_PLANNER_HPP_
