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
// the projected views of the IR (compiled_constraints_utils) and the cheapest candidate that passes
// is taken. As stated in constraints_compiler.hpp, everything is measured on the reference_path of
// the current cycle; the Spline2D of autoware_frenet_planner is not used. Why not sample l(t)
// directly: starting from standstill both s and l would rise as t^3, which puts the initial heading
// off the ego heading and rejects every candidate in the kinematic check.

#include "../../utils/boundary_simplifier.hpp"
#include "../../utils/turn_indicator_decider.hpp"
#include "../trajectory_planner_interface.hpp"
#include "compiled_constraints_utils.hpp"
#include "constraints_compiler.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner::experimental
{

class FrenetSamplingBasedPlanner : public TrajectoryPlannerInterface
{
public:
  std::string get_name() const override { return "frenet_sampling_based_planner"; }

  void on_initialize(
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    const Params & params) override;

  TrajectoryPlannerResult plan_trajectories(const TrajectoryPlannerInput & input) override;

private:
  // The turn signal is decided after the fact from the reference_path, the map and the ego state;
  // one decider per output, since each holds its own anti-chatter and latch state
  TurnIndicatorDecider normal_turn_indicator_decider_{TurnSignalParams{}};
  TurnIndicatorDecider cautious_turn_indicator_decider_{TurnSignalParams{}};

  //! Output of the previous cycle, one per side, the reference of the continuity cost. Not cleared
  //! on a route change: the cost is soft, so a previous output that no longer fits the new
  //! reference_path is outvoted by the other terms within a cycle or two
  std::optional<Trajectory> normal_previous_trajectory_;
  std::optional<Trajectory> cautious_previous_trajectory_;

  //! Shared by both sides: the cache keys on the geometry, so the second side pays a hash only
  std::unique_ptr<BoundarySimplifier> soft_boundary_simplifier_;
  std::unique_ptr<BoundarySimplifier> hard_boundary_simplifier_;

  //! Upper bounds of the global ScalarBound constraints, per quantity: the ones
  //! collect_kinematic_limits does not read (LAT_ACCEL, LON_JERK, STEER_ANGLE, STEER_RATE)
  struct GlobalBounds
  {
    double lat_accel{INF};
    double lon_jerk{INF};
    double steer_angle{INF};
    double steer_rate{INF};
  };

  //! What evaluate() looks up per point, tabulated once per call over cells of the arc length:
  //! without it every point scans every scalar bound and every vertex of every boundary polyline,
  //! about a hundred thousand times per cycle
  class ReferenceGrid;

  struct ConstraintTables
  {
    ConstraintTables(
      const PlannerContext & context, const ReferenceGrid & grid,
      const CompiledConstraints & compiled_constraints, double resolution);

    //! Index of the cell holding the arc length s, clamped to the table
    std::size_t cell(double s) const;

    double res{1.0};  //!< [m] cell size
    std::size_t cells{1};
    KinematicLimits limits;
    GlobalBounds bounds;
    //! [m/s] the velocity limit in the cell, the global bound and the speed limit zones together
    std::vector<double> v_max;
    //! Per lateral bound of the IR, in the same order: the boundary l the footprint has to stay
    //! clear of, per cell (see tabulate_lateral_bound). Empty for the hard ones, which are checked
    //! on boundary_profiles
    std::vector<std::vector<double>> lateral_extreme_l;
    std::vector<bool> lateral_is_hard;
    //! Per cell, the hard lateral bounds in the frame at the start of the cell
    std::vector<BoundaryProfile> boundary_profiles;
  };

  //! The reference path resampled at a fixed spacing. Every geometric query of the sampling loops
  //! reads this instead of the spline: one Trajectory::compute() runs about ten spline evaluations
  //! and allocates the lane_ids of the point it returns, and the loops query the geometry about a
  //! hundred thousand times per cycle
  class ReferenceGrid
  {
  public:
    ReferenceGrid(const PathPointTrajectory & path, double resolution);

    double curvature(double s) const;
    double dkappa(double s) const;  //!< [1/m^2] central difference over the grid spacing
    double azimuth(double s) const;
    double z(double s) const;
    //! World position of the point at the lateral offset l from the centerline point at s
    Point2d position(double s, double l) const;

  private:
    double res_{1.0};  //!< the spacing, an exact divisor of the path length
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> z_;
    std::vector<double> yaw_;
    std::vector<double> cos_yaw_;
    std::vector<double> sin_yaw_;
    std::vector<double> curvature_;
    std::vector<double> dkappa_;
  };

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
    std::vector<double> yaw;     //!< [rad] heading in world coordinates
    std::vector<double> kappa;   //!< [1/m]
    std::vector<double> metric;  //!< [-] d sigma / d s, path arc length per reference arc length
    //! Whether the path, followed to its end, stays clear of the hard boundaries and within the
    //! steer limit; the candidates check only the part within the time horizon
    bool feasible_to_end{true};
    //! Cost of the soft boundaries along the whole path, the same for every velocity profile
    double soft_bound_cost{0.0};
    std::string tag;
  };

  //! Longitudinal profile sampled in time (t_k = k * time_step_s)
  struct VelocityProfile
  {
    std::vector<double> s;
    std::vector<double> v;
    std::vector<double> a;
    std::string tag;
  };

  //! Lateral profile l(s) of the previous output, measured on the reference_path of this cycle.
  //! Compared in space and not in time: the ego has moved on by one cycle, so the point of the
  //! previous trajectory at a given time is not the one at the same place any more
  struct PreviousLateral
  {
    PreviousLateral() = default;
    PreviousLateral(const PathPointTrajectory & reference_path, const Trajectory & previous);

    std::vector<double> s;  //!< strictly increasing
    std::vector<double> l;
    //! l at the arc length query_s, held constant beyond both ends; empty when there is no
    //! previous output to compare against
    std::optional<double> at(double query_s) const;
  };

  //! A trajectory candidate: one path combined with one velocity profile. Frenet quantities only,
  //! sampled at t_k = k * time_step_s; the world pose is built in to_trajectory_msg, for the
  //! winner alone
  struct Candidate
  {
    //! The path it was combined from, for the heading of the output. Points into the paths of the
    //! current call, which outlive the candidates
    const PathCandidate * path{nullptr};
    std::vector<double> s;      //!< [m] s(t_k)
    std::vector<double> l;      //!< [m] l(s(t_k))
    std::vector<double> kappa;  //!< [1/m] kappa(s(t_k)), kept at double precision for the checks
    std::vector<double> v;      //!< [m/s] along the path, the profile speed times the metric
    std::vector<double> a;      //!< [m/s^2]
    double cost{0.0};
    bool valid{true};
    std::string tag;
  };

  //! Not const: the boundary simplifier carries a cache
  std::optional<Trajectory> plan_one_side(
    const PlannerContext & context, const ReferenceGrid & grid,
    const std::vector<Constraint> & constraints,
    const std::optional<Trajectory> & previous_trajectory, TrajectoryPlannerDebug & debug);

  InitialState compute_initial_state(
    const PlannerContext & context, const ReferenceGrid & grid) const;

  //! Path of constant curvature, the one the ego is on with its current steer
  PathCandidate hold_steer_path(
    const PlannerContext & context, const ReferenceGrid & grid,
    const InitialState & initial_state) const;

  //! Samples the quintic l(s) for one terminal state (arc length length, lateral position
  //! l_target). With a positive return_length it comes back to the centerline over that arc length
  //! instead of holding l_target
  PathCandidate sample_path(
    const PlannerContext & context, const ReferenceGrid & grid, const InitialState & initial_state,
    const double length, const double l_target, const double return_length = 0.0) const;

  std::vector<PathCandidate> generate_paths(
    const PlannerContext & context, const ReferenceGrid & grid,
    const InitialState & initial_state) const;

  std::vector<VelocityProfile> generate_velocity_profiles(
    const PlannerContext & context, const ReferenceGrid & grid, const ConstraintTables & tables,
    const InitialState & initial_state, const CompiledConstraints & compiled_constraints) const;

  //! Last resort when no candidate is valid: hold the current lateral position and stop at the
  //! hardest deceleration
  VelocityProfile make_stop_profile(
    const InitialState & initial_state, const KinematicLimits & limits) const;

  //! Interpolates l and the curvature at s(t_k) along the path
  Candidate combine(const PathCandidate & path, const VelocityProfile & profile) const;

  //! Evaluates the hard constraints and accumulates the soft cost, writing valid and cost
  void evaluate(
    const PlannerContext & context, const ReferenceGrid & grid,
    const CompiledConstraints & compiled_constraints, const ConstraintTables & tables,
    const double l_goal, const PreviousLateral & previous_lateral, Candidate & candidate) const;

  Trajectory to_trajectory_msg(const PlannerContext & context, const Candidate & candidate) const;

  void append_debug_markers(
    const PlannerContext & context, const ReferenceGrid & grid,
    const std::vector<Candidate> & candidates, MarkerArray & debug_markers) const;

  //! The lateral bounds of the projected views, drawn at a constant spacing along the
  //! reference_path as thin lines from the centerline to each boundary along the normal
  MarkerArray make_lateral_bounds_markers(
    const PlannerContext & context, const ReferenceGrid & grid,
    const CompiledConstraints & compiled_constraints) const;
};

}  // namespace autoware::safety_planner::experimental

#endif  // TRAJECTORY_PLANNER__FRENET_SAMPLING_BASED_PLANNER__FRENET_SAMPLING_BASED_PLANNER_HPP_
