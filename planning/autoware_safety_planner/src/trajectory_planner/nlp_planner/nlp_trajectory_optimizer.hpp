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

#ifndef AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__NLP_TRAJECTORY_OPTIMIZER_HPP_
#define AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__NLP_TRAJECTORY_OPTIMIZER_HPP_

#include "ssc_corridor.hpp"
#include "trajectory_optimizer_interface.hpp"

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner
{

// Trajectory optimization as a time-parameterized non-convex NLP.
//
//   state x = [px, py, theta, k, v, a]     input u = [w, j]   (w = dk/dt, j = da/dt)
//   px' = v cos(theta), py' = v sin(theta), theta' = v k, k' = w, v' = a, a' = j
//   (RK4, one step per stage)
//
// Carrying the curvature in the state lets the steer angle |atan(L k)| and the steer rate
// L w / (1 + (L k)^2) be written as hard boxes, which the SSC formulation structurally cannot. The
// price is non-convexity, covered by a three-level fallback that drops the comfort rows and then
// the safety rows, and by an independent verification.
//
// The solver is **acados** (SQP with PARTIAL_CONDENSING_HPIPM). The structure of the problem is
// fixed once at code generation time (generators/nlp_time_ocp.py); a cycle only injects the
// parameters, the bounds and the reference and solves. Dropping a level opens the bounds to
// +-1e6 rather than removing rows, so the solver is never rebuilt.
//
// The two lateral planes of the corridor are the **semantic corridor of SSC**, reused: nothing else
// carves half spaces per stage, and a cube (s, l, t) already certifies the free space of its time
// span, so a stage point can simply be confined to it. There is no corridor carving of its own,
// which would put the non-convex search, i.e. the choice of the homotopy, in two places. Only the
// forward cut ignores the cubes and comes straight from the occupancies and stop lines that are in
// effect ahead at the time of the stage (make_stage_planes).

//! The levels of the fallback
enum class SolveLevel : std::uint8_t {
  LEVEL_1,  //!< every safety and comfort row, aiming at the nominal trajectory
  LEVEL_2,  //!< the comfort rows disabled on every stage, still aiming at the nominal trajectory
  LEVEL_3,  //!< the safety rows disabled as well, aiming at a stop from the certificate
};

//! Parameters of the NLP (ROS namespace `trajectory_optimizer.nlp.*`)
struct NlpParams
{
  //! The corridor carving. The implementation is the one of SSC, but the parameters live in
  //! their own namespace so that tuning one formulation does not move the other
  SscCorridorParams corridor;

  //! [1/(m s)] hard bound on |w|. No plugin emits a rate of curvature, so it lives here; the
  //! bound on the curvature itself is derived from the steer angle in the IR
  double curvature_rate_max{0.6};
  //! [m] within this distance the end of the tracking reference is replaced by the goal itself;
  //! 0 disables the replacement
  double goal_capture_distance_m{5.0};

  // cost weights
  double weight_pos{1.0};
  double weight_yaw{1.0};
  double weight_v{0.5};
  double weight_kappa{200.0};
  double weight_w{100.0};
  double weight_a{0.1};
  double weight_j{0.1};
  double weight_terminal_v{10.0};
  double weight_terminal_a{1.0};

  //! Quadratic penalty on the slacks. An L1 penalty is not used; it wrecked the convergence
  double slack_safety{1.0e6};
  double slack_comfort{1.0e4};

  // iteration budget per level
  int max_iter_level1{20};
  int max_iter_level2{8};
  int max_iter_level3{6};
};

//! The kinematic limits read from the IR, i.e. KinematicLimits plus the jerk, the curvature and
//! the steer rate. Nothing that reads the projected views needs those, so they do not live in
//! sl_view_utils.
struct NlpLimits
{
  KinematicLimits base;          //!< v, a and a_lat, from the global ScalarBound constraints
  double j_hard{5.0};            //!< [m/s^3] hard bound on |j|
  double j_nom{1.6};             //!< [m/s^3] comfort bound on |j|
  double kappa_max{0.156};       //!< [1/m] bound on |k|, derived from the steer angle
  double steer_rate_hard{3.0};   //!< [rad/s] hard bound on the steer rate
  double steer_rate_nom{0.995};  //!< [rad/s] comfort bound on the steer rate
  double a_lat_hard{6.0};        //!< [m/s^2] hard bound on v^2|k|, used by the verification only
};

class NlpTrajectoryOptimizer : public TrajectoryOptimizerInterface
{
public:
  NlpTrajectoryOptimizer();
  ~NlpTrajectoryOptimizer() override;

  std::string get_name() const override { return "nlp"; }

  TrajectoryOptimizerResult optimize(const TrajectoryOptimizerInput & input) override;

private:
  //! Reads the ROS parameters; on_initialize must have run
  NlpParams read_params() const;

  //! A thin wrapper around the generated acados solver, kept opaque so that the generated headers
  //! stay out of this header. The solver lives across cycles, its structure built once
  class Solver;
  std::unique_ptr<Solver> solver_;
};

// ---------------------------------------------------------------------------------------------
// free functions, so that they can be tested outside the plugin
// ---------------------------------------------------------------------------------------------

//! The half space n.p <= d, n being a unit vector in the planning frame
struct HalfPlane
{
  double nx{1.0};
  double ny{0.0};
  double d{0.0};
};

//! The corridor of one stage: the two lateral planes from the cube, then the forward cut from the
//! occupancies and stop lines. There is no cut behind: v >= 0 keeps the ego from reversing and
//! there is nothing behind to protect, while the s0 of the first cube is clipped at the start of
//! the path, so a rear cut would structurally push the rear of the footprint outside and let the
//! slack drive the solution forward.
inline constexpr std::size_t NUM_PLANES = 3;
using StagePlanes = std::array<HalfPlane, NUM_PLANES>;

//! Resamples the rough plan onto N+1 points. It is the identity when the counts already match; if
//! the grid of the rough planner differs from the number of stages the solver was generated with,
//! both the points and s are interpolated linearly in t.
RoughPlan resample_rough_plan(const RoughPlan & plan, std::size_t num_points);

//! Collects the limits the NLP needs from the global ScalarBound constraints. The bound on the
//! steer angle comes from the vehicle dimensions, and the wheel base is the fallback when the IR
//! carries none.
NlpLimits collect_nlp_limits(
  const CompiledConstraints & compiled_constraints, const VehicleInfo & vehicle_info);

//! The discrete map F(x, u): explicit RK4, one step per stage. The dynamics of the NLP, the
//! forward simulation of the certificate and the residual of the verification all share it.
OptimizedTrajectoryPoint integrate_rk4(
  const OptimizedTrajectoryPoint & state, double w, double j, double dt);

//! The last-resort stop trajectory, simulated forward at the hardest comfortable deceleration. It
//! satisfies the dynamics exactly and the vehicle boxes on every stage, and so witnesses that a
//! feasible point exists.
OptimizedTrajectory make_certificate(
  const OptimizedTrajectoryPoint & initial, const NlpLimits & limits, std::size_t num_points,
  double dt);

//! Turns the measured ego state into an initial state inside the vehicle boxes. The lower bound
//! -sqrt(2 j_hard v0) on a0 is what keeps v >= 0 while the deceleration is released at j_hard.
OptimizedTrajectoryPoint condition_initial_state(
  const OptimizedTrajectoryPoint & raw, const NlpLimits & limits);

//! Turns the cubes into the half spaces of each stage. Stage k is anchored at the arc length s_k
//! of the seed, and the tangent and normal of the centerline there approximate the two lateral
//! faces of its cube as straight lines in world coordinates. The forward cut is placed, margin
//! included, in front of the nearest occupancy or stop line that is in effect ahead within the
//! lateral band at t_k, or max_longitudinal_inflation_m ahead when there is none.
std::vector<StagePlanes> make_stage_planes(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const std::vector<SemanticCube> & cubes, const std::vector<CorridorSeedPoint> & seed,
  const SscCorridorParams & corridor);

//! Result of the verification. The status of the solver decides nothing here: solved is not
//! satisfied.
struct NlpVerification
{
  bool dynamics_ok{false};  //!< the residual of the dynamics is within tolerance
  bool vehicle_ok{false};   //!< the vehicle boxes and the hard steer rate hold
  bool safety_ok{false};    //!< the corridor geometry and the speed limits hold
  bool comfort_ok{false};   //!< the comfort bounds hold
  double max_dynamics_residual{0.0};
  std::string message;  //!< which row failed first, for the report

  //! What the given level requires to count as a success
  bool passes(SolveLevel level) const;
};

//! An independent verification, measured again in plain C++; the geometry goes through the
//! projected views
NlpVerification verify_trajectory(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const OptimizedTrajectory & trajectory, const NlpLimits & limits);

}  // namespace autoware::safety_planner

#endif  // AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__NLP_TRAJECTORY_OPTIMIZER_HPP_
