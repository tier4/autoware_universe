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

#include "ssc_qp_trajectory_optimizer.hpp"

#include "bezier.hpp"

#include <autoware/osqp_interface/osqp_interface.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

constexpr double EPS = 1e-9;
//! Lower bound on (1 - k_ref l): the Frenet scale collapses near the center of curvature of the
//! reference, so it is cut conservatively
constexpr double MIN_FRENET_SCALE = 0.1;
//! [m] shortest travel over which the curvature is differenced, which avoids dividing by 0 at a
//! standstill
constexpr double MIN_ARC_FOR_CURVATURE_M = 1e-3;

//! variables per cube: six control points for s and six for l
constexpr int VARS_PER_CUBE = 2 * BEZIER_CONTROL_POINTS;
constexpr int DIM_S = 0;
constexpr int DIM_L = 1;

//! Column of control point i of cube j, in dimension s or l
int column_of(const int cube, const int dim, const int i)
{
  return cube * VARS_PER_CUBE + dim * BEZIER_CONTROL_POINTS + i;
}

//! One row of the linear constraints
struct Row
{
  std::vector<std::pair<int, double>> coefficients;
  double lower{0.0};
  double upper{0.0};

  void add(const int column, const double value) { coefficients.emplace_back(column, value); }
};

//! Adds d^k f/dt^k to a row, as a linear combination of the control points. at_end selects the
//! value at the end of the piece (control point i = m - k) rather than at its start (i = 0). In
//! real time the value is alpha^(1-k) times that, which the caller folds into scale.
void add_boundary_derivative(
  Row & row, const int cube, const int dim, const int k, const bool at_end, const double scale)
{
  const auto d = hodograph_matrix(BEZIER_DEGREE, k);
  const int index = at_end ? BEZIER_DEGREE - k : 0;
  for (int c = 0; c <= BEZIER_DEGREE; ++c) {
    const double value = d[static_cast<std::size_t>(index)][static_cast<std::size_t>(c)];
    if (std::abs(value) > EPS) {
      row.add(column_of(cube, dim, c), scale * value);
    }
  }
}

}  // namespace

// =============================================================================================
// boundary conditions
// =============================================================================================

SscBoundaryState to_frenet_boundary_state(
  const PlannerContext & context, const RoughPlanPoint & point, const double s, const double l)
{
  const auto & path = context.reference_path;
  const double s_clamped = std::clamp(s, 0.0, path.length());
  const double ref_yaw = path.azimuth(s_clamped);
  const double ref_curvature = path.curvature(s_clamped);
  const double scale = std::max(1.0 - ref_curvature * l, MIN_FRENET_SCALE);
  const double delta_yaw = autoware_utils_math::normalize_radian(point.pose.yaw - ref_yaw);

  SscBoundaryState state;
  state.s = s;
  state.l = l;
  state.s_dot = point.v * std::cos(delta_yaw) / scale;
  state.l_dot = point.v * std::sin(delta_yaw);
  // The acceleration uses the small-deviation approximation of a constant heading and drops the
  // centripetal term. SSC carries no heading, so being exact here would not survive the QP
  state.s_ddot = point.a * std::cos(delta_yaw) / scale;
  state.l_ddot = point.a * std::sin(delta_yaw);
  return state;
}

double cube_velocity_upper(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const SemanticCube & cube, const KinematicLimits & limits)
{
  double upper = limits.v_hard;
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (bound.quantity != BoundedQuantity::VELOCITY) {
      continue;
    }
    if (bound.s1 < cube.s0 || bound.s0 > cube.s1) {
      continue;  // outside the s range of the cube
    }
    upper = std::min(upper, bound.max);
  }

  // Take the largest curvature over the s range of the cube, which is the conservative choice
  const auto & path = context.reference_path;
  const double length = path.length();
  double max_curvature = 0.0;
  constexpr int CURVATURE_SAMPLES = 5;
  for (int i = 0; i <= CURVATURE_SAMPLES; ++i) {
    const double ratio = static_cast<double>(i) / CURVATURE_SAMPLES;
    const double s = std::clamp(cube.s0 * (1.0 - ratio) + cube.s1 * ratio, 0.0, length);
    max_curvature = std::max(max_curvature, std::abs(path.curvature(s)));
  }
  if (max_curvature > EPS) {
    upper = std::min(upper, std::sqrt(limits.a_lat_nom / max_curvature));
  }
  return std::max(upper, 0.0);
}

// =============================================================================================
// QP
// =============================================================================================

std::optional<SscQpSolution> solve_ssc_qp(
  const std::vector<SemanticCube> & cubes, const std::vector<double> & velocity_upper,
  const SscBoundaryState & initial, const SscBoundaryState & terminal,
  const KinematicLimits & limits, const SscQpParams & params)
{
  const int n = static_cast<int>(cubes.size());
  if (n < 1 || velocity_upper.size() != cubes.size()) {
    return std::nullopt;
  }
  const double alpha = cubes.front().t1 - cubes.front().t0;
  if (!(alpha > EPS)) {
    return std::nullopt;
  }
  const int num_vars = n * VARS_PER_CUBE;

  // ---- objective: the squared jerk integral ----
  // (1/alpha^3) p' Q p per piece and dimension. OSQP minimizes 0.5 x'Px + q'x, hence P = 2 w Q.
  // The objective staying the same whatever semantic elements are present is the point of SSC:
  // everything that is added shows up only in the bounds of the cubes, i.e. in the inequalities
  // below
  const auto jerk_q = jerk_hessian(BEZIER_DEGREE);
  Eigen::MatrixXd p_matrix = Eigen::MatrixXd::Zero(num_vars, num_vars);
  const double jerk_weight[2] = {params.weight_jerk_s, params.weight_jerk_l};
  for (int j = 0; j < n; ++j) {
    for (int dim = 0; dim < 2; ++dim) {
      const double coefficient = 2.0 * jerk_weight[dim] / (alpha * alpha * alpha);
      for (int r = 0; r <= BEZIER_DEGREE; ++r) {
        for (int c = 0; c <= BEZIER_DEGREE; ++c) {
          p_matrix(column_of(j, dim, r), column_of(j, dim, c)) +=
            coefficient * jerk_q[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)];
        }
      }
    }
  }
  // Regularization against the null space of the jerk Hessian, the polynomials of degree two and
  // below
  for (int i = 0; i < num_vars; ++i) {
    p_matrix(i, i) += params.regularization;
  }

  std::vector<Row> rows;

  // ---- (1) the initial and terminal state, as equalities on k = 0, 1, 2 ----
  const auto add_boundary_rows =
    [&](const int cube, const bool at_end, const SscBoundaryState & state) {
      const double value[2][3] = {
        {state.s, state.s_dot, state.s_ddot}, {state.l, state.l_dot, state.l_ddot}};
      for (int dim = 0; dim < 2; ++dim) {
        for (int k = 0; k <= 2; ++k) {
          Row row;
          add_boundary_derivative(row, cube, dim, k, at_end, std::pow(alpha, 1 - k));
          row.lower = value[dim][k];
          row.upper = value[dim][k];
          rows.push_back(std::move(row));
        }
      }
    };
  add_boundary_rows(0, false, initial);
  add_boundary_rows(n - 1, true, terminal);

  // ---- (2) continuity between the pieces, for k = 0..3 ----
  // alpha is the same for every cube, so the scales cancel; they are written out anyway, so that
  // this keeps holding if alpha ever varies per piece
  for (int j = 0; j + 1 < n; ++j) {
    for (int dim = 0; dim < 2; ++dim) {
      for (int k = 0; k <= 3; ++k) {
        Row row;
        add_boundary_derivative(row, j, dim, k, true, std::pow(alpha, 1 - k));
        add_boundary_derivative(row, j + 1, dim, k, false, -std::pow(alpha, 1 - k));
        rows.push_back(std::move(row));  // lower = upper = 0
      }
    }
  }

  // ---- (3) free space, (P1) at k = 0: alpha p_i inside the box of the cube ----
  for (int j = 0; j < n; ++j) {
    const double box[2][2] = {{cubes[j].s0, cubes[j].s1}, {cubes[j].l0, cubes[j].l1}};
    for (int dim = 0; dim < 2; ++dim) {
      if (box[dim][0] > box[dim][1]) {
        return std::nullopt;  // a collapsed cube means the carving already failed
      }
      for (int i = 0; i <= BEZIER_DEGREE; ++i) {
        Row row;
        row.add(column_of(j, dim, i), alpha);
        row.lower = box[dim][0];
        row.upper = box[dim][1];
        rows.push_back(std::move(row));
      }
    }
  }

  // ---- (4) dynamics, (P1) at k = 1 and 2: the control points of the derivatives in a box ----
  // This is sufficient for the **whole** speed and acceleration profile to stay in that box
  for (int j = 0; j < n; ++j) {
    const double bound[2][2][2] = {
      // {[lower, upper] at k = 1, [lower, upper] at k = 2}
      {{0.0, velocity_upper[static_cast<std::size_t>(j)]},  // s' >= 0, i.e. no reversing
       {limits.a_hard_min, limits.a_hard_max}},
      {{-params.lateral_rate_max_mps, params.lateral_rate_max_mps},
       {-params.lateral_accel_max_mps2, params.lateral_accel_max_mps2}},
    };
    for (int k = 1; k <= 2; ++k) {
      const auto d = hodograph_matrix(BEZIER_DEGREE, k);
      const double scale = std::pow(alpha, 1 - k);
      for (int dim = 0; dim < 2; ++dim) {
        for (int i = 0; i + k <= BEZIER_DEGREE; ++i) {
          Row row;
          for (int c = 0; c <= BEZIER_DEGREE; ++c) {
            const double value = d[static_cast<std::size_t>(i)][static_cast<std::size_t>(c)];
            if (std::abs(value) > EPS) {
              row.add(column_of(j, dim, c), scale * value);
            }
          }
          row.lower = bound[dim][k - 1][0];
          row.upper = bound[dim][k - 1][1];
          rows.push_back(std::move(row));
        }
      }
    }
  }

  // ---- hand it to OSQP ----
  const int num_rows = static_cast<int>(rows.size());
  Eigen::MatrixXd a_matrix = Eigen::MatrixXd::Zero(num_rows, num_vars);
  std::vector<double> lower(static_cast<std::size_t>(num_rows));
  std::vector<double> upper(static_cast<std::size_t>(num_rows));
  for (int r = 0; r < num_rows; ++r) {
    const auto & row = rows[static_cast<std::size_t>(r)];
    for (const auto & [column, value] : row.coefficients) {
      a_matrix(r, column) += value;
    }
    lower[static_cast<std::size_t>(r)] = row.lower;
    upper[static_cast<std::size_t>(r)] = row.upper;
  }

  const std::vector<double> q_vector(static_cast<std::size_t>(num_vars), 0.0);
  osqp_interface::OSQPInterface solver(params.osqp_eps_abs, true);
  const auto result = solver.optimize(p_matrix, a_matrix, q_vector, lower, upper);
  // Anything but OSQP_SOLVED is discarded, the inaccurate statuses included, so that a solution
  // that solved but violates the constraints never reaches the consumers
  if (result.solution_status != OSQP_SOLVED) {
    return std::nullopt;
  }
  if (static_cast<int>(result.primal_solution.size()) != num_vars) {
    return std::nullopt;
  }
  for (const double value : result.primal_solution) {
    if (!std::isfinite(value)) {
      return std::nullopt;
    }
  }

  SscQpSolution solution;
  solution.control_points = result.primal_solution;
  solution.alpha = alpha;
  return solution;
}

// =============================================================================================
// sampling the solution, from Frenet back to world coordinates
// =============================================================================================

OptimizedTrajectory sample_ssc_solution(
  const PlannerContext & context, const std::vector<SemanticCube> & cubes,
  const SscQpSolution & solution, const std::vector<double> & sample_times)
{
  OptimizedTrajectory trajectory;
  const int n = static_cast<int>(cubes.size());
  const double alpha = solution.alpha;
  if (n < 1 || !(alpha > EPS) || sample_times.empty()) {
    return trajectory;
  }

  const auto & path = context.reference_path;
  const double length = path.length();
  const double t_origin = cubes.front().t0;

  trajectory.points.reserve(sample_times.size());
  for (const double t : sample_times) {
    // Which piece the time belongs to. The pieces meet exactly at their common time and the
    // continuity equalities make the values agree, so either choice gives the same result
    const int j = std::clamp(static_cast<int>(std::floor((t - t_origin) / alpha)), 0, n - 1);
    const double u = std::clamp((t - cubes[static_cast<std::size_t>(j)].t0) / alpha, 0.0, 1.0);

    std::vector<double> p_s(BEZIER_CONTROL_POINTS);
    std::vector<double> p_l(BEZIER_CONTROL_POINTS);
    for (int i = 0; i <= BEZIER_DEGREE; ++i) {
      p_s[static_cast<std::size_t>(i)] =
        solution.control_points[static_cast<std::size_t>(column_of(j, DIM_S, i))];
      p_l[static_cast<std::size_t>(i)] =
        solution.control_points[static_cast<std::size_t>(column_of(j, DIM_L, i))];
    }

    const double s = alpha * evaluate_derivative(p_s, 0, u);
    const double s_dot = evaluate_derivative(p_s, 1, u);
    const double l = alpha * evaluate_derivative(p_l, 0, u);
    const double l_dot = evaluate_derivative(p_l, 1, u);

    const double s_clamped = std::clamp(s, 0.0, length);
    const double ref_curvature = path.curvature(s_clamped);
    const double frenet_scale = std::max(1.0 - ref_curvature * l, MIN_FRENET_SCALE);

    OptimizedTrajectoryPoint point;
    point.t = t;
    point.pose = to_world_pose(path, s_clamped, l);
    // The heading comes from the Frenet slope. At a standstill atan2(0, 0) is 0 and it falls back
    // to the centerline tangent
    point.pose.yaw = autoware_utils_math::normalize_radian(
      point.pose.yaw + std::atan2(l_dot, frenet_scale * s_dot));
    // Speed in world coordinates; s' >= 0 was imposed, so it always points forward
    point.v = std::hypot(frenet_scale * s_dot, l_dot);
    trajectory.points.push_back(point);
  }

  // The curvature, the acceleration and the inputs are differenced on the time grid. Neither the
  // heading nor the curvature is a variable of SSC, so this is the only place the output contract
  // of OptimizedTrajectoryPoint can be met
  auto & points = trajectory.points;
  const std::size_t count = points.size();
  for (std::size_t k = 0; k + 1 < count; ++k) {
    const double dt = points[k + 1].t - points[k].t;
    if (!(dt > EPS)) {
      continue;
    }
    const double d_yaw =
      autoware_utils_math::normalize_radian(points[k + 1].pose.yaw - points[k].pose.yaw);
    const double arc = std::max(
      std::hypot(
        points[k + 1].pose.position.x() - points[k].pose.position.x(),
        points[k + 1].pose.position.y() - points[k].pose.position.y()),
      MIN_ARC_FOR_CURVATURE_M);
    points[k].kappa = d_yaw / arc;
    points[k].a = (points[k + 1].v - points[k].v) / dt;
  }
  if (count >= 2) {
    points[count - 1].kappa = points[count - 2].kappa;
    points[count - 1].a = points[count - 2].a;
  }
  for (std::size_t k = 0; k + 1 < count; ++k) {
    const double dt = points[k + 1].t - points[k].t;
    if (!(dt > EPS)) {
      continue;
    }
    points[k].w = (points[k + 1].kappa - points[k].kappa) / dt;
    points[k].j = (points[k + 1].a - points[k].a) / dt;
  }
  // The last point has no input, so w and j stay 0

  return trajectory;
}

// =============================================================================================
// the plugin
// =============================================================================================

SscQpParams SscQpTrajectoryOptimizer::read_params() const
{
  const auto & p = params_.trajectory_optimizer.ssc_qp;
  SscQpParams params;
  params.corridor.cube_duration_s = p.cube_duration_s;
  params.corridor.margin_m = p.margin_m;
  params.corridor.inflation_step_m = p.inflation_step_m;
  params.corridor.max_lateral_inflation_m = p.max_lateral_inflation_m;
  params.corridor.max_longitudinal_inflation_m = p.max_longitudinal_inflation_m;
  params.lateral_rate_max_mps = p.lateral_rate_max_mps;
  params.lateral_accel_max_mps2 = p.lateral_accel_max_mps2;
  params.weight_jerk_s = p.weight_jerk_s;
  params.weight_jerk_l = p.weight_jerk_l;
  params.regularization = p.regularization;
  params.osqp_eps_abs = p.osqp_eps_abs;
  return params;
}

TrajectoryOptimizerResult SscQpTrajectoryOptimizer::optimize(const TrajectoryOptimizerInput & input)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  TrajectoryOptimizerResult result;
  const auto params = read_params();
  const auto limits = collect_kinematic_limits(input.compiled_constraints);

  // 1. the seed is (s(t), l(t)) of the rough plan, which has settled the homotopy
  const auto seed = make_corridor_seed(input.context, input.rough_plan);
  if (seed.size() < 2) {
    result.status = TrajectoryOptimizerStatus::INFEASIBLE;
    result.debug.message = "rough plan is too short to seed a corridor";
    return result;
  }

  // 2. carve the (s, l, t) cubes around the seed
  const auto cubes =
    generate_semantic_corridor(input.context, input.compiled_constraints, seed, params.corridor);
  if (cubes.empty()) {
    result.status = TrajectoryOptimizerStatus::INFEASIBLE;
    result.debug.message = "no semantic cube was generated";
    return result;
  }

  // 3. the longitudinal speed bound of each cube, from the speed limits and the curvature
  std::vector<double> velocity_upper;
  velocity_upper.reserve(cubes.size());
  for (const auto & cube : cubes) {
    velocity_upper.push_back(
      cube_velocity_upper(input.context, input.compiled_constraints, cube, limits));
  }

  // 4. the initial and terminal state, the ends of the rough plan in Frenet coordinates
  const auto initial = to_frenet_boundary_state(
    input.context, input.rough_plan.points.front(), seed.front().s, seed.front().l);
  const auto terminal = to_frenet_boundary_state(
    input.context, input.rough_plan.points.back(), seed.back().s, seed.back().l);

  // 5. the convex QP over the piecewise Bezier
  const auto solution = solve_ssc_qp(cubes, velocity_upper, initial, terminal, limits, params);
  const double z_base = input.context.odometry.pose.pose.position.z;
  if (!solution) {
    result.status = TrajectoryOptimizerStatus::INFEASIBLE;
    result.debug.message = "QP is infeasible";
    result.debug.debug_markers = make_corridor_markers(input.context, cubes, z_base);
    return result;
  }

  // 6. sample it on the time grid of the rough plan and convert back to world coordinates
  std::vector<double> sample_times;
  sample_times.reserve(seed.size());
  for (const auto & point : seed) {
    sample_times.push_back(point.t);
  }
  result.trajectory = sample_ssc_solution(input.context, cubes, *solution, sample_times);
  if (result.trajectory.points.empty()) {
    result.status = TrajectoryOptimizerStatus::SOLVER_ERROR;
    result.debug.message = "failed to sample the QP solution";
    result.debug.debug_markers = make_corridor_markers(input.context, cubes, z_base);
    return result;
  }

  result.status = TrajectoryOptimizerStatus::SUCCESS;
  result.debug.which_level = 1;
  result.debug.debug_markers = make_corridor_markers(input.context, cubes, z_base);
  return result;
}

}  // namespace autoware::safety_planner

PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::SscQpTrajectoryOptimizer,
  autoware::safety_planner::TrajectoryOptimizerInterface)
