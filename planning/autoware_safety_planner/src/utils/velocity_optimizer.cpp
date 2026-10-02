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

#include "velocity_optimizer.hpp"

#include <Eigen/Sparse>
#include <autoware/osqp_interface/osqp_interface.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

constexpr double STOP_VELOCITY_MPS = 1.0e-3;

//! JerkFilteredSmoother::forwardJerkFilter on a uniform grid: from (v, a), accelerate at the jerk
//! j_max up to a_max, clipped by v_max. Only the velocity is kept, the QP takes nothing else
std::vector<double> forward_jerk_filter(
  double v, double a, const double a_max, const double j_max, const std::vector<double> & v_max,
  const double ds)
{
  // The time a start from rest at the jerk j_max takes over ds, which bounds the step at low speed
  const double max_dt = std::cbrt(6.0 * ds / j_max);
  std::vector<double> filtered(v_max.size());
  for (std::size_t i = 0; i < v_max.size(); ++i) {
    if (i > 0) {
      const double dt = std::min(ds / std::max(v, 1.0e-6), max_dt);
      if (a + j_max * dt >= a_max) {
        const double j = std::min((a_max - a) / dt, j_max);
        v += a * dt + 0.5 * j * dt * dt;
        a = a_max;
      } else {
        v += a * dt + 0.5 * j_max * dt * dt;
        a += j_max * dt;
      }
    }
    if (v > v_max[i]) {
      v = v_max[i];
      a = 0.0;
    }
    if (v < 0.0) {
      v = 0.0;
      a = 0.0;
    }
    filtered[i] = v;
  }
  return filtered;
}

//! JerkFilteredSmoother::mergeFilteredTrajectory: the smaller of the two, except that an ego
//! faster than the backward profile first brakes into it at the jerk j_min. Unlike there, the
//! braking is released at -j_min once the excess left is what the release takes: braking at j_min
//! all the way meets the backward profile with a kink, and the QP, bounded by this profile, can
//! only stay under it by braking harder than a_min before it. Nor is it capped by the forward
//! profile, which cuts an ego over v_max down to v_max on its first point. The steps follow the
//! discretization of the QP (b' = 2a over each interval), so that the QP can ride on the profile
std::vector<double> merge_filtered(
  const double v0, const double a0, const double a_min, const double j_min,
  const std::vector<double> & forward, const std::vector<double> & backward, const double ds)
{
  const std::size_t n = forward.size();
  std::vector<double> merged(n);
  std::size_t i = 0;
  if (backward.front() < v0) {
    const double max_dt = std::cbrt(6.0 * ds / std::abs(j_min));
    double v = v0;
    double a = a0;
    while (i + 1 < n && backward[i] < v) {
      merged[i] = v;
      v = std::sqrt(std::max(v * v + 2.0 * a * ds, 0.0));
      const double dt = std::min(ds / std::max(v, 1.0e-6), max_dt);
      const double release_drop = a * a / (2.0 * std::abs(j_min));
      const double j = a < 0.0 && v - release_drop <= backward[i + 1]
                         ? std::min(-a / dt, std::abs(j_min))
                         : std::max((a_min - a) / dt, j_min);
      a += j * dt;
      ++i;
    }
  }
  for (; i < n; ++i) {
    merged[i] = std::min(forward[i], backward[i]);
  }
  return merged;
}

osqp_interface::CSC_Matrix to_csc(const Eigen::SparseMatrix<double> & matrix)
{
  osqp_interface::CSC_Matrix csc;
  const auto nnz = static_cast<std::size_t>(matrix.nonZeros());
  csc.m_vals.assign(matrix.valuePtr(), matrix.valuePtr() + nnz);
  csc.m_row_idxs.assign(matrix.innerIndexPtr(), matrix.innerIndexPtr() + nnz);
  csc.m_col_idxs.assign(matrix.outerIndexPtr(), matrix.outerIndexPtr() + matrix.outerSize() + 1);
  return csc;
}

}  // namespace

std::optional<VelocityOptimizerResult> optimize_velocity(
  const std::vector<double> & v_max, const double ds, const double v0,
  const std::optional<double> a0, const VelocityOptimizerParams & params)
{
  // The filters and the linearization need a start; with a0 free it is the one at rest
  const double a_start = a0.value_or(0.0);
  // The points after the first stop stay stopped and are left out of the QP
  const auto stop_it =
    std::find_if(v_max.begin(), v_max.end(), [](const double v) { return v < STOP_VELOCITY_MPS; });
  const auto n = static_cast<std::size_t>(
    stop_it == v_max.end() ? v_max.size() : std::distance(v_max.begin(), stop_it) + 1);
  const std::vector<double> limit(v_max.begin(), v_max.begin() + static_cast<std::ptrdiff_t>(n));

  // The filtered profile is both the soft bound on v and the velocity the pseudo jerk is
  // linearized around, as in JerkFilteredSmoother
  const auto forward =
    forward_jerk_filter(v0, std::max(a_start, params.a_min), params.a_max, params.j_max, limit, ds);
  std::vector<double> reversed(limit.rbegin(), limit.rend());
  auto backward = forward_jerk_filter(
    reversed.front(), 0.0, std::abs(params.a_min), std::abs(params.j_min), reversed, ds);
  std::reverse(backward.begin(), backward.end());
  auto ref = merge_filtered(v0, a_start, params.a_min, params.j_min, forward, backward, ds);
  // The second point is where (v0, a0) leads whatever the plan; a bound below it is always
  // violated, which slows OSQP down to thousands of iterations
  if (a0 && n > 1) {
    ref[1] = std::max(ref[1], std::sqrt(std::max(v0 * v0 + 2.0 * *a0 * ds, 0.0)));
  }

  // x = [b (v^2), a, delta (over v^2), sigma (over a), gamma (over j)], n each
  const auto ni = static_cast<int>(n);
  const int ib = 0;
  const int ia = ni;
  const int idelta = 2 * ni;
  const int isigma = 3 * ni;
  const int igamma = 4 * ni;
  const int num_variables = 5 * ni;
  const int num_rows = 4 * ni;

  // Only the upper triangle of P, as OSQP takes it
  std::vector<Eigen::Triplet<double>> p_entries;
  std::vector<double> q(static_cast<std::size_t>(num_variables), 0.0);
  for (int i = 0; i + 1 < ni; ++i) {
    const double ref_v = 0.5 * (ref[i] + ref[i + 1]);
    const double w = params.jerk_weight * (ref_v / ds) * (ref_v / ds) * ds;
    p_entries.emplace_back(ia + i, ia + i, w);
    p_entries.emplace_back(ia + i + 1, ia + i + 1, w);
    p_entries.emplace_back(ia + i, ia + i + 1, -w);
  }
  for (int i = 0; i < ni; ++i) {
    // Maximizes v^2 / v_ref^2 over the arc length; left out where v_ref is too small to divide by
    if (ref[i] > 0.01) {
      q[static_cast<std::size_t>(ib + i)] = -(i + 1 < ni ? ds : 1.0) / (ref[i] * ref[i]);
    }
    p_entries.emplace_back(idelta + i, idelta + i, params.over_v_weight);
    p_entries.emplace_back(isigma + i, isigma + i, params.over_a_weight);
    p_entries.emplace_back(igamma + i, igamma + i, params.over_j_weight);
  }
  Eigen::SparseMatrix<double> p_matrix(num_variables, num_variables);
  p_matrix.setFromTriplets(p_entries.begin(), p_entries.end());

  std::vector<Eigen::Triplet<double>> a_entries;
  std::vector<double> lower(static_cast<std::size_t>(num_rows));
  std::vector<double> upper(static_cast<std::size_t>(num_rows));
  int row = 0;
  const auto bound = [&](const double lo, const double hi) {
    lower[static_cast<std::size_t>(row)] = lo;
    upper[static_cast<std::size_t>(row)] = hi;
    ++row;
  };
  // b may go negative through delta: b >= 0 alone is infeasible for v = 0 with a < 0, and a
  // negative b is read as a stop
  for (int i = 0; i < ni; ++i) {
    a_entries.emplace_back(row, ib + i, 1.0);
    a_entries.emplace_back(row, idelta + i, -1.0);
    bound(0.0, ref[i] * ref[i]);
  }
  for (int i = 0; i < ni; ++i) {
    a_entries.emplace_back(row, ia + i, 1.0);
    a_entries.emplace_back(row, isigma + i, -1.0);
    if (ref[i] < STOP_VELOCITY_MPS) {
      bound(0.0, 0.0);
    } else {
      bound(params.a_min, params.a_max);
    }
  }
  // j = da/ds * v, with v frozen at the reference
  for (int i = 0; i + 1 < ni; ++i) {
    const double ref_v = 0.5 * (ref[i] + ref[i + 1]);
    a_entries.emplace_back(row, ia + i, -ref_v);
    a_entries.emplace_back(row, ia + i + 1, ref_v);
    a_entries.emplace_back(row, igamma + i, -ds);
    bound(params.j_min * ds, params.j_max * ds);
  }
  // db/ds = 2a
  for (int i = 0; i + 1 < ni; ++i) {
    a_entries.emplace_back(row, ib + i, -1.0);
    a_entries.emplace_back(row, ib + i + 1, 1.0);
    a_entries.emplace_back(row, ia + i, -2.0 * ds);
    bound(0.0, 0.0);
  }
  a_entries.emplace_back(row, ib, 1.0);
  bound(v0 * v0, v0 * v0);
  // Hard when free: a slack on the first acceleration would go straight to the controller
  a_entries.emplace_back(row, ia, 1.0);
  if (a0) {
    bound(*a0, *a0);
  } else {
    bound(params.a_min, params.a_max);
  }
  Eigen::SparseMatrix<double> a_matrix(num_rows, num_variables);
  a_matrix.setFromTriplets(a_entries.begin(), a_entries.end());

  // eps_rel 1e-3 (OSQP_interface sets 1e-4) is 0.1 on b ~ 100, 0.005 m/s; at 1e-4 a start at the
  // limit takes a few thousand iterations
  constexpr double OSQP_EPS_ABS = 1.0e-4;
  constexpr double OSQP_EPS_REL = 1.0e-3;
  osqp_interface::OSQPInterface solver(
    to_csc(p_matrix), to_csc(a_matrix), q, lower, upper, OSQP_EPS_ABS);
  solver.updateEpsRel(OSQP_EPS_REL);
  const auto result = solver.optimize();
  if (
    result.solution_status != OSQP_SOLVED ||
    result.primal_solution.size() != static_cast<std::size_t>(num_variables)) {
    return std::nullopt;
  }

  VelocityOptimizerResult output;
  output.v.assign(v_max.size(), 0.0);
  output.a.assign(v_max.size(), 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    const double b = result.primal_solution[static_cast<std::size_t>(ib) + i];
    const double a = result.primal_solution[static_cast<std::size_t>(ia) + i];
    if (!std::isfinite(b) || !std::isfinite(a)) {
      return std::nullopt;
    }
    output.v[i] = std::sqrt(std::max(b, 0.0));
    output.a[i] = a;
  }
  return output;
}

}  // namespace autoware::safety_planner
