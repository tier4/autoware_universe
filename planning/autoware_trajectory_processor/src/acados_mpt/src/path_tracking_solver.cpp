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

#include "temporal_mpt/path_tracking_solver.hpp"

#include "autoware/trajectory_processor/acados_interface.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace temporal_mpt
{
namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
}  // namespace

struct PathTrackingSolver::Impl
{
  AcadosInterface solver;
  double lf{1.0};
  double lr{1.0};
  double tau_a{0.15};
  double tau_d{0.08};
  double max_steer_rate{3.0};

  bool warm_start_enabled{true};
  bool have_prev_solution{false};
  std::array<std::array<double, NX>, N + 1> prev_x_world{};
  std::array<std::array<double, NU>, N> prev_u{};

  bool have_external_u{false};
  std::array<std::array<double, NU>, N> external_u{};
};

PathTrackingSolver::PathTrackingSolver() : impl_(std::make_unique<Impl>())
{
}

PathTrackingSolver::~PathTrackingSolver() = default;

PathTrackingSolver::PathTrackingSolver(PathTrackingSolver &&) noexcept = default;

PathTrackingSolver & PathTrackingSolver::operator=(PathTrackingSolver &&) noexcept = default;

void PathTrackingSolver::setModelParameters(
  const double lf, const double lr, const double tau_a, const double tau_d,
  const double max_steer_rate_rad_s)
{
  impl_->lf = lf;
  impl_->lr = lr;
  impl_->tau_a = std::max(1.0e-4, tau_a);
  impl_->tau_d = std::max(1.0e-4, tau_d);
  impl_->max_steer_rate = std::max(1.0e-6, max_steer_rate_rad_s);
}

void PathTrackingSolver::setPreviousSolutionWarmStartEnabled(const bool enabled)
{
  impl_->warm_start_enabled = enabled;
  if (!enabled) {
    resetWarmStart();
  }
}

void PathTrackingSolver::resetWarmStart()
{
  impl_->have_prev_solution = false;
  impl_->have_external_u = false;
}

void PathTrackingSolver::setWarmStartControls(
  const std::vector<double> & accel_cmd, const std::vector<double> & steer_cmd)
{
  const std::size_t n = std::min({accel_cmd.size(), steer_cmd.size(), static_cast<std::size_t>(N)});
  if (n == 0U) {
    impl_->have_external_u = false;
    return;
  }
  for (std::size_t k = 0; k < N; ++k) {
    const std::size_t src = std::min(k, n - 1U);
    impl_->external_u[k][0] = accel_cmd[src];
    impl_->external_u[k][1] = steer_cmd[src];
  }
  impl_->have_external_u = true;
}

std::size_t PathTrackingSolver::horizon()
{
  return N;
}

PathTrackingResult PathTrackingSolver::solve(
  const PathTrackingInitialState & x0, const PathTrackingReference & reference)
{
  PathTrackingResult result;
  result.accel_cmd.assign(N, 0.0);
  result.steer_cmd.assign(N, 0.0);

  const std::size_t n_pts =
    std::min({reference.x.size(), reference.y.size(), reference.yaw.size(), reference.v.size()});
  if (n_pts < 2U) {
    result.status = -2;
    return result;
  }

  const std::array<double, NP> model_params = {impl_->lf, impl_->lr, impl_->tau_a, impl_->tau_d};
  impl_->solver.setParametersAllStages(model_params);
  impl_->solver.setSteerRateLimit(impl_->max_steer_rate);

  const std::array<double, NX> x0_world = {x0.x,     x0.y,    x0.yaw, std::max(0.0, x0.v),
                                           x0.accel, x0.steer};

  size_t start_idx = 0;
  {
    double best_d2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < n_pts; ++i) {
      const double dx = reference.x[i] - x0_world[0];
      const double dy = reference.y[i] - x0_world[1];
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        start_idx = i;
      }
    }
  }

  const double yaw_at_start = reference.yaw[start_idx];
  const double psi_bias = std::round((x0_world[2] - yaw_at_start) / kTwoPi) * kTwoPi;
  const double x_off = x0_world[0];
  const double y_off = x0_world[1];

  for (size_t k = 0; k < N; ++k) {
    if (k == 0) {
      // [x,y,psi,v,a,delta, a_cmd_ref, delta_cmd_ref]
      const std::array<double, NY> yref = {
        x0_world[0] - x_off, x0_world[1] - y_off, x0_world[2], x0_world[3],
        x0_world[4],         x0_world[5],         0.0,         0.0};
      impl_->solver.setStageReference(static_cast<int>(k), yref);
      continue;
    }
    const size_t idx = std::min(start_idx + k, n_pts - 1);
    const double yaw = reference.yaw[idx] + psi_bias;
    const double v_ref = std::max(0.0, reference.v[idx]);
    const std::array<double, NY> yref = {
      reference.x[idx] - x_off, reference.y[idx] - y_off, yaw, v_ref, 0.0, 0.0, 0.0, 0.0};
    impl_->solver.setStageReference(static_cast<int>(k), yref);
  }

  const size_t terminal_idx = std::min(start_idx + N, n_pts - 1);
  const double terminal_yaw = reference.yaw[terminal_idx] + psi_bias;
  const double terminal_v_ref = std::max(0.0, reference.v[terminal_idx]);
  impl_->solver.setTerminalReference(
    {reference.x[terminal_idx] - x_off, reference.y[terminal_idx] - y_off, terminal_yaw,
     terminal_v_ref, 0.0, 0.0});

  const std::array<double, NX> x0_local = {x0_world[0] - x_off, x0_world[1] - y_off, x0_world[2],
                                           x0_world[3],         x0_world[4],         x0_world[5]};

  // Warm-start: prefer external u (e.g. shifted MPPI u_opt), else shift previous t-MPT solution.
  if (impl_->have_external_u) {
    std::array<std::array<double, NX>, N + 1> x_guess{};
    for (size_t k = 0; k <= N; ++k) {
      x_guess[k] = x0_local;
    }
    impl_->solver.setWarmStartTrajectory(x_guess, impl_->external_u);
    impl_->have_external_u = false;
  } else if (impl_->warm_start_enabled && impl_->have_prev_solution) {
    std::array<std::array<double, NX>, N + 1> x_guess{};
    std::array<std::array<double, NU>, N> u_guess{};
    for (size_t k = 0; k < N; ++k) {
      const size_t src = std::min(k + 1, N - 1);
      u_guess[k] = impl_->prev_u[src];
    }
    x_guess[0] = x0_local;
    for (size_t k = 1; k <= N; ++k) {
      const size_t src = std::min(k + 1, N);
      auto xw = impl_->prev_x_world[src];
      xw[0] -= x_off;
      xw[1] -= y_off;
      x_guess[k] = xw;
    }
    impl_->solver.setWarmStartTrajectory(x_guess, u_guess);
  } else {
    impl_->solver.setWarmStart(x0_local, {0.0, 0.0});
  }

  const AcadosSolution solution = impl_->solver.getControl(x0_local);
  result.status = solution.status;
  result.ok = (solution.status == 0);
  if (!result.ok) {
    // Keep previous warm-start on failure so the next cycle can still try.
    return result;
  }

  for (size_t k = 0; k < N; ++k) {
    result.accel_cmd[k] = solution.utraj[k][0];
    result.steer_cmd[k] = solution.utraj[k][1];
  }

  // Cache world-frame state + controls for the next cycle's shifted warm-start.
  impl_->prev_u = solution.utraj;
  for (size_t k = 0; k <= N; ++k) {
    auto xw = solution.xtraj[k];
    xw[0] += x_off;
    xw[1] += y_off;
    impl_->prev_x_world[k] = xw;
  }
  impl_->have_prev_solution = impl_->warm_start_enabled;
  return result;
}

}  // namespace temporal_mpt
