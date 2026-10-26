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

#ifndef TEMPORAL_MPT__PATH_TRACKING_SOLVER_HPP_
#define TEMPORAL_MPT__PATH_TRACKING_SOLVER_HPP_

#include <cstddef>
#include <memory>
#include <vector>

namespace temporal_mpt
{

/** Time-indexed reference path (x, y, yaw, v) in world frame. */
struct PathTrackingReference
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> v;
};

struct PathTrackingInitialState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
  double accel{0.0};
  double steer{0.0};
};

struct PathTrackingResult
{
  bool ok{false};
  int status{-1};
  /** Stage controls u[k] = (accel_cmd, steer_cmd), length = horizon N. */
  std::vector<double> accel_cmd;
  std::vector<double> steer_cmd;
};

/**
 * Thin host wrapper around the temporal first-order-Dubins acados OCP
 * (MPPI plant without input-delay FIFOs).
 */
class PathTrackingSolver
{
public:
  PathTrackingSolver();
  ~PathTrackingSolver();

  PathTrackingSolver(const PathTrackingSolver &) = delete;
  PathTrackingSolver & operator=(const PathTrackingSolver &) = delete;
  PathTrackingSolver(PathTrackingSolver &&) noexcept;
  PathTrackingSolver & operator=(PathTrackingSolver &&) noexcept;

  /**
   * Model parameters: ``L = lf + lr`` (= wheel_base), first-order lags, steer-rate limit.
   * ``tau_a`` / ``tau_d`` match MPPI ``acc_time_constant`` / ``steer_time_constant``.
   */
  void setModelParameters(
    double lf, double lr, double tau_a, double tau_d, double max_steer_rate_rad_s = 3.0);

  /**
   * When enabled (default), a successful solve seeds the next NLP with the previous
   * solution shifted by one stage (standard MPC warm-start).
   */
  void setPreviousSolutionWarmStartEnabled(bool enabled);

  /** Drop stored previous solution (e.g. after stop / tracking reset). */
  void resetWarmStart();

  /**
   * Optional external warm-start from a prior control sequence (e.g. shifted MPPI u_opt).
   * Consumed on the next ``solve`` (after an internal one-step shift is not applied to these;
   * pass already time-aligned commands). Cleared after use.
   */
  void setWarmStartControls(
    const std::vector<double> & accel_cmd, const std::vector<double> & steer_cmd);

  PathTrackingResult solve(
    const PathTrackingInitialState & x0, const PathTrackingReference & reference);

  static std::size_t horizon();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace temporal_mpt

#endif  // TEMPORAL_MPT__PATH_TRACKING_SOLVER_HPP_
