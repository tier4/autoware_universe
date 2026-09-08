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

#ifndef AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_RUNTIME_OPTIONS_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_RUNTIME_OPTIONS_HPP_

#include <string>

namespace autoware::mppi_optimizer
{

/** Debug logging and ablation options from mppi_optimizer.param.yaml. */
struct FirstOrderDubinsMppiRuntimeOptions
{
  bool enable_debug_trajectory_log{false};
  /** Empty -> process current working directory. */
  std::string debug_trajectory_log_directory;
  bool ignore_obstacles{false};
  bool ignore_road_borders{false};
  bool ignore_drivable_area{false};
  bool force_cold_start_each_step{false};
  bool skip_if_invalid{false};
  /** Skip optimization for stopping trajectories shorter than this arc length in meters. */
  float min_optimization_length{0.0F};
  /** Warm-start u_nom from shifted previous optimized controls (else reseed from DP each cycle).
   *  Ignored when use_temporal_mpt_as_nominal is true (t-MPT uses its own shifted solution). */
  bool use_last_control_as_nominal{false};
  /**
   * When true (and not forced nominal), seed u_nom from acados temporal MPT
   * instead of the geometric diffusion seed. Falls back to diffusion seed on solve failure.
   * t-MPT warm-starts from its own previous x/u shifted one stage, not from MPPI u_opt.
   */
  bool use_temporal_mpt_as_nominal{false};
  /** Prevent MPPI rollouts from integrating longitudinal velocity below zero. */
  bool prevent_reverse_velocity{true};
  /**
   * When false, ignore vehicle acc/steer time delays in the MPPI plant (N_acc = N_steer = 0).
   * Vehicle τ (first-order lag) is unchanged. Default true preserves delay compensation.
   */
  bool enable_input_delay_compensation{true};
};

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_RUNTIME_OPTIONS_HPP_
