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
  /** Empty -> $XDG_CACHE_HOME/autoware/mppi_debug_log or $HOME/.cache/autoware/mppi_debug_log. */
  std::string debug_trajectory_log_directory;
  bool ignore_obstacles{false};
  bool ignore_drivable_area{false};
  bool force_cold_start_each_step{false};
  bool skip_if_invalid{false};
  /** Warm-start u_nom from shifted previous optimized controls (else reseed from DP each cycle). */
  bool use_last_control_as_nominal{false};
};

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_RUNTIME_OPTIONS_HPP_
