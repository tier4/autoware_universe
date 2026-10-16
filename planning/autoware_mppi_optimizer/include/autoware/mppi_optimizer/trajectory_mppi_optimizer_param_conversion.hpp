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

#ifndef AUTOWARE__MPPI_OPTIMIZER__TRAJECTORY_MPPI_OPTIMIZER_PARAM_CONVERSION_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__TRAJECTORY_MPPI_OPTIMIZER_PARAM_CONVERSION_HPP_

#include "autoware/mppi_optimizer/curvature_adaptive_steering_filter.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_runtime_options.hpp"
#include <autoware_mppi_optimizer/trajectory_mppi_optimizer_parameters.hpp>

namespace autoware::mppi_optimizer
{

FirstOrderDubinsMppiCostParams costParamsFromTrajectoryMppiOptimizerParams(
  const trajectory_mppi_optimizer::Params & params);

FirstOrderDubinsMppiRuntimeOptions runtimeOptionsFromTrajectoryMppiOptimizerParams(
  const trajectory_mppi_optimizer::Params & params);

CurvatureAdaptiveSteeringFilterParams curvatureAdaptiveSteeringFilterParamsFromTrajectoryMppiOptimizerParams(
  const trajectory_mppi_optimizer::Params & params);

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__TRAJECTORY_MPPI_OPTIMIZER_PARAM_CONVERSION_HPP_
