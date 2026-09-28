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

#ifndef AUTOWARE__MPPI_OPTIMIZER__DETAIL__TRAJECTORY_VALIDATOR_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__DETAIL__TRAJECTORY_VALIDATOR_HPP_

#include "autoware/mppi_optimizer/detail/trajectory_utils.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"

#include <cstddef>
#include <vector>

namespace autoware::mppi_optimizer::detail
{

template <class Cost>
[[nodiscard]] FirstOrderDubinsMppiValidationResult validateOptimizedTrajectory(
  const Cost & cost, const std::vector<OptimizedState> & states)
{
  for (std::size_t i = 0; i < states.size(); ++i) {
    const auto & state = states[i];
    const int timestep = static_cast<int>(i);
    auto reasons = FirstOrderDubinsMppiInvalidityReason::none;
    if (cost.exceedsLateralBoundary(state.x, state.y)) {
      reasons = reasons | FirstOrderDubinsMppiInvalidityReason::lateral_boundary;
    }
    if (cost.egoIntersectsObstacleAtStep(state.x, state.y, state.yaw, timestep)) {
      reasons = reasons | FirstOrderDubinsMppiInvalidityReason::obstacle;
    }
    if (cost.egoIntersectsRoadBorder(state.x, state.y, state.yaw)) {
      reasons = reasons | FirstOrderDubinsMppiInvalidityReason::road_border;
    }
    if (state.velocity < 0.0) {
      reasons = reasons | FirstOrderDubinsMppiInvalidityReason::reverse;
    }
    if (reasons != FirstOrderDubinsMppiInvalidityReason::none) {
      return FirstOrderDubinsMppiValidationResult{reasons, i};
    }
  }
  return {};
}

}  // namespace autoware::mppi_optimizer::detail

#endif  // AUTOWARE__MPPI_OPTIMIZER__DETAIL__TRAJECTORY_VALIDATOR_HPP_
