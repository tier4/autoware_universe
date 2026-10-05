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

#ifndef TRAJECTORY_PLANNER__REFERENCE_PATH_FOLLOWING_PLANNER__REFERENCE_PATH_FOLLOWING_PLANNER_HPP_
#define TRAJECTORY_PLANNER__REFERENCE_PATH_FOLLOWING_PLANNER__REFERENCE_PATH_FOLLOWING_PLANNER_HPP_

#include "../../utils/turn_indicator_decider.hpp"
#include "../trajectory_planner_interface.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner::experimental
{

class ReferencePathFollowingPlanner : public TrajectoryPlannerInterface
{
public:
  std::string get_name() const override { return "reference_path_following_planner"; }

  void on_initialize(
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    const Params & params) override;

  TrajectoryPlannerResult plan_trajectories(const TrajectoryPlannerInput & input) override;

private:
  PlannedTrajectory plan_one_side(
    TurnIndicatorDecider & turn_indicator_decider, const PlannerContext & context,
    const std::vector<Constraint> & constraints, std::optional<Trajectory> & previous_trajectory,
    TrajectoryPlannerDebug & debug) const;

  // One decider per output, since each holds its own anti-chatter and latch state
  TurnIndicatorDecider normal_turn_indicator_decider_{TurnSignalParams{}};
  TurnIndicatorDecider cautious_turn_indicator_decider_{TurnSignalParams{}};
  //! Output of the previous cycle, one per side, where the initial acceleration is read from
  std::optional<Trajectory> normal_previous_trajectory_;
  std::optional<Trajectory> cautious_previous_trajectory_;
};

}  // namespace autoware::safety_planner::experimental

// clang-format off
#endif  // TRAJECTORY_PLANNER__REFERENCE_PATH_FOLLOWING_PLANNER__REFERENCE_PATH_FOLLOWING_PLANNER_HPP_  // NOLINT
// clang-format on
