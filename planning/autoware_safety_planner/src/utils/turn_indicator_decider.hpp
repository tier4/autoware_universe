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

#ifndef UTILS__TURN_INDICATOR_DECIDER_HPP_
#define UTILS__TURN_INDICATOR_DECIDER_HPP_

#include "../context.hpp"
#include "../type_alias.hpp"

#include <cstdint>
#include <optional>

namespace autoware::safety_planner
{

struct TurnSignalParams
{
  double search_distance{30.0};            //!< [m] activation distance floor
  double min_blink_duration{3.0};          //!< [s] min on-time once lit (anti-chatter)
  double stopped_velocity_threshold{0.1};  //!< [m/s] at/below this ego counts as stopped
  double heading_align_threshold{0.15};    //!< [rad] ego-vs-exit yaw gap that ends a maneuver
};

class TurnIndicatorDecider
{
public:
  explicit TurnIndicatorDecider(const TurnSignalParams & params) : params_(params) {}

  void update_params(const TurnSignalParams & params) { params_ = params; }

  TurnIndicatorsCommand decide(const PlannerContext & context, const Trajectory & trajectory);

private:
  TurnSignalParams params_;
  uint8_t held_command_{TurnIndicatorsCommand::DISABLE};
  double held_since_{0.0};
  uint8_t pull_out_latch_{TurnIndicatorsCommand::DISABLE};
  bool arrived_at_goal_{false};
  std::optional<Pose> latched_goal_pose_;
};

}  // namespace autoware::safety_planner

#endif  // UTILS__TURN_INDICATOR_DECIDER_HPP_
