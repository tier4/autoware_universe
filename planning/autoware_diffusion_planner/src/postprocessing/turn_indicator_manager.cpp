// Copyright 2025 TIER IV, Inc.
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

#include "autoware/diffusion_planner/postprocessing/turn_indicator_manager.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::diffusion_planner::postprocess
{
namespace
{
uint8_t raw_state_to_command(const std::size_t raw_state)
{
  switch (raw_state) {
    case TURN_INDICATOR_OUTPUT_DISABLE:
      return TurnIndicatorsCommand::DISABLE;
    case TURN_INDICATOR_OUTPUT_ENABLE_LEFT:
      return TurnIndicatorsCommand::ENABLE_LEFT;
    case TURN_INDICATOR_OUTPUT_ENABLE_RIGHT:
      return TurnIndicatorsCommand::ENABLE_RIGHT;
    default:
      return TurnIndicatorsCommand::DISABLE;
  }
}
}  // namespace

TurnIndicatorManager::TurnIndicatorManager(
  const rclcpp::Duration & hold_duration, const rclcpp::Duration & on_confirmation_duration)
: hold_duration_(hold_duration), on_confirmation_duration_(on_confirmation_duration)
{
}

void TurnIndicatorManager::set_durations(
  const rclcpp::Duration & hold_duration, const rclcpp::Duration & on_confirmation_duration)
{
  hold_duration_ = hold_duration;
  on_confirmation_duration_ = on_confirmation_duration;
}

TurnIndicatorsCommand TurnIndicatorManager::evaluate(
  const std::vector<float> & turn_indicator_logit, const rclcpp::Time & stamp)
{
  TurnIndicatorsCommand command_msg;
  command_msg.stamp = stamp;

  const bool malformed_shape =
    turn_indicator_logit.size() != static_cast<std::size_t>(TURN_INDICATOR_OUTPUT_DIM);
  // std::max_element compares with operator<, and every comparison against NaN is false: a
  // NaN in the first slot pins the argmax there (a decisive RIGHT is silently published as
  // DISABLE) and a NaN elsewhere silently excludes its own class. Treat a non-finite logit
  // as malformed instead of letting it enter the debounce as a legitimate observation.
  const bool non_finite =
    !malformed_shape &&
    std::any_of(turn_indicator_logit.begin(), turn_indicator_logit.end(), [](const float value) {
      return !std::isfinite(value);
    });
  if (malformed_shape || non_finite) {
    // A missing, stale-shape or non-finite auxiliary output must never leave the previous
    // command latched indefinitely.
    stable_command_ = TurnIndicatorsCommand::DISABLE;
    has_candidate_ = false;
    has_contrary_ = false;
    has_last_stamp_ = false;
    command_msg.command = TurnIndicatorsCommand::DISABLE;
    return command_msg;
  }

  // A backwards timestamp (simulation reset, bag loop) invalidates any evidence
  // window in progress; keep the stable command and restart confirmation.
  if (has_last_stamp_ && stamp < last_stamp_) {
    has_candidate_ = false;
    has_contrary_ = false;
  }
  last_stamp_ = stamp;
  has_last_stamp_ = true;

  const auto max_it = std::max_element(turn_indicator_logit.begin(), turn_indicator_logit.end());
  const uint8_t observed = raw_state_to_command(
    static_cast<std::size_t>(std::distance(turn_indicator_logit.begin(), max_it)));

  if (observed == stable_command_) {
    // Agreement with the published command discards all pending contrary evidence.
    has_candidate_ = false;
    has_contrary_ = false;
  } else {
    // Disagreement with the active command is tracked on its own clock. Losing confidence
    // in a direction is release evidence and needs no agreement on a replacement, whereas
    // asserting a new direction is a stronger action that keeps the per-identity window.
    if (!has_contrary_) {
      has_contrary_ = true;
      contrary_since_ = stamp;
    }
    if (!has_candidate_ || candidate_command_ != observed) {
      has_candidate_ = true;
      candidate_command_ = observed;
      candidate_since_ = stamp;
    }
    // Activating from DISABLE only needs the short window; releasing an active
    // signal (turn-off or direction flip) needs the long one.
    const bool active = stable_command_ != TurnIndicatorsCommand::DISABLE;
    const rclcpp::Duration & required_duration =
      active ? hold_duration_ : on_confirmation_duration_;
    if ((stamp - candidate_since_) >= required_duration) {
      // A single observation stayed consistent for the window: activate, or flip direction.
      stable_command_ = candidate_command_;
      has_candidate_ = false;
      has_contrary_ = false;
    } else if (active && (stamp - contrary_since_) >= required_duration) {
      // The model disagreed for the whole window without settling on one replacement.
      // Oscillation is exactly the state in which a definite turn intent must not keep
      // being asserted, so fall back to the fail-safe rather than holding the lamp on.
      stable_command_ = TurnIndicatorsCommand::DISABLE;
      has_candidate_ = false;
      has_contrary_ = false;
    }
  }

  command_msg.command = stable_command_;
  return command_msg;
}

}  // namespace autoware::diffusion_planner::postprocess
