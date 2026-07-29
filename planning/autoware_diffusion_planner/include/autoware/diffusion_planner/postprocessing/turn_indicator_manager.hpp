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

#ifndef AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__TURN_INDICATOR_MANAGER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__TURN_INDICATOR_MANAGER_HPP_

#include "autoware/diffusion_planner/dimensions.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <cstdint>
#include <vector>

namespace autoware::diffusion_planner::postprocess
{
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

/**
 * @brief Debounces the model's per-cycle turn-indicator prediction into a stable command.
 *
 * The model emits three dense state classes (DISABLE, ENABLE_LEFT, ENABLE_RIGHT) every
 * planning cycle; KEEP is not a model class, so command persistence is deployment-side
 * logic.  This manager is an asymmetric-hysteresis state machine over the published
 * command:
 *
 *  - The per-cycle argmax is treated as an observation.  Temporal consistency is the
 *    confidence filter: a command change is published only after evidence has persisted
 *    for a confirmation window, and an observation agreeing with the published command
 *    clears all pending evidence.  (No logit-margin gate: it would add a
 *    calibration-sensitive knob, while consecutive agreement filters the same
 *    single-frame glitches.)
 *  - Turning ON from DISABLE uses a short window (on_confirmation_duration) so the
 *    blinker starts promptly while single-frame glitches are rejected.  Only the *same*
 *    observation counts, so alternating noise never activates a signal.
 *  - Leaving an active signal (LEFT/RIGHT -> DISABLE, or a direct direction flip) uses
 *    the longer hold_duration window, which also guarantees a minimum on-time of
 *    hold_duration.  Two kinds of evidence are timed separately here: a *consistent*
 *    contrary observation confirms whatever it asserts (turn off, or flip direction),
 *    while *any* sustained disagreement - including observations that keep changing
 *    identity - releases to DISABLE.  Oscillation is precisely the state in which a
 *    definite turn intent must not keep being asserted, so it must not latch the lamp on.
 *  - A malformed or non-finite logit vector immediately resets to DISABLE so a stale
 *    command is never latched; a backwards timestamp (sim reset / bag loop) restarts the
 *    evidence window.
 *  - The windows are pure timestamp differences and deliberately carry no minimum
 *    observation count: under a degraded cycle rate a count would stop confirming at all,
 *    which would latch the published command - a strictly worse failure than the early
 *    release a stalled planner can otherwise cause, since that release lands on the
 *    fail-safe state.
 */
class TurnIndicatorManager
{
public:
  /**
   * @brief Constructs a manager with asymmetric confirmation windows.
   *
   * @param hold_duration Contrary evidence required to release (turn off or flip) an
   *        active LEFT/RIGHT command; also its minimum on-time.
   * @param on_confirmation_duration Consistent evidence required to activate a signal
   *        from DISABLE.
   */
  TurnIndicatorManager(
    const rclcpp::Duration & hold_duration, const rclcpp::Duration & on_confirmation_duration);

  /**
   * @brief Evaluates three-class logits into a debounced command.
   *
   * @param turn_indicator_logit Logits in Python order: disable, left, right.
   * @param stamp Timestamp for the command message.
   * @return TurnIndicatorsCommand with the selected command and stamp.
   */
  TurnIndicatorsCommand evaluate(
    const std::vector<float> & turn_indicator_logit, const rclcpp::Time & stamp);

  /**
   * @brief Updates both confirmation windows.
   *
   * @param hold_duration New release-confirmation window.
   * @param on_confirmation_duration New activation-confirmation window.
   */
  void set_durations(
    const rclcpp::Duration & hold_duration, const rclcpp::Duration & on_confirmation_duration);

private:
  /// Contrary-evidence window to release an active command (and its minimum on-time).
  rclcpp::Duration hold_duration_;
  /// Consistent-evidence window to activate a signal from DISABLE.
  rclcpp::Duration on_confirmation_duration_;

  uint8_t stable_command_{TurnIndicatorsCommand::DISABLE};

  /// Pending contrary observation being confirmed; only the same identity accumulates.
  uint8_t candidate_command_{TurnIndicatorsCommand::DISABLE};
  bool has_candidate_{false};
  rclcpp::Time candidate_since_{};

  /// Any disagreement with an active command, regardless of which class was observed.
  bool has_contrary_{false};
  rclcpp::Time contrary_since_{};

  /// Last evaluation stamp, used to detect time regressions.
  rclcpp::Time last_stamp_{};
  bool has_last_stamp_{false};
};

}  // namespace autoware::diffusion_planner::postprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__TURN_INDICATOR_MANAGER_HPP_
