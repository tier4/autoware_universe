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

#ifndef AUTOWARE__TENSORRT_E2E__BEV_WARP_TARGET_HPP_
#define AUTOWARE__TENSORRT_E2E__BEV_WARP_TARGET_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace autoware::tensorrt_e2e
{

/**
 * @brief The pose the BEV maps are warped into under `planning_time`, with the turn during L
 * smoothed.
 *
 * Under `planning_time` the BEV maps are warped from the cloud's pose (T) into the planning
 * pose (T + L). Measured, the heading change over those ~0.11 s carries the car's own steering
 * wobble: 2.2-2.9 mrad rms (p99 7-9) on top of a ~20 mrad turn in the 2026-10-23 taxi turns.
 * The model was trained on the human's smooth motion there and, on the car's lateral chain,
 * planned along that wobble: a 1.6 Hz steering oscillation in turns, which the same warp with a
 * smooth turn removes (replayed closed loop, 30 taxi turns).
 *
 * The target here keeps the planning position, and takes the heading as the cloud's heading plus
 * a low-passed yaw rate times L. The yaw rate is filtered first order with `tau_s`, one update
 * per planning tick. `tau_s <= 0` is off: the planning pose as measured, unchanged. Poses are
 * `[x, y, cos, sin]`.
 *
 * Header-only, with no ROS types, so the rule is tested without a ROS build.
 */
class BevWarpTarget
{
public:
  explicit BevWarpTarget(const double tau_s) : tau_s_(tau_s) {}

  [[nodiscard]] bool enabled() const { return tau_s_ > 0.0; }

  /// Forget the filter (a localization reset or a gap restarts it from the next yaw rate).
  void reset() { filtered_ = false; }

  /// The warp target for this tick. `stamp_s` is the planning time; `latency_s` the time from
  /// the cloud to it; `yaw_rate` the measured yaw rate at the planning time (rad/s).
  std::array<double, 4> update(
    const std::array<double, 4> & sensor_pose, const std::array<double, 4> & planning_pose,
    const double yaw_rate, const double latency_s, const double stamp_s)
  {
    if (!enabled() || !std::isfinite(yaw_rate) || !std::isfinite(stamp_s)) {
      return planning_pose;
    }
    const double dt = stamp_s - last_stamp_s_;
    if (!filtered_ || dt < 0.0 || dt > max_gap_s) {
      yaw_rate_ = yaw_rate;  // first tick, a rewind, or a gap: start over
      filtered_ = true;
    } else if (dt > 0.0) {
      yaw_rate_ += (1.0 - std::exp(-dt / tau_s_)) * (yaw_rate - yaw_rate_);
    }  // a second pass at the same stamp does not filter twice
    last_stamp_s_ = stamp_s;
    const double heading =
      std::atan2(sensor_pose[3], sensor_pose[2]) + yaw_rate_ * std::max(latency_s, 0.0);
    return {planning_pose[0], planning_pose[1], std::cos(heading), std::sin(heading)};
  }

  [[nodiscard]] std::optional<double> filtered_yaw_rate() const
  {
    return filtered_ ? std::optional<double>(yaw_rate_) : std::nullopt;
  }

  //! A longer gap between ticks than this restarts the filter.
  static constexpr double max_gap_s = 1.0;

private:
  double tau_s_;
  bool filtered_{false};
  double yaw_rate_{0.0};
  double last_stamp_s_{0.0};
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__BEV_WARP_TARGET_HPP_
