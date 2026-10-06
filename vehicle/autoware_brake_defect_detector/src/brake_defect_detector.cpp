// Copyright 2026 The Autoware Contributors
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

#include "autoware_brake_defect_detector/brake_defect_detector.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::brake_defect_detector
{

BrakeDefectDetector::BrakeDefectDetector(const DetectorParams & params) : params_(params)
{
}

void BrakeDefectDetector::reset()
{
  commands_.clear();
  last_update_time_.reset();
  settling_until_sec_ = 0.0;
  filtered_residual_ = 0.0;
  cusum_statistic_ = 0.0;
  filter_initialized_ = false;
}

void BrakeDefectDetector::observe_command(const double acceleration, const double time_sec)
{
  if (!std::isfinite(acceleration) || !std::isfinite(time_sec)) {
    reset();
    return;
  }

  if (!commands_.empty()) {
    const auto & previous = commands_.back();
    const double dt = time_sec - previous.time_sec;
    if (dt <= 0.0) {
      reset();
    } else if (std::abs(acceleration - previous.acceleration) / dt > params_.jerk_limit_mps3) {
      settling_until_sec_ = time_sec + params_.settling_time_sec;
    }
  }

  commands_.push_back({time_sec, acceleration});

  // Retain one command before the useful window for zero-order hold lookup.
  const double oldest_useful = time_sec - params_.actuation_delay_sec - params_.max_update_gap_sec;
  while (commands_.size() > 1 && commands_[1].time_sec <= oldest_useful) {
    commands_.pop_front();
  }
}

DiagnosticStatus BrakeDefectDetector::update(
  const double acceleration_command, const double brake_command, const double measured_acceleration,
  const double speed, const double pitch_rad, const double time_sec)
{
  if (
    !std::isfinite(acceleration_command) || !std::isfinite(brake_command) ||
    !std::isfinite(measured_acceleration) || !std::isfinite(speed) || !std::isfinite(pitch_rad) ||
    !std::isfinite(time_sec)) {
    reset();
    return {};
  }

  double dt = 0.0;
  if (last_update_time_) {
    dt = time_sec - *last_update_time_;
    if (dt <= 0.0 || dt > params_.max_update_gap_sec) {
      reset();
      return {};
    }
  }
  last_update_time_ = time_sec;

  const double delayed_time = time_sec - params_.actuation_delay_sec;
  while (commands_.size() > 1 && commands_[1].time_sec <= delayed_time) {
    commands_.pop_front();
  }
  if (commands_.empty() || commands_.front().time_sec > delayed_time) {
    return {};
  }

  const double delayed_command = commands_.front().acceleration;
  const double expected_acceleration = delayed_command + 9.81 * std::sin(pitch_rad);
  // With negative deceleration commands, missing brake torque makes measured
  // acceleration greater than expected. Positive residual therefore means
  // insufficient braking.
  const double residual = measured_acceleration - expected_acceleration;
  if (!std::isfinite(residual)) {
    reset();
    return {};
  }

  const bool valid = dt > 0.0 && acceleration_command < params_.max_decel_cmd &&
                     delayed_command < params_.max_decel_cmd &&
                     brake_command >= params_.brake_cmd_min &&
                     brake_command <= params_.brake_cmd_max && speed >= params_.min_speed_mps &&
                     time_sec >= settling_until_sec_;

  if (valid) {
    if (!filter_initialized_) {
      filtered_residual_ = residual;
      filter_initialized_ = true;
    } else {
      const double alpha = params_.residual_filter_tau_sec > 0.0
                             ? 1.0 - std::exp(-dt / params_.residual_filter_tau_sec)
                             : 1.0;
      filtered_residual_ += alpha * (residual - filtered_residual_);
    }
    cusum_statistic_ =
      std::max(0.0, cusum_statistic_ + (filtered_residual_ - params_.cusum_drift_k) * dt);
  } else {
    filter_initialized_ = false;
    filtered_residual_ = 0.0;
    cusum_statistic_ = std::max(0.0, cusum_statistic_ - params_.cusum_drift_k * dt);
  }

  return {
    valid && cusum_statistic_ >= params_.cusum_threshold_h, filtered_residual_, cusum_statistic_,
    valid};
}

}  // namespace autoware::brake_defect_detector
