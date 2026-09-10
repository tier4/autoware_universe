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

#ifndef AUTOWARE__MPPI_OPTIMIZER__DETAIL__TRAJECTORY_UTILS_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__DETAIL__TRAJECTORY_UTILS_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>

#include <cstddef>
#include <optional>
#include <vector>

namespace autoware::mppi_optimizer::detail
{

inline constexpr int kMppiHorizon = 80;
inline constexpr float kMppiDt = 0.1F;

struct InitialState
{
  float x{0.0F};
  float y{0.0F};
  float yaw{0.0F};
  float velocity{0.0F};
  float acceleration{0.0F};
  float steering{0.0F};
};

struct ReferenceSample
{
  float time{0.0F};
  float x{0.0F};
  float y{0.0F};
  float yaw{0.0F};
  float velocity{0.0F};
  std::optional<float> max_velocity;
  /** Cumulative polyline chord length [m] along the DP trajectory at this sample's source index. */
  float arc_length_s{0.0F};
};

struct OptimizedState
{
  float x{0.0F};
  float y{0.0F};
  float yaw{0.0F};
  float velocity{0.0F};
  float acceleration{0.0F};
  float steering{0.0F};
};

/**
 * Deterministic longitudinal profile used while an external or pointwise maximum velocity is
 * restrictive. Steering commands are copied from the supplied MPPI control sequence unchanged.
 */
struct ActiveVelocityLimitProfile
{
  bool active{false};
  float target_velocity{0.0F};
  /** Effective maximum at each post-step sample; nullopt denotes no pointwise maximum. */
  std::vector<std::optional<float>> maximum_velocities;
  std::vector<FirstOrderDubinsMppiControl> controls;
  /** Post-step velocity and acceleration states aligned with controls. */
  std::vector<float> velocities;
  std::vector<float> accelerations;
};

[[nodiscard]] bool isOptimizationRequired(const Trajectory & trajectory, double min_length);

void setInitialEngageVelocity(
  Trajectory & trajectory, const std::optional<float> & max_velocity = std::nullopt);

[[nodiscard]] InitialState makeInitialState(
  const Odometry & odometry,
  const std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> & acceleration,
  const std::optional<autoware_vehicle_msgs::msg::SteeringReport> & steering_status,
  const FirstOrderDubinsMppiVehicleParams & vehicle_params);

[[nodiscard]] std::vector<ReferenceSample> buildReferenceHorizon(
  const Trajectory & trajectory, const InitialState & ego, const int horizon = kMppiHorizon,
  const float dt = kMppiDt, const size_t start_idx = 0U,
  const std::vector<float> * cumulative_chord_length_s = nullptr,
  const std::vector<std::optional<float>> * maximum_velocities = nullptr);

/** Resolve the external/map minimum at every reference point. */
[[nodiscard]] std::vector<std::optional<float>> buildEffectiveMaximumVelocityProfile(
  std::size_t point_count, const FirstOrderDubinsMppiKinematicLimits & limits);

/** Return the common finite maximum when every point has the same limit. */
[[nodiscard]] std::optional<float> getUniformMaximumVelocity(
  const std::vector<std::optional<float>> & maximum_velocities);

/**
 * Build the fastest delay-, lag-, acceleration-, and jerk-aware profile under the effective
 * external/map maximum-velocity envelope. Uniform limits retain the existing scalar behavior;
 * varying limits use a backwards reachable envelope so braking begins before a lower future cap.
 */
[[nodiscard]] ActiveVelocityLimitProfile buildActiveVelocityLimitProfile(
  const std::vector<FirstOrderDubinsMppiControl> & controls, const InitialState & initial_state,
  const FirstOrderDubinsMppiKinematicLimits & limits,
  const FirstOrderDubinsMppiVehicleParams & vehicle_params, int acceleration_delay_steps = 0,
  const std::vector<float> & acceleration_delay_buffer = {}, float dt = kMppiDt,
  bool keep_active = false, const std::vector<float> & reference_velocities = {});

/** Apply an active profile to trajectory velocity/acceleration fields; inactive is an exact no-op.
 */
void applyActiveVelocityLimitProfile(
  Trajectory & trajectory, const ActiveVelocityLimitProfile & profile);

[[nodiscard]] std::vector<FirstOrderDubinsMppiControl> buildDiffusionNominalControl(
  const Trajectory & reference, std::size_t start_idx,
  const FirstOrderDubinsMppiVehicleParams & vehicle_params, int horizon = kMppiHorizon,
  float min_chord_length_m = 1.5F);

[[nodiscard]] std::vector<FirstOrderDubinsMppiControl> buildForcedNominalControl(
  const std::vector<float> & acceleration_commands, const std::vector<float> & steering_commands,
  const FirstOrderDubinsMppiVehicleParams & vehicle_params, int horizon = kMppiHorizon);

/**
 * Filter an MPPI nominal control sequence through optional longitudinal kinematic limits.
 *
 * The filter predicts the first-order acceleration state through the pending input-delay queue.
 * It preserves steering, clamps acceleration commands to active acceleration bounds, applies
 * active jerk bounds at the time each command reaches the plant, and biases acceleration toward
 * the admissible boundary while predicted velocity is outside the active pointwise interval.
 */
[[nodiscard]] std::vector<FirstOrderDubinsMppiControl> filterNominalControlWithKinematicLimits(
  const std::vector<FirstOrderDubinsMppiControl> & nominal, const InitialState & initial_state,
  const FirstOrderDubinsMppiKinematicLimits & limits,
  const FirstOrderDubinsMppiVehicleParams & vehicle_params, int acceleration_delay_steps = 0,
  const std::vector<float> & acceleration_delay_buffer = {}, float dt = kMppiDt);

/**
 * Low-pass steering commands with alpha increasing linearly from straight_alpha at zero steering
 * to turn_alpha at turn_angle_rad. The larger of the previous and target steering magnitudes sets
 * alpha, preserving fast response both into and out of turns. Acceleration is preserved exactly.
 */
void filterSteeringCommandsWithCurvatureAdaptiveEma(
  std::vector<FirstOrderDubinsMppiControl> & controls, float initial_steering, float straight_alpha,
  float turn_alpha, float turn_angle_rad);

[[nodiscard]] std::vector<FirstOrderDubinsMppiControl> shiftNominalControl(
  const std::vector<FirstOrderDubinsMppiControl> & previous, int horizon = kMppiHorizon);

/**
 * Time-align a no-delay nominal (e.g. t-MPT) to MPPI issued commands with per-channel dead time.
 * Issued u_nom[k] enters the delay FIFO; the plant sees it acc_delay / steer_delay steps later.
 * Maps u_nom[k] <- u_no_delay[k + delay] so MPPI stage k uses the t-MPT intent for stage k.
 */
[[nodiscard]] std::vector<FirstOrderDubinsMppiControl> shiftNominalControlForInputDelay(
  const std::vector<FirstOrderDubinsMppiControl> & nominal, int acceleration_delay_steps,
  int steer_delay_steps);

[[nodiscard]] Trajectory buildOptimizedTrajectory(
  const Trajectory & input, const std::vector<OptimizedState> & post_step_states,
  const std::vector<FirstOrderDubinsMppiControl> & controls);

[[nodiscard]] float computeMengerCurvatureWithMinChord(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, std::size_t target_idx,
  float min_chord_length_m = 1.5F) noexcept;

/**
 * @brief Cumulative chord length along a trajectory polyline: s[0]=0,
 *        s[i]=s[i-1]+||p[i]-p[i-1]||. Same length as points; empty input → empty output.
 */
[[nodiscard]] std::vector<float> computeCumulativeChordLength(const Trajectory & trajectory);

}  // namespace autoware::mppi_optimizer::detail

#endif  // AUTOWARE__MPPI_OPTIMIZER__DETAIL__TRAJECTORY_UTILS_HPP_
