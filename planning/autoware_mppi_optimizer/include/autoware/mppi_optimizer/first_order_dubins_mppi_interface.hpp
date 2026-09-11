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

#ifndef AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_INTERFACE_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_INTERFACE_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_runtime_options.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params.hpp"

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::mppi_optimizer
{

using autoware_perception_msgs::msg::TrackedObjects;
using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

/** Open-loop plant replay vs measured ego over elapsed wall time since the previous MPPI cycle. */
struct FirstOrderDubinsMppiPredictionAccuracy
{
  bool valid{false};
  double elapsed_s{0.0};
  int full_steps{0};
  float remainder_s{0.0F};
  int integration_steps{0};
  float pos_error_m{0.0F};
  float yaw_error_rad{0.0F};
  float vel_error_mps{0.0F};
  float predicted_x{0.0F};
  float predicted_y{0.0F};
  float predicted_yaw{0.0F};
  float predicted_vel{0.0F};
};

struct FirstOrderDubinsMppiState
{
  float x{0.0F};
  float y{0.0F};
  float yaw{0.0F};
  float vel_x{0.0F};
};

struct FirstOrderDubinsMppiControl
{
  float accel_cmd{0.0F};
  float steer_cmd{0.0F};
};

enum class FirstOrderDubinsMppiNominalSeedSource : std::uint8_t {
  diffusion_reference,
  previous_optimized,
  temporal_mpt,
  forced,
  mpc_predicted_trajectory
};

enum class FirstOrderDubinsMppiNominalResetReason : std::uint8_t {
  none,
  unavailable,
  forced_cold_start,
  stopped,
  invalid_timestamp,
  expired,
  prediction_error,
  reference_discontinuity,
  initial_steering_discontinuity,
  rejected,
  skipped,
  configuration_changed,
  externally_invalidated
};

inline const char * to_string(const FirstOrderDubinsMppiNominalSeedSource source)
{
  switch (source) {
    case FirstOrderDubinsMppiNominalSeedSource::diffusion_reference:
      return "diffusion_reference";
    case FirstOrderDubinsMppiNominalSeedSource::previous_optimized:
      return "previous_optimized";
    case FirstOrderDubinsMppiNominalSeedSource::temporal_mpt:
      return "temporal_mpt";
    case FirstOrderDubinsMppiNominalSeedSource::forced:
      return "forced";
    case FirstOrderDubinsMppiNominalSeedSource::mpc_predicted_trajectory:
      return "mpc_predicted_trajectory";
  }
  return "unknown";
}

inline const char * to_string(const FirstOrderDubinsMppiNominalResetReason reason)
{
  switch (reason) {
    case FirstOrderDubinsMppiNominalResetReason::none:
      return "none";
    case FirstOrderDubinsMppiNominalResetReason::unavailable:
      return "unavailable";
    case FirstOrderDubinsMppiNominalResetReason::forced_cold_start:
      return "forced_cold_start";
    case FirstOrderDubinsMppiNominalResetReason::stopped:
      return "stopped";
    case FirstOrderDubinsMppiNominalResetReason::invalid_timestamp:
      return "invalid_timestamp";
    case FirstOrderDubinsMppiNominalResetReason::expired:
      return "expired";
    case FirstOrderDubinsMppiNominalResetReason::prediction_error:
      return "prediction_error";
    case FirstOrderDubinsMppiNominalResetReason::reference_discontinuity:
      return "reference_discontinuity";
    case FirstOrderDubinsMppiNominalResetReason::initial_steering_discontinuity:
      return "initial_steering_discontinuity";
    case FirstOrderDubinsMppiNominalResetReason::rejected:
      return "rejected";
    case FirstOrderDubinsMppiNominalResetReason::skipped:
      return "skipped";
    case FirstOrderDubinsMppiNominalResetReason::configuration_changed:
      return "configuration_changed";
    case FirstOrderDubinsMppiNominalResetReason::externally_invalidated:
      return "externally_invalidated";
  }
  return "unknown";
}

struct FirstOrderDubinsMppiPostprocessingContext
{
  FirstOrderDubinsMppiNominalSeedSource seed_source{
    FirstOrderDubinsMppiNominalSeedSource::diffusion_reference};
  /** Number of samples removed from a previous accepted horizon. */
  int shift_count{0};
  /** True only when optimized steering u[0] still equals its already-filtered shifted seed. */
  bool preserve_first_steering_command{false};
};

enum class FirstOrderDubinsMppiMpcNominalSeedStatus : std::uint8_t {
  disabled,
  previous_mppi_applied,
  unavailable,
  stale,
  invalid,
  forced_nominal,
  optimization_not_run,
  used
};

inline const char * to_string(const FirstOrderDubinsMppiMpcNominalSeedStatus status)
{
  switch (status) {
    case FirstOrderDubinsMppiMpcNominalSeedStatus::disabled:
      return "disabled";
    case FirstOrderDubinsMppiMpcNominalSeedStatus::previous_mppi_applied:
      return "previous_mppi_applied";
    case FirstOrderDubinsMppiMpcNominalSeedStatus::unavailable:
      return "unavailable";
    case FirstOrderDubinsMppiMpcNominalSeedStatus::stale:
      return "stale";
    case FirstOrderDubinsMppiMpcNominalSeedStatus::invalid:
      return "invalid";
    case FirstOrderDubinsMppiMpcNominalSeedStatus::forced_nominal:
      return "forced_nominal";
    case FirstOrderDubinsMppiMpcNominalSeedStatus::optimization_not_run:
      return "optimization_not_run";
    case FirstOrderDubinsMppiMpcNominalSeedStatus::used:
      return "used";
  }
  return "unknown";
}

constexpr FirstOrderDubinsMppiMpcNominalSeedStatus resolveMpcNominalSeedStatus(
  const bool enabled, const bool previous_mppi_applied, const bool prediction_available,
  const bool prediction_fresh, const bool prediction_valid)
{
  if (!enabled) return FirstOrderDubinsMppiMpcNominalSeedStatus::disabled;
  if (previous_mppi_applied) {
    return FirstOrderDubinsMppiMpcNominalSeedStatus::previous_mppi_applied;
  }
  if (!prediction_available) return FirstOrderDubinsMppiMpcNominalSeedStatus::unavailable;
  if (!prediction_fresh) return FirstOrderDubinsMppiMpcNominalSeedStatus::stale;
  if (!prediction_valid) return FirstOrderDubinsMppiMpcNominalSeedStatus::invalid;
  return FirstOrderDubinsMppiMpcNominalSeedStatus::used;
}

/** Optional host-side output conditioning supplied by the interface caller. */
using FirstOrderDubinsMppiControlSequencePostprocessor = std::function<void(
  std::vector<FirstOrderDubinsMppiControl> &, const FirstOrderDubinsMppiPostprocessingContext &)>;

/** Nominal control sequence supplied to MPPI before sampling and optimization. */
struct FirstOrderDubinsMppiNominalControlProfile
{
  float time_step_s{0.0F};
  std::vector<float> acceleration_commands_mps2;
  std::vector<float> steering_commands_rad;
};

/** Result of anchoring nominal steer u[0] to the steering state at command application time. */
struct FirstOrderDubinsMppiNominalSteeringContinuity
{
  bool active{false};
  bool clamped{false};
  float application_steering_rad{0.0F};
  float unguarded_command_rad{0.0F};
  float guarded_command_rad{0.0F};
};

struct FirstOrderDubinsMppiRollout
{
  std::vector<std::pair<float, float>> points;
  float cost{0.0F};
  /** One-based MPPI optimization iteration; zero denotes a legacy/offline aggregate sample. */
  int iteration{0};
  /** True when this sample was selected as a high-cost (worst) viz sample, not top-weighted. */
  bool is_worst{false};
};

/** Optional kinematic bounds supplied by external and map velocity-limit sources. */
struct FirstOrderDubinsMppiKinematicLimits
{
  /** Global maximum supplied by the external VelocityLimit message. */
  std::optional<float> max_velocity;
  /** Optional map maximum aligned with each point of the input reference trajectory. */
  std::vector<std::optional<float>> max_velocity_by_reference_point;
  std::optional<float> min_longitudinal_acceleration;
  std::optional<float> max_longitudinal_acceleration;
  std::optional<float> min_longitudinal_jerk;
  std::optional<float> max_longitudinal_jerk;
};

/** Host reconstruction of the cost assigned to the selected MPPI trajectory. */
struct FirstOrderDubinsMppiCostBreakdown
{
  float spatial_overspeed{0.0F};
  float track{0.0F};
  float heading{0.0F};
  float terminal_error{0.0F};
  float terminal_heading{0.0F};
  float lateral_distance{0.0F};
  float lateral_boundary{0.0F};
  float lateral_yaw_error{0.0F};
  float remaining_distance{0.0F};
  float path_overshoot{0.0F};
  float track_center{0.0F};
  float corner_buffer{0.0F};
  float drivable_area{0.0F};
  float obstacle{0.0F};
  float road_border{0.0F};
  float acceleration_command{0.0F};
  float steering_command{0.0F};
  float lateral_acceleration{0.0F};
  float lateral_jerk{0.0F};
  float longitudinal_jerk{0.0F};
  float steering_rate{0.0F};
  float initial_steering_rate{0.0F};
  float acceleration_command_rate{0.0F};
  float steering_command_rate{0.0F};
  float kinematic_velocity_overlimit{0.0F};
  float kinematic_acceleration_overlimit{0.0F};
  float kinematic_jerk_overlimit{0.0F};
  float running_total{0.0F};
  float terminal_total{0.0F};
  float total{0.0F};
  /** Signed cross-track at the first post-step state [m]; + = left of path tangent. */
  float signed_lateral_error_m{0.0F};
  std::size_t evaluated_timesteps{0U};

  [[nodiscard]] float componentTotal() const
  {
    return spatial_overspeed + track + heading + terminal_error + terminal_heading +
           lateral_distance + lateral_boundary + lateral_yaw_error + remaining_distance +
           path_overshoot + track_center + corner_buffer + drivable_area + acceleration_command +
           steering_command + lateral_acceleration + lateral_jerk + longitudinal_jerk +
           steering_rate + initial_steering_rate + acceleration_command_rate +
           steering_command_rate + kinematic_velocity_overlimit + kinematic_acceleration_overlimit +
           kinematic_jerk_overlimit + obstacle + road_border;
  }
};

enum class FirstOrderDubinsMppiInvalidityReason : std::uint8_t {
  none = 0U,
  lateral_boundary = 1U << 0U,
  obstacle = 1U << 1U,
  road_border = 1U << 2U,
  reverse = 1U << 3U,
  nonfinite_state = 1U << 4U,
  no_eligible_rollouts = 1U << 5U,
  insufficient_progress = 1U << 6U,
};

inline std::string to_string(FirstOrderDubinsMppiInvalidityReason reason)
{
  if (reason == FirstOrderDubinsMppiInvalidityReason::none) {
    return "none";
  }

  std::string result;
  const auto val = static_cast<std::uint8_t>(reason);

  if (val & static_cast<std::uint8_t>(FirstOrderDubinsMppiInvalidityReason::lateral_boundary)) {
    result += "lateral_boundary | ";
  }
  if (val & static_cast<std::uint8_t>(FirstOrderDubinsMppiInvalidityReason::obstacle)) {
    result += "obstacle | ";
  }
  if (val & static_cast<std::uint8_t>(FirstOrderDubinsMppiInvalidityReason::road_border)) {
    result += "road_border | ";
  }
  if (val & static_cast<std::uint8_t>(FirstOrderDubinsMppiInvalidityReason::reverse)) {
    result += "reverse | ";
  }
  if (val & static_cast<std::uint8_t>(FirstOrderDubinsMppiInvalidityReason::nonfinite_state)) {
    result += "nonfinite_state | ";
  }

  // Remove the trailing " | " if the string is not empty
  if (val & static_cast<std::uint8_t>(FirstOrderDubinsMppiInvalidityReason::no_eligible_rollouts)) {
    result += "no_eligible_rollouts | ";
  }
  if (
    val & static_cast<std::uint8_t>(FirstOrderDubinsMppiInvalidityReason::insufficient_progress)) {
    result += "insufficient_progress | ";
  }
  if (!result.empty()) {
    result.resize(result.size() - 3);
  } else {
    // Fallback for an unknown bit pattern
    result = "unknown(" + std::to_string(val) + ")";
  }

  return result;
}

constexpr FirstOrderDubinsMppiInvalidityReason operator|(
  const FirstOrderDubinsMppiInvalidityReason lhs, const FirstOrderDubinsMppiInvalidityReason rhs)
{
  return static_cast<FirstOrderDubinsMppiInvalidityReason>(
    static_cast<std::uint8_t>(lhs) | static_cast<std::uint8_t>(rhs));
}

constexpr bool hasInvalidityReason(
  const FirstOrderDubinsMppiInvalidityReason reasons,
  const FirstOrderDubinsMppiInvalidityReason reason)
{
  return (static_cast<std::uint8_t>(reasons) & static_cast<std::uint8_t>(reason)) != 0U;
}

struct FirstOrderDubinsMppiValidationResult
{
  /** Reasons detected at the first invalid trajectory point. */
  FirstOrderDubinsMppiInvalidityReason reasons{FirstOrderDubinsMppiInvalidityReason::none};
  std::optional<std::size_t> first_invalid_index;

  [[nodiscard]] bool isValid() const
  {
    return reasons == FirstOrderDubinsMppiInvalidityReason::none;
  }
};

struct FirstOrderDubinsMppiTiming
{
  /** Wall time for seedNominalControl (t-MPT / last-u / diffusion). */
  double seed_nominal_ms{0.0};
  /** Total optimizeTrajectory wall time. */
  double total_ms{0.0};
};

/** Post-step delay-bicycle state after u₀ (internal MPPI plant; for closed-loop sim feedback). */
struct FirstOrderDubinsMppiAppliedPlantState
{
  float x{0.0F};
  float y{0.0F};
  float yaw{0.0F};
  float velocity{0.0F};
  float acceleration{0.0F};
  float steering{0.0F};
  float sim_time{0.0F};
  std::vector<float> accel_cmd_delay_buffer;
  std::vector<float> steer_cmd_delay_buffer;
  FirstOrderDubinsMppiControl applied_control;
  bool valid{false};
};

struct FirstOrderDubinsMppiDebug
{
  Trajectory reference_trajectory;
  Trajectory optimized_trajectory;
  /** Open-loop rollout of the seeded u_nom warm-start (accel/steer cmds in a / front_wheel). */
  Trajectory nominal_trajectory;
  std::vector<std::pair<float, float>> optimal_horizon;
  std::vector<FirstOrderDubinsMppiRollout> rollouts;
  FirstOrderDubinsMppiNominalControlProfile nominal_control_profile;
  /** Cost of the pre-optimization nominal control rollout. */
  FirstOrderDubinsMppiCostBreakdown nominal_cost_breakdown;
  /** Cost of the final selected control rollout. */
  FirstOrderDubinsMppiCostBreakdown cost_breakdown;
  FirstOrderDubinsMppiTiming timing;
  FirstOrderDubinsMppiKinematicLimits active_kinematic_limits;
  float baseline_cost{0.0F};
  /** ESS for every MPPI optimization iteration in the most recent control step. */
  std::vector<float> iteration_effective_sample_sizes;
  /** Lambda used for those weights and the adapted value prepared for the next control step. */
  float lambda_used{0.0F};
  float lambda_next{0.0F};
  /** Maximum finite raw cost in the final rollout population. */
  float max_rollout_cost{0.0F};
  /** Robust upper raw cost used to normalize the final rollout population. */
  float normalization_upper_cost{0.0F};
  /** Fraction of final-iteration rollouts that encountered a collision/safety violation. */
  float unsafe_rollout_fraction{0.0F};
  int eligible_rollout_count{0};
  int minimum_cost_rollout_count{0};
  bool unsafe_rollout_population{false};
  /** Hard-constraint validation of the generated post-step states. */
  FirstOrderDubinsMppiValidationResult validation;
  /** True while the deterministic external-only maximum-velocity profile is applied. */
  bool external_velocity_limit_active{false};
  /** True while any deterministic external/map maximum-velocity profile is applied. */
  bool velocity_limit_profile_active{false};
  /** True when at least one valid map-derived pointwise maximum was supplied. */
  bool map_velocity_limit_active{false};
  /** Effective external/map minimum aligned with reference_trajectory.points. */
  std::vector<std::optional<float>> effective_max_velocity_by_reference_point;
  /** True when skip_if_invalid replaced the optimized trajectory with the input trajectory. */
  bool was_rejected{false};
  /** Internal plant after runStep(); use for sim feedback to avoid duplicate integration. */
  FirstOrderDubinsMppiAppliedPlantState applied_plant;
  /** Open-loop delay-bicycle replay vs measured ego since the previous MPPI cycle. */
  FirstOrderDubinsMppiPredictionAccuracy prediction_accuracy;
  FirstOrderDubinsMppiNominalSeedSource nominal_seed_source{
    FirstOrderDubinsMppiNominalSeedSource::diffusion_reference};
  FirstOrderDubinsMppiNominalResetReason nominal_reset_reason{
    FirstOrderDubinsMppiNominalResetReason::unavailable};
  int nominal_shift_count{0};
  FirstOrderDubinsMppiNominalSteeringContinuity nominal_steering_continuity;
  /** Whether the optimized MPPI trajectory was applied on the preceding plugin cycle. */
  bool previous_mppi_trajectory_applied{false};
  FirstOrderDubinsMppiMpcNominalSeedStatus mpc_nominal_seed_status{
    FirstOrderDubinsMppiMpcNominalSeedStatus::disabled};
};

struct FirstOrderDubinsMppiOptimizationResult
{
  Trajectory trajectory;
  FirstOrderDubinsMppiDebug debug;
  /** Number of leading points generated from the MPPI control horizon. */
  std::size_t optimized_point_count{0U};
};

/** Static 2D line segment supplied to the MPPI cost function in map coordinates. */
struct Segment
{
  float x0{0.0F};
  float y0{0.0F};
  float x1{0.0F};
  float y1{0.0F};
};

/**
 * @brief Host-side interface to the first-order Dubins MPPI controller used in the
 *        two-lane double-park path-tracking example.
 */
class FirstOrderDubinsMppiInterface
{
public:
  FirstOrderDubinsMppiInterface();
  ~FirstOrderDubinsMppiInterface();

  FirstOrderDubinsMppiInterface(const FirstOrderDubinsMppiInterface &) = delete;
  FirstOrderDubinsMppiInterface & operator=(const FirstOrderDubinsMppiInterface &) = delete;
  FirstOrderDubinsMppiInterface(FirstOrderDubinsMppiInterface &&) noexcept;
  FirstOrderDubinsMppiInterface & operator=(FirstOrderDubinsMppiInterface &&) noexcept;

  /** Initialize GPU resources and the two-lane double-park scenario. */
  void initialize();

  /** Whether initialize() completed successfully. */
  bool isInitialized() const;

  /** Configure vehicle geometry and limits from Autoware vehicle_info. */
  void setVehicleParams(const FirstOrderDubinsMppiVehicleParams & params);

  /** Configure MPPI cost weights (FirstOrderDubinsBicycleCostParams). */
  void setCostParams(const FirstOrderDubinsMppiCostParams & params);

  /** Configure debug logging and ablation options. */
  void setRuntimeOptions(const FirstOrderDubinsMppiRuntimeOptions & options);

  /**
   * @brief Optionally write reference/optimized trajectories for offline viz.
   * @param enable When true, each optimizeTrajectory writes CSVs under directory.
   * @param directory Output folder (created if missing). Ignored when enable is false.
   */
  void setDebugTrajectoryLogging(bool enable, const std::string & directory = "");

  /**
   * @brief Ablation options to mirror mppi_offline_retune conditions in online sim.
   * @param use_last_control_as_nominal When true, reuse a recent applied optimized control
   *        sequence while its plant replay and shifted reference remain continuous. Otherwise
   *        seed from the diffusion reference.
   */
  void setAblationOptions(
    const bool ignore_obstacles, const bool ignore_road_borders, const bool ignore_drivable_area,
    const bool force_cold_start_each_step, const bool skip_if_invalid,
    bool use_last_control_as_nominal = false);

  /**
   * @brief Copy per-rollout raw costs and normalized importance weights from the last
   *        optimizeTrajectory / computeStep call (for offline retune histograms).
   * @param stride Keep every N-th sample (1 = all rollouts). Use >1 to limit CSV size.
   */
  bool copySampleCostDistribution(
    std::vector<float> & raw_costs, std::vector<float> & normalized_weights, int stride = 1) const;

  /**
   * @brief Replace the per-iteration sampled debug rollouts with top-K weighted samples plus
   *        worst-K high-cost samples from the final iteration (CPU replay; ~tens of ms). Enable
   *        only for offline retune; leave false for online planning and debug trajectory logging.
   */
  void setRolloutVisualizationEnabled(bool enable);

  /**
   * @brief Force the next optimizeTrajectory / seedNominalControl to use this horizon as u_nom
   *        (offline retune replay of logged NNNNNN_nominal.csv). Cleared after one use.
   *        Sequences are truncated/padded to the MPPI horizon; values are clamped to vehicle
   * limits.
   */
  void setForcedNominalControl(
    const std::vector<float> & accel_cmd, const std::vector<float> & steer_cmd);

  /**
   * @brief Seed the last two applied commands tracked for debug CSV export.
   *        Offline retune only; vendor Savitzky–Golay no longer reads cross-cycle history.
   *        Order: (accel/steer) at t-2, then (accel/steer) at t-1.
   */
  void setControlHistory(float accel_tm2, float steer_tm2, float accel_tm1, float steer_tm1);

  /**
   * @brief Seed per-channel input-delay FIFOs with already-sent commands (oldest first).
   *        Accel uses the first N_acc samples; steer uses the first N_steer samples.
   *        Empty clears / disables forced seeding (falls back to measured hold).
   */
  void setInputDelayBuffer(
    const std::vector<float> & accel_cmd, const std::vector<float> & steer_cmd);

  /**
   * @brief Copy the last optimized control sequence (after optimizeTrajectory / computeStep).
   *        Used by offline retune to warm-start a subsequent MPPI pass (Re-seed).
   * @return false if the controller has not produced a control sequence yet.
   */
  bool copyLastOptimizedControl(
    std::vector<float> & accel_cmd, std::vector<float> & steer_cmd) const;

  /**
   * @brief Run one MPPI control step and propagate the vehicle state forward.
   * @param state Current ego state (updated in place).
   * @param sim_time Current simulation time [s].
   */
  FirstOrderDubinsMppiControl computeStep(FirstOrderDubinsMppiState & state, float sim_time);

  /**
   * @brief Track a diffusion-planner reference (poses + velocities) with one MPPI step.
   *
   * Uses the diffusion trajectory directly as the MPPI reference horizon (x, y, yaw, v).
   * When use_temporal_mpt_as_nominal is set, seeds u_nom from acados t-MPT (self warm-start).
   * Otherwise seeds from the previous optimized controls when use_last_control_as_nominal is set
   * (else from the reference trajectory). Returns the MPPI-predicted feasible state rollout.
   *
   * @param input Reference trajectory from the diffusion planner (map frame).
   * @param odometry Current ego odometry in the same frame as the trajectory.
   * @param acceleration Optional ego longitudinal acceleration [m/s^2] in base_link.
   * @param steering_status Optional ego tire steering angle [rad] from vehicle status.
   * @param tracked_objects Perception tracked objects used as dynamic obstacles
   * (constant-velocity).
   * @param road_borders Static road-border segments used by the gradual optimizer cost and hard
   *        output validator.
   * @param drivable_area Static drivable-area boundary segments used as a gradual constraint.
   * @param kinematic_limits Optional external scalar and map pointwise velocity bounds, plus
   *        external acceleration and jerk bounds.
   * @param control_postprocessor Optional caller-owned conditioning applied to the optimized
   *        control horizon before state recomputation and applied-control bookkeeping. The second
   *        callback argument reports the nominal source and shift count. Its steering-preservation
   *        flag is true only when optimized u[0] still equals the already-filtered shifted steering
   *        seed, so an output filter can avoid filtering it twice.
   * @param defer_commit Require commitPendingTrajectory() before recording candidate commands
   *        in the accepted execution history; use this when the caller can reject or shadow output.
   * @param mpc_predicted_trajectory Optional path whose geometry supplies the nominal steering
   *        prefix. Eligibility based on previous application and freshness is decided by the
   * caller.
   * @throws std::length_error If non-ignored scene geometry exceeds GPU/validator capacity.
   */
  FirstOrderDubinsMppiOptimizationResult optimizeTrajectory(
    const Trajectory & input, const Odometry & odometry,
    const std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> & acceleration,
    const std::optional<autoware_vehicle_msgs::msg::SteeringReport> & steering_status,
    const TrackedObjects & tracked_objects, const std::vector<Segment> & road_borders,
    const std::vector<Segment> & drivable_area,
    const FirstOrderDubinsMppiKinematicLimits & kinematic_limits = {},
    const FirstOrderDubinsMppiControlSequencePostprocessor & control_postprocessor = {},
    bool defer_commit = false,
    const std::optional<Trajectory> & mpc_predicted_trajectory = std::nullopt);

  /** Commit the most recent deferred, non-rejected candidate only after accepting its output.
   * This records an assumed first actuator command, not feedback from the downstream controller.
   * Callers with actual command history should supply that history through the existing seed APIs.
   * A subsequent optimizeTrajectory call discards an uncommitted candidate automatically.
   * Changes to parameters, ablations, or control/delay seeds also discard it.
   */
  void commitPendingTrajectory();
  void discardPendingTrajectory() noexcept;

  /** Prevent reuse of an accepted horizon after execution switches to another trajectory. */
  void invalidateNominalWarmStart() noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_INTERFACE_HPP_
