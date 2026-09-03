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

#include "autoware/mppi_optimizer/detail/plant_prediction_utils.hpp"
#include "autoware/mppi_optimizer/detail/temporal_mpt_nominal.hpp"
#include "autoware/mppi_optimizer/detail/trajectory_utils.hpp"
#include "autoware/mppi_optimizer/detail/trajectory_validator.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"
#include "autoware/mppi_optimizer/mppi_debug_trajectory_logger.hpp"
#include "autoware/mppi_optimizer/tracked_objects_obstacles.hpp"

#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh>
#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost_bridge.hpp>
#include <mppi/cost_functions/moving_car_obstacles.hpp>
#include <mppi/dynamics/dubins/first_order_dubins_bicycle.cuh>
#include <mppi/feedback_controllers/zero_feedback.cuh>
#include <mppi/sampling_distributions/colored_noise/colored_noise.cuh>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>
#include <mppi/sampling_distributions/smooth-MPPI/smooth-MPPI.cuh>
#include <mppi/utils/gpu_err_chk.cuh>
#include <rclcpp/logging.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::mppi_optimizer
{
namespace
{
constexpr int kMppiHorizon = detail::kMppiHorizon;
constexpr int kRefHorizon = kMppiHorizon;
constexpr float kDt = detail::kMppiDt;
constexpr int kNumRollouts = 16 * 1024;
constexpr int kSampledDebugRollouts = 128;
constexpr int kMaxVizRollouts = 256;
constexpr int kMaxWorstVizRollouts = 128;
constexpr char kLoggerName[] = "first_order_dubins_mppi";

rclcpp::Logger mppiLogger()
{
  return rclcpp::get_logger(kLoggerName);
}

bool isValidSignedInterval(const std::optional<float> & lower, const std::optional<float> & upper)
{
  return lower && upper && std::isfinite(*lower) && std::isfinite(*upper) && *lower <= 0.0F &&
         *upper >= 0.0F && *lower <= *upper;
}

FirstOrderDubinsMppiKinematicLimits sanitizeKinematicLimits(
  const FirstOrderDubinsMppiKinematicLimits & input)
{
  FirstOrderDubinsMppiKinematicLimits result;
  if (input.max_velocity && std::isfinite(*input.max_velocity) && *input.max_velocity >= 0.0F) {
    result.max_velocity = input.max_velocity;
  }
  result.max_velocity_by_reference_point.reserve(input.max_velocity_by_reference_point.size());
  for (const auto & maximum : input.max_velocity_by_reference_point) {
    result.max_velocity_by_reference_point.push_back(
      maximum && std::isfinite(*maximum) && *maximum >= 0.0F ? maximum : std::nullopt);
  }
  if (isValidSignedInterval(
        input.min_longitudinal_acceleration, input.max_longitudinal_acceleration)) {
    result.min_longitudinal_acceleration = input.min_longitudinal_acceleration;
    result.max_longitudinal_acceleration = input.max_longitudinal_acceleration;
  }
  if (isValidSignedInterval(input.min_longitudinal_jerk, input.max_longitudinal_jerk)) {
    result.min_longitudinal_jerk = input.min_longitudinal_jerk;
    result.max_longitudinal_jerk = input.max_longitudinal_jerk;
  }
  return result;
}

FirstOrderDubinsBicycleKinematicLimitData makeKinematicLimitCostData(
  const FirstOrderDubinsMppiKinematicLimits & limits)
{
  FirstOrderDubinsBicycleKinematicLimitData result;
  if (limits.max_velocity) {
    result.active_mask |= kVelocityLimitActive;
    result.min_velocity = 0.0F;
    result.max_velocity = *limits.max_velocity;
  }
  if (limits.min_longitudinal_acceleration && limits.max_longitudinal_acceleration) {
    result.active_mask |= kAccelerationLimitActive;
    result.min_longitudinal_acceleration = *limits.min_longitudinal_acceleration;
    result.max_longitudinal_acceleration = *limits.max_longitudinal_acceleration;
  }
  if (limits.min_longitudinal_jerk && limits.max_longitudinal_jerk) {
    result.active_mask |= kJerkLimitActive;
    result.min_longitudinal_jerk = *limits.min_longitudinal_jerk;
    result.max_longitudinal_jerk = *limits.max_longitudinal_jerk;
  }
  return result;
}

using DYN = FirstOrderDubinsBicycle;
using COST = FirstOrderDubinsBicycleCost<kRefHorizon>;
using FB = ZeroFeedback<DYN, kMppiHorizon>;

#define USE_COLOURED_NOISE

#ifdef USE_COLOURED_NOISE
using SAMPLER = mppi::sampling_distributions::ColoredNoiseDistribution<DYN::DYN_PARAMS_T>;
#elif defined(USE_SMOOTH_MPPI)
using SAMPLER = mppi::sampling_distributions::SmoothMPPIDistribution<DYN::DYN_PARAMS_T>;
#else
using SAMPLER = mppi::sampling_distributions::GaussianDistribution<DYN::DYN_PARAMS_T>;
#endif

using Mppi = VanillaMPPIController<DYN, COST, FB, kMppiHorizon, kNumRollouts, SAMPLER>;
using CostBreakdown = FirstOrderDubinsMppiCostBreakdown;

constexpr std::array<float CostBreakdown::*, 28> kCostBreakdownFields = {
  &CostBreakdown::spatial_overspeed,
  &CostBreakdown::track,
  &CostBreakdown::heading,
  &CostBreakdown::terminal_error,
  &CostBreakdown::terminal_heading,
  &CostBreakdown::lateral_distance,
  &CostBreakdown::lateral_boundary,
  &CostBreakdown::lateral_yaw_error,
  &CostBreakdown::remaining_distance,
  &CostBreakdown::path_overshoot,
  &CostBreakdown::track_center,
  &CostBreakdown::corner_buffer,
  &CostBreakdown::drivable_area,
  &CostBreakdown::obstacle,
  &CostBreakdown::road_border,
  &CostBreakdown::acceleration_command,
  &CostBreakdown::steering_command,
  &CostBreakdown::lateral_acceleration,
  &CostBreakdown::lateral_jerk,
  &CostBreakdown::longitudinal_jerk,
  &CostBreakdown::steering_rate,
  &CostBreakdown::initial_steering_rate,
  &CostBreakdown::kinematic_velocity_overlimit,
  &CostBreakdown::kinematic_acceleration_overlimit,
  &CostBreakdown::kinematic_jerk_overlimit,
  &CostBreakdown::running_total,
  &CostBreakdown::terminal_total,
  &CostBreakdown::total,
};

void accumulateCostBreakdown(CostBreakdown & total, const CostBreakdown & value)
{
  for (const auto field : kCostBreakdownFields) {
    total.*field += value.*field;
  }
}

void scaleCostBreakdown(CostBreakdown & breakdown, const float scale)
{
  for (const auto field : kCostBreakdownFields) {
    breakdown.*field *= scale;
  }
}

CostBreakdown reconstructControlTrajectoryCost(
  const COST & cost, DYN & model, const DYN::state_array & initial_state,
  const Mppi::control_trajectory & controls)
{
  CostBreakdown result;
  const int horizon = std::min(kMppiHorizon, static_cast<int>(controls.cols()));
  if (horizon <= 0) {
    return result;
  }

  int crash_status = 0;
  DYN::state_array state = initial_state;
  DYN::state_array next_state = DYN::state_array::Zero();
  DYN::state_array state_derivative = DYN::state_array::Zero();
  DYN::output_array output = DYN::output_array::Zero();
  for (int timestep = 0; timestep < horizon; ++timestep) {
    // Replay the same constrained, post-step outputs used by the GPU rollout, including x[H].
    DYN::control_array control = controls.col(timestep);
    model.enforceConstraints(state, control);
    model.step(
      state, next_state, state_derivative, control, output, static_cast<float>(timestep), kDt);
    if (timestep == 0) {
      using OutputIndex = FirstOrderDubinsBicycleParams::OutputIndex;
      result.signed_lateral_error_m = cost
                                        .computeLateralPathMetrics(
                                          output(static_cast<int>(OutputIndex::BASELINK_POS_I_X)),
                                          output(static_cast<int>(OutputIndex::BASELINK_POS_I_Y)),
                                          output(static_cast<int>(OutputIndex::YAW)))
                                        .lateral_distance;
    }
    accumulateCostBreakdown(
      result, cost.computeRunningCostBreakdown(output, control, timestep, &crash_status));
    state = next_state;
  }

  // The GPU applies terminalCost() to the final post-step output and divides both running and
  // terminal costs by the configured horizon.
  accumulateCostBreakdown(result, cost.computeTerminalCostBreakdown(output));

  // MPPI-Generic stores the horizon-average of both running and terminal costs.
  scaleCostBreakdown(result, 1.0F / static_cast<float>(horizon));
  result.evaluated_timesteps = static_cast<std::size_t>(horizon);
  return result;
}

Mppi::control_trajectory makeNominalControlTrajectory(
  const std::vector<float> & acceleration_commands, const std::vector<float> & steering_commands)
{
  Mppi::control_trajectory controls = Mppi::control_trajectory::Zero();
  const int horizon = std::min(
    {kMppiHorizon, static_cast<int>(acceleration_commands.size()),
     static_cast<int>(steering_commands.size())});
  const int accel_idx =
    static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
  const int steer_idx = static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);
  for (int timestep = 0; timestep < horizon; ++timestep) {
    controls(accel_idx, timestep) = acceleration_commands[static_cast<std::size_t>(timestep)];
    controls(steer_idx, timestep) = steering_commands[static_cast<std::size_t>(timestep)];
  }
  return controls;
}

std::string formatCostBreakdown(const CostBreakdown & cost)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << "{timesteps=" << cost.evaluated_timesteps
         << ", total=" << cost.total << ", running=" << cost.running_total
         << ", terminal=" << cost.terminal_total << ", spatial_overspeed=" << cost.spatial_overspeed
         << ", track=" << cost.track << ", heading=" << cost.heading
         << ", terminal_error=" << cost.terminal_error
         << ", terminal_heading=" << cost.terminal_heading
         << ", lateral_distance=" << cost.lateral_distance
         << ", lateral_boundary=" << cost.lateral_boundary
         << ", lateral_yaw=" << cost.lateral_yaw_error << ", remaining=" << cost.remaining_distance
         << ", overshoot=" << cost.path_overshoot << ", track_center=" << cost.track_center
         << ", corner=" << cost.corner_buffer << ", drivable=" << cost.drivable_area
         << ", obstacle=" << cost.obstacle << ", road_border=" << cost.road_border
         << ", accel_cmd=" << cost.acceleration_command << ", steer_cmd=" << cost.steering_command
         << ", lateral_accel=" << cost.lateral_acceleration
         << ", lateral_jerk=" << cost.lateral_jerk
         << ", longitudinal_jerk=" << cost.longitudinal_jerk
         << ", steer_rate=" << cost.steering_rate
         << ", initial_steer_rate=" << cost.initial_steering_rate
         << ", velocity_overlimit=" << cost.kinematic_velocity_overlimit
         << ", acceleration_overlimit=" << cost.kinematic_acceleration_overlimit
         << ", jerk_overlimit=" << cost.kinematic_jerk_overlimit << '}';
  return stream.str();
}

/** Expose vendor Savitzky–Golay control_history_ for offline retune parity. */
class MppiWithHistoryAccess : public Mppi
{
public:
  using Mppi::Mppi;

  struct IterationRolloutSnapshot
  {
    int iteration{0};
    std::vector<float> controls;
    std::vector<float> costs;
  };

  void computeControl(
    const Eigen::Ref<const Mppi::state_array> & state, const int optimization_stride = 1) override
  {
    if (iteration_rollout_capture_enabled_) {
      // Retain each snapshot's vector capacity across planning cycles.
      iteration_rollout_snapshots_.resize(static_cast<std::size_t>(this->getNumIters()));
    } else {
      iteration_rollout_snapshots_.clear();
    }
    Mppi::computeControl(state, optimization_stride);

    // Preserve this interface's existing unsmoothed-control behavior without downloading the
    // device mean a second time. Vanilla MPPI snapshots the final mean before its generic host
    // smoothing and zero-state constraint pass.
    this->control_ = this->getDeviceOptimalControlSequence();

    // First reconstruct the predicted state at each command. Constraints such as reverse
    // prevention depend on velocity, so applying them against the vendor's zero_state would erase
    // every braking command. Then constrain the stored controls against their corresponding
    // predicted states and replay once more to keep control_, state_, and output_ consistent.
    this->computeStateTrajectory(state);
    for (int timestep = 0; timestep < this->getNumTimesteps(); ++timestep) {
      this->model_->enforceConstraints(this->state_.col(timestep), this->control_.col(timestep));
    }
    this->computeStateTrajectory(state);
  }

  const std::vector<MppiWithHistoryAccess::IterationRolloutSnapshot> & iterationRolloutSnapshots()
    const
  {
    return iteration_rollout_snapshots_;
  }

  void setIterationRolloutCaptureEnabled(const bool enable)
  {
    iteration_rollout_capture_enabled_ = enable;
    if (!enable) {
      iteration_rollout_snapshots_.clear();
    }
  }

  /** Replace the optimized sequence and keep its host state rollout consistent. */
  void setControlSequenceAndRecomputeState(
    const Mppi::control_trajectory & controls, const Mppi::state_array & initial_state)
  {
    this->control_ = controls;
    this->computeStateTrajectory(initial_state);
  }

  void setControlHistory(
    const float accel_tm2, const float steer_tm2, const float accel_tm1, const float steer_tm1)
  {
    const int accel_idx =
      static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
    const int steer_idx = static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);
    this->control_history_(accel_idx, 0) = accel_tm2;
    this->control_history_(steer_idx, 0) = steer_tm2;
    this->control_history_(accel_idx, 1) = accel_tm1;
    this->control_history_(steer_idx, 1) = steer_tm1;
  }

  void copyControlHistory(
    float & accel_tm2, float & steer_tm2, float & accel_tm1, float & steer_tm1) const
  {
    const int accel_idx =
      static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
    const int steer_idx = static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);
    accel_tm2 = this->control_history_(accel_idx, 0);
    steer_tm2 = this->control_history_(steer_idx, 0);
    accel_tm1 = this->control_history_(accel_idx, 1);
    steer_tm1 = this->control_history_(steer_idx, 1);
  }

protected:
  bool requiresRawRolloutCostsForIteration() const override
  {
    return iteration_rollout_capture_enabled_;
  }

  void optimizationIterationComplete(const int iteration) override
  {
    if (
      !iteration_rollout_capture_enabled_ || iteration < 0 ||
      static_cast<std::size_t>(iteration) >= iteration_rollout_snapshots_.size()) {
      return;
    }
    const int rollout_count = std::min(kSampledDebugRollouts, kNumRollouts);
    const std::size_t controls_per_rollout =
      static_cast<std::size_t>(kMppiHorizon) * static_cast<std::size_t>(DYN::CONTROL_DIM);

    auto & snapshot = iteration_rollout_snapshots_[static_cast<std::size_t>(iteration)];
    snapshot.iteration = iteration + 1;
    snapshot.controls.resize(static_cast<std::size_t>(rollout_count) * controls_per_rollout);
    snapshot.costs.resize(static_cast<std::size_t>(rollout_count));

    // SamplingDistribution stores [rollout][timestep][control] contiguously. Copying the first
    // debug subset as one block avoids the per-rollout cudaMemcpyAsync calls used by the vendor
    // visualization staging path.
    HANDLE_ERROR(cudaMemcpyAsync(
      snapshot.controls.data(), this->sampler_->getControlSample(0, 0, 0),
      snapshot.controls.size() * sizeof(float), cudaMemcpyDeviceToHost, this->stream_));
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));
    std::copy_n(this->trajectory_costs_.data(), rollout_count, snapshot.costs.data());
  }

private:
  bool iteration_rollout_capture_enabled_{false};
  std::vector<IterationRolloutSnapshot> iteration_rollout_snapshots_;
};

void applyUserCostParams(
  FirstOrderDubinsBicycleCostParams<kRefHorizon> & cost_params,
  const FirstOrderDubinsMppiCostParams & user)
{
  cost_params.spatial_overspeed_coeff = user.spatial_overspeed_coeff;
  cost_params.track_coeff = user.track_coeff;
  cost_params.track_terminal_scale = user.track_terminal_scale;
  cost_params.terminal_error_coeff = user.terminal_error_coeff;
  cost_params.terminal_heading_coeff = user.terminal_heading_coeff;
  cost_params.heading_coeff = user.heading_coeff;
  cost_params.lateral_distance_coeff = user.lateral_distance_coeff;
  cost_params.lateral_yaw_error_coeff = user.lateral_yaw_error_coeff;
  cost_params.remaining_distance_coeff = user.remaining_distance_coeff;
  cost_params.path_overshoot_coeff = user.path_overshoot_coeff;
  cost_params.track_center_coeff = user.track_center_coeff;
  cost_params.corner_buffer_coeff = user.corner_buffer_coeff;
  cost_params.corner_safe_margin = user.corner_safe_margin;
  cost_params.boundary_threshold = user.boundary_threshold;
  cost_params.lateral_boundary_soft_margin = user.lateral_boundary_soft_margin;
  const auto calc_weight = [](float penalty, float soft_margin) {
    // calculate the weight such that the cost is 0 at the start of the soft_margin and "penalty" at
    // the hard margin
    const float m = std::max(soft_margin, 1.0E-3F);
    return penalty / (m * m);
  };
  cost_params.lateral_boundary_barrier_weight =
    calc_weight(user.crash_contact_penalty, user.lateral_boundary_soft_margin);
  cost_params.accel_cmd_coeff = user.accel_cmd_coeff;
  cost_params.steer_cmd_coeff = user.steer_cmd_coeff;
  cost_params.steer_rate_coeff = user.steer_rate_coeff;
  cost_params.initial_steer_rate_coeff = user.initial_steer_rate_coeff;
  cost_params.overlimit_coeff = user.overlimit_coeff;
  cost_params.lateral_acceleration_coeff = user.lateral_acceleration_coeff;
  cost_params.lateral_jerk_coeff = user.lateral_jerk_coeff;
  cost_params.longitudinal_jerk_coeff = user.longitudinal_jerk_coeff;
  cost_params.obstacle_collision_margin = user.obstacle_collision_margin;
  cost_params.road_border_collision_margin = user.road_border_collision_margin;
  cost_params.obstacle_safe_margin = user.obstacle_safe_margin;
  cost_params.obstacle_barrier_weight =
    calc_weight(user.crash_contact_penalty, user.obstacle_safe_margin);
  cost_params.road_border_safe_margin = user.road_border_safe_margin;
  cost_params.road_border_barrier_weight =
    calc_weight(user.crash_contact_penalty, user.road_border_safe_margin);
  cost_params.drivable_area_safe_margin = user.drivable_area_safe_margin;
  cost_params.drivable_area_barrier_weight = user.drivable_area_barrier_weight;
  cost_params.crash_contact_penalty = user.crash_contact_penalty;
}

FirstOrderDubinsMppiState toHostState(const DYN::state_array & x)
{
  FirstOrderDubinsMppiState state;
  state.x = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X));
  state.y = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y));
  state.yaw = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW));
  state.vel_x = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X));
  return state;
}

void fromHostState(DYN::state_array & x, const FirstOrderDubinsMppiState & state)
{
  x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X)) = state.x;
  x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y)) = state.y;
  x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW)) = state.yaw;
  x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X)) = state.vel_x;
}

float yawFromOdometry(const Odometry & odometry)
{
  return static_cast<float>(tf2::getYaw(odometry.pose.pose.orientation));
}

float longitudinalAccelerationMps2(
  const std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> & acceleration)
{
  if (!acceleration.has_value()) {
    return 0.0F;
  }
  return static_cast<float>(acceleration->accel.accel.linear.x);
}

float steeringTireAngleRad(
  const std::optional<autoware_vehicle_msgs::msg::SteeringReport> & steering_status)
{
  if (!steering_status.has_value()) {
    return 0.0F;
  }
  return steering_status->steering_tire_angle;
}

/**
 * Sampled ZOH dead-time length at time t:
 *   N(t) = t/dt - floor((t - τ)/dt)
 * Prefer floor (not C int truncation) so negative intermediates stay correct.
 * On an exact grid t = n·dt this equals ceil(τ/dt).
 */
int sampledDelaySteps(const float t, const float delay_s, const float dt)
{
  if (delay_s <= 0.0F || dt <= 0.0F) {
    return 0;
  }
  const double steps = static_cast<double>(t) / static_cast<double>(dt) -
                       std::floor(static_cast<double>(t - delay_s) / static_cast<double>(dt));
  const int n = static_cast<int>(std::lround(steps));
  return clampInputDelaySteps(std::max(0, n));
}

void checkCuda(const char * where)
{
  const cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    throw std::runtime_error(
      std::string("MPPI CUDA error at ") + where + ": " + cudaGetErrorString(err));
  }
}

/** Open-loop integrate logged u_nom from the optimization IC; post-step states aligned to DP. */
Trajectory buildNominalTrajectory(
  DYN & model, const DYN::state_array & x0, const Trajectory & input,
  const std::vector<float> & accel_cmd, const std::vector<float> & steer_cmd)
{
  const size_t n = std::min(
    {input.points.size(), accel_cmd.size(), steer_cmd.size(),
     static_cast<size_t>(std::max(0, kMppiHorizon))});
  if (n == 0U) {
    return Trajectory{};
  }

  const int pos_x_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X);
  const int pos_y_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y);
  const int yaw_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW);
  const int vel_x_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X);
  const int accel_state_idx =
    static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::ACCELERATION);
  const int accel_idx =
    static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
  const int steer_idx = static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);

  std::vector<detail::OptimizedState> states;
  std::vector<FirstOrderDubinsMppiControl> controls;
  states.reserve(n);
  controls.reserve(n);

  DYN::state_array x = x0;
  DYN::state_array x_next = model.getZeroState();
  DYN::state_array xdot = model.getZeroState();
  DYN::output_array y = DYN::output_array::Zero();
  DYN::control_array u = DYN::control_array::Zero();

  for (size_t i = 0; i < n; ++i) {
    FirstOrderDubinsMppiControl cmd;
    cmd.accel_cmd = accel_cmd[i];
    cmd.steer_cmd = steer_cmd[i];
    u(accel_idx) = cmd.accel_cmd;
    u(steer_idx) = cmd.steer_cmd;
    model.enforceConstraints(x, u);
    model.step(x, x_next, xdot, u, y, static_cast<float>(i), kDt);
    x = x_next;

    detail::OptimizedState state;
    state.x = x(pos_x_idx);
    state.y = x(pos_y_idx);
    state.yaw = x(yaw_idx);
    state.velocity = x(vel_x_idx);
    state.acceleration = x(accel_state_idx);
    // Publish steer_cmd (not tire angle) so the live viz δ_cmd panel shows u_nom.
    state.steering = cmd.steer_cmd;
    states.push_back(state);
    controls.push_back(cmd);
  }
  return detail::buildOptimizedTrajectory(input, states, controls);
}

void replayRolloutPoints(
  DYN & model, const DYN::state_array & x0, const float * controls, const int horizon,
  const float dt, std::vector<std::pair<float, float>> & points)
{
  const int pos_x_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X);
  const int pos_y_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y);

  points.clear();
  points.reserve(static_cast<size_t>(horizon) + 1U);
  points.emplace_back(x0(pos_x_idx), x0(pos_y_idx));

  DYN::state_array x = x0;
  DYN::state_array x_next = model.getZeroState();
  DYN::state_array xdot = model.getZeroState();
  DYN::output_array y = DYN::output_array::Zero();
  DYN::control_array u = DYN::control_array::Zero();

  for (int t = 0; t < horizon; ++t) {
    for (int d = 0; d < DYN::CONTROL_DIM; ++d) {
      u(d) = controls
        [(static_cast<size_t>(t) * static_cast<size_t>(DYN::CONTROL_DIM)) + static_cast<size_t>(d)];
    }
    model.enforceConstraints(x, u);
    model.step(x, x_next, xdot, u, y, static_cast<float>(t), dt);
    points.emplace_back(x_next(pos_x_idx), x_next(pos_y_idx));
    x = x_next;
  }
}

void fillOptimalHorizonPoints(
  const Mppi::state_trajectory & state_trajectory, std::vector<std::pair<float, float>> & points)
{
  const int pos_x_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X);
  const int pos_y_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y);
  const int horizon = static_cast<int>(state_trajectory.cols());

  points.clear();
  points.reserve(static_cast<size_t>(horizon));
  for (int col = 0; col < horizon; ++col) {
    points.emplace_back(state_trajectory(pos_x_idx, col), state_trajectory(pos_y_idx, col));
  }
}

void copySamplerControlsToHost(
  SAMPLER & sampler, const int horizon, const std::vector<int> & rollout_indices,
  std::vector<float> & host_controls)
{
  const size_t rollout_stride =
    static_cast<size_t>(horizon) * static_cast<size_t>(DYN::CONTROL_DIM) * sizeof(float);
  host_controls.assign(
    rollout_indices.size() * static_cast<size_t>(horizon) * static_cast<size_t>(DYN::CONTROL_DIM),
    0.0F);

  size_t out_idx = 0;
  for (const int rollout : rollout_indices) {
    float * device_controls = sampler.getControlSample(rollout, 0, 0);
    HANDLE_ERROR(cudaMemcpy(
      host_controls.data() +
        out_idx * static_cast<size_t>(horizon) * static_cast<size_t>(DYN::CONTROL_DIM),
      device_controls, rollout_stride, cudaMemcpyDeviceToHost));
    ++out_idx;
  }
}

void selectTopRolloutIndices(
  const std::vector<float> & normalized_weights, const std::vector<float> & raw_costs,
  const int top_n, std::vector<int> & rollout_indices, std::vector<float> & selected_costs)
{
  const int num_rollouts = static_cast<int>(normalized_weights.size());
  rollout_indices.resize(static_cast<size_t>(num_rollouts));
  selected_costs = raw_costs;
  std::iota(rollout_indices.begin(), rollout_indices.end(), 0);

  if (top_n <= 0 || top_n >= num_rollouts) {
    return;
  }

  std::partial_sort(
    rollout_indices.begin(), rollout_indices.begin() + top_n, rollout_indices.end(),
    [&](const int a, const int b) {
      return normalized_weights[static_cast<size_t>(a)] >
             normalized_weights[static_cast<size_t>(b)];
    });
  rollout_indices.resize(static_cast<size_t>(top_n));
  selected_costs.resize(static_cast<size_t>(top_n));
  std::transform(
    rollout_indices.begin(), rollout_indices.end(), selected_costs.begin(),
    [&](const int rollout) { return raw_costs[static_cast<size_t>(rollout)]; });
}

/** Highest raw-cost samples, excluding any indices already chosen as top-weighted. */
void selectWorstRolloutIndices(
  const std::vector<float> & raw_costs, const std::vector<int> & exclude_indices, const int worst_n,
  std::vector<int> & rollout_indices, std::vector<float> & selected_costs)
{
  rollout_indices.clear();
  selected_costs.clear();
  const int num_rollouts = static_cast<int>(raw_costs.size());
  if (worst_n <= 0 || num_rollouts <= 0) {
    return;
  }

  std::vector<char> excluded(static_cast<size_t>(num_rollouts), 0);
  for (const int idx : exclude_indices) {
    if (idx >= 0 && idx < num_rollouts) {
      excluded[static_cast<size_t>(idx)] = 1;
    }
  }

  std::vector<int> candidates;
  candidates.reserve(static_cast<size_t>(num_rollouts));
  for (int i = 0; i < num_rollouts; ++i) {
    if (!excluded[static_cast<size_t>(i)]) {
      candidates.push_back(i);
    }
  }
  if (candidates.empty()) {
    return;
  }

  const int keep = std::min(worst_n, static_cast<int>(candidates.size()));
  std::partial_sort(
    candidates.begin(), candidates.begin() + keep, candidates.end(), [&](const int a, const int b) {
      return raw_costs[static_cast<size_t>(a)] > raw_costs[static_cast<size_t>(b)];
    });
  candidates.resize(static_cast<size_t>(keep));
  rollout_indices = std::move(candidates);
  selected_costs.resize(rollout_indices.size());
  std::transform(
    rollout_indices.begin(), rollout_indices.end(), selected_costs.begin(),
    [&](const int rollout) { return raw_costs[static_cast<size_t>(rollout)]; });
}

void appendReplayedRollouts(
  DYN & model, const DYN::state_array & x0, const std::vector<float> & host_controls,
  const std::vector<float> & selected_costs, const bool is_worst,
  std::vector<FirstOrderDubinsMppiRollout> & rollouts)
{
  size_t out_idx = 0;
  for (const float cost : selected_costs) {
    FirstOrderDubinsMppiRollout rollout;
    rollout.cost = cost;
    rollout.is_worst = is_worst;
    const float * controls = host_controls.data() + out_idx * static_cast<size_t>(kMppiHorizon) *
                                                      static_cast<size_t>(DYN::CONTROL_DIM);
    replayRolloutPoints(model, x0, controls, kMppiHorizon, kDt, rollout.points);
    rollouts.push_back(std::move(rollout));
    ++out_idx;
  }
}

float rawCostFromMinMaxWeight(
  const float weight, const float min_cost, const float max_cost, const float lambda)
{
  const float cost_range = max_cost - min_cost;
  if (!std::isfinite(cost_range) || cost_range < 1.0E-6F) {
    return min_cost;
  }
  if (!(weight > 0.0F) || !std::isfinite(weight)) {
    return max_cost;
  }
  const float normalized_cost = std::clamp(-lambda * std::log(weight), 0.0F, 1.0F);
  return min_cost + normalized_cost * cost_range;
}

void buildRolloutVisualization(
  MppiWithHistoryAccess & controller, SAMPLER & sampler, DYN & model,
  const DYN::state_array & x_at_optimization, FirstOrderDubinsMppiDebug & debug)
{
  const Mppi::state_trajectory state_trajectory = controller.getActualStateSeq();
  fillOptimalHorizonPoints(state_trajectory, debug.optimal_horizon);
  debug.baseline_cost = controller.getBaselineCost();

  controller.downloadImportanceWeightsToHost();
  // IMPORTANT: take by value — see copySampleCostDistribution for nvcc temporary lifetime note.
  const Mppi::sampled_cost_traj importance = controller.getSampledCostSeq();
  const float min_cost = controller.getLastMinRolloutCost();
  const float max_cost = controller.getLastMaxRolloutCost();
  const float unnormalized_weight_sum = controller.getLastUnnormalizedWeightSum();
  const float lambda = controller.getLastWeightLambda();
  const int num_rollouts = static_cast<int>(importance.size());

  std::vector<float> raw_costs(static_cast<size_t>(num_rollouts));
  std::vector<float> normalized_weights(static_cast<size_t>(num_rollouts));
  const float normalizer = controller.getNormalizerCost();
  for (size_t i = 0; i < normalized_weights.size(); ++i) {
    const float w = static_cast<float>(importance(static_cast<int>(i)));
    normalized_weights[i] = (normalizer > 0.0F) ? w / normalizer : 0.0F;
    raw_costs[i] = rawCostFromMinMaxWeight(w * unnormalized_weight_sum, min_cost, max_cost, lambda);
  }

  std::vector<int> top_indices;
  std::vector<float> top_costs;
  selectTopRolloutIndices(normalized_weights, raw_costs, kMaxVizRollouts, top_indices, top_costs);

  std::vector<int> worst_indices;
  std::vector<float> worst_costs;
  selectWorstRolloutIndices(
    raw_costs, top_indices, kMaxWorstVizRollouts, worst_indices, worst_costs);

  std::vector<int> all_indices = top_indices;
  all_indices.insert(all_indices.end(), worst_indices.begin(), worst_indices.end());

  std::vector<float> host_controls;
  copySamplerControlsToHost(sampler, kMppiHorizon, all_indices, host_controls);

  debug.rollouts.clear();
  debug.rollouts.reserve(all_indices.size());
  const size_t top_ctrl_floats =
    top_indices.size() * static_cast<size_t>(kMppiHorizon) * static_cast<size_t>(DYN::CONTROL_DIM);
  const std::vector<float> top_controls(
    host_controls.begin(), host_controls.begin() + static_cast<std::ptrdiff_t>(std::min(
                                                     top_ctrl_floats, host_controls.size())));
  appendReplayedRollouts(model, x_at_optimization, top_controls, top_costs, false, debug.rollouts);
  if (!worst_indices.empty()) {
    const std::vector<float> worst_controls(
      host_controls.begin() + static_cast<std::ptrdiff_t>(top_ctrl_floats), host_controls.end());
    appendReplayedRollouts(
      model, x_at_optimization, worst_controls, worst_costs, true, debug.rollouts);
  }
}

void buildIterationDebugRollouts(
  const std::vector<MppiWithHistoryAccess::IterationRolloutSnapshot> & snapshots, DYN & model,
  const DYN::state_array & x_at_optimization, FirstOrderDubinsMppiDebug & debug)
{
  debug.rollouts.clear();
  debug.rollouts.reserve(snapshots.size() * static_cast<std::size_t>(kSampledDebugRollouts));
  const std::size_t controls_per_rollout =
    static_cast<std::size_t>(kMppiHorizon) * static_cast<std::size_t>(DYN::CONTROL_DIM);

  for (const auto & snapshot : snapshots) {
    const std::size_t rollout_count = std::min(
      snapshot.costs.size(),
      controls_per_rollout > 0U ? snapshot.controls.size() / controls_per_rollout : 0U);
    for (std::size_t rollout_index = 0; rollout_index < rollout_count; ++rollout_index) {
      FirstOrderDubinsMppiRollout rollout;
      rollout.cost = snapshot.costs[rollout_index];
      rollout.iteration = snapshot.iteration;
      rollout.is_worst = false;
      const float * controls = snapshot.controls.data() + rollout_index * controls_per_rollout;
      // Preserve the existing debug convention: initial state plus horizon - 1 post-step states.
      replayRolloutPoints(
        model, x_at_optimization, controls, kMppiHorizon - 1, kDt, rollout.points);
      debug.rollouts.push_back(std::move(rollout));
    }
  }
}

}  // namespace

struct FirstOrderDubinsMppiInterface::Impl
{
  Trajectory diffusion_reference;
  /** Cumulative chord length [m] along diffusion_reference.points (s[0]=0). */
  std::vector<float> diffusion_reference_chord_length_s;
  TrackedObjects tracked_objects;
  std::vector<Segment> road_borders;
  std::vector<Segment> drivable_area;
  std::vector<mppi::cost::MovingCarObstacle> obstacles;

  DYN model;
  FirstOrderDubinsBicycleParams dyn;
  FirstOrderDubinsMppiVehicleParams vehicle_params{};
  FirstOrderDubinsMppiCostParams user_cost_params_{};
  FirstOrderDubinsMppiKinematicLimits active_kinematic_limits{};
  std::vector<std::optional<float>> effective_max_velocity_by_reference_point;
  std::optional<float> uniform_effective_max_velocity;
  bool has_map_velocity_limit{false};
  detail::ActiveVelocityLimitProfile active_velocity_limit_profile;
  MppiDebugTrajectoryLogger debug_trajectory_logger;
  COST cost;
  FirstOrderDubinsBicycleCostParams<kRefHorizon> cost_params{};
  SAMPLER sampler;
  FB feedback;
  std::unique_ptr<MppiWithHistoryAccess> controller;
  Mppi::control_trajectory u_nom = Mppi::control_trajectory::Zero();
  Mppi::control_trajectory u_opt = Mppi::control_trajectory::Zero();
  DYN::state_array x = DYN::state_array::Zero();

  std::vector<float> obs_traj_x;
  std::vector<float> obs_traj_y;
  std::vector<float> obs_traj_yaw;
  std::vector<float> obs_half_length;
  std::vector<float> obs_half_width;
  std::vector<float> road_border_x0;
  std::vector<float> road_border_y0;
  std::vector<float> road_border_x1;
  std::vector<float> road_border_y1;
  std::vector<float> drivable_area_x0;
  std::vector<float> drivable_area_y0;
  std::vector<float> drivable_area_x1;
  std::vector<float> drivable_area_y1;

  bool initialized{false};
  bool road_border_capacity_warning_emitted{false};
  bool drivable_area_capacity_warning_emitted{false};
  int step_count{0};
  size_t tracking_start_idx{0U};
  float sim_time{0.0F};
  bool ignore_obstacles{false};
  bool ignore_road_borders{false};
  bool ignore_drivable_area{false};
  bool force_cold_start_each_step{false};
  bool skip_if_invalid{false};
  float min_optimization_length{0.0F};
  /** Warm-start u_nom from shifted previous u_opt when available. */
  bool use_last_control_as_nominal{false};
  /** Cold-seed u_nom from acados temporal MPT instead of geometric diffusion seed. */
  bool use_temporal_mpt_as_nominal{false};
  /** Prevent acceleration commands and integrated states from producing reverse velocity. */
  bool prevent_reverse_velocity{true};
  /** When false, force N_acc = N_steer = 0 (vehicle delay params ignored). */
  bool enable_input_delay_compensation{true};
  detail::TemporalMptNominalSeeder temporal_mpt_nominal_seeder;
  /** Fill debug.rollouts with top-K weighted samples (CPU replay). Offline retune only by default.
   */
  bool enable_rollout_visualization{false};
  /** Capture and replay a sampled rollout subset from every internal MPPI iteration. */
  bool enable_iteration_rollout_debug{false};
  /** One-shot u_nom override from offline retune (NNNNNN_nominal.csv). */
  bool forced_nominal_pending{false};
  std::vector<float> forced_nominal_accel;
  std::vector<float> forced_nominal_steer;
  /** Snapshot of u_nom after seeding, written to NNNNNN_nominal.csv. */
  std::vector<float> logged_nominal_accel;
  std::vector<float> logged_nominal_steer;
  /** Wall time of the most recent seedNominalControl call [ms]. */
  double last_seed_nominal_ms{0.0};

  /**
   * Per-channel discrete ZOH input delay (in dynamics taps, not host IC pre-roll):
   * N_acc(t) / N_steer(t) via sampledDelaySteps; FIFOs seed the delay pipes in x.
   */
  int acc_delay_steps{0};
  int steer_delay_steps{0};
  std::vector<float> accel_delay_buffer;
  std::vector<float> steer_delay_buffer;
  bool delay_buffer_seeded{false};

  /** Pending offline seeds applied after setup() / step_count==0 reset. */
  bool pending_control_history{false};
  float pending_hist_accel_tm2{0.0F};
  float pending_hist_steer_tm2{0.0F};
  float pending_hist_accel_tm1{0.0F};
  float pending_hist_steer_tm1{0.0F};
  bool pending_delay_buffer{false};
  std::vector<float> pending_delay_accel;
  std::vector<float> pending_delay_steer;

  /** Snapshots written to debug CSVs for exact offline replay. */
  float logged_hist_accel_tm2{0.0F};
  float logged_hist_steer_tm2{0.0F};
  float logged_hist_accel_tm1{0.0F};
  float logged_hist_steer_tm1{0.0F};
  std::vector<float> logged_delay_accel;
  std::vector<float> logged_delay_steer;
  float logged_applied_accel{0.0F};
  float logged_applied_steer{0.0F};

  /** Anchor plant IC + stamp captured before each runStep (for next-cycle prediction metric). */
  detail::FirstOrderDubinsMppiPlantSnapshot prediction_anchor_{};
  std::vector<detail::FirstOrderDubinsMppiControlHistoryEntry> prediction_control_history_{};

  Impl() : feedback(&model, kDt), sampler(SAMPLER::SAMPLING_PARAMS_T{}) {}

  void setup()
  {
    dyn = FirstOrderDubinsBicycleParams{};
    dyn.wheel_base = vehicle_params.wheel_base;
    dyn.max_steer_angle = vehicle_params.max_steer_angle;
    dyn.accel_time_constant = vehicle_params.acc_time_constant;
    dyn.steer_time_constant = vehicle_params.steer_time_constant;
    dyn.max_steer_rate = vehicle_params.steer_rate_lim;
    dyn.min_accel = vehicle_params.min_accel();
    dyn.max_accel = vehicle_params.max_accel();
    dyn.prevent_reverse_velocity = prevent_reverse_velocity;
    model.setParams(dyn);
    temporal_mpt_nominal_seeder.setBicycleParameters(
      vehicle_params.wheel_base, vehicle_params.ego_axle_to_box_center,
      vehicle_params.acc_time_constant, vehicle_params.steer_time_constant,
      vehicle_params.steer_rate_lim);

    acc_delay_steps = enable_input_delay_compensation
                        ? sampledDelaySteps(0.0F, vehicle_params.acc_time_delay, kDt)
                        : 0;
    steer_delay_steps = enable_input_delay_compensation
                          ? sampledDelaySteps(0.0F, vehicle_params.steer_time_delay, kDt)
                          : 0;
    dyn.acc_delay_steps = acc_delay_steps;
    dyn.steer_delay_steps = steer_delay_steps;
    model.setParams(dyn);
    accel_delay_buffer.clear();
    steer_delay_buffer.clear();
    delay_buffer_seeded = false;

    cost.GPUSetup();

    cost_params = FirstOrderDubinsBicycleCostParams<kRefHorizon>{};
    applyUserCostParams(cost_params, user_cost_params_);
    mppi::cost::fillFirstOrderDubinsBicycleCostGeometry<kRefHorizon>(cost_params, dyn);

    cost_params.ego_length = vehicle_params.ego_length;
    cost_params.ego_width = vehicle_params.ego_width;
    cost_params.ego_axle_to_box_center = vehicle_params.ego_axle_to_box_center;
    // Comfort-cost steer-rate clamp must match vehicle config (simulator_model steer_rate_lim).
    cost_params.max_steer_rate = vehicle_params.steer_rate_lim;
    cost.setParams(cost_params);

    const float kMaxSteer = dyn.max_steer_angle;
    std::array<float2, DYN::CONTROL_DIM> u_rng{};
    u_rng[static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD)] = {
      dyn.min_accel, dyn.max_accel};
    u_rng[static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD)] = {
      -kMaxSteer, kMaxSteer};
    model.setControlRanges(u_rng);

    SAMPLER::SAMPLING_PARAMS_T sp{};
    sp.std_dev[static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD)] =
      user_cost_params_.accel_cmd_std_dev;
    sp.std_dev[static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD)] =
      user_cost_params_.steer_cmd_std_dev;
    sp.std_dev_decay = user_cost_params_.std_dev_decay;
    sp.sum_strides = std::max(32, (kNumRollouts + 1023) / 1024);
#ifdef USE_COLOURED_NOISE
    sp.exponents[static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD)] =
      user_cost_params_.accel_cmd_noise_exponent;
    sp.exponents[static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD)] =
      user_cost_params_.steer_cmd_noise_exponent;
#elif defined(USE_SMOOTH_MPPI)
    // Smooth-MPPI samples action derivatives and integrates with dt.
    sp.dt = kDt;
#endif
    sampler = SAMPLER(sp);

    const float lambda = user_cost_params_.lambda;
    controller = std::make_unique<MppiWithHistoryAccess>(
      &model, &cost, &feedback, &sampler, kDt, user_cost_params_.max_iter, lambda, 0.0F,
      kMppiHorizon, u_nom);
    auto cp = controller->getParams();
    cp.lambda_ = lambda;
    cp.dynamics_rollout_dim_ = dim3(32, 2, 1);
    cp.cost_rollout_dim_ = dim3(32, 2, 1);
    cp.seed_ = 1U;
    controller->setParams(cp);
    controller->configureEssLambdaAdaptation(
      user_cost_params_.target_ess_ratio, user_cost_params_.lambda_adaptation_gain,
      user_cost_params_.lambda_min, user_cost_params_.lambda_max,
      user_cost_params_.unsafe_rollout_fraction_threshold,
      user_cost_params_.cost_normalization_percentile);
    controller->setIterationRolloutCaptureEnabled(
      enable_iteration_rollout_debug && !enable_rollout_visualization);
    // Per-iteration debug controls are captured as one contiguous block by
    // MppiWithHistoryAccess. Disable the vendor visualization staging buffers, whose
    // per-rollout copies only preserve the final optimization iteration.
    controller->setPercentageSampledControlTrajectories(0.0F);

    model.GPUSetup();

    initialized = true;
    step_count = 0;
    tracking_start_idx = 0U;
    sim_time = 0.0F;
    RCLCPP_INFO(
      mppiLogger(),
      "MPPI GPU initialized (horizon=%d, rollouts=%d, iterations=%d, dt=%.2f, lambda=%.3f, "
      "wheel_base=%.2f, max_steer=%.2f, accel_std=%.3f, steer_std=%.3f, "
      "std_decay=%.3f, acc_tau=%.2f, steer_tau=%.2f, "
      "acc_delay=%.3f (%d steps), steer_delay=%.3f (%d steps), "
      "steer_rate_lim=%.2f, vel_rate_lim=%.2f, ego=%.2fx%.2f, axle_to_center=%.2f, "
      "boundary_threshold=%.2f, obs_margin=%.2f, road_border_margin=%.2f, "
      "lateral_barrier=%.2f@%.2f, obs_barrier=%.2f@%.2f, road_barrier=%.2f@%.2f, "
      "drive_barrier=%.2f@%.2f, crash_contact_penalty=%.2f)",
      kMppiHorizon, kNumRollouts, user_cost_params_.max_iter, kDt, user_cost_params_.lambda,
      vehicle_params.wheel_base, vehicle_params.max_steer_angle,
      user_cost_params_.accel_cmd_std_dev, user_cost_params_.steer_cmd_std_dev,
      user_cost_params_.std_dev_decay, vehicle_params.acc_time_constant,
      vehicle_params.steer_time_constant, vehicle_params.acc_time_delay, acc_delay_steps,
      vehicle_params.steer_time_delay, steer_delay_steps, vehicle_params.steer_rate_lim,
      vehicle_params.vel_rate_lim, vehicle_params.ego_length, vehicle_params.ego_width,
      vehicle_params.ego_axle_to_box_center, cost_params.boundary_threshold,
      cost_params.obstacle_collision_margin, cost_params.road_border_collision_margin,
      cost_params.lateral_boundary_barrier_weight, cost_params.lateral_boundary_soft_margin,
      cost_params.obstacle_barrier_weight, cost_params.obstacle_safe_margin,
      cost_params.road_border_barrier_weight, cost_params.road_border_safe_margin,
      cost_params.drivable_area_barrier_weight, cost_params.drivable_area_safe_margin,
      cost_params.crash_contact_penalty);
  }

  void resetTrackingState()
  {
    step_count = 0;
    u_opt.setZero();
    sim_time = 0.0F;
    accel_delay_buffer.clear();
    steer_delay_buffer.clear();
    delay_buffer_seeded = false;
    temporal_mpt_nominal_seeder.resetWarmStart();
    prediction_anchor_.valid = false;
    prediction_control_history_.clear();
  }

  void syncDelayStepsToModel()
  {
    if (enable_input_delay_compensation) {
      acc_delay_steps = sampledDelaySteps(sim_time, vehicle_params.acc_time_delay, kDt);
      steer_delay_steps = sampledDelaySteps(sim_time, vehicle_params.steer_time_delay, kDt);
    } else {
      acc_delay_steps = 0;
      steer_delay_steps = 0;
    }
    dyn.acc_delay_steps = acc_delay_steps;
    dyn.steer_delay_steps = steer_delay_steps;
    model.setParams(dyn);
  }

  /// @brief Elevate active limits to hard rollout constraints. If no dynamic limit is active,
  /// fallback to the vehicle's physical hardware limits.
  void syncKinematicLimitsToModel()
  {
    dyn.min_accel =
      active_kinematic_limits.min_longitudinal_acceleration
        ? std::max(
            vehicle_params.min_accel(), *active_kinematic_limits.min_longitudinal_acceleration)
        : vehicle_params.min_accel();

    dyn.max_accel =
      active_kinematic_limits.max_longitudinal_acceleration
        ? std::min(
            vehicle_params.max_accel(), *active_kinematic_limits.max_longitudinal_acceleration)
        : vehicle_params.max_accel();

    model.setParams(dyn);
  }

  void resizeChannelDelayBuffer(std::vector<float> & buffer, const int n, const float hold)
  {
    if (n <= 0) {
      buffer.clear();
      return;
    }
    const size_t target = static_cast<size_t>(n);
    if (buffer.size() == target) {
      return;
    }
    if (buffer.size() < target) {
      buffer.insert(buffer.begin(), target - buffer.size(), hold);
    } else {
      buffer.erase(
        buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(buffer.size() - target));
    }
  }

  void ensureDelayBufferSeeded()
  {
    if (acc_delay_steps <= 0 && steer_delay_steps <= 0) {
      accel_delay_buffer.clear();
      steer_delay_buffer.clear();
      delay_buffer_seeded = true;
      return;
    }
    const float accel_hold =
      x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::ACCELERATION));
    const float steer_hold =
      x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::STEER_ANGLE));
    if (!delay_buffer_seeded) {
      if (acc_delay_steps > 0) {
        accel_delay_buffer.assign(static_cast<size_t>(acc_delay_steps), accel_hold);
      } else {
        accel_delay_buffer.clear();
      }
      if (steer_delay_steps > 0) {
        steer_delay_buffer.assign(static_cast<size_t>(steer_delay_steps), steer_hold);
      } else {
        steer_delay_buffer.clear();
      }
      delay_buffer_seeded = true;
      return;
    }
    resizeChannelDelayBuffer(accel_delay_buffer, acc_delay_steps, accel_hold);
    resizeChannelDelayBuffer(steer_delay_buffer, steer_delay_steps, steer_hold);
  }

  void loadDelayPipesIntoState()
  {
    using S = FirstOrderDubinsBicycleParams::StateIndex;
    for (int i = 0; i < FirstOrderDubinsBicycleParams::kMaxInputDelaySteps; ++i) {
      x(static_cast<int>(S::ACCEL_CMD_D0) + i) = 0.0F;
      x(static_cast<int>(S::STEER_CMD_D0) + i) = 0.0F;
    }
    for (int i = 0; i < acc_delay_steps; ++i) {
      x(static_cast<int>(S::ACCEL_CMD_D0) + i) = accel_delay_buffer[static_cast<size_t>(i)];
    }
    for (int i = 0; i < steer_delay_steps; ++i) {
      x(static_cast<int>(S::STEER_CMD_D0) + i) = steer_delay_buffer[static_cast<size_t>(i)];
    }
  }

  void applyPendingControlHistory()
  {
    if (!pending_control_history || !controller) {
      return;
    }
    controller->setControlHistory(
      pending_hist_accel_tm2, pending_hist_steer_tm2, pending_hist_accel_tm1,
      pending_hist_steer_tm1);
    pending_control_history = false;
  }

  void applyPendingDelayBuffer()
  {
    if (!pending_delay_buffer) {
      return;
    }
    pending_delay_buffer = false;
    if (pending_delay_accel.empty() && pending_delay_steer.empty()) {
      accel_delay_buffer.clear();
      steer_delay_buffer.clear();
      delay_buffer_seeded = false;
      return;
    }
    if (acc_delay_steps > 0) {
      if (pending_delay_accel.empty()) {
        accel_delay_buffer.assign(
          static_cast<size_t>(acc_delay_steps),
          x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::ACCELERATION)));
      } else {
        const float accel_hold = pending_delay_accel.back();
        accel_delay_buffer.resize(static_cast<size_t>(acc_delay_steps));
        for (int i = 0; i < acc_delay_steps; ++i) {
          accel_delay_buffer[static_cast<size_t>(i)] =
            static_cast<size_t>(i) < pending_delay_accel.size()
              ? pending_delay_accel[static_cast<size_t>(i)]
              : accel_hold;
        }
      }
    } else {
      accel_delay_buffer.clear();
    }
    if (steer_delay_steps > 0) {
      if (pending_delay_steer.empty()) {
        steer_delay_buffer.assign(
          static_cast<size_t>(steer_delay_steps),
          x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::STEER_ANGLE)));
      } else {
        const float steer_hold = pending_delay_steer.back();
        steer_delay_buffer.resize(static_cast<size_t>(steer_delay_steps));
        for (int i = 0; i < steer_delay_steps; ++i) {
          steer_delay_buffer[static_cast<size_t>(i)] =
            static_cast<size_t>(i) < pending_delay_steer.size()
              ? pending_delay_steer[static_cast<size_t>(i)]
              : steer_hold;
        }
      }
    } else {
      steer_delay_buffer.clear();
    }
    delay_buffer_seeded = true;
  }

  void snapshotControlHistoryForLog()
  {
    logged_hist_accel_tm2 = 0.0F;
    logged_hist_steer_tm2 = 0.0F;
    logged_hist_accel_tm1 = 0.0F;
    logged_hist_steer_tm1 = 0.0F;
    if (!controller) {
      return;
    }
    controller->copyControlHistory(
      logged_hist_accel_tm2, logged_hist_steer_tm2, logged_hist_accel_tm1, logged_hist_steer_tm1);
  }

  void snapshotDelayBufferForLog()
  {
    logged_delay_accel = accel_delay_buffer;
    logged_delay_steer = steer_delay_buffer;
    // Pad unequal channel lengths so the two-column CSV writer stays rectangular.
    const size_t n = std::max(logged_delay_accel.size(), logged_delay_steer.size());
    if (n == 0U) {
      return;
    }
    const float accel_pad = logged_delay_accel.empty() ? 0.0F : logged_delay_accel.back();
    const float steer_pad = logged_delay_steer.empty() ? 0.0F : logged_delay_steer.back();
    while (logged_delay_accel.size() < n) {
      logged_delay_accel.push_back(accel_pad);
    }
    while (logged_delay_steer.size() < n) {
      logged_delay_steer.push_back(steer_pad);
    }
  }

  void pushDelayedInput(const DYN::control_array & u)
  {
    const int accel_idx =
      static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
    const int steer_idx = static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);
    if (acc_delay_steps > 0) {
      ensureDelayBufferSeeded();
      if (accel_delay_buffer.size() != static_cast<size_t>(acc_delay_steps)) {
        resizeChannelDelayBuffer(
          accel_delay_buffer, acc_delay_steps,
          x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::ACCELERATION)));
      }
      for (int i = 0; i < acc_delay_steps - 1; ++i) {
        accel_delay_buffer[static_cast<size_t>(i)] = accel_delay_buffer[static_cast<size_t>(i + 1)];
      }
      accel_delay_buffer[static_cast<size_t>(acc_delay_steps - 1)] = u(accel_idx);
    } else {
      accel_delay_buffer.clear();
    }
    if (steer_delay_steps > 0) {
      ensureDelayBufferSeeded();
      if (steer_delay_buffer.size() != static_cast<size_t>(steer_delay_steps)) {
        resizeChannelDelayBuffer(
          steer_delay_buffer, steer_delay_steps,
          x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::STEER_ANGLE)));
      }
      for (int i = 0; i < steer_delay_steps - 1; ++i) {
        steer_delay_buffer[static_cast<size_t>(i)] = steer_delay_buffer[static_cast<size_t>(i + 1)];
      }
      steer_delay_buffer[static_cast<size_t>(steer_delay_steps - 1)] = u(steer_idx);
    } else {
      steer_delay_buffer.clear();
    }
  }

  void seedNominalControlFromLastOptimized()
  {
    // Drop the control already applied at the previous cycle; hold the terminal command.
    for (int t = 0; t < kMppiHorizon - 1; ++t) {
      u_nom.col(t) = u_opt.col(t + 1);
    }
    u_nom.col(kMppiHorizon - 1) = u_opt.col(kMppiHorizon - 1);
  }

  void seedNominalControlFromDiffusionReference(
    const Trajectory & reference, const size_t start_idx)
  {
    const int accel_idx =
      static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
    const int steer_idx = static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);
    const auto nominal = detail::buildDiffusionNominalControl(
      reference, start_idx, vehicle_params, kMppiHorizon,
      user_cost_params_.nominal_curvature_min_chord_length_m);
    for (int t = 0; t < kMppiHorizon; ++t) {
      u_nom(accel_idx, t) = nominal[static_cast<size_t>(t)].accel_cmd;
      u_nom(steer_idx, t) = nominal[static_cast<size_t>(t)].steer_cmd;
    }
  }

  void seedNominalControlFromTemporalMpt(
    const Trajectory & reference, const detail::InitialState & ego)
  {
    auto nominal = temporal_mpt_nominal_seeder.solve(reference, ego, vehicle_params, kMppiHorizon);
    if (!nominal) {
      temporal_mpt_nominal_seeder.resetWarmStart();
      seedNominalControlFromDiffusionReference(reference, tracking_start_idx);
      return;
    }
    if (enable_input_delay_compensation && (acc_delay_steps > 0 || steer_delay_steps > 0)) {
      nominal =
        detail::shiftNominalControlForInputDelay(*nominal, acc_delay_steps, steer_delay_steps);
    }
    const int accel_idx =
      static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
    const int steer_idx = static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);
    for (int t = 0; t < kMppiHorizon; ++t) {
      u_nom(accel_idx, t) = (*nominal)[static_cast<size_t>(t)].accel_cmd;
      u_nom(steer_idx, t) = (*nominal)[static_cast<size_t>(t)].steer_cmd;
    }
  }

  void applyActiveVelocityLimitToNominal()
  {
    if (!active_velocity_limit_profile.active) {
      return;
    }
    const int accel_idx =
      static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
    const int count =
      std::min(kMppiHorizon, static_cast<int>(active_velocity_limit_profile.controls.size()));
    for (int timestep = 0; timestep < count; ++timestep) {
      u_nom(accel_idx, timestep) =
        active_velocity_limit_profile.controls[static_cast<std::size_t>(timestep)].accel_cmd;
    }
  }

  void seedNominalControlFromForced()
  {
    const int accel_idx =
      static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
    const int steer_idx = static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);
    const auto nominal = detail::buildForcedNominalControl(
      forced_nominal_accel, forced_nominal_steer, vehicle_params, kMppiHorizon);
    for (int t = 0; t < kMppiHorizon; ++t) {
      u_nom(accel_idx, t) = nominal[static_cast<size_t>(t)].accel_cmd;
      u_nom(steer_idx, t) = nominal[static_cast<size_t>(t)].steer_cmd;
    }
  }

  void filterNominalControl(const detail::InitialState & ego)
  {
    const int accel_idx =
      static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
    const int steer_idx = static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);
    std::vector<FirstOrderDubinsMppiControl> nominal(static_cast<size_t>(kMppiHorizon));
    for (int t = 0; t < kMppiHorizon; ++t) {
      nominal[static_cast<size_t>(t)] = {u_nom(accel_idx, t), u_nom(steer_idx, t)};
    }
    const auto filtered = detail::filterNominalControlWithKinematicLimits(
      nominal, ego, active_kinematic_limits, vehicle_params, acc_delay_steps, accel_delay_buffer,
      kDt);
    for (int t = 0; t < kMppiHorizon; ++t) {
      u_nom(accel_idx, t) = filtered[static_cast<size_t>(t)].accel_cmd;
      u_nom(steer_idx, t) = filtered[static_cast<size_t>(t)].steer_cmd;
    }
  }

  void snapshotNominalForLog()
  {
    const int accel_idx =
      static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
    const int steer_idx = static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);
    logged_nominal_accel.resize(static_cast<size_t>(kMppiHorizon));
    logged_nominal_steer.resize(static_cast<size_t>(kMppiHorizon));
    for (int t = 0; t < kMppiHorizon; ++t) {
      logged_nominal_accel[static_cast<size_t>(t)] = u_nom(accel_idx, t);
      logged_nominal_steer[static_cast<size_t>(t)] = u_nom(steer_idx, t);
    }
  }

  void seedNominalControl(
    const Trajectory & reference, const size_t start_idx, const detail::InitialState & ego)
  {
    if (forced_nominal_pending) {
      seedNominalControlFromForced();
      forced_nominal_pending = false;
      snapshotNominalForLog();
      return;
    }

    // After a tracking reset, step_count is 0 and u_opt was cleared — fall back to DP / MPT seed.
    // Also reseed when departing from a stop: shifted last u_opt is usually near-zero / braking.
    constexpr float kStoppedVelocityMps = 0.05F;
    const bool started_from_stop = std::abs(ego.velocity) < kStoppedVelocityMps;
    const bool have_last_u = use_last_control_as_nominal && step_count > 0 && !started_from_stop;

    if (use_temporal_mpt_as_nominal) {
      // t-MPT warm-starts from its own previous x/u, shifted one stage. Do not inject MPPI u_opt.
      if (started_from_stop || step_count == 0) {
        temporal_mpt_nominal_seeder.resetWarmStart();
      }
      seedNominalControlFromTemporalMpt(reference, ego);
      filterNominalControl(ego);
      snapshotNominalForLog();
      return;
    }
    if (have_last_u) {
      seedNominalControlFromLastOptimized();
      snapshotNominalForLog();
      return;
    }
    seedNominalControlFromDiffusionReference(reference, start_idx);
    filterNominalControl(ego);
    snapshotNominalForLog();
  }

  void updateDiffusionReference(
    const Trajectory & reference, const Odometry & odometry,
    const std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> & acceleration,
    const std::optional<autoware_vehicle_msgs::msg::SteeringReport> & steering_status,
    const TrackedObjects & tracked_objects_in, const std::vector<Segment> & road_borders_in,
    const std::vector<Segment> & drivable_area_in,
    const FirstOrderDubinsMppiKinematicLimits & kinematic_limits)
  {
    if (!initialized) {
      setup();
    }

    if (force_cold_start_each_step) {
      resetTrackingState();
    }

    diffusion_reference = reference;
    active_kinematic_limits = sanitizeKinematicLimits(kinematic_limits);
    effective_max_velocity_by_reference_point = detail::buildEffectiveMaximumVelocityProfile(
      diffusion_reference.points.size(), active_kinematic_limits);
    uniform_effective_max_velocity =
      detail::getUniformMaximumVelocity(effective_max_velocity_by_reference_point);
    has_map_velocity_limit = std::any_of(
      active_kinematic_limits.max_velocity_by_reference_point.begin(),
      active_kinematic_limits.max_velocity_by_reference_point.end(),
      [](const auto & value) { return value.has_value(); });
    auto cost_limits = active_kinematic_limits;
    cost_limits.max_velocity = uniform_effective_max_velocity;
    cost.setKinematicLimits(makeKinematicLimitCostData(cost_limits));
    syncKinematicLimitsToModel();
    diffusion_reference_chord_length_s = detail::computeCumulativeChordLength(diffusion_reference);
    tracked_objects = ignore_obstacles ? TrackedObjects{} : tracked_objects_in;
    road_borders = ignore_road_borders ? std::vector<Segment>() : road_borders_in;
    drivable_area = ignore_drivable_area ? std::vector<Segment>() : drivable_area_in;
    if (road_borders.size() > static_cast<size_t>(COST::kMaxRoadBorderSegments)) {
      RCLCPP_WARN(
        mppiLogger(), "Road-border segment count %zu exceeds GPU capacity %d; truncating",
        road_borders.size(), COST::kMaxRoadBorderSegments);
    }
    if (drivable_area.size() > static_cast<size_t>(COST::kMaxDrivableAreaSegments)) {
      RCLCPP_WARN(
        mppiLogger(), "Drivable-area segment count %zu exceeds GPU capacity %d; truncating",
        drivable_area.size(), COST::kMaxDrivableAreaSegments);
    }
    obstacles.clear();
    // Boundary crash is disabled on this stack (isEgoOutsideDrivableArea always false).
    // ignore_drivable_area remains an ablation API flag; it does not reintroduce road borders.
    (void)ignore_drivable_area;

    // Optimize from measured ego; delay is applied in dynamics (no reference time shift).
    tracking_start_idx = 0U;
    if (step_count == 0) {
      resetTrackingState();
    }
    // Offline retune seeds must land after the step_count==0 reset (which clears the delay FIFO).
    applyPendingControlHistory();
    syncDelayStepsToModel();
    applyPendingDelayBuffer();

    const auto initial_state =
      detail::makeInitialState(odometry, acceleration, steering_status, vehicle_params);
    cost.setInitialSteeringAngle(initial_state.steering);
    const std::vector<FirstOrderDubinsMppiControl> profile_seed(
      std::max(static_cast<std::size_t>(kMppiHorizon), diffusion_reference.points.size()));
    std::vector<float> profile_reference_velocities(profile_seed.size(), 0.0F);
    if (!diffusion_reference.points.empty()) {
      for (std::size_t index = 0; index < profile_reference_velocities.size(); ++index) {
        profile_reference_velocities[index] =
          diffusion_reference.points[std::min(index, diffusion_reference.points.size() - 1U)]
            .longitudinal_velocity_mps;
      }
    }
    const auto next_profile_maximums =
      detail::buildEffectiveMaximumVelocityProfile(profile_seed.size(), active_kinematic_limits);
    const auto next_uniform_maximum = detail::getUniformMaximumVelocity(next_profile_maximums);
    constexpr float kSameVelocityLimitToleranceMps = 1.0E-3F;
    const bool same_uniform_velocity_limit =
      next_uniform_maximum &&
      std::abs(*next_uniform_maximum - active_velocity_limit_profile.target_velocity) <=
        kSameVelocityLimitToleranceMps;
    const bool same_pointwise_velocity_limits =
      next_profile_maximums.size() == active_velocity_limit_profile.maximum_velocities.size() &&
      std::equal(
        next_profile_maximums.begin(), next_profile_maximums.end(),
        active_velocity_limit_profile.maximum_velocities.begin(),
        [](const auto & lhs, const auto & rhs) {
          if (lhs.has_value() != rhs.has_value()) {
            return false;
          }
          return !lhs || std::abs(*lhs - *rhs) <= kSameVelocityLimitToleranceMps;
        });
    const bool keep_velocity_limit_active =
      active_velocity_limit_profile.active &&
      (same_uniform_velocity_limit || same_pointwise_velocity_limits);
    active_velocity_limit_profile = detail::buildActiveVelocityLimitProfile(
      profile_seed, initial_state, active_kinematic_limits, vehicle_params, acc_delay_steps,
      accel_delay_buffer, kDt, keep_velocity_limit_active, profile_reference_velocities);
    detail::applyActiveVelocityLimitProfile(diffusion_reference, active_velocity_limit_profile);
    const auto seed_t0 = std::chrono::steady_clock::now();
    seedNominalControl(diffusion_reference, tracking_start_idx, initial_state);
    last_seed_nominal_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - seed_t0).count();
    applyActiveVelocityLimitToNominal();
    if (active_velocity_limit_profile.active) {
      snapshotNominalForLog();
    }

    x = model.getZeroState();
    x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X)) = initial_state.x;
    x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y)) = initial_state.y;
    x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW)) = initial_state.yaw;
    x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X)) = initial_state.velocity;
    x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::ACCELERATION)) =
      initial_state.acceleration;
    x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::STEER_ANGLE)) =
      initial_state.steering;
    ensureDelayBufferSeeded();
    loadDelayPipesIntoState();
    snapshotDelayBufferForLog();
  }

  void capturePredictionAnchor(const Odometry & odometry)
  {
    prediction_anchor_.valid = true;
    prediction_anchor_.stamp = odometry.header.stamp;
    prediction_anchor_.sim_time = sim_time;
    auto & plant = prediction_anchor_.plant;
    plant.valid = true;
    plant.x = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X));
    plant.y = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y));
    plant.yaw = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW));
    plant.velocity = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X));
    plant.acceleration =
      x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::ACCELERATION));
    plant.steering = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::STEER_ANGLE));
    plant.sim_time = sim_time;
    plant.accel_cmd_delay_buffer = accel_delay_buffer;
    plant.steer_cmd_delay_buffer = steer_delay_buffer;
  }

  FirstOrderDubinsMppiPredictionAccuracy evaluatePredictionAccuracy(const Odometry & odometry) const
  {
    if (!prediction_anchor_.valid) {
      return {};
    }
    detail::PlantPredictionReplayInput input;
    input.vehicle = vehicle_params;
    input.enable_input_delay_compensation = enable_input_delay_compensation;
    input.integration_dt = kDt;
    input.anchor = prediction_anchor_;
    input.control_history = prediction_control_history_;
    input.measurement_stamp = odometry.header.stamp;
    input.measured_x = static_cast<float>(odometry.pose.pose.position.x);
    input.measured_y = static_cast<float>(odometry.pose.pose.position.y);
    input.measured_yaw = yawFromOdometry(odometry);
    input.measured_vel = static_cast<float>(odometry.twist.twist.linear.x);
    return detail::evaluatePlantPredictionAccuracy(input);
  }

  void recordAppliedControl(const Odometry & odometry, const FirstOrderDubinsMppiControl & control)
  {
    detail::FirstOrderDubinsMppiControlHistoryEntry entry;
    entry.stamp = odometry.header.stamp;
    entry.control = control;
    prediction_control_history_.push_back(entry);
    constexpr std::size_t kMaxHistoryEntries = 256U;
    if (prediction_control_history_.size() > kMaxHistoryEntries) {
      prediction_control_history_.erase(
        prediction_control_history_.begin(),
        prediction_control_history_.end() - static_cast<std::ptrdiff_t>(kMaxHistoryEntries));
    }
  }

  void uploadBoundarySegments()
  {
    if (road_borders.empty()) {
      cost.clearRoadBorders();
    } else {
      cost.setRoadBorderSegments(road_borders);
    }
    if (drivable_area.empty()) {
      cost.clearDrivableAreaSegments();
    } else {
      cost.setDrivableAreaSegments(drivable_area);
    }
  }

  FirstOrderDubinsMppiControl runStep(
    const FirstOrderDubinsMppiControlSequencePostprocessor & control_postprocessor)
  {
    // History taps used by this cycle's Savitzky–Golay (before slideControlSequence).
    snapshotControlHistoryForLog();

    // Measured ego IC; per-channel delay lives in dynamics taps (no host pre-roll / ref shift).
    detail::InitialState ego;
    ego.x = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X));
    ego.y = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y));
    ego.yaw = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW));
    ego.velocity = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X));
    auto prepared_reference = detail::buildReferenceHorizon(
      diffusion_reference, ego, kRefHorizon, kDt, tracking_start_idx,
      &diffusion_reference_chord_length_s,
      uniform_effective_max_velocity ? nullptr : &effective_max_velocity_by_reference_point);
    if (uniform_effective_max_velocity && !active_velocity_limit_profile.active) {
      for (auto & sample : prepared_reference) {
        sample.velocity = std::clamp(sample.velocity, 0.0F, *uniform_effective_max_velocity);
      }
    }
    std::vector<mppi::path::PathReferenceSample> ref(prepared_reference.size());
    for (size_t i = 0; i < prepared_reference.size(); ++i) {
      ref[i].t = prepared_reference[i].time;
      ref[i].x = prepared_reference[i].x;
      ref[i].y = prepared_reference[i].y;
      ref[i].yaw = prepared_reference[i].yaw;
      ref[i].v = prepared_reference[i].velocity;
      ref[i].arc_length_s = prepared_reference[i].arc_length_s;
      if (prepared_reference[i].max_velocity) {
        ref[i].max_velocity = *prepared_reference[i].max_velocity;
        ref[i].velocity_limit_active = 1U;
      }
    }
    mppi::path::PathReferenceSample terminal_reference = ref.back();
    if (!diffusion_reference.points.empty()) {
      const auto & terminal_point = diffusion_reference.points.back();
      terminal_reference.x = static_cast<float>(terminal_point.pose.position.x);
      terminal_reference.y = static_cast<float>(terminal_point.pose.position.y);
      terminal_reference.yaw = static_cast<float>(tf2::getYaw(terminal_point.pose.orientation));
    }
    mppi::cost::fillFirstOrderDubinsBicycleCostFromPathReference<kRefHorizon>(
      cost, ref, &terminal_reference);

    // Lateral crash / soft lateral distance use the full DP polyline + chord lengths.
    {
      const auto & pts = diffusion_reference.points;
      const int n_src = static_cast<int>(pts.size());
      if (n_src >= 2) {
        const int max_n = COST::kMaxLateralCorridorPoints;
        const int n = std::min(n_src, max_n);
        std::vector<float> corridor_x(static_cast<size_t>(n));
        std::vector<float> corridor_y(static_cast<size_t>(n));
        std::vector<float> corridor_s(static_cast<size_t>(n));
        std::vector<float> corridor_ref_velocity(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
          const int src =
            (n_src <= max_n) ? i : ((i == n - 1) ? (n_src - 1) : (i * (n_src - 1) / (n - 1)));
          corridor_x[static_cast<size_t>(i)] =
            static_cast<float>(pts[static_cast<size_t>(src)].pose.position.x);
          corridor_y[static_cast<size_t>(i)] =
            static_cast<float>(pts[static_cast<size_t>(src)].pose.position.y);
          corridor_s[static_cast<size_t>(i)] =
            (static_cast<size_t>(src) < diffusion_reference_chord_length_s.size())
              ? diffusion_reference_chord_length_s[static_cast<size_t>(src)]
              : 0.0F;
          corridor_ref_velocity[static_cast<size_t>(i)] =
            pts[static_cast<size_t>(src)].longitudinal_velocity_mps;
        }
        cost.setLateralCorridor(
          corridor_x.data(), corridor_y.data(), n, corridor_s.data(), corridor_ref_velocity.data());
      } else {
        cost.clearLateralCorridor();
      }
    }

    int obstacle_count = 0;
    if (!tracked_objects.objects.empty()) {
      buildObstacleTrajectoryBuffersFromTrackedObjects(
        tracked_objects, kDt, kRefHorizon, obs_traj_x, obs_traj_y, obs_traj_yaw, obs_half_length,
        obs_half_width, 0.0F);
      obstacle_count = trackedObjectObstacleCount(tracked_objects);
    } else if (!obstacles.empty()) {
      mppi::cost::buildObstacleTrajectoryBuffers(
        obstacles, sim_time, kDt, kRefHorizon, obs_traj_x, obs_traj_y, obs_traj_yaw,
        obs_half_length, obs_half_width);
      obstacle_count =
        static_cast<int>(std::min(obstacles.size(), static_cast<size_t>(kMaxMppiObstacles)));
    } else {
      obs_traj_x.clear();
      obs_traj_y.clear();
      obs_traj_yaw.clear();
      obs_half_length.clear();
      obs_half_width.clear();
    }
    mppi::cost::fillFirstOrderDubinsBicycleCostObstacleTrajectories<kRefHorizon>(
      cost, obstacle_count > 0 ? obs_traj_x.data() : nullptr,
      obstacle_count > 0 ? obs_traj_y.data() : nullptr,
      obstacle_count > 0 ? obs_traj_yaw.data() : nullptr,
      obstacle_count > 0 ? obs_half_length.data() : nullptr,
      obstacle_count > 0 ? obs_half_width.data() : nullptr, obstacle_count, kRefHorizon);
    uploadBoundarySegments();
    cost.renderDistanceMapTextureDebug();

    controller->updateImportanceSampler(u_nom);
    controller->computeControl(x, 1);
    checkCuda("computeControl");

    Mppi::control_trajectory u_opt_traj = controller->getControlSeq();
    bool control_sequence_modified = false;
    if (control_postprocessor) {
      const int accel_idx =
        static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
      const int steer_idx =
        static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);
      std::vector<FirstOrderDubinsMppiControl> optimized_controls(
        static_cast<std::size_t>(u_opt_traj.cols()));
      for (int timestep = 0; timestep < u_opt_traj.cols(); ++timestep) {
        optimized_controls[static_cast<std::size_t>(timestep)] = {
          u_opt_traj(accel_idx, timestep), u_opt_traj(steer_idx, timestep)};
      }
      control_postprocessor(optimized_controls);
      if (optimized_controls.size() != static_cast<std::size_t>(u_opt_traj.cols())) {
        throw std::invalid_argument("MPPI control postprocessor must preserve the horizon size");
      }
      for (int timestep = 0; timestep < u_opt_traj.cols(); ++timestep) {
        u_opt_traj(accel_idx, timestep) =
          optimized_controls[static_cast<std::size_t>(timestep)].accel_cmd;
        u_opt_traj(steer_idx, timestep) =
          optimized_controls[static_cast<std::size_t>(timestep)].steer_cmd;
      }
      control_sequence_modified = true;
    }
    if (active_velocity_limit_profile.active) {
      const int accel_idx =
        static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
      const int count = std::min(
        static_cast<int>(u_opt_traj.cols()),
        static_cast<int>(active_velocity_limit_profile.controls.size()));
      for (int timestep = 0; timestep < count; ++timestep) {
        u_opt_traj(accel_idx, timestep) =
          active_velocity_limit_profile.controls[static_cast<std::size_t>(timestep)].accel_cmd;
      }
    }
    if (control_sequence_modified) {
      controller->setControlSequenceAndRecomputeState(u_opt_traj, x);
    }
    u_opt = u_opt_traj;

    DYN::control_array u_apply = u_opt_traj.col(0);
    model.enforceConstraints(x, u_apply);
    pushDelayedInput(u_apply);

    DYN::state_array x_next = model.getZeroState();
    DYN::state_array xdot = model.getZeroState();
    DYN::output_array y = DYN::output_array::Zero();
    model.step(x, x_next, xdot, u_apply, y, static_cast<float>(step_count), kDt);
    x = x_next;

    ++step_count;
    sim_time += kDt;

    FirstOrderDubinsMppiControl control;
    control.accel_cmd =
      u_apply(static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD));
    control.steer_cmd =
      u_apply(static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD));
    logged_applied_accel = control.accel_cmd;
    logged_applied_steer = control.steer_cmd;

    RCLCPP_DEBUG(
      mppiLogger(),
      "MPPI track step %d: start_idx=%zu ref_v0=%.2f u_accel=%.3f u_steer=%.3f "
      "ego_v=%.2f baseline_cost=%.2f",
      step_count, tracking_start_idx, ref.empty() ? 0.0F : ref.front().v, control.accel_cmd,
      control.steer_cmd, x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X)),
      controller->getBaselineCost());

    return control;
  }

  void teardown()
  {
    controller.reset();
    cost.freeCudaMem();
    model.freeCudaMem();
    sampler.freeCudaMem();
    initialized = false;
  }
};

FirstOrderDubinsMppiInterface::FirstOrderDubinsMppiInterface() : impl_(std::make_unique<Impl>())
{
}

FirstOrderDubinsMppiInterface::~FirstOrderDubinsMppiInterface()
{
  if (impl_) {
    impl_->teardown();
  }
}

FirstOrderDubinsMppiInterface::FirstOrderDubinsMppiInterface(
  FirstOrderDubinsMppiInterface && other) noexcept = default;

FirstOrderDubinsMppiInterface & FirstOrderDubinsMppiInterface::operator=(
  FirstOrderDubinsMppiInterface && other) noexcept = default;

void FirstOrderDubinsMppiInterface::initialize()
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  impl_->setup();
}

bool FirstOrderDubinsMppiInterface::isInitialized() const
{
  return impl_ && impl_->initialized;
}

void FirstOrderDubinsMppiInterface::setVehicleParams(
  const FirstOrderDubinsMppiVehicleParams & params)
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  if (impl_->initialized) {
    impl_->teardown();
  }
  impl_->vehicle_params = params;
}

void FirstOrderDubinsMppiInterface::setCostParams(const FirstOrderDubinsMppiCostParams & params)
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  if (
    !std::isfinite(params.overlimit_coeff) || params.overlimit_coeff < 0.0F ||
    !std::isfinite(params.initial_steer_rate_coeff) || params.initial_steer_rate_coeff < 0.0F ||
    !std::isfinite(params.crash_contact_penalty) || params.crash_contact_penalty < 0.0F ||
    !std::isfinite(params.std_dev_decay) || params.std_dev_decay < 0.0F ||
    params.std_dev_decay > 1.0F || !std::isfinite(params.lambda) ||
    !std::isfinite(params.lambda_min) || !std::isfinite(params.lambda_max) ||
    !std::isfinite(params.target_ess_ratio) || !std::isfinite(params.lambda_adaptation_gain) ||
    !std::isfinite(params.unsafe_rollout_fraction_threshold) ||
    !std::isfinite(params.cost_normalization_percentile) || params.lambda_min <= 0.0F ||
    params.lambda_max < params.lambda_min || params.lambda < params.lambda_min ||
    params.lambda > params.lambda_max || params.target_ess_ratio < 0.0F ||
    params.target_ess_ratio > 1.0F || params.lambda_adaptation_gain < 0.0F ||
    params.unsafe_rollout_fraction_threshold < 0.0F ||
    params.unsafe_rollout_fraction_threshold > 1.0F ||
    params.cost_normalization_percentile < 0.0F || params.cost_normalization_percentile > 1.0F ||
    params.max_iter <= 0) {
    throw std::invalid_argument(
      "MPPI cost parameters must have finite non-negative penalties, std_dev_decay in [0, 1], "
      "lambda within positive [lambda_min, lambda_max], target_ess_ratio in [0, 1], a "
      "non-negative lambda_adaptation_gain, unsafe_rollout_fraction_threshold and "
      "cost_normalization_percentile in [0, 1], and max_iter greater than zero");
  }
  if (impl_->initialized) {
    impl_->teardown();
  }
  impl_->user_cost_params_ = params;
}

void FirstOrderDubinsMppiInterface::setRuntimeOptions(
  const FirstOrderDubinsMppiRuntimeOptions & options)
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  impl_->prevent_reverse_velocity = options.prevent_reverse_velocity;
  setDebugTrajectoryLogging(
    options.enable_debug_trajectory_log, options.debug_trajectory_log_directory);
  impl_->cost.setDistanceMapTextureDebugEnabled(options.enable_distance_map_texture_debug);
  impl_->enable_iteration_rollout_debug = options.enable_iteration_rollout_debug;
  if (impl_->controller) {
    impl_->controller->setIterationRolloutCaptureEnabled(
      options.enable_iteration_rollout_debug && !impl_->enable_rollout_visualization);
  }
  setAblationOptions(
    options.ignore_obstacles, options.ignore_road_borders, options.ignore_drivable_area,
    options.force_cold_start_each_step, options.skip_if_invalid,
    options.use_last_control_as_nominal);
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  impl_->use_temporal_mpt_as_nominal = options.use_temporal_mpt_as_nominal;
  impl_->enable_input_delay_compensation = options.enable_input_delay_compensation;
  impl_->min_optimization_length = options.min_optimization_length;
  impl_->dyn.prevent_reverse_velocity = options.prevent_reverse_velocity;
  if (impl_->initialized) {
    impl_->syncDelayStepsToModel();
    if (!impl_->enable_input_delay_compensation) {
      impl_->accel_delay_buffer.clear();
      impl_->steer_delay_buffer.clear();
      impl_->delay_buffer_seeded = false;
      impl_->loadDelayPipesIntoState();
    }
  }
  RCLCPP_INFO(
    mppiLogger(),
    "MPPI nominal seed: use_temporal_mpt_as_nominal=%s enable_input_delay_compensation=%s "
    "prevent_reverse_velocity=%s",
    options.use_temporal_mpt_as_nominal ? "true" : "false",
    options.enable_input_delay_compensation ? "true" : "false",
    options.prevent_reverse_velocity ? "true" : "false");
}
void FirstOrderDubinsMppiInterface::setDebugTrajectoryLogging(
  const bool enable, const std::string & directory)
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  impl_->debug_trajectory_logger.configure(enable, directory);
  impl_->debug_trajectory_logger.writeParamsOnce(impl_->user_cost_params_, impl_->vehicle_params);
}

void FirstOrderDubinsMppiInterface::setAblationOptions(
  const bool ignore_obstacles, const bool ignore_road_borders, const bool ignore_drivable_area,
  const bool force_cold_start_each_step, const bool skip_if_invalid,
  const bool use_last_control_as_nominal)
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  impl_->ignore_obstacles = ignore_obstacles;
  impl_->ignore_road_borders = ignore_road_borders;
  impl_->ignore_drivable_area = ignore_drivable_area;
  impl_->force_cold_start_each_step = force_cold_start_each_step;
  impl_->skip_if_invalid = skip_if_invalid;
  impl_->use_last_control_as_nominal = use_last_control_as_nominal;
  RCLCPP_INFO(
    mppiLogger(),
    "MPPI ablation options: ignore_obstacles=%s ignore_road_borders=%s ignore_drivable_area=%s "
    "force_cold_start_each_step=%s skip_if_invalid=%s use_last_control_as_nominal=%s",
    ignore_obstacles ? "true" : "false", ignore_road_borders ? "true" : "false",
    ignore_drivable_area ? "true" : "false", force_cold_start_each_step ? "true" : "false",
    skip_if_invalid ? "true" : "false", use_last_control_as_nominal ? "true" : "false");
  FirstOrderDubinsMppiRuntimeOptions runtime{};
  runtime.ignore_obstacles = ignore_obstacles;
  runtime.ignore_road_borders = ignore_road_borders;
  runtime.ignore_drivable_area = ignore_drivable_area;
  runtime.force_cold_start_each_step = force_cold_start_each_step;
  runtime.skip_if_invalid = skip_if_invalid;
  runtime.min_optimization_length = impl_->min_optimization_length;
  runtime.use_last_control_as_nominal = use_last_control_as_nominal;
  runtime.use_temporal_mpt_as_nominal = impl_->use_temporal_mpt_as_nominal;
  runtime.prevent_reverse_velocity = impl_->prevent_reverse_velocity;
  runtime.enable_input_delay_compensation = impl_->enable_input_delay_compensation;
  runtime.enable_iteration_rollout_debug = impl_->enable_iteration_rollout_debug;
  impl_->debug_trajectory_logger.writeRuntimeOptionsOnce(runtime);
}

void FirstOrderDubinsMppiInterface::setRolloutVisualizationEnabled(const bool enable)
{
  if (!impl_) {
    return;
  }
  impl_->enable_rollout_visualization = enable;
  if (impl_->controller) {
    impl_->controller->setIterationRolloutCaptureEnabled(
      impl_->enable_iteration_rollout_debug && !enable);
  }
}

void FirstOrderDubinsMppiInterface::setForcedNominalControl(
  const std::vector<float> & accel_cmd, const std::vector<float> & steer_cmd)
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  impl_->forced_nominal_accel = accel_cmd;
  impl_->forced_nominal_steer = steer_cmd;
  impl_->forced_nominal_pending = !accel_cmd.empty() || !steer_cmd.empty();
}

void FirstOrderDubinsMppiInterface::setControlHistory(
  const float accel_tm2, const float steer_tm2, const float accel_tm1, const float steer_tm1)
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  impl_->pending_hist_accel_tm2 = accel_tm2;
  impl_->pending_hist_steer_tm2 = steer_tm2;
  impl_->pending_hist_accel_tm1 = accel_tm1;
  impl_->pending_hist_steer_tm1 = steer_tm1;
  impl_->pending_control_history = true;
  if (impl_->controller) {
    impl_->applyPendingControlHistory();
  }
}

void FirstOrderDubinsMppiInterface::setInputDelayBuffer(
  const std::vector<float> & accel_cmd, const std::vector<float> & steer_cmd)
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  // Always defer until updateDiffusionReference (after step_count==0 reset) so offline seeds
  // are not wiped by resetTrackingState.
  impl_->pending_delay_accel = accel_cmd;
  impl_->pending_delay_steer = steer_cmd;
  impl_->pending_delay_buffer = true;
}

bool FirstOrderDubinsMppiInterface::copyLastOptimizedControl(
  std::vector<float> & accel_cmd, std::vector<float> & steer_cmd) const
{
  accel_cmd.clear();
  steer_cmd.clear();
  if (!impl_ || !impl_->controller || !impl_->initialized) {
    return false;
  }
  const Mppi::control_trajectory u_opt = impl_->controller->getControlSeq();
  const int n = static_cast<int>(u_opt.cols());
  if (n <= 0) {
    return false;
  }
  const int accel_idx =
    static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
  const int steer_idx = static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);
  accel_cmd.resize(static_cast<size_t>(n));
  steer_cmd.resize(static_cast<size_t>(n));
  for (int t = 0; t < n; ++t) {
    accel_cmd[static_cast<size_t>(t)] = u_opt(accel_idx, t);
    steer_cmd[static_cast<size_t>(t)] = u_opt(steer_idx, t);
  }
  return true;
}

bool FirstOrderDubinsMppiInterface::copySampleCostDistribution(
  std::vector<float> & raw_costs, std::vector<float> & normalized_weights, const int stride) const
{
  raw_costs.clear();
  normalized_weights.clear();
  if (!impl_ || !impl_->controller || !impl_->initialized) {
    return false;
  }

  impl_->controller->downloadImportanceWeightsToHost();
  // IMPORTANT: take by value (not const-ref-to-temporary). nvcc has historically broken
  // lifetime extension for large Eigen return temporaries, which caused heap corruption
  // (munmap_chunk: invalid pointer) when reading getSampledCostSeq() via const auto&.
  const Mppi::sampled_cost_traj importance = impl_->controller->getSampledCostSeq();
  const float min_cost = impl_->controller->getLastMinRolloutCost();
  const float max_cost = impl_->controller->getLastMaxRolloutCost();
  const float unnormalized_weight_sum = impl_->controller->getLastUnnormalizedWeightSum();
  const float normalizer = impl_->controller->getNormalizerCost();
  const float lambda = impl_->controller->getLastWeightLambda();
  const int stride_n = std::max(1, stride);
  const int num_rollouts = static_cast<int>(importance.size());
  const int kept = (num_rollouts + stride_n - 1) / stride_n;
  raw_costs.reserve(static_cast<size_t>(kept));
  normalized_weights.reserve(static_cast<size_t>(kept));

  for (int i = 0; i < num_rollouts; i += stride_n) {
    const float w = importance(i);
    normalized_weights.push_back((normalizer > 0.0F) ? (w / normalizer) : 0.0F);
    raw_costs.push_back(
      rawCostFromMinMaxWeight(w * unnormalized_weight_sum, min_cost, max_cost, lambda));
  }
  return !raw_costs.empty();
}

FirstOrderDubinsMppiControl FirstOrderDubinsMppiInterface::computeStep(
  FirstOrderDubinsMppiState & state, float sim_time)
{
  if (!impl_ || !impl_->initialized) {
    throw std::runtime_error(
      "FirstOrderDubinsMppiInterface must be initialized before computeStep");
  }

  fromHostState(impl_->x, state);
  impl_->sim_time = sim_time;
  const FirstOrderDubinsMppiControl control = impl_->runStep({});
  state = toHostState(impl_->x);
  // Advance the vendor control history after the applied command is consumed so the
  // next cycle's Savitzky-Golay left-edge taps are the previously applied controls.
  impl_->controller->slideControlSequence(1);
  return control;
}

FirstOrderDubinsMppiOptimizationResult FirstOrderDubinsMppiInterface::optimizeTrajectory(
  const Trajectory & input, const Odometry & odometry,
  const std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> & acceleration,
  const std::optional<autoware_vehicle_msgs::msg::SteeringReport> & steering_status,
  const TrackedObjects & tracked_objects, const std::vector<Segment> & road_borders,
  const std::vector<Segment> & drivable_area,
  const FirstOrderDubinsMppiKinematicLimits & kinematic_limits,
  const FirstOrderDubinsMppiControlSequencePostprocessor & control_postprocessor)
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  FirstOrderDubinsMppiOptimizationResult result;
  const auto not_enough_input_points = input.points.size() < 2U;
  const auto optimization_required =
    detail::isOptimizationRequired(input, impl_->min_optimization_length);
  if (not_enough_input_points || !optimization_required) {
    RCLCPP_WARN(
      mppiLogger(), "MPPI skipped: %s",
      not_enough_input_points ? "trajectory has fewer than 2 points"
                              : "trajectory does not require optimization");

    result.trajectory = input;
    result.debug.reference_trajectory = input;
    result.debug.optimized_trajectory = input;
    return result;
  }

  const auto start_time = std::chrono::steady_clock::now();

  impl_->updateDiffusionReference(
    input, odometry, acceleration, steering_status, tracked_objects, road_borders, drivable_area,
    kinematic_limits);
  result.debug.prediction_accuracy = impl_->evaluatePredictionAccuracy(odometry);
  impl_->capturePredictionAnchor(odometry);
  // Capture IC before runStep advances the ego state with the applied control.
  const DYN::state_array x_at_optimization = impl_->x;
  const FirstOrderDubinsMppiControl control = impl_->runStep(control_postprocessor);
  impl_->recordAppliedControl(odometry, control);

  FirstOrderDubinsMppiAppliedPlantState & applied_plant = result.debug.applied_plant;
  applied_plant.valid = true;
  applied_plant.x = impl_->x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X));
  applied_plant.y = impl_->x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y));
  applied_plant.yaw = impl_->x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW));
  applied_plant.velocity =
    impl_->x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X));
  applied_plant.acceleration =
    impl_->x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::ACCELERATION));
  applied_plant.steering =
    impl_->x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::STEER_ANGLE));
  applied_plant.sim_time = impl_->sim_time;
  applied_plant.accel_cmd_delay_buffer = impl_->accel_delay_buffer;
  applied_plant.steer_cmd_delay_buffer = impl_->steer_delay_buffer;
  applied_plant.applied_control = control;

  const auto state_trajectory = impl_->controller->getActualStateSeq();
  const Mppi::control_trajectory u_opt_traj = impl_->controller->getControlSeq();
  const int pos_x_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X);
  const int pos_y_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y);
  const int yaw_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW);
  const int vel_x_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X);
  const int accel_state_idx =
    static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::ACCELERATION);
  const int steer_x_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::STEER_ANGLE);
  const int accel_cmd_idx =
    static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
  const int steer_cmd_idx =
    static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);

  // Cost/plot alignment: MPPI costs post-step state at t against ref[t]=DP[t] (DP starts at
  // ≈dt). Map published points[i] <- state[i+1], control[i] so index 0 is state after first
  // step vs DP[0] — not ego. Ego is the IC only; it is never a cost or plot sample.
  //
  // Vendor mismatch: GPU rollouts take H steps (mppi_common.cu: t = 0..H-1) and cost the
  // final post-step state x[H], but host getActualStateSeq() only runs H-1 steps
  // (controller.cuh: i < num_timesteps - 1) and stores x[0..H-1]. Reconstruct x[H] here
  // with one dynamics step so the last DP point is not left as a duplicate of x[H-1].
  const int n_state = static_cast<int>(state_trajectory.cols());
  const int n_ctrl = static_cast<int>(u_opt_traj.cols());
  const size_t num_points = std::min(
    {input.points.size(), static_cast<size_t>(std::max(0, n_state)),
     static_cast<size_t>(std::max(0, n_ctrl))});
  result.optimized_point_count = num_points;

  DYN::state_array x_final = DYN::state_array::Zero();
  DYN::state_array x_final_dot = DYN::state_array::Zero();
  DYN::output_array y_final = DYN::output_array::Zero();
  const bool have_final_state = n_state > 0 && n_ctrl > 0;
  if (have_final_state) {
    DYN::state_array x_tail = state_trajectory.col(n_state - 1);
    DYN::control_array u_tail = u_opt_traj.col(n_ctrl - 1);
    impl_->model.enforceConstraints(x_tail, u_tail);
    impl_->model.step(
      x_tail, x_final, x_final_dot, u_tail, y_final, static_cast<float>(n_state - 1), kDt);
  }

  std::vector<detail::OptimizedState> optimized_states;
  std::vector<FirstOrderDubinsMppiControl> optimized_controls;
  optimized_states.reserve(num_points);
  optimized_controls.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    const int control_col = static_cast<int>(i);
    const bool use_final = (control_col + 1 >= n_state) && have_final_state;
    detail::OptimizedState state;
    state.x = use_final ? x_final(pos_x_idx) : state_trajectory(pos_x_idx, control_col + 1);
    state.y = use_final ? x_final(pos_y_idx) : state_trajectory(pos_y_idx, control_col + 1);
    state.yaw = use_final ? x_final(yaw_idx) : state_trajectory(yaw_idx, control_col + 1);
    state.velocity = use_final ? x_final(vel_x_idx) : state_trajectory(vel_x_idx, control_col + 1);
    state.acceleration =
      use_final ? x_final(accel_state_idx) : state_trajectory(accel_state_idx, control_col + 1);
    state.steering =
      use_final ? x_final(steer_x_idx) : state_trajectory(steer_x_idx, control_col + 1);
    optimized_states.push_back(state);
    FirstOrderDubinsMppiControl optimized_control;
    optimized_control.accel_cmd = u_opt_traj(accel_cmd_idx, control_col);
    optimized_control.steer_cmd = u_opt_traj(steer_cmd_idx, control_col);
    optimized_controls.push_back(optimized_control);
  }

  Trajectory output = detail::buildOptimizedTrajectory(input, optimized_states, optimized_controls);
  if (impl_->active_velocity_limit_profile.active) {
    // buildOptimizedTrajectory intentionally preserves the suffix outside the MPPI horizon.
    // An active velocity profile must not allow that suffix to jump back to its input speed.
    const auto & profile = impl_->active_velocity_limit_profile;
    for (std::size_t index = num_points; index < output.points.size(); ++index) {
      auto & point = output.points[index];
      if (index < profile.velocities.size()) {
        point.longitudinal_velocity_mps = profile.velocities[index];
        point.acceleration_mps2 = profile.accelerations[index];
      } else {
        point.longitudinal_velocity_mps = profile.target_velocity;
        point.acceleration_mps2 = 0.0F;
      }
    }
  }

  // Validate only states that come from getActualStateSeq. The host-extrapolated x_final
  // (vendor GPU/host horizon mismatch) is appended so the published path matches the DP
  // length, but it is not an MPPI-scored state and often sits >boundary_threshold off the
  // corridor even when the optimized prefix is fine — that was causing false rejects.
  std::vector<detail::OptimizedState> states_to_validate;
  states_to_validate.reserve(optimized_states.size());
  for (size_t i = 0; i < optimized_states.size(); ++i) {
    const bool use_final = (static_cast<int>(i) + 1 >= n_state) && have_final_state;
    if (!use_final) {
      states_to_validate.push_back(optimized_states[i]);
    }
  }
  if (states_to_validate.empty()) {
    states_to_validate = optimized_states;
  }
  const auto validation = detail::validateOptimizedTrajectory(impl_->cost, states_to_validate);
  float max_pos_delta = 0.0F;
  float max_vel_delta = 0.0F;
  for (size_t i = 0; i < optimized_states.size(); ++i) {
    const auto & state = optimized_states[i];
    const auto & in_point = input.points[i];
    const float ref_x = static_cast<float>(in_point.pose.position.x);
    const float ref_y = static_cast<float>(in_point.pose.position.y);
    const float ref_v = in_point.longitudinal_velocity_mps;
    max_pos_delta = std::max(max_pos_delta, std::hypot(state.x - ref_x, state.y - ref_y));
    max_vel_delta = std::max(max_vel_delta, std::abs(state.velocity - ref_v));
  }

  const auto initial_effective_maximum =
    !impl_->effective_max_velocity_by_reference_point.empty()
      ? impl_->effective_max_velocity_by_reference_point.front()
      : std::nullopt;
  detail::setInitialEngageVelocity(output, initial_effective_maximum);

  result.trajectory = output;
  result.debug.reference_trajectory = input;
  result.debug.optimized_trajectory = output;
  result.debug.active_kinematic_limits = impl_->active_kinematic_limits;
  result.debug.effective_max_velocity_by_reference_point =
    impl_->effective_max_velocity_by_reference_point;
  result.debug.map_velocity_limit_active = impl_->has_map_velocity_limit;
  result.debug.nominal_trajectory = buildNominalTrajectory(
    impl_->model, x_at_optimization, input, impl_->logged_nominal_accel,
    impl_->logged_nominal_steer);
  result.debug.timing.seed_nominal_ms = impl_->last_seed_nominal_ms;
  result.debug.nominal_control_profile.time_step_s = kDt;
  result.debug.nominal_control_profile.acceleration_commands_mps2 = impl_->logged_nominal_accel;
  result.debug.nominal_control_profile.steering_commands_rad = impl_->logged_nominal_steer;
  result.debug.iteration_effective_sample_sizes =
    impl_->controller->getIterationEffectiveSampleSizes();
  result.debug.lambda_used = impl_->controller->getLastWeightLambda();
  result.debug.lambda_next = impl_->controller->getNextWeightLambda();
  result.debug.max_rollout_cost = impl_->controller->getLastMaxRolloutCost();
  result.debug.normalization_upper_cost = impl_->controller->getLastNormalizationUpperCost();
  result.debug.unsafe_rollout_fraction = impl_->controller->getLastUnsafeRolloutFraction();
  result.debug.validation = validation;
  result.debug.velocity_limit_profile_active = impl_->active_velocity_limit_profile.active;
  result.debug.external_velocity_limit_active =
    impl_->active_velocity_limit_profile.active && impl_->active_kinematic_limits.max_velocity;
  if (impl_->enable_rollout_visualization) {
    buildRolloutVisualization(
      *impl_->controller, impl_->sampler, impl_->model, x_at_optimization, result.debug);
  } else {
    fillOptimalHorizonPoints(impl_->controller->getActualStateSeq(), result.debug.optimal_horizon);
    result.debug.baseline_cost = impl_->controller->getBaselineCost();
    if (impl_->enable_iteration_rollout_debug) {
      buildIterationDebugRollouts(
        impl_->controller->iterationRolloutSnapshots(), impl_->model, x_at_optimization,
        result.debug);
    }
  }
  if (impl_->debug_trajectory_logger.enabled() && n_state > 0 && n_ctrl > 0) {
    result.debug.cost_breakdown =
      reconstructControlTrajectoryCost(impl_->cost, impl_->model, x_at_optimization, u_opt_traj);
    const auto nominal_controls =
      makeNominalControlTrajectory(impl_->logged_nominal_accel, impl_->logged_nominal_steer);
    result.debug.nominal_cost_breakdown = reconstructControlTrajectoryCost(
      impl_->cost, impl_->model, x_at_optimization, nominal_controls);
  }

  MppiDebugEgoState ego;
  ego.x = odometry.pose.pose.position.x;
  ego.y = odometry.pose.pose.position.y;
  ego.z = odometry.pose.pose.position.z;
  ego.yaw = yawFromOdometry(odometry);
  ego.v = odometry.twist.twist.linear.x;
  ego.accel = longitudinalAccelerationMps2(acceleration);
  ego.steer = steeringTireAngleRad(steering_status);
  impl_->debug_trajectory_logger.writeParamsOnce(impl_->user_cost_params_, impl_->vehicle_params);
  {
    FirstOrderDubinsMppiRuntimeOptions runtime{};
    runtime.ignore_obstacles = impl_->ignore_obstacles;
    runtime.ignore_road_borders = impl_->ignore_road_borders;
    runtime.ignore_drivable_area = impl_->ignore_drivable_area;
    runtime.force_cold_start_each_step = impl_->force_cold_start_each_step;
    runtime.skip_if_invalid = impl_->skip_if_invalid;
    runtime.min_optimization_length = impl_->min_optimization_length;
    runtime.use_last_control_as_nominal = impl_->use_last_control_as_nominal;
    runtime.use_temporal_mpt_as_nominal = impl_->use_temporal_mpt_as_nominal;
    runtime.prevent_reverse_velocity = impl_->prevent_reverse_velocity;
    runtime.enable_input_delay_compensation = impl_->enable_input_delay_compensation;
    runtime.enable_iteration_rollout_debug = impl_->enable_iteration_rollout_debug;
    impl_->debug_trajectory_logger.writeRuntimeOptionsOnce(runtime);
  }
  impl_->debug_trajectory_logger.logFrame(
    result.debug.reference_trajectory, result.debug.optimized_trajectory,
    result.debug.nominal_trajectory, ego, result.debug.baseline_cost, impl_->logged_nominal_accel,
    impl_->logged_nominal_steer, road_borders, drivable_area, tracked_objects,
    impl_->logged_hist_accel_tm2, impl_->logged_hist_steer_tm2, impl_->logged_hist_accel_tm1,
    impl_->logged_hist_steer_tm1, impl_->logged_delay_accel, impl_->logged_delay_steer,
    impl_->logged_applied_accel, impl_->logged_applied_steer, impl_->active_kinematic_limits);

  const auto validation_reasons = to_string(result.debug.validation.reasons);
  const auto cost_breakdown = formatCostBreakdown(result.debug.cost_breakdown);
  result.debug.timing.total_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time)
      .count();
  RCLCPP_INFO(
    mppiLogger(),
    "MPPI tracked diffusion ref in %.1f ms (seed_nominal=%.1f): start_idx=%zu steps=%d "
    "output points size=%zu points=%zu rollouts=%zu "
    "obstacles=%zu road_borders=%zu drivable_segments=%zu u_accel=%.3f u_steer=%.3f "
    "best_sample_cost=%.2f selected_cost=%s validity=%s max_pos_err=%.3f m "
    "max_vel_err=%.3f m/s",
    result.debug.timing.total_ms, result.debug.timing.seed_nominal_ms, impl_->tracking_start_idx,
    impl_->step_count, output.points.size(), num_points, result.debug.rollouts.size(),
    tracked_objects.objects.size(), road_borders.size(), drivable_area.size(), control.accel_cmd,
    control.steer_cmd, result.debug.baseline_cost, cost_breakdown.c_str(),
    validation_reasons.c_str(), max_pos_delta, max_vel_delta);

  if (impl_->skip_if_invalid && !validation.isValid()) {
    result.trajectory = input;
    detail::applyActiveVelocityLimitProfile(
      result.trajectory, impl_->active_velocity_limit_profile);
    result.debug.reference_trajectory = input;
    result.debug.optimized_trajectory = result.trajectory;
    // Keep nominal_trajectory: still shows the warm-start open-loop path.
    result.debug.was_rejected = true;
    RCLCPP_WARN(
      mppiLogger(),
      "MPPI output rejected with invalidity_mask=%u at point=%s; returning the input trajectory "
      "unchanged",
      static_cast<unsigned int>(validation.reasons),
      validation.first_invalid_index.has_value()
        ? std::to_string(validation.first_invalid_index.value()).c_str()
        : "unknown");
  }

  // Advance the vendor control history after all getControlSeq / getActualStateSeq reads
  // so the next cycle's Savitzky-Golay left-edge taps are the previously applied controls.
  impl_->controller->slideControlSequence(1);

  return result;
}

namespace detail
{
namespace
{

constexpr float kPlantPredictionTimeEpsilonS = 1.0E-6F;

int clampInputDelayStepsForPrediction(const int steps)
{
  return std::clamp(steps, 0, FirstOrderDubinsBicycleParams::kMaxInputDelaySteps);
}

int sampledDelayStepsForPrediction(const float t, const float delay_s, const float dt)
{
  if (delay_s <= 0.0F || dt <= 0.0F) {
    return 0;
  }
  const double steps = static_cast<double>(t) / static_cast<double>(dt) -
                       std::floor(static_cast<double>(t - delay_s) / static_cast<double>(dt));
  return clampInputDelayStepsForPrediction(std::max(0, static_cast<int>(std::lround(steps))));
}

float wrapPiForPrediction(const float yaw)
{
  float wrapped = yaw;
  while (wrapped > static_cast<float>(M_PI)) {
    wrapped -= 2.0F * static_cast<float>(M_PI);
  }
  while (wrapped < -static_cast<float>(M_PI)) {
    wrapped += 2.0F * static_cast<float>(M_PI);
  }
  return wrapped;
}

void loadPlantIntoStateForPrediction(
  const FirstOrderDubinsMppiAppliedPlantState & plant, FirstOrderDubinsBicycle::state_array & x)
{
  using S = FirstOrderDubinsBicycleParams::StateIndex;
  x = FirstOrderDubinsBicycle::state_array::Zero();
  x(static_cast<int>(S::POS_X)) = plant.x;
  x(static_cast<int>(S::POS_Y)) = plant.y;
  x(static_cast<int>(S::YAW)) = plant.yaw;
  x(static_cast<int>(S::VEL_X)) = plant.velocity;
  x(static_cast<int>(S::ACCELERATION)) = plant.acceleration;
  x(static_cast<int>(S::STEER_ANGLE)) = plant.steering;
}

void loadDelayPipesIntoStateForPrediction(
  FirstOrderDubinsBicycle::state_array & x, const int acc_delay_steps, const int steer_delay_steps,
  const std::vector<float> & accel_delay_buffer, const std::vector<float> & steer_delay_buffer)
{
  using S = FirstOrderDubinsBicycleParams::StateIndex;
  for (int i = 0; i < FirstOrderDubinsBicycleParams::kMaxInputDelaySteps; ++i) {
    x(static_cast<int>(S::ACCEL_CMD_D0) + i) = 0.0F;
    x(static_cast<int>(S::STEER_CMD_D0) + i) = 0.0F;
  }
  for (int i = 0; i < acc_delay_steps; ++i) {
    if (static_cast<size_t>(i) < accel_delay_buffer.size()) {
      x(static_cast<int>(S::ACCEL_CMD_D0) + i) = accel_delay_buffer[static_cast<size_t>(i)];
    }
  }
  for (int i = 0; i < steer_delay_steps; ++i) {
    if (static_cast<size_t>(i) < steer_delay_buffer.size()) {
      x(static_cast<int>(S::STEER_CMD_D0) + i) = steer_delay_buffer[static_cast<size_t>(i)];
    }
  }
}

void pushDelayedInputForPrediction(
  const FirstOrderDubinsBicycle::control_array & u, const int acc_delay_steps,
  const int steer_delay_steps, FirstOrderDubinsBicycle::state_array & x,
  std::vector<float> & accel_delay_buffer, std::vector<float> & steer_delay_buffer)
{
  using S = FirstOrderDubinsBicycleParams::StateIndex;
  using C = FirstOrderDubinsBicycleParams::ControlIndex;
  if (acc_delay_steps > 0) {
    if (accel_delay_buffer.size() != static_cast<size_t>(acc_delay_steps)) {
      accel_delay_buffer.assign(
        static_cast<size_t>(acc_delay_steps), x(static_cast<int>(S::ACCELERATION)));
    }
    for (int i = 0; i < acc_delay_steps - 1; ++i) {
      accel_delay_buffer[static_cast<size_t>(i)] = accel_delay_buffer[static_cast<size_t>(i + 1)];
    }
    accel_delay_buffer[static_cast<size_t>(acc_delay_steps - 1)] =
      u(static_cast<int>(C::ACCELERATION_CMD));
  } else {
    accel_delay_buffer.clear();
  }

  if (steer_delay_steps > 0) {
    if (steer_delay_buffer.size() != static_cast<size_t>(steer_delay_steps)) {
      steer_delay_buffer.assign(
        static_cast<size_t>(steer_delay_steps), x(static_cast<int>(S::STEER_ANGLE)));
    }
    for (int i = 0; i < steer_delay_steps - 1; ++i) {
      steer_delay_buffer[static_cast<size_t>(i)] = steer_delay_buffer[static_cast<size_t>(i + 1)];
    }
    steer_delay_buffer[static_cast<size_t>(steer_delay_steps - 1)] =
      u(static_cast<int>(C::STEER_CMD));
  } else {
    steer_delay_buffer.clear();
  }
}

FirstOrderDubinsBicycleParams makePlantPredictionDynamicsParams(
  const FirstOrderDubinsMppiVehicleParams & vehicle, const int acc_delay_steps,
  const int steer_delay_steps, const bool prevent_reverse_velocity)
{
  FirstOrderDubinsBicycleParams dyn{};
  dyn.wheel_base = vehicle.wheel_base;
  dyn.max_steer_angle = vehicle.max_steer_angle;
  dyn.accel_time_constant = vehicle.acc_time_constant;
  dyn.steer_time_constant = vehicle.steer_time_constant;
  dyn.max_steer_rate = vehicle.steer_rate_lim;
  dyn.min_accel = vehicle.min_accel();
  dyn.max_accel = vehicle.max_accel();
  dyn.acc_delay_steps = acc_delay_steps;
  dyn.steer_delay_steps = steer_delay_steps;
  dyn.prevent_reverse_velocity = prevent_reverse_velocity;
  return dyn;
}

FirstOrderDubinsMppiControl controlAtTimeForPrediction(
  const std::vector<FirstOrderDubinsMppiControlHistoryEntry> & history,
  const builtin_interfaces::msg::Time & query_time, const FirstOrderDubinsMppiControl & fallback)
{
  FirstOrderDubinsMppiControl selected = fallback;
  const double query_s = stampToSeconds(query_time);
  double best_s = -std::numeric_limits<double>::infinity();
  for (const auto & entry : history) {
    const double entry_s = stampToSeconds(entry.stamp);
    if (entry_s <= query_s + kPlantPredictionTimeEpsilonS && entry_s >= best_s) {
      best_s = entry_s;
      selected = entry.control;
    }
  }
  return selected;
}

}  // namespace

double stampToSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0E-9;
}

double elapsedSeconds(
  const builtin_interfaces::msg::Time & from, const builtin_interfaces::msg::Time & to)
{
  return stampToSeconds(to) - stampToSeconds(from);
}

FirstOrderDubinsMppiPredictionAccuracy evaluatePlantPredictionAccuracy(
  const PlantPredictionReplayInput & input)
{
  using C = FirstOrderDubinsBicycleParams::ControlIndex;
  using S = FirstOrderDubinsBicycleParams::StateIndex;

  FirstOrderDubinsMppiPredictionAccuracy result;
  if (!input.anchor.valid || input.integration_dt <= kPlantPredictionTimeEpsilonS) {
    return result;
  }

  const double elapsed = elapsedSeconds(input.anchor.stamp, input.measurement_stamp);
  result.elapsed_s = elapsed;
  if (elapsed <= kPlantPredictionTimeEpsilonS) {
    return result;
  }

  const auto dts = buildChunkedIntegrationDts(static_cast<float>(elapsed), input.integration_dt);
  if (dts.empty()) {
    return result;
  }

  result.full_steps =
    static_cast<int>(std::floor((elapsed + kPlantPredictionTimeEpsilonS) / input.integration_dt));
  result.remainder_s =
    static_cast<float>(elapsed) - static_cast<float>(result.full_steps) * input.integration_dt;
  if (result.remainder_s < kPlantPredictionTimeEpsilonS) {
    result.remainder_s = 0.0F;
  }
  result.integration_steps = static_cast<int>(dts.size());

  FirstOrderDubinsBicycle model;
  float sim_time = input.anchor.sim_time;
  int acc_delay_steps = 0;
  int steer_delay_steps = 0;
  if (input.enable_input_delay_compensation) {
    acc_delay_steps =
      sampledDelayStepsForPrediction(sim_time, input.vehicle.acc_time_delay, input.integration_dt);
    steer_delay_steps = sampledDelayStepsForPrediction(
      sim_time, input.vehicle.steer_time_delay, input.integration_dt);
  }
  model.setParams(makePlantPredictionDynamicsParams(
    input.vehicle, acc_delay_steps, steer_delay_steps, /*prevent_reverse_velocity=*/true));

  FirstOrderDubinsBicycle::state_array x = FirstOrderDubinsBicycle::state_array::Zero();
  loadPlantIntoStateForPrediction(input.anchor.plant, x);
  std::vector<float> accel_delay_buffer = input.anchor.plant.accel_cmd_delay_buffer;
  std::vector<float> steer_delay_buffer = input.anchor.plant.steer_cmd_delay_buffer;
  loadDelayPipesIntoStateForPrediction(
    x, acc_delay_steps, steer_delay_steps, accel_delay_buffer, steer_delay_buffer);

  const FirstOrderDubinsMppiControl fallback_control = input.control_history.empty()
                                                         ? FirstOrderDubinsMppiControl{}
                                                         : input.control_history.back().control;

  builtin_interfaces::msg::Time query_time = input.anchor.stamp;
  float integration_time = 0.0F;
  for (const float step_dt : dts) {
    const auto control =
      controlAtTimeForPrediction(input.control_history, query_time, fallback_control);
    FirstOrderDubinsBicycle::control_array u = FirstOrderDubinsBicycle::control_array::Zero();
    u(static_cast<int>(C::ACCELERATION_CMD)) = control.accel_cmd;
    u(static_cast<int>(C::STEER_CMD)) = control.steer_cmd;

    pushDelayedInputForPrediction(
      u, acc_delay_steps, steer_delay_steps, x, accel_delay_buffer, steer_delay_buffer);
    loadDelayPipesIntoStateForPrediction(
      x, acc_delay_steps, steer_delay_steps, accel_delay_buffer, steer_delay_buffer);

    if (input.enable_input_delay_compensation) {
      acc_delay_steps = sampledDelayStepsForPrediction(
        sim_time, input.vehicle.acc_time_delay, input.integration_dt);
      steer_delay_steps = sampledDelayStepsForPrediction(
        sim_time, input.vehicle.steer_time_delay, input.integration_dt);
      model.setParams(makePlantPredictionDynamicsParams(
        input.vehicle, acc_delay_steps, steer_delay_steps, /*prevent_reverse_velocity=*/true));
    }

    FirstOrderDubinsBicycle::state_array x_next = model.getZeroState();
    FirstOrderDubinsBicycle::state_array xdot = model.getZeroState();
    FirstOrderDubinsBicycle::output_array y = FirstOrderDubinsBicycle::output_array::Zero();
    model.enforceConstraints(x, u);
    model.step(x, x_next, xdot, u, y, sim_time, step_dt);
    x = x_next;
    sim_time += step_dt;
    integration_time += step_dt;

    const double anchor_s = stampToSeconds(input.anchor.stamp);
    query_time.sec = static_cast<int32_t>(std::floor(anchor_s + integration_time));
    query_time.nanosec = static_cast<uint32_t>(
      (anchor_s + integration_time - static_cast<double>(query_time.sec)) * 1.0E9);
  }

  result.valid = true;
  result.predicted_x = x(static_cast<int>(S::POS_X));
  result.predicted_y = x(static_cast<int>(S::POS_Y));
  result.predicted_yaw = x(static_cast<int>(S::YAW));
  result.predicted_vel = x(static_cast<int>(S::VEL_X));
  result.pos_error_m =
    std::hypot(result.predicted_x - input.measured_x, result.predicted_y - input.measured_y);
  result.yaw_error_rad = wrapPiForPrediction(result.predicted_yaw - input.measured_yaw);
  result.vel_error_mps = result.predicted_vel - input.measured_vel;
  return result;
}

}  // namespace detail

}  // namespace autoware::mppi_optimizer
