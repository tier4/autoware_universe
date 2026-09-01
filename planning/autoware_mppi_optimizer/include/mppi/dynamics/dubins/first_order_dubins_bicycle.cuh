/**
 * Kinematic Dubins bicycle with first-order actuation on longitudinal acceleration and steering.
 *
 * State: speed, yaw, position, steer angle, applied acceleration, plus optional discrete
 * ZOH command-delay pipelines (oldest tap = next applied) for accel/steer independently.
 * Controls: acceleration command [m/s^2], steer angle command [rad].
 *
 *   d(accel)/dt = (u_accel_delayed - accel) / accel_time_constant
 *   d(v)/dt     = accel
 *   d(steer)/dt = (u_steer_delayed - steer) / steer_time_constant   (rate-limited)
 *   d(yaw)/dt   = (v / L) * tan(steer)
 *   d(x,y)/dt   = v * [cos(yaw), sin(yaw)]
 */
#pragma once

#ifndef MPPIGENERIC_FIRST_ORDER_DUBINS_BICYCLE_CUH
#define MPPIGENERIC_FIRST_ORDER_DUBINS_BICYCLE_CUH

#include <mppi/dynamics/dynamics.cuh>
#include <mppi/utils/angle_utils.cuh>

#include <cmath>

struct FirstOrderDubinsBicycleParams : public DynamicsParams
{
  /** Max discrete dead-time taps per channel (covers τ up to ~0.8 s at dt=0.1). */
  static constexpr int kMaxInputDelaySteps = 8;
  /** Fixed MPPI integration period used by the reverse-velocity control constraint. */
  static constexpr float kControlDt = 0.1F;

  enum class StateIndex : int {
    VEL_X = 0,
    YAW,
    POS_X,
    POS_Y,
    STEER_ANGLE,
    ACCELERATION,
    /** Oldest accel cmd in the wire (applied this step when acc_delay_steps > 0). */
    ACCEL_CMD_D0,
    ACCEL_CMD_D1,
    ACCEL_CMD_D2,
    ACCEL_CMD_D3,
    ACCEL_CMD_D4,
    ACCEL_CMD_D5,
    ACCEL_CMD_D6,
    ACCEL_CMD_D7,
    STEER_CMD_D0,
    STEER_CMD_D1,
    STEER_CMD_D2,
    STEER_CMD_D3,
    STEER_CMD_D4,
    STEER_CMD_D5,
    STEER_CMD_D6,
    STEER_CMD_D7,
    NUM_STATES
  };

  enum class ControlIndex : int { ACCELERATION_CMD = 0, STEER_CMD, NUM_CONTROLS };

  enum class OutputIndex : int {
    BASELINK_VEL_B_X = 0,
    BASELINK_VEL_B_Y,
    BASELINK_POS_I_X,
    BASELINK_POS_I_Y,
    YAW,
    STEER_ANGLE,
    ACCELERATION,
    TOTAL_VELOCITY,
    LATERAL_JERK,
    LONGITUDINAL_JERK,
    NUM_OUTPUTS
  };

  float wheel_base = 0.32F;
  /** PathTrackerFeedback: steer_cmd [rad] = atan(kappa * L) * steer_angle_scale /
   * steer_command_angle_scale */
  float steer_angle_scale = 1.0F;
  float steer_command_angle_scale = 1.0F;
  /** First-order lag: accel_dot = (u_accel - accel) / accel_time_constant */
  float accel_time_constant = 0.15F;
  /** First-order lag: steer_dot = (u_steer - steer) / steer_time_constant */
  float steer_time_constant = 0.08F;
  float max_steer_angle = 0.45F;
  float max_steer_rate = 3.0F;
  float min_accel = -6.0F;
  float max_accel = 4.0F;
  /** Prevent acceleration commands and integrated states from producing reverse velocity. */
  bool prevent_reverse_velocity = true;
  /** Discrete ZOH delay steps (0 = no delay). Clamped to [0, kMaxInputDelaySteps]. */
  int acc_delay_steps = 0;
  int steer_delay_steps = 0;
};

static_assert(
  static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::ACCEL_CMD_D7) -
      static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::ACCEL_CMD_D0) + 1 ==
    FirstOrderDubinsBicycleParams::kMaxInputDelaySteps,
  "accel delay taps must match kMaxInputDelaySteps");
static_assert(
  static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::STEER_CMD_D7) -
      static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::STEER_CMD_D0) + 1 ==
    FirstOrderDubinsBicycleParams::kMaxInputDelaySteps,
  "steer delay taps must match kMaxInputDelaySteps");

/** Apply the steering-rate limit shared by the dynamics and comfort-cost models. */
template <class PARAMS_T>
__host__ __device__ inline float clampSteerRate(const PARAMS_T & params, const float steer_rate)
{
  return fmaxf(fminf(steer_rate, params.max_steer_rate), -params.max_steer_rate);
}

/** Clamp delay step count into the fixed pipeline capacity. */
__host__ __device__ inline int clampInputDelaySteps(const int steps)
{
  const int max_steps = FirstOrderDubinsBicycleParams::kMaxInputDelaySteps;
  if (steps <= 0) {
    return 0;
  }
  return steps > max_steps ? max_steps : steps;
}

using namespace MPPI_internal;

template <class CLASS_T, class PARAMS_T = FirstOrderDubinsBicycleParams>
class FirstOrderDubinsBicycleImpl : public Dynamics<CLASS_T, PARAMS_T>
{
public:
  using PARENT_CLASS = Dynamics<CLASS_T, PARAMS_T>;
  using state_array = typename PARENT_CLASS::state_array;
  using control_array = typename PARENT_CLASS::control_array;
  using output_array = typename PARENT_CLASS::output_array;
  using dfdx = typename PARENT_CLASS::dfdx;
  using dfdu = typename PARENT_CLASS::dfdu;
  // Keep the parent host overload: MPPI's generic host pass supplies a placeholder zero state.
  using PARENT_CLASS::enforceConstraints;
  using PARENT_CLASS::updateState;

  FirstOrderDubinsBicycleImpl(cudaStream_t stream = nullptr);

  FirstOrderDubinsBicycleImpl(PARAMS_T & params, cudaStream_t stream = nullptr);

  std::string getDynamicsModelName() const override { return "First-Order Dubins Bicycle"; }

  void computeDynamics(
    const Eigen::Ref<const state_array> & state, const Eigen::Ref<const control_array> & control,
    Eigen::Ref<state_array> state_der);

  void updateState(
    const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
    Eigen::Ref<state_array> state_der, const float dt);

  __device__ void updateState(float * state, float * next_state, float * state_der, const float dt);

  /** Host step: continuous plant with discrete per-channel command delay taps. */
  void step(
    Eigen::Ref<state_array> state, Eigen::Ref<state_array> next_state,
    Eigen::Ref<state_array> state_der, const Eigen::Ref<const control_array> & control,
    Eigen::Ref<output_array> output, const float t, const float dt);

  /** Device step: same discrete delay advance as host (rollout kernel entry). */
  __device__ void step(
    float * state, float * next_state, float * state_der, float * control, float * output,
    float * theta_s, const float t, const float dt);

  state_array interpolateState(
    const Eigen::Ref<state_array> state_1, const Eigen::Ref<state_array> state_2,
    const float alpha);

  bool computeGrad(
    const Eigen::Ref<const state_array> & state, const Eigen::Ref<const control_array> & control,
    Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B);

  __device__ void computeDynamics(
    float * state, float * control, float * state_der, float * theta = nullptr);

  /** Host state-aware control filtering with prevent_reverse_velocity. */
  void enforceConstraints(Eigen::Ref<state_array> state, Eigen::Ref<control_array> control);

  /** Device state-aware control filtering. */
  __device__ void enforceConstraints(float * state, float * control);

  void stateToOutput(const Eigen::Ref<const state_array> & state, Eigen::Ref<output_array> output);

  __host__ __device__ void stateToOutput(const float * state, float * output);

  state_array stateFromMap(const std::map<std::string, float> & map) override;
};

class FirstOrderDubinsBicycle : public FirstOrderDubinsBicycleImpl<FirstOrderDubinsBicycle>
{
public:
  FirstOrderDubinsBicycle(cudaStream_t stream = nullptr)
  : FirstOrderDubinsBicycleImpl<FirstOrderDubinsBicycle>(stream)
  {
  }

  FirstOrderDubinsBicycle(FirstOrderDubinsBicycleParams & params, cudaStream_t stream = nullptr)
  : FirstOrderDubinsBicycleImpl<FirstOrderDubinsBicycle>(params, stream)
  {
  }
};

#if __CUDACC__
#include "first_order_dubins_bicycle.cu"
#endif

#endif  // MPPIGENERIC_FIRST_ORDER_DUBINS_BICYCLE_CUH
