#include <mppi/dynamics/dubins/first_order_dubins_bicycle.cuh>

#include <cmath>

namespace
{
using S = FirstOrderDubinsBicycleParams::StateIndex;
using C = FirstOrderDubinsBicycleParams::ControlIndex;

__host__ __device__ __forceinline__ float clampSteerRate(
  const FirstOrderDubinsBicycleParams & p, const float steer_dot)
{
  return fmaxf(fminf(steer_dot, p.max_steer_rate), -p.max_steer_rate);
}

/**
 * Delayed ZOH of the command: u_eff[k] = u[k - n] from the shift register (u_now, z1, z2).
 */
__host__ __device__ __forceinline__ float delayedCommand(
  const float u_now, const float z1, const float z2, const int delay_steps)
{
  if (delay_steps <= 0) {
    return u_now;
  }
  if (delay_steps == 1) {
    return z1;
  }
  return z2;
}

__host__ __device__ __forceinline__ void firstOrderDubinsBicycleDeriv(
  const FirstOrderDubinsBicycleParams & p, const float * __restrict__ state,
  const float * __restrict__ control, float * __restrict__ state_der)
{
  const float v = state[static_cast<int>(S::VEL_X)];
  const float yaw = state[static_cast<int>(S::YAW)];
  const float steer = state[static_cast<int>(S::STEER_ANGLE)];
  const float accel = state[static_cast<int>(S::ACCELERATION)];
  const float accel_cmd = control[static_cast<int>(C::ACCELERATION_CMD)];
  const float steer_cmd = control[static_cast<int>(C::STEER_CMD)];
  const float accel_z1 = state[static_cast<int>(S::ACC_CMD_Z1)];
  const float accel_z2 = state[static_cast<int>(S::ACC_CMD_Z2)];
  const float steer_z1 = state[static_cast<int>(S::STEER_CMD_Z1)];
  const float steer_z2 = state[static_cast<int>(S::STEER_CMD_Z2)];

  const float accel_cmd_eff =
    delayedCommand(accel_cmd, accel_z1, accel_z2, p.accel_delay_steps);
  const float steer_cmd_eff =
    delayedCommand(steer_cmd, steer_z1, steer_z2, p.steer_delay_steps);

  const float accel_tau = fmaxf(p.accel_time_constant, 1.0E-4F);
  const float steer_tau = fmaxf(p.steer_time_constant, 1.0E-4F);
  const float inv_dt = p.inv_dt;

  state_der[static_cast<int>(S::ACCELERATION)] = (accel_cmd_eff - accel) / accel_tau;
  state_der[static_cast<int>(S::VEL_X)] = accel;
  state_der[static_cast<int>(S::YAW)] = (v / p.wheel_base) * tanf(steer);

  float sin_yaw = 0.0F;
  float cos_yaw = 0.0F;
#ifdef __CUDA_ARCH__
  __sincosf(yaw, &sin_yaw, &cos_yaw);
#else
  sincosf(yaw, &sin_yaw, &cos_yaw);
#endif
  state_der[static_cast<int>(S::POS_X)] = v * cos_yaw;
  state_der[static_cast<int>(S::POS_Y)] = v * sin_yaw;

  state_der[static_cast<int>(S::STEER_ANGLE)] =
    clampSteerRate(p, (steer_cmd_eff - steer) / steer_tau);

  // Discrete shift Z2 <- Z1 <- u as Euler ders: next = state + der * dt.
  state_der[static_cast<int>(S::ACC_CMD_Z1)] = (accel_cmd - accel_z1) * inv_dt;
  state_der[static_cast<int>(S::ACC_CMD_Z2)] = (accel_z1 - accel_z2) * inv_dt;
  state_der[static_cast<int>(S::STEER_CMD_Z1)] = (steer_cmd - steer_z1) * inv_dt;
  state_der[static_cast<int>(S::STEER_CMD_Z2)] = (steer_z1 - steer_z2) * inv_dt;
}
}  // namespace

template <class CLASS_T, class PARAMS_T>
FirstOrderDubinsBicycleImpl<CLASS_T, PARAMS_T>::FirstOrderDubinsBicycleImpl(cudaStream_t stream)
: Dynamics<CLASS_T, PARAMS_T>(stream)
{
  this->params_ = PARAMS_T();
}

template <class CLASS_T, class PARAMS_T>
FirstOrderDubinsBicycleImpl<CLASS_T, PARAMS_T>::FirstOrderDubinsBicycleImpl(
  PARAMS_T & params, cudaStream_t stream)
: Dynamics<CLASS_T, PARAMS_T>(params, stream)
{
}

template <class CLASS_T, class PARAMS_T>
void FirstOrderDubinsBicycleImpl<CLASS_T, PARAMS_T>::computeDynamics(
  const Eigen::Ref<const state_array> & state, const Eigen::Ref<const control_array> & control,
  Eigen::Ref<state_array> state_der)
{
  firstOrderDubinsBicycleDeriv(this->params_, state.data(), control.data(), state_der.data());
}

template <class CLASS_T, class PARAMS_T>
bool FirstOrderDubinsBicycleImpl<CLASS_T, PARAMS_T>::computeGrad(
  const Eigen::Ref<const state_array> &, const Eigen::Ref<const control_array> &, Eigen::Ref<dfdx>,
  Eigen::Ref<dfdu>)
{
  return false;
}

template <class CLASS_T, class PARAMS_T>
void FirstOrderDubinsBicycleImpl<CLASS_T, PARAMS_T>::updateState(
  const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
  Eigen::Ref<state_array> state_der, const float dt)
{
  next_state = state + state_der * dt;
  next_state(static_cast<int>(S::YAW)) =
    angle_utils::normalizeAngle(next_state(static_cast<int>(S::YAW)));
  next_state(static_cast<int>(S::STEER_ANGLE)) = fmaxf(
    fminf(next_state(static_cast<int>(S::STEER_ANGLE)), this->params_.max_steer_angle),
    -this->params_.max_steer_angle);
  next_state(static_cast<int>(S::ACCELERATION)) = fmaxf(
    fminf(next_state(static_cast<int>(S::ACCELERATION)), this->params_.max_accel),
    this->params_.min_accel);
}

template <class CLASS_T, class PARAMS_T>
FirstOrderDubinsBicycleImpl<CLASS_T, PARAMS_T>::state_array
FirstOrderDubinsBicycleImpl<CLASS_T, PARAMS_T>::interpolateState(
  const Eigen::Ref<state_array> state_1, const Eigen::Ref<state_array> state_2, const float alpha)
{
  state_array result = (1.0F - alpha) * state_1 + alpha * state_2;
  result(static_cast<int>(S::YAW)) = angle_utils::interpolateEulerAngleLinear(
    state_1(static_cast<int>(S::YAW)), state_2(static_cast<int>(S::YAW)), alpha);
  return result;
}

template <class CLASS_T, class PARAMS_T>
__device__ void FirstOrderDubinsBicycleImpl<CLASS_T, PARAMS_T>::updateState(
  float * state, float * next_state, float * state_der, const float dt)
{
  // Same pattern as RacerDubins: striping across threadIdx.y; delay states use Euler too.
  for (int i = threadIdx.y; i < PARENT_CLASS::STATE_DIM; i += blockDim.y) {
    next_state[i] = state[i] + state_der[i] * dt;
    if (i == static_cast<int>(S::YAW)) {
      next_state[i] = angle_utils::normalizeAngle(next_state[i]);
    } else if (i == static_cast<int>(S::STEER_ANGLE)) {
      next_state[i] =
        fmaxf(fminf(next_state[i], this->params_.max_steer_angle), -this->params_.max_steer_angle);
    } else if (i == static_cast<int>(S::ACCELERATION)) {
      next_state[i] = fmaxf(fminf(next_state[i], this->params_.max_accel), this->params_.min_accel);
    }
  }
}

template <class CLASS_T, class PARAMS_T>
__device__ void FirstOrderDubinsBicycleImpl<CLASS_T, PARAMS_T>::computeDynamics(
  float * state, float * control, float * state_der, float *)
{
  // Parent step() calls computeDynamics from all y-threads; only one writer needed.
  // __syncthreads() in Dynamics::step runs after this returns.
  if (threadIdx.y == 0) {
    firstOrderDubinsBicycleDeriv(this->params_, state, control, state_der);
  }
}

template <class CLASS_T, class PARAMS_T>
void FirstOrderDubinsBicycleImpl<CLASS_T, PARAMS_T>::stateToOutput(
  const Eigen::Ref<const state_array> & state, Eigen::Ref<output_array> output)
{
  stateToOutput(state.data(), output.data());
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ void FirstOrderDubinsBicycleImpl<CLASS_T, PARAMS_T>::stateToOutput(
  const float * state, float * output)
{
  using O = FirstOrderDubinsBicycleParams::OutputIndex;
  const float v = state[static_cast<int>(S::VEL_X)];

  output[static_cast<int>(O::BASELINK_VEL_B_X)] = v;
  output[static_cast<int>(O::BASELINK_VEL_B_Y)] = 0.0F;
  output[static_cast<int>(O::BASELINK_POS_I_X)] = state[static_cast<int>(S::POS_X)];
  output[static_cast<int>(O::BASELINK_POS_I_Y)] = state[static_cast<int>(S::POS_Y)];
  output[static_cast<int>(O::YAW)] = state[static_cast<int>(S::YAW)];
  output[static_cast<int>(O::STEER_ANGLE)] = state[static_cast<int>(S::STEER_ANGLE)];
  output[static_cast<int>(O::ACCELERATION)] = state[static_cast<int>(S::ACCELERATION)];
  output[static_cast<int>(O::TOTAL_VELOCITY)] = fabsf(v);
  output[static_cast<int>(O::LONGITUDINAL_JERK)] = 0.0F;
  output[static_cast<int>(O::LATERAL_JERK)] = 0.0F;
}

template <class CLASS_T, class PARAMS_T>
FirstOrderDubinsBicycleImpl<CLASS_T, PARAMS_T>::state_array
FirstOrderDubinsBicycleImpl<CLASS_T, PARAMS_T>::stateFromMap(
  const std::map<std::string, float> & map)
{
  state_array s = state_array::Zero();
  const auto set_if = [&map, &s](const char * key, const int idx) {
    const auto it = map.find(key);
    if (it != map.end()) {
      s(idx) = it->second;
    }
  };
  set_if("VEL_X", static_cast<int>(S::VEL_X));
  set_if("YAW", static_cast<int>(S::YAW));
  set_if("POS_X", static_cast<int>(S::POS_X));
  set_if("POS_Y", static_cast<int>(S::POS_Y));
  set_if("STEER_ANGLE", static_cast<int>(S::STEER_ANGLE));
  set_if("ACCELERATION", static_cast<int>(S::ACCELERATION));
  set_if("ACC_CMD_Z1", static_cast<int>(S::ACC_CMD_Z1));
  set_if("ACC_CMD_Z2", static_cast<int>(S::ACC_CMD_Z2));
  set_if("STEER_CMD_Z1", static_cast<int>(S::STEER_CMD_Z1));
  set_if("STEER_CMD_Z2", static_cast<int>(S::STEER_CMD_Z2));
  return s;
}
