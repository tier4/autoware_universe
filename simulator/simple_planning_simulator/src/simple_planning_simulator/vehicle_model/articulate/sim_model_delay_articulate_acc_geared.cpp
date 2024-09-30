// Copyright 2024 The Autoware Foundation.
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

#include "simple_planning_simulator/vehicle_model/articulate/sim_model_delay_articulate_acc_geared.hpp"

#include "autoware_vehicle_msgs/msg/gear_command.hpp"

#include <algorithm>

SimModelDelayArticulateAccGeared::SimModelDelayArticulateAccGeared(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim,
  double front_wheelbase, double rear_wheelbase, double dt, double acc_delay,
  double acc_time_constant, double steer_delay, double steer_time_constant, double steer_dead_band,
  double steer_bias, double rear_slip_coeff, double debug_acc_scaling_factor,
  double debug_steer_scaling_factor)
: SimModelInterface(6 /* dim x */, 2 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  front_wheelbase_(front_wheelbase),
  rear_wheelbase_(rear_wheelbase),
  acc_delay_(acc_delay),
  acc_time_constant_(std::max(acc_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  steer_dead_band_(steer_dead_band),
  steer_bias_(steer_bias),
  rear_slip_coeff_(rear_slip_coeff),
  debug_acc_scaling_factor_(std::max(debug_acc_scaling_factor, 0.0)),
  debug_steer_scaling_factor_(std::max(debug_steer_scaling_factor, 0.0))
{
  initializeInputQueue(dt);
}

double SimModelDelayArticulateAccGeared::getX()
{
  return state_(IDX::X);
}
double SimModelDelayArticulateAccGeared::getY()
{
  return state_(IDX::Y);
}
double SimModelDelayArticulateAccGeared::getYaw()
{
  return state_(IDX::YAW);
}
double SimModelDelayArticulateAccGeared::getVx()
{
  return state_(IDX::VX);
}
double SimModelDelayArticulateAccGeared::getVy()
{
  return 0.0;
}
double SimModelDelayArticulateAccGeared::getAx()
{
  return state_(IDX::ACCX);
}
double SimModelDelayArticulateAccGeared::getWz()
{
  const double steer = state_(IDX::STEER);
  const double c_steer = std::cos(steer);

  return (state_(IDX::VX) * std::sin(steer) -
          state_steer_rate_ * front_wheelbase_ * c_steer * rear_slip_coeff_) /
         (front_wheelbase_ + rear_wheelbase_ * c_steer);
}
double SimModelDelayArticulateAccGeared::getSteer()
{
  // return measured values with bias added to actual values
  return state_(IDX::STEER) + steer_bias_;
}
void SimModelDelayArticulateAccGeared::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  acc_input_queue_.push_back(input_(IDX_U::ACCX_DES));
  delayed_input(IDX_U::ACCX_DES) = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();

  const auto prev_state = state_;
  updateRungeKutta(dt, delayed_input);

  // take velocity limit explicitly
  state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));

  // consider gear
  // update position and velocity first, and then acceleration is calculated naturally
  updateStateWithGear(state_, prev_state, gear_, dt);
}

void SimModelDelayArticulateAccGeared::initializeInputQueue(const double & dt)
{
  size_t acc_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  acc_input_queue_.resize(acc_input_queue_size);
  std::fill(acc_input_queue_.begin(), acc_input_queue_.end(), 0.0);

  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_input_queue_.resize(steer_input_queue_size);
  std::fill(steer_input_queue_.begin(), steer_input_queue_.end(), 0.0);
}

Eigen::VectorXd SimModelDelayArticulateAccGeared::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };

  const double vel = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const double acc = sat(state(IDX::ACCX), vx_rate_lim_, -vx_rate_lim_);
  const double yaw = state(IDX::YAW);
  const double steer = state(IDX::STEER);
  const double acc_des =
    sat(input(IDX_U::ACCX_DES), vx_rate_lim_, -vx_rate_lim_) * debug_acc_scaling_factor_;
  const double steer_des =
    sat(input(IDX_U::STEER_DES), steer_lim_, -steer_lim_) * debug_steer_scaling_factor_;
  // NOTE: `steer_des` is calculated by control from measured values. getSteer() also gets the
  // measured value. The steer_rate used in the motion calculation is obtained from these
  // differences.
  const double steer_diff = getSteer() - steer_des;
  const double steer_diff_with_dead_band = std::invoke([&]() {
    if (steer_diff > steer_dead_band_) {
      return steer_diff - steer_dead_band_;
    } else if (steer_diff < -steer_dead_band_) {
      return steer_diff + steer_dead_band_;
    } else {
      return 0.0;
    }
  });
  double state_steer_rate_ =
    sat(-steer_diff_with_dead_band / steer_time_constant_, steer_rate_lim_, -steer_rate_lim_);

  const double c_steer = std::cos(steer);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vel * cos(yaw);
  d_state(IDX::Y) = vel * sin(yaw);
  d_state(IDX::YAW) =
    (vel * std::sin(steer) - state_steer_rate_ * front_wheelbase_ * c_steer * rear_slip_coeff_) /
    (front_wheelbase_ + rear_wheelbase_ * c_steer);
  d_state(IDX::VX) = acc;
  d_state(IDX::STEER) = state_steer_rate_;
  d_state(IDX::ACCX) = -(acc - acc_des) / acc_time_constant_;

  return d_state;
}

void SimModelDelayArticulateAccGeared::updateStateWithGear(
  Eigen::VectorXd & state, const Eigen::VectorXd & prev_state, const uint8_t gear, const double dt)
{
  const auto setStopState = [&]() {
    state(IDX::VX) = 0.0;
    state(IDX::X) = prev_state(IDX::X);
    state(IDX::Y) = prev_state(IDX::Y);
    state(IDX::YAW) = prev_state(IDX::YAW);
    state(IDX::ACCX) = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
  };

  using autoware_vehicle_msgs::msg::GearCommand;
  if (
    gear == GearCommand::DRIVE || gear == GearCommand::DRIVE_2 || gear == GearCommand::DRIVE_3 ||
    gear == GearCommand::DRIVE_4 || gear == GearCommand::DRIVE_5 || gear == GearCommand::DRIVE_6 ||
    gear == GearCommand::DRIVE_7 || gear == GearCommand::DRIVE_8 || gear == GearCommand::DRIVE_9 ||
    gear == GearCommand::DRIVE_10 || gear == GearCommand::DRIVE_11 ||
    gear == GearCommand::DRIVE_12 || gear == GearCommand::DRIVE_13 ||
    gear == GearCommand::DRIVE_14 || gear == GearCommand::DRIVE_15 ||
    gear == GearCommand::DRIVE_16 || gear == GearCommand::DRIVE_17 ||
    gear == GearCommand::DRIVE_18 || gear == GearCommand::LOW || gear == GearCommand::LOW_2) {
    if (state(IDX::VX) < 0.0) {
      setStopState();
    }
  } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
    if (state(IDX::VX) > 0.0) {
      setStopState();
    }
  } else if (gear == GearCommand::PARK) {
    setStopState();
  } else {
    setStopState();
  }
}
