// Copyright 2025 The Autoware Contributors
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

#include "filter.hpp"

#include <autoware_command_mode_types/sources.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace autoware::control_command_gate
{

CommandFilter::CommandFilter(std::unique_ptr<CommandOutput> && output, rclcpp::Node & node)
: CommandBridge(std::move(output)), node_(node), vehicle_status_(node)
{
  enable_command_limit_filter_ = node_.declare_parameter<bool>("enable_command_limit_filter");
  transition_flag_ = false;
  debug_pub_ = node_.create_publisher<Float32MultiArrayStamped>("~/debug/steer_accel_limit", 1);
}

void CommandFilter::set_nominal_filter_params(const VehicleCmdFilterParam & p)
{
  nominal_filter_.setParam(p);
}

void CommandFilter::set_transition_filter_params(const VehicleCmdFilterParam & p)
{
  transition_filter_.setParam(p);
}

void CommandFilter::set_transition_flag(bool flag)
{
  transition_flag_ = flag;
}

ClipDiag * CommandFilter::create_diag_task()
{
  if (clip_diag_) {
    throw std::logic_error("clip diag has already been created");
  }
  clip_diag_ = std::make_unique<ClipDiag>("steer_accel_limit");
  return clip_diag_.get();
}

double CommandFilter::get_delta_time()
{
  const auto curr_time = node_.now();
  if (!prev_time_) {
    prev_time_ = curr_time;
    return 0.0;
  }
  const auto delta_time = (curr_time - *prev_time_).seconds();
  prev_time_ = curr_time;
  return delta_time;
}

Control CommandFilter::filter_command(uint16_t source_id, const Control & msg)
{
  const auto dt = get_delta_time();
  const bool apply_steer_accel_limit = source_id != autoware::command_mode_types::sources::builtin;
  const auto current_steering = vehicle_status_.get_current_steering();
  const auto current_velocity = vehicle_status_.get_current_velocity();

  IsFilterActivated is_filter_activated;
  Control out = msg;

  nominal_filter_.setCurrentSpeed(current_velocity);
  transition_filter_.setCurrentSpeed(current_velocity);

  const auto & filter = transition_flag_ ? transition_filter_ : nominal_filter_;
  double steer_angle_rate_clip = 0.0;
  double steer_rotation_rate_clip = 0.0;
  filter.filterAll(
    dt, current_steering, out, is_filter_activated, apply_steer_accel_limit, steer_angle_rate_clip,
    steer_rotation_rate_clip);

  // set prev value for both to keep consistency over switching:
  // Actual steer, vel, acc should be considered in manual mode to prevent sudden motion when
  // switching from manual to autonomous
  const auto is_autoware_control_enabled = vehicle_status_.is_autoware_control_enabled();
  const auto is_vehicle_stopped = vehicle_status_.is_vehicle_stopped();
  const auto current_status_command = vehicle_status_.get_actual_status_as_command();
  Control prev_command = is_autoware_control_enabled ? out : current_status_command;
  if (is_vehicle_stopped) {
    prev_command.longitudinal = out.longitudinal;
  }

  const bool is_valid_steer_accel_cycle = dt >= VehicleCmdFilter::DT_MIN_STEER_ACCEL_LIMIT;
  const double steer_accel_dt = std::min(dt, VehicleCmdFilter::DT_MAX_STEER_ACCEL_LIMIT);
  const double out_rotation_rate = out.lateral.steering_tire_rotation_rate;

  Float32MultiArrayStamped debug;
  debug.stamp = node_.now();
  debug.data.resize(2, std::numeric_limits<float>::quiet_NaN());
  const double steer_accel_lim = filter.getSteerAccelLimForSteerCmd();
  debug.data.push_back(static_cast<float>(steer_accel_lim));
  debug.data.push_back(static_cast<float>(-steer_accel_lim));

  const auto set_prev_steer_rates = [this](const double angle_rate, const double rotation_rate) {
    nominal_filter_.setPrevSteerRates(angle_rate, rotation_rate);
    transition_filter_.setPrevSteerRates(angle_rate, rotation_rate);
  };
  if (is_valid_steer_accel_cycle) {
    const double steer_angle_rate =
      (out.lateral.steering_tire_angle - filter.getPrevCmd().lateral.steering_tire_angle) /
      steer_accel_dt;
    debug.data.at(0) =
      static_cast<float>((steer_angle_rate - filter.getPrevSteerAngleRate()) / steer_accel_dt);
    debug.data.at(1) =
      static_cast<float>((out_rotation_rate - filter.getPrevSteerRotationRate()) / steer_accel_dt);
    if (is_autoware_control_enabled && apply_steer_accel_limit) {
      set_prev_steer_rates(steer_angle_rate, out_rotation_rate);
    }
  }
  if (!is_autoware_control_enabled) {
    set_prev_steer_rates(0.0, 0.0);
  } else if (!apply_steer_accel_limit) {
    set_prev_steer_rates(0.0, out_rotation_rate);
  }

  const auto integrate_clip = [steer_accel_dt](double & integral, const double clip) {
    integral = clip > 0.0 ? integral + clip * steer_accel_dt : 0.0;
  };
  if (!apply_steer_accel_limit) {
    steer_angle_rate_clip_integral_ = 0.0;
    steer_rotation_rate_clip_integral_ = 0.0;
  } else if (is_valid_steer_accel_cycle) {
    integrate_clip(steer_angle_rate_clip_integral_, steer_angle_rate_clip);
    integrate_clip(steer_rotation_rate_clip_integral_, steer_rotation_rate_clip);
    const double threshold = filter.getParam().steer_accel_clip_integral_th_diag;
    if (
      steer_angle_rate_clip_integral_ > threshold ||
      steer_rotation_rate_clip_integral_ > threshold) {
      if (clip_diag_) {
        clip_diag_->notify();
      }
      steer_angle_rate_clip_integral_ = 0.0;
      steer_rotation_rate_clip_integral_ = 0.0;
    }
  }

  // TODO(Horibe): To prevent sudden acceleration/deceleration when switching from manual to
  // autonomous, the filter should be applied for actual speed and acceleration during manual
  // driving. However, this means that the output command from Gate will always be close to the
  // driving state during manual driving. Here, let autoware publish the stop command when the ego
  // is stopped to intend the autoware is trying to keep stopping.
  nominal_filter_.setPrevCmd(prev_command);
  transition_filter_.setPrevCmd(prev_command);

  debug_pub_->publish(debug);

  return out;
}

void CommandFilter::on_control(uint16_t source_id, const Control & msg)
{
  const auto out = enable_command_limit_filter_ ? filter_command(source_id, msg) : msg;
  CommandBridge::on_control(source_id, out);
}

}  // namespace autoware::control_command_gate
