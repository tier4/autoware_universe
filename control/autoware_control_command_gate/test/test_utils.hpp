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

#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

#include "common/control_command_filter.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace autoware::control_command_gate::test
{

constexpr double cycle = 0.03;

inline VehicleCmdFilterParam make_filter_param()
{
  VehicleCmdFilterParam p;
  p.wheel_base = 4.76;
  p.vel_lim = 25.0;
  p.reference_speed_points = {0.1, 0.3, 1.0, 3.0, 5.0, 20.0, 30.0};
  p.steer_cmd_lim = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.8};
  p.steer_rate_lim_for_steer_cmd = {0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6};
  p.lon_acc_lim_for_lon_vel = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 4.0};
  p.lon_jerk_lim_for_lon_acc = {80.0, 5.0, 5.0, 5.0, 5.0, 5.0, 4.0};
  p.lat_acc_lim_for_steer_cmd = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
  p.lat_jerk_lim_for_steer_cmd = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  p.lat_jerk_lim_for_steer_rate = 1.0;
  p.steer_cmd_diff_lim_from_current_steer = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.8};
  p.enable_steer_accel_limit = false;
  p.steer_accel_lim_for_steer_cmd = {0.8, 0.76, 0.64, 0.39, 0.3, 0.3, 0.3};
  p.steer_accel_clip_integral_th_diag = 0.2;
  return p;
}

inline VehicleCmdFilterParam make_steer_accel_param()
{
  auto p = make_filter_param();
  p.enable_steer_accel_limit = true;
  return p;
}

inline VehicleCmdFilterParam make_isolated_steer_accel_param()
{
  auto p = make_steer_accel_param();
  p.lat_acc_lim_for_steer_cmd.assign(p.reference_speed_points.size(), 1000.0);
  p.lat_jerk_lim_for_steer_cmd.assign(p.reference_speed_points.size(), 1000.0);
  p.lat_jerk_lim_for_steer_rate = 1000.0;
  return p;
}

inline double accel_limit_of(const VehicleCmdFilterParam & p, const double speed)
{
  VehicleCmdFilter filter;
  filter.setParam(p);
  filter.setCurrentSpeed(speed);
  return filter.getSteerAccelLimForSteerCmd();
}

inline double tolerance_of_accel(const double max_abs_steer, const double dt)
{
  const float x = static_cast<float>(std::max(max_abs_steer, 1.0e-6));
  const double ulp = std::nextafter(x, std::numeric_limits<float>::infinity()) - x;
  return 2.0 * ulp / (dt * dt);
}

struct SteerAccelStep
{
  double dt;
  Control stage;
  Control out;
  double steer_angle_rate_clip;
  double steer_rotation_rate_clip;
};

class SteerAccelDriver
{
public:
  explicit SteerAccelDriver(const VehicleCmdFilterParam & p) { filter_.setParam(p); }

  void reset(const double steer, const double steer_rate, const double rotation_rate)
  {
    prev_out_ = Control();
    prev_out_.lateral.steering_tire_angle = static_cast<float>(steer);
    prev_out_.lateral.steering_tire_rotation_rate = static_cast<float>(rotation_rate);
    prev_steer_rate_ = steer_rate;
    prev_rotation_rate_ = rotation_rate;
  }

  SteerAccelStep step(
    const double dt, const double speed, const Control & cmd, const double current_steer)
  {
    filter_.setCurrentSpeed(speed);
    filter_.setPrevCmd(prev_out_);
    filter_.setPrevSteerRates(prev_steer_rate_, prev_rotation_rate_);

    SteerAccelStep result{dt, cmd, cmd, 0.0, 0.0};
    double clip = 0.0;
    double clip_field = 0.0;
    filter_.limitLateralSteerAccel(dt, result.stage, clip, clip_field);
    IsFilterActivated activated;
    filter_.filterAll(
      dt, current_steer, result.out, activated, true, result.steer_angle_rate_clip,
      result.steer_rotation_rate_clip);

    if (dt >= VehicleCmdFilter::DT_MIN_STEER_ACCEL_LIMIT) {
      prev_steer_rate_ =
        (result.out.lateral.steering_tire_angle - prev_out_.lateral.steering_tire_angle) /
        std::min(dt, VehicleCmdFilter::DT_MAX_STEER_ACCEL_LIMIT);
      prev_rotation_rate_ = result.out.lateral.steering_tire_rotation_rate;
    }
    prev_out_ = result.out;
    return result;
  }

  SteerAccelStep step_steer(const double dt, const double speed, const double steer)
  {
    Control cmd;
    cmd.lateral.steering_tire_angle = static_cast<float>(steer);
    cmd.lateral.steering_tire_rotation_rate = prev_out_.lateral.steering_tire_rotation_rate;
    return step(dt, speed, cmd, prev_out_.lateral.steering_tire_angle);
  }

  const Control & prev_out() const { return prev_out_; }

private:
  VehicleCmdFilter filter_;
  Control prev_out_;
  double prev_steer_rate_ = 0.0;
  double prev_rotation_rate_ = 0.0;
};

inline void expect_steer_accels_within(
  const std::vector<Control> & outputs, const size_t begin, const double steer0, const double rate0,
  const std::vector<double> & dts, const double limit, const std::string & tag)
{
  double prev_steer = steer0;
  double prev_rate = rate0;
  double prev_prev_steer = steer0;
  for (size_t k = begin; k < outputs.size(); ++k) {
    const double steer = outputs.at(k).lateral.steering_tire_angle;
    const double rate = (steer - prev_steer) / dts.at(k);
    const double accel = (rate - prev_rate) / dts.at(k);
    const double max_abs =
      std::max({std::abs(steer), std::abs(prev_steer), std::abs(prev_prev_steer)});
    EXPECT_LE(std::abs(accel), limit * (1.0 + 1e-9) + tolerance_of_accel(max_abs, dts.at(k)))
      << tag << " k=" << k << " accel=" << accel << " limit=" << limit;
    prev_prev_steer = prev_steer;
    prev_steer = steer;
    prev_rate = rate;
  }
}

inline void expect_rotation_rate_accels_within(
  const std::vector<Control> & outputs, const size_t begin, const double rate0,
  const std::vector<double> & dts, const double limit, const std::string & tag)
{
  double prev = rate0;
  for (size_t k = begin; k < outputs.size(); ++k) {
    const double rate = outputs.at(k).lateral.steering_tire_rotation_rate;
    EXPECT_LE(std::abs(rate - prev) / dts.at(k), limit * (1.0 + 1e-9) + 1e-4) << tag << " k=" << k;
    prev = rate;
  }
}

}  // namespace autoware::control_command_gate::test

#endif  // TEST_UTILS_HPP_
