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
#include "control_command_gate.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::control_command_gate::test
{

inline const LimitArray & reference_speed_points()
{
  static const LimitArray points{0.1, 0.3, 1.0, 3.0, 5.0, 20.0, 30.0};
  return points;
}

inline const LimitArray & provisional_steer_accel_lim()
{
  static const LimitArray limits{0.8, 0.76, 0.64, 0.39, 0.3, 0.3, 0.3};
  return limits;
}

inline VehicleCmdFilterParam make_filter_param()
{
  VehicleCmdFilterParam p;
  p.wheel_base = 4.76;
  p.vel_lim = 25.0;
  p.reference_speed_points = reference_speed_points();
  p.steer_cmd_lim = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.8};
  p.steer_rate_lim_for_steer_cmd = {0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6};
  p.lon_acc_lim_for_lon_vel = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 4.0};
  p.lon_jerk_lim_for_lon_acc = {80.0, 5.0, 5.0, 5.0, 5.0, 5.0, 4.0};
  p.lat_acc_lim_for_steer_cmd = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
  p.lat_jerk_lim_for_steer_cmd = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  p.lat_jerk_lim_for_steer_rate = 1.0;
  p.steer_cmd_diff_lim_from_current_steer = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.8};
  p.enable_steer_accel_limit = false;
  p.steer_accel_lim_for_steer_cmd = provisional_steer_accel_lim();
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

inline double tolerance_of_accel(const double max_abs_steer, const double dt)
{
  const float x = static_cast<float>(std::max(max_abs_steer, 1.0e-6));
  const double ulp = std::nextafter(x, std::numeric_limits<float>::infinity()) - x;
  return 2.0 * ulp / (dt * dt);
}

struct SteerAccelStep
{
  double dt;
  double speed;
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
    const double dt, const double speed, const Control & cmd, const double current_steer,
    const bool apply = true)
  {
    filter_.setCurrentSpeed(speed);
    filter_.setPrevCmd(prev_out_);
    filter_.setPrevSteerRates(prev_steer_rate_, prev_rotation_rate_);

    SteerAccelStep result{dt, speed, cmd, cmd, 0.0, 0.0};
    if (apply) {
      double clip = 0.0;
      double clip_field = 0.0;
      filter_.limitLateralSteerAccel(dt, result.stage, clip, clip_field);
    }
    IsFilterActivated activated;
    filter_.filterAll(
      dt, current_steer, result.out, activated, apply, result.steer_angle_rate_clip,
      result.steer_rotation_rate_clip);

    if (!apply) {
      prev_steer_rate_ = 0.0;
      prev_rotation_rate_ = result.out.lateral.steering_tire_rotation_rate;
    } else if (dt >= VehicleCmdFilter::DT_MIN_STEER_ACCEL_LIMIT) {
      prev_steer_rate_ =
        (result.out.lateral.steering_tire_angle - prev_out_.lateral.steering_tire_angle) /
        std::min(dt, VehicleCmdFilter::DT_MAX_STEER_ACCEL_LIMIT);
      prev_rotation_rate_ = result.out.lateral.steering_tire_rotation_rate;
    }
    prev_out_ = result.out;
    return result;
  }

  SteerAccelStep step_steer(
    const double dt, const double speed, const double steer, const bool apply = true)
  {
    Control cmd;
    cmd.lateral.steering_tire_angle = static_cast<float>(steer);
    cmd.lateral.steering_tire_rotation_rate = prev_out_.lateral.steering_tire_rotation_rate;
    return step(dt, speed, cmd, prev_out_.lateral.steering_tire_angle, apply);
  }

  const Control & prev_out() const { return prev_out_; }
  double prev_steer_rate() const { return prev_steer_rate_; }
  double prev_rotation_rate() const { return prev_rotation_rate_; }
  const VehicleCmdFilter & filter() const { return filter_; }

private:
  VehicleCmdFilter filter_;
  Control prev_out_;
  double prev_steer_rate_ = 0.0;
  double prev_rotation_rate_ = 0.0;
};

struct AccelSample
{
  double accel;
  double tolerance;
};

inline std::vector<AccelSample> steer_accels(
  const double steer0, const double rate0, const std::vector<double> & dts,
  const std::vector<double> & steers)
{
  std::vector<AccelSample> result;
  double prev_steer = steer0;
  double prev_rate = rate0;
  double prev_prev_steer = steer0;
  for (size_t i = 0; i < steers.size(); ++i) {
    const double rate = (steers.at(i) - prev_steer) / dts.at(i);
    const double max_abs =
      std::max({std::abs(steers.at(i)), std::abs(prev_steer), std::abs(prev_prev_steer)});
    result.push_back({(rate - prev_rate) / dts.at(i), tolerance_of_accel(max_abs, dts.at(i))});
    prev_prev_steer = prev_steer;
    prev_steer = steers.at(i);
    prev_rate = rate;
  }
  return result;
}

inline std::vector<rclcpp::Parameter> load_default_parameters()
{
  const auto path = ament_index_cpp::get_package_share_directory("autoware_control_command_gate") +
                    "/config/default.param.yaml";
  return rclcpp::parameter_map_from_yaml_file(path).at("/**");
}

inline const rclcpp::Parameter & find_parameter(
  const std::vector<rclcpp::Parameter> & params, const std::string & name)
{
  const auto it = std::find_if(
    params.begin(), params.end(), [&name](const auto & param) { return param.get_name() == name; });
  if (it == params.end()) {
    throw std::out_of_range("parameter not found: " + name);
  }
  return *it;
}

inline VehicleCmdFilterParam to_filter_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & ns, const double wheel_base)
{
  const auto get = [&params, &ns](const std::string & name) {
    return find_parameter(params, ns + name);
  };
  VehicleCmdFilterParam p;
  p.wheel_base = wheel_base;
  p.vel_lim = get("vel_lim").as_double();
  p.reference_speed_points = get("reference_speed_points").as_double_array();
  p.lon_acc_lim_for_lon_vel = get("lon_acc_lim_for_lon_vel").as_double_array();
  p.lon_jerk_lim_for_lon_acc = get("lon_jerk_lim_for_lon_acc").as_double_array();
  p.lat_acc_lim_for_steer_cmd = get("lat_acc_lim_for_steer_cmd").as_double_array();
  p.lat_jerk_lim_for_steer_cmd = get("lat_jerk_lim_for_steer_cmd").as_double_array();
  p.steer_cmd_lim = get("steer_cmd_lim").as_double_array();
  p.steer_rate_lim_for_steer_cmd = get("steer_rate_lim_for_steer_cmd").as_double_array();
  p.steer_cmd_diff_lim_from_current_steer =
    get("steer_cmd_diff_lim_from_current_steer").as_double_array();
  p.lat_jerk_lim_for_steer_rate = get("lat_jerk_lim_for_steer_rate").as_double();
  p.enable_steer_accel_limit = get("enable_steer_accel_limit").as_bool();
  p.steer_accel_lim_for_steer_cmd = get("steer_accel_lim_for_steer_cmd").as_double_array();
  p.steer_accel_clip_integral_th_diag = get("steer_accel_clip_integral_th_diag").as_double();
  return p;
}

inline std::vector<rclcpp::Parameter> vehicle_info_parameters()
{
  return {
    rclcpp::Parameter("wheel_radius", 0.39),  rclcpp::Parameter("wheel_width", 0.42),
    rclcpp::Parameter("wheel_base", 2.74),    rclcpp::Parameter("wheel_tread", 1.63),
    rclcpp::Parameter("front_overhang", 1.0), rclcpp::Parameter("rear_overhang", 1.03),
    rclcpp::Parameter("left_overhang", 0.1),  rclcpp::Parameter("right_overhang", 0.1),
    rclcpp::Parameter("vehicle_height", 2.5), rclcpp::Parameter("max_steer_angle", 0.7),
  };
}

inline std::shared_ptr<ControlCmdGate> create_gate(std::vector<rclcpp::Parameter> params)
{
  const auto vehicle_info = vehicle_info_parameters();
  params.insert(params.end(), vehicle_info.begin(), vehicle_info.end());
  rclcpp::NodeOptions options;
  options.parameter_overrides(params);
  options.use_clock_thread(false);
  return std::make_shared<ControlCmdGate>(options);
}

inline std::vector<rclcpp::Parameter> remove_parameters(
  std::vector<rclcpp::Parameter> params, const std::vector<std::string> & names)
{
  params.erase(
    std::remove_if(
      params.begin(), params.end(),
      [&names](const auto & param) {
        return std::find(names.begin(), names.end(), param.get_name()) != names.end();
      }),
    params.end());
  return params;
}

}  // namespace autoware::control_command_gate::test

#endif  // TEST_UTILS_HPP_
