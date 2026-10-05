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

#include "common/control_command_filter.hpp"
#include "test_utils.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::control_command_gate::test
{

namespace
{

constexpr double wheel_base = 4.76;

void expect_set_param_exits(const VehicleCmdFilterParam & p)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  VehicleCmdFilter filter;
  EXPECT_EXIT(filter.setParam(p), ::testing::ExitedWithCode(EXIT_FAILURE), "");
}

VehicleCmdFilterParam make_previous_default_nominal_param()
{
  VehicleCmdFilterParam p;
  p.wheel_base = wheel_base;
  p.vel_lim = 25.0;
  p.reference_speed_points = {0.1, 0.3, 20.0, 30.0};
  p.steer_cmd_lim = {1.0, 1.0, 1.0, 0.8};
  p.steer_rate_lim_for_steer_cmd = {1.0, 1.0, 1.0, 0.8};
  p.lon_acc_lim_for_lon_vel = {5.0, 5.0, 5.0, 4.0};
  p.lon_jerk_lim_for_lon_acc = {80.0, 5.0, 5.0, 4.0};
  p.lat_acc_lim_for_steer_cmd = {5.0, 5.0, 5.0, 4.0};
  p.lat_jerk_lim_for_steer_cmd = {7.0, 7.0, 7.0, 6.0};
  p.lat_jerk_lim_for_steer_rate = 10.0;
  p.steer_cmd_diff_lim_from_current_steer = {1.0, 1.0, 1.0, 0.8};
  p.enable_steer_accel_limit = false;
  p.steer_accel_lim_for_steer_cmd = {1.0, 1.0, 1.0, 1.0};
  p.steer_accel_clip_integral_th_diag = 0.2;
  return p;
}

VehicleCmdFilterParam make_previous_default_transition_param()
{
  VehicleCmdFilterParam p;
  p.wheel_base = wheel_base;
  p.vel_lim = 50.0;
  p.reference_speed_points = {20.0, 30.0};
  p.steer_cmd_lim = {1.0, 0.8};
  p.steer_rate_lim_for_steer_cmd = {1.0, 0.8};
  p.lon_acc_lim_for_lon_vel = {1.0, 0.9};
  p.lon_jerk_lim_for_lon_acc = {0.5, 0.4};
  p.lat_acc_lim_for_steer_cmd = {2.0, 1.8};
  p.lat_jerk_lim_for_steer_cmd = {7.0, 6.0};
  p.lat_jerk_lim_for_steer_rate = 10.0;
  p.steer_cmd_diff_lim_from_current_steer = {1.0, 0.8};
  p.enable_steer_accel_limit = false;
  p.steer_accel_lim_for_steer_cmd = {1.0, 1.0};
  p.steer_accel_clip_integral_th_diag = 0.2;
  return p;
}

Control make_large_command()
{
  Control cmd;
  cmd.lateral.steering_tire_angle = 1.4;
  cmd.lateral.steering_tire_rotation_rate = 10.0;
  cmd.longitudinal.velocity = 100.0;
  cmd.longitudinal.acceleration = 100.0;
  cmd.longitudinal.jerk = 100.0;
  return cmd;
}

void expect_same_command(const Control & expected, const Control & actual, const std::string & tag)
{
  EXPECT_FLOAT_EQ(expected.lateral.steering_tire_angle, actual.lateral.steering_tire_angle) << tag;
  EXPECT_FLOAT_EQ(
    expected.lateral.steering_tire_rotation_rate, actual.lateral.steering_tire_rotation_rate)
    << tag;
  EXPECT_FLOAT_EQ(expected.longitudinal.velocity, actual.longitudinal.velocity) << tag;
  EXPECT_FLOAT_EQ(expected.longitudinal.acceleration, actual.longitudinal.acceleration) << tag;
  EXPECT_FLOAT_EQ(expected.longitudinal.jerk, actual.longitudinal.jerk) << tag;
}

void expect_same_limits(
  const VehicleCmdFilterParam & previous, const VehicleCmdFilterParam & current)
{
  VehicleCmdFilter previous_filter;
  VehicleCmdFilter current_filter;
  previous_filter.setParam(previous);
  current_filter.setParam(current);

  const std::vector<
    std::pair<std::string, std::function<void(const VehicleCmdFilter &, Control &)>>>
    stages{
      {"steer", [](const auto & f, auto & c) { f.limitLateralSteer(c); }},
      {"steer_rate", [](const auto & f, auto & c) { f.limitLateralSteerRate(1.0, c); }},
      {"lon_acc", [](const auto & f, auto & c) { f.limitLongitudinalWithAcc(1.0, c); }},
      {"lon_jerk", [](const auto & f, auto & c) { f.limitLongitudinalWithJerk(1.0, c); }},
      {"lat_acc", [](const auto & f, auto & c) { f.limitLateralWithLatAcc(1.0, c); }},
      {"lat_jerk", [](const auto & f, auto & c) { f.limitLateralWithLatJerk(1.0, c); }},
      {"actual_steer_diff", [](const auto & f, auto & c) { f.limitActualSteerDiff(0.0, c); }},
    };

  for (int i = 0; i <= 3500; ++i) {
    const double v = 0.01 * i;
    previous_filter.setCurrentSpeed(v);
    current_filter.setCurrentSpeed(v);
    for (const auto & [name, stage] : stages) {
      for (const double sign : {1.0, -1.0}) {
        auto previous_cmd = make_large_command();
        previous_cmd.lateral.steering_tire_angle *= sign;
        previous_cmd.lateral.steering_tire_rotation_rate *= sign;
        previous_cmd.longitudinal.acceleration *= sign;
        auto current_cmd = previous_cmd;
        stage(previous_filter, previous_cmd);
        stage(current_filter, current_cmd);
        expect_same_command(previous_cmd, current_cmd, name + " v=" + std::to_string(v));
      }
    }
  }
}

}  // namespace

TEST(VehicleCmdFilterSteerAccelParam, InterpolatesSteerAccelLimitBySpeed)
{
  VehicleCmdFilter filter;
  filter.setParam(make_filter_param());

  const std::vector<std::pair<double, double>> expected{
    {-5.0, 0.3}, {-0.1, 0.8}, {0.0, 0.8},  {0.05, 0.8},  {0.1, 0.8},  {0.2, 0.78},
    {0.3, 0.76}, {0.65, 0.7}, {1.0, 0.64}, {2.0, 0.515}, {3.0, 0.39}, {4.0, 0.345},
    {5.0, 0.3},  {12.5, 0.3}, {20.0, 0.3}, {25.0, 0.3},  {30.0, 0.3}, {40.0, 0.3},
  };
  for (const auto & [v, limit] : expected) {
    filter.setCurrentSpeed(v);
    EXPECT_NEAR(filter.getSteerAccelLimForSteerCmd(), limit, 1e-12) << "v=" << v;
  }
}

TEST(VehicleCmdFilterSteerAccelParamDeathTest, ExitsWhenSteerAccelLimitIsShorter)
{
  auto p = make_filter_param();
  p.steer_accel_lim_for_steer_cmd.pop_back();
  expect_set_param_exits(p);
}

TEST(VehicleCmdFilterSteerAccelParamDeathTest, ExitsWhenSteerAccelLimitIsLonger)
{
  auto p = make_filter_param();
  p.steer_accel_lim_for_steer_cmd.push_back(0.3);
  expect_set_param_exits(p);
}

TEST(VehicleCmdFilterSteerAccelParamDeathTest, ExitsWhenOnlyReferenceSpeedPointsAreRefined)
{
  auto p = make_previous_default_nominal_param();
  p.reference_speed_points = reference_speed_points();
  expect_set_param_exits(p);
}

TEST(VehicleCmdFilterSteerAccelParamDeathTest, ExitsWhenSteerAccelLimitIsZero)
{
  auto p = make_filter_param();
  p.steer_accel_lim_for_steer_cmd.assign(p.steer_accel_lim_for_steer_cmd.size(), 0.0);
  expect_set_param_exits(p);
}

TEST(VehicleCmdFilterSteerAccelParamDeathTest, ExitsWhenSteerAccelLimitHasNegativeElement)
{
  auto p = make_filter_param();
  p.steer_accel_lim_for_steer_cmd.at(3) = -0.39;
  expect_set_param_exits(p);
}

TEST(VehicleCmdFilterSteerAccelParam, AcceptsTinyPositiveSteerAccelLimit)
{
  auto p = make_filter_param();
  p.steer_accel_lim_for_steer_cmd.assign(p.steer_accel_lim_for_steer_cmd.size(), 1e-9);
  VehicleCmdFilter filter;
  filter.setParam(p);
  EXPECT_EQ(filter.getParam().steer_accel_lim_for_steer_cmd, p.steer_accel_lim_for_steer_cmd);
}

TEST(VehicleCmdFilterSteerAccelParam, RefinedDefaultConfigKeepsExistingNominalLimits)
{
  const auto params = load_default_parameters();
  expect_same_limits(
    make_previous_default_nominal_param(), to_filter_param(params, "nominal_filter.", wheel_base));
}

TEST(VehicleCmdFilterSteerAccelParam, RefinedDefaultConfigKeepsExistingTransitionLimits)
{
  const auto params = load_default_parameters();
  expect_same_limits(
    make_previous_default_transition_param(),
    to_filter_param(params, "transition_filter.", wheel_base));
}

}  // namespace autoware::control_command_gate::test
