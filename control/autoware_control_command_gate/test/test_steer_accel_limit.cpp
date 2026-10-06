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

#include <algorithm>
#include <cmath>
#include <functional>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace autoware::control_command_gate::test
{

namespace
{

constexpr double wheel_base = 4.76;

const std::vector<std::pair<double, double>> & accel_limit_by_speed()
{
  static const std::vector<std::pair<double, double>> table{
    {0.0, 0.8},  {0.1, 0.8},   {0.2, 0.78}, {0.3, 0.76},  {0.65, 0.7},
    {1.0, 0.64}, {2.0, 0.515}, {3.0, 0.39}, {4.0, 0.345}, {5.0, 0.3},
    {12.5, 0.3}, {20.0, 0.3},  {25.0, 0.3}, {30.0, 0.3},  {35.0, 0.3},
  };
  return table;
}

struct AccelSample
{
  double accel;
  double tolerance;
};

bool within_limit(const AccelSample & sample, const double limit)
{
  return std::abs(sample.accel) <= limit * (1.0 + 1e-9) + sample.tolerance;
}

std::vector<Control> outputs_of(const std::vector<SteerAccelStep> & steps)
{
  std::vector<Control> result;
  for (const auto & s : steps) {
    result.push_back(s.out);
  }
  return result;
}

std::vector<AccelSample> stage_accels(
  const double steer0, const double rate0, const std::vector<SteerAccelStep> & steps)
{
  std::vector<AccelSample> result;
  double prev_steer = steer0;
  double prev_rate = rate0;
  for (const auto & s : steps) {
    const double stage_rate = (s.stage.lateral.steering_tire_angle - prev_steer) / s.dt;
    const double max_abs = std::max(
      std::abs(static_cast<double>(s.stage.lateral.steering_tire_angle)), std::abs(prev_steer));
    result.push_back({(stage_rate - prev_rate) / s.dt, tolerance_of_accel(max_abs, s.dt)});
    const double out_rate = (s.out.lateral.steering_tire_angle - prev_steer) / s.dt;
    prev_steer = s.out.lateral.steering_tire_angle;
    prev_rate = out_rate;
  }
  return result;
}

struct SteerScenario
{
  std::string name;
  double initial_steer;
  std::function<double(double)> command;
};

std::vector<SteerScenario> step_ramp_reverse_scenarios()
{
  return {
    {"step", 0.0, [](double) { return 0.2; }},
    {"ramp", 0.0, [](double t) { return std::min(0.5 * t, 0.3); }},
    {"reverse", 0.0,
     [](double t) {
       if (t < 0.5) return 0.5 * t;
       if (t < 1.5) return 0.25 - 0.5 * (t - 0.5);
       return -0.25;
     }},
    {"step_to_zero", 0.3, [](double) { return 0.0; }},
  };
}

std::vector<SteerAccelStep> run_scenario(
  SteerAccelDriver & driver, const SteerScenario & scenario, const double speed,
  const std::vector<double> & dts)
{
  driver.reset(scenario.initial_steer, 0.0, 0.0);
  std::vector<SteerAccelStep> steps;
  double t = 0.0;
  for (const double dt : dts) {
    t += dt;
    steps.push_back(driver.step_steer(dt, speed, scenario.command(t)));
  }
  return steps;
}

void expect_accels_within(
  const std::vector<AccelSample> & accels, const double limit, const std::string & tag)
{
  for (size_t i = 0; i < accels.size(); ++i) {
    EXPECT_TRUE(within_limit(accels.at(i), limit))
      << tag << " k=" << i << " accel=" << accels.at(i).accel << " limit=" << limit;
  }
}

double steer_rate_limit(const double speed)
{
  return std::min(0.6, 1.0 * wheel_base / std::max(speed * speed, 0.001));
}

double lat_acc(const double speed, const double steer)
{
  return speed * speed * std::tan(steer) / wheel_base;
}

}  // namespace

TEST(SteerAccelLimit, SteerSideStaysWithinLimitAndConverges)
{
  for (const auto & [speed, limit] : accel_limit_by_speed()) {
    for (const auto & scenario : step_ramp_reverse_scenarios()) {
      SteerAccelDriver driver(make_isolated_steer_accel_param());
      const std::vector<double> dts(200, cycle);
      const auto steps = run_scenario(driver, scenario, speed, dts);
      const auto tag = scenario.name + " v=" + std::to_string(speed);
      expect_accels_within(stage_accels(scenario.initial_steer, 0.0, steps), limit, tag + " stage");
      expect_steer_accels_within(
        outputs_of(steps), 0, scenario.initial_steer, 0.0, dts, limit, tag + " out");

      const double target = scenario.command(200 * cycle);
      EXPECT_LT(std::abs(steps.back().out.lateral.steering_tire_angle - target), 1e-6)
        << scenario.name << " v=" << speed << " limit=" << limit;
      const double direction = target >= scenario.initial_steer ? 1.0 : -1.0;
      for (const auto & s : steps) {
        EXPECT_LE(direction * (s.stage.lateral.steering_tire_angle - target), 1e-4)
          << scenario.name << " v=" << speed;
      }
    }
  }
}

TEST(SteerAccelLimit, RotationRateIsLimitedIndependently)
{
  const double accel_lim = 0.8;
  {
    SteerAccelDriver driver(make_isolated_steer_accel_param());
    driver.reset(0.0, 0.0, 0.0);
    std::vector<SteerAccelStep> steps;
    for (int k = 0; k < 60; ++k) {
      Control cmd;
      cmd.lateral.steering_tire_rotation_rate = 0.5f;
      steps.push_back(driver.step(cycle, 0.0, cmd, 0.0));
      EXPECT_EQ(steps.back().out.lateral.steering_tire_angle, 0.0f) << "field only k=" << k;
    }
    expect_rotation_rate_accels_within(
      outputs_of(steps), 0, 0.0, std::vector<double>(steps.size(), cycle), accel_lim, "field only");
  }
  {
    SteerAccelDriver driver(make_isolated_steer_accel_param());
    driver.reset(0.0, 0.0, 0.0);
    std::vector<SteerAccelStep> steps;
    std::vector<double> dts;
    for (int k = 0; k < 60; ++k) {
      Control cmd;
      cmd.lateral.steering_tire_angle = 0.2f;
      steps.push_back(driver.step(cycle, 0.0, cmd, driver.prev_out().lateral.steering_tire_angle));
      dts.push_back(cycle);
      EXPECT_EQ(steps.back().out.lateral.steering_tire_rotation_rate, 0.0f) << "steer only k=" << k;
    }
    expect_steer_accels_within(outputs_of(steps), 0, 0.0, 0.0, dts, accel_lim, "steer only");
  }
  {
    SteerAccelDriver driver(make_isolated_steer_accel_param());
    driver.reset(0.0, 0.0, 0.0);
    std::vector<SteerAccelStep> steps;
    std::vector<double> dts;
    double prev_steer = 0.0;
    for (int k = 0; k < 20; ++k) {
      Control cmd;
      cmd.lateral.steering_tire_angle = -0.2f;
      cmd.lateral.steering_tire_rotation_rate = 0.5f;
      steps.push_back(driver.step(cycle, 0.0, cmd, driver.prev_out().lateral.steering_tire_angle));
      dts.push_back(cycle);
      const double new_rate = (steps.back().stage.lateral.steering_tire_angle - prev_steer) / cycle;
      EXPECT_NE(steps.back().out.lateral.steering_tire_rotation_rate, static_cast<float>(new_rate))
        << "both k=" << k;
      prev_steer = steps.back().out.lateral.steering_tire_angle;
    }
    expect_steer_accels_within(outputs_of(steps), 0, 0.0, 0.0, dts, accel_lim, "both");
    expect_rotation_rate_accels_within(outputs_of(steps), 0, 0.0, dts, accel_lim, "both field");
  }
}

TEST(SteerAccelLimit, DisabledFlagLeavesCommandUntouched)
{
  std::mt19937 engine(2);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  VehicleCmdFilter filter;
  filter.setParam(make_filter_param());
  for (const double dt : {0.0, 0.002, 0.03, 2.5}) {
    Control prev;
    prev.lateral.steering_tire_angle = static_cast<float>(uniform(engine));
    prev.lateral.steering_tire_rotation_rate = static_cast<float>(uniform(engine));
    filter.setPrevCmd(prev);
    filter.setPrevSteerRates(uniform(engine), uniform(engine));
    filter.setCurrentSpeed(10.0 * (uniform(engine) + 1.0));
    Control cmd;
    cmd.lateral.steering_tire_angle = static_cast<float>(uniform(engine));
    cmd.lateral.steering_tire_rotation_rate = static_cast<float>(2.0 * uniform(engine));
    cmd.longitudinal.velocity = static_cast<float>(10.0 * uniform(engine));
    cmd.longitudinal.acceleration = static_cast<float>(uniform(engine));
    cmd.longitudinal.jerk = static_cast<float>(uniform(engine));
    auto out = cmd;
    double clip = -1.0;
    double clip_field = -1.0;
    filter.limitLateralSteerAccel(dt, out, clip, clip_field);
    EXPECT_TRUE(out == cmd) << "dt=" << dt;
    EXPECT_EQ(clip, 0.0);
    EXPECT_EQ(clip_field, 0.0);
  }
}

TEST(SteerAccelLimit, SkipsShortCycleAndRestoresRotationRate)
{
  for (const double dt : {0.0, 0.004999, 0.005, 0.005001}) {
    VehicleCmdFilter filter;
    filter.setParam(make_steer_accel_param());
    filter.setCurrentSpeed(0.0);
    Control prev;
    prev.lateral.steering_tire_angle = 0.1f;
    filter.setPrevCmd(prev);
    filter.setPrevSteerRates(0.2, 0.3);
    Control cmd;
    cmd.lateral.steering_tire_angle = 0.5f;
    cmd.lateral.steering_tire_rotation_rate = 0.9f;
    auto out = cmd;
    double clip = 0.0;
    double clip_field = 0.0;
    filter.limitLateralSteerAccel(dt, out, clip, clip_field);
    const auto tag = "dt=" + std::to_string(dt);
    if (dt < VehicleCmdFilter::DT_MIN_STEER_ACCEL_LIMIT) {
      EXPECT_EQ(out.lateral.steering_tire_rotation_rate, 0.3f) << tag;
      EXPECT_EQ(out.lateral.steering_tire_angle, cmd.lateral.steering_tire_angle) << tag;
      EXPECT_EQ(clip, 0.0) << tag;
      EXPECT_EQ(clip_field, 0.0) << tag;
    } else {
      EXPECT_NEAR(out.lateral.steering_tire_rotation_rate, 0.3 + 0.8 * dt, 1e-6) << tag;
      EXPECT_NEAR(out.lateral.steering_tire_angle, 0.1 + (0.2 + 0.8 * dt) * dt, 1e-6) << tag;
      EXPECT_GT(clip, 0.0) << tag;
      EXPECT_GT(clip_field, 0.0) << tag;
    }
  }
}

TEST(SteerAccelLimit, IsAppliedBeforeExistingLateralStages)
{
  struct Case
  {
    std::string name;
    double speed;
    double prev_steer;
    double prev_rate;
    double command;
  };
  const std::vector<Case> cases{
    {"steer_limit", 0.0, 0.95, 0.5, 1.5},
    {"steer_rate", 0.0, 0.2, 0.58, 0.25},
    {"lat_jerk", 10.0, 0.02, 0.04, 0.1},
  };
  for (const auto & c : cases) {
    VehicleCmdFilter filter;
    filter.setParam(make_steer_accel_param());
    filter.setCurrentSpeed(c.speed);
    Control prev;
    prev.lateral.steering_tire_angle = static_cast<float>(c.prev_steer);
    filter.setPrevCmd(prev);
    filter.setPrevSteerRates(c.prev_rate, 0.0);
    Control cmd;
    cmd.lateral.steering_tire_angle = static_cast<float>(c.command);

    auto expected = cmd;
    double clip = 0.0;
    double clip_field = 0.0;
    filter.limitLateralSteerAccel(cycle, expected, clip, clip_field);
    filter.limitLateralSteer(expected);
    filter.limitLateralSteerRate(cycle, expected);
    filter.limitLongitudinalWithJerk(cycle, expected);
    filter.limitLongitudinalWithAcc(cycle, expected);
    filter.limitLongitudinalWithVel(expected);
    filter.limitLateralWithLatJerk(cycle, expected);
    filter.limitLateralWithLatAcc(cycle, expected);
    filter.limitActualSteerDiff(c.prev_steer, expected);

    auto out = cmd;
    IsFilterActivated activated;
    filter.filterAll(cycle, c.prev_steer, out, activated, true, clip, clip_field);
    EXPECT_TRUE(out == expected) << c.name;
  }

  VehicleCmdFilter filter;
  filter.setParam(make_steer_accel_param());
  filter.setCurrentSpeed(0.0);
  Control prev;
  prev.lateral.steering_tire_angle = 0.99f;
  filter.setPrevCmd(prev);
  filter.setPrevSteerRates(0.6, 0.0);
  Control cmd;
  cmd.lateral.steering_tire_angle = 1.0f;
  IsFilterActivated activated;
  double clip = 0.0;
  double clip_field = 0.0;
  filter.filterAll(cycle, 0.99, cmd, activated, true, clip, clip_field);
  EXPECT_LE(cmd.lateral.steering_tire_angle, 1.0f);
}

TEST(SteerAccelLimit, ExistingStageConstraintsHoldForRandomInputs)
{
  std::mt19937 engine(3);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::uniform_real_distribution<double> dt_dist(0.02, 0.1);
  std::uniform_real_distribution<double> speed_dist(0.0, 20.0);
  constexpr double eps = 1e-5;

  struct Violations
  {
    bool steer_rate = false;
    bool lat_jerk = false;
    bool lat_acc = false;
  };
  const auto final_violations =
    [&](const Control & out, const Control & prev, const double speed, const double dt) {
      Violations v;
      const double r_lim = steer_rate_limit(speed);
      v.steer_rate = std::abs(out.lateral.steering_tire_angle - prev.lateral.steering_tire_angle) >
                     r_lim * dt + eps;
      v.lat_jerk = std::abs(
                     lat_acc(speed, out.lateral.steering_tire_angle) -
                     lat_acc(speed, prev.lateral.steering_tire_angle)) > 1.0 * dt + eps;
      v.lat_acc = std::abs(lat_acc(speed, out.lateral.steering_tire_angle)) > 1.5 + eps;
      return v;
    };

  for (int i = 0; i < 10000; ++i) {
    const double speed = speed_dist(engine);
    const double dt = dt_dist(engine);
    const double r_lim = steer_rate_limit(speed);
    Control prev;
    prev.lateral.steering_tire_angle = static_cast<float>(0.9 * unit(engine));
    prev.lateral.steering_tire_rotation_rate = static_cast<float>(0.6 * unit(engine));
    const double prev_rate = 0.6 * unit(engine);
    const double current_steer = prev.lateral.steering_tire_angle + 1.5 * unit(engine);
    Control cmd;
    cmd.lateral.steering_tire_angle =
      static_cast<float>(prev.lateral.steering_tire_angle + 0.5 * unit(engine));
    cmd.lateral.steering_tire_rotation_rate = static_cast<float>(unit(engine));
    const auto tag = "i=" + std::to_string(i);

    std::vector<Control> finals;
    for (const bool enable : {true, false}) {
      auto p = make_filter_param();
      p.enable_steer_accel_limit = enable;
      VehicleCmdFilter filter;
      filter.setParam(p);
      filter.setCurrentSpeed(speed);
      filter.setPrevCmd(prev);
      filter.setPrevSteerRates(prev_rate, prev.lateral.steering_tire_rotation_rate);

      auto c = cmd;
      double clip = 0.0;
      double clip_field = 0.0;
      filter.limitLateralSteerAccel(dt, c, clip, clip_field);
      filter.limitLateralSteer(c);
      EXPECT_LE(std::abs(c.lateral.steering_tire_angle), 1.0 + eps) << tag;
      filter.limitLateralSteerRate(dt, c);
      EXPECT_LE(
        std::abs(c.lateral.steering_tire_angle - prev.lateral.steering_tire_angle),
        r_lim * dt + eps)
        << tag;
      EXPECT_LE(std::abs(c.lateral.steering_tire_rotation_rate), r_lim + eps) << tag;
      filter.limitLongitudinalWithJerk(dt, c);
      filter.limitLongitudinalWithAcc(dt, c);
      filter.limitLongitudinalWithVel(c);
      filter.limitLateralWithLatJerk(dt, c);
      EXPECT_LE(
        std::abs(
          lat_acc(speed, c.lateral.steering_tire_angle) -
          lat_acc(speed, prev.lateral.steering_tire_angle)),
        1.0 * dt + eps)
        << tag;
      filter.limitLateralWithLatAcc(dt, c);
      EXPECT_LE(std::abs(lat_acc(speed, c.lateral.steering_tire_angle)), 1.5 + eps) << tag;
      filter.limitActualSteerDiff(current_steer, c);
      EXPECT_LE(std::abs(c.lateral.steering_tire_angle - current_steer), 1.0 + eps) << tag;

      auto out = cmd;
      IsFilterActivated activated;
      filter.filterAll(dt, current_steer, out, activated, true, clip, clip_field);
      EXPECT_TRUE(out == c) << tag;
      finals.push_back(out);
    }

    const auto with_limit = final_violations(finals.at(0), prev, speed, dt);
    const auto without_limit = final_violations(finals.at(1), prev, speed, dt);
    EXPECT_FALSE(with_limit.steer_rate && !without_limit.steer_rate) << tag;
    EXPECT_FALSE(with_limit.lat_jerk && !without_limit.lat_jerk) << tag;
    EXPECT_FALSE(with_limit.lat_acc && !without_limit.lat_acc) << tag;
  }
}

}  // namespace autoware::control_command_gate::test
