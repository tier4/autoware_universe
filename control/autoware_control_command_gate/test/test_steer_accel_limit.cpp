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
constexpr double cycle = 0.03;

const std::vector<std::pair<double, double>> & accel_limit_by_speed()
{
  static const std::vector<std::pair<double, double>> table{
    {0.0, 0.8},  {0.1, 0.8},   {0.2, 0.78}, {0.3, 0.76},  {0.65, 0.7},
    {1.0, 0.64}, {2.0, 0.515}, {3.0, 0.39}, {4.0, 0.345}, {5.0, 0.3},
    {12.5, 0.3}, {20.0, 0.3},  {25.0, 0.3}, {30.0, 0.3},  {35.0, 0.3},
  };
  return table;
}

bool within_limit(const AccelSample & sample, const double limit)
{
  return std::abs(sample.accel) <= limit * (1.0 + 1e-9) + sample.tolerance;
}

std::vector<double> steers_of(const std::vector<SteerAccelStep> & steps, const bool stage)
{
  std::vector<double> result;
  for (const auto & s : steps) {
    result.push_back(
      stage ? s.stage.lateral.steering_tire_angle : s.out.lateral.steering_tire_angle);
  }
  return result;
}

std::vector<double> rotation_rates_of(const std::vector<SteerAccelStep> & steps)
{
  std::vector<double> result;
  for (const auto & s : steps) {
    result.push_back(s.out.lateral.steering_tire_rotation_rate);
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

TEST(SteerAccelLimit, SteerSideStaysWithinLimit)
{
  for (const auto & [speed, limit] : accel_limit_by_speed()) {
    for (const auto & scenario : step_ramp_reverse_scenarios()) {
      SteerAccelDriver driver(make_isolated_steer_accel_param());
      const std::vector<double> dts(200, cycle);
      const auto steps = run_scenario(driver, scenario, speed, dts);
      const auto tag = scenario.name + " v=" + std::to_string(speed);
      expect_accels_within(stage_accels(scenario.initial_steer, 0.0, steps), limit, tag + " stage");
      expect_accels_within(
        steer_accels(scenario.initial_steer, 0.0, dts, steers_of(steps, false)), limit,
        tag + " out");
    }
  }
}

TEST(SteerAccelLimit, SteerSideConvergesToCommand)
{
  for (const auto & [speed, limit] : accel_limit_by_speed()) {
    for (const auto & scenario : step_ramp_reverse_scenarios()) {
      SteerAccelDriver driver(make_isolated_steer_accel_param());
      const std::vector<double> dts(200, cycle);
      const auto steps = run_scenario(driver, scenario, speed, dts);
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

TEST(SteerAccelLimit, KeepsCommandWithinBand)
{
  for (const double speed : {0.3, 3.0, 20.0}) {
    SteerAccelDriver driver(make_isolated_steer_accel_param());
    VehicleCmdFilter probe;
    probe.setParam(make_isolated_steer_accel_param());
    probe.setCurrentSpeed(speed);
    const double accel_lim = probe.getSteerAccelLimForSteerCmd();
    const double amplitude = 0.005;
    const double omega = 0.5 * accel_lim * cycle / amplitude;
    ASSERT_LE(amplitude * omega * omega, 0.9 * accel_lim);
    ::testing::Test::RecordProperty(
      "band_sine_frequency_v" + std::to_string(static_cast<int>(speed * 10)),
      std::to_string(omega / (2.0 * M_PI)));
    ::testing::Test::RecordProperty(
      "band_sine_max_rate_v" + std::to_string(static_cast<int>(speed * 10)),
      std::to_string(accel_lim * cycle));
    const auto steer_at = [&](double t) { return amplitude * std::sin(omega * t); };
    const auto rate_at = [&](double t) { return amplitude * omega * std::cos(omega * t); };
    const float s_m1 = static_cast<float>(steer_at(-cycle));
    const float s_m2 = static_cast<float>(steer_at(-2.0 * cycle));
    driver.reset(s_m1, (s_m1 - s_m2) / cycle, static_cast<float>(rate_at(-cycle)));
    for (int k = 0; k < 400; ++k) {
      const double t = k * cycle;
      Control cmd;
      cmd.lateral.steering_tire_angle = static_cast<float>(steer_at(t));
      cmd.lateral.steering_tire_rotation_rate = static_cast<float>(rate_at(t));
      const auto s = driver.step(cycle, speed, cmd, driver.prev_out().lateral.steering_tire_angle);
      const auto tag = "v=" + std::to_string(speed) + " k=" + std::to_string(k);
      EXPECT_EQ(s.stage.lateral.steering_tire_angle, cmd.lateral.steering_tire_angle) << tag;
      EXPECT_EQ(
        s.stage.lateral.steering_tire_rotation_rate, cmd.lateral.steering_tire_rotation_rate)
        << tag;
      EXPECT_NEAR(s.out.lateral.steering_tire_angle, cmd.lateral.steering_tire_angle, 1e-7) << tag;
      EXPECT_EQ(s.out.lateral.steering_tire_rotation_rate, cmd.lateral.steering_tire_rotation_rate)
        << tag;
      EXPECT_EQ(s.steer_angle_rate_clip, 0.0) << tag;
      EXPECT_EQ(s.steer_rotation_rate_clip, 0.0) << tag;
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
    const auto rates = rotation_rates_of(steps);
    double prev = 0.0;
    for (size_t k = 0; k < rates.size(); ++k) {
      EXPECT_LE(std::abs(rates.at(k) - prev) / cycle, accel_lim * (1.0 + 1e-9) + 1e-4)
        << "field only k=" << k;
      prev = rates.at(k);
    }
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
    expect_accels_within(
      steer_accels(0.0, 0.0, dts, steers_of(steps, false)), accel_lim, "steer only");
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
    expect_accels_within(steer_accels(0.0, 0.0, dts, steers_of(steps, false)), accel_lim, "both");
    const auto rates = rotation_rates_of(steps);
    double prev = 0.0;
    for (size_t k = 0; k < rates.size(); ++k) {
      EXPECT_LE(std::abs(rates.at(k) - prev) / cycle, accel_lim * (1.0 + 1e-9) + 1e-4)
        << "both field k=" << k;
      prev = rates.at(k);
    }
  }
}

TEST(SteerAccelLimit, StaysWithinLimitUnderVaryingCycle)
{
  std::mt19937 engine(1);
  std::uniform_real_distribution<double> jitter(0.02, 0.05);
  std::bernoulli_distribution choose_slow(0.5);
  std::vector<std::pair<std::string, std::vector<double>>> dt_sequences{
    {"jitter", {}}, {"slow", std::vector<double>(200, 0.1)}, {"mixed", {}}};
  for (int k = 0; k < 200; ++k) {
    dt_sequences.at(0).second.push_back(jitter(engine));
    dt_sequences.at(2).second.push_back(choose_slow(engine) ? 0.1 : 0.03);
  }
  for (const double speed : {0.0, 1.0, 3.0, 10.0}) {
    VehicleCmdFilter probe;
    probe.setParam(make_isolated_steer_accel_param());
    probe.setCurrentSpeed(speed);
    const double limit = probe.getSteerAccelLimForSteerCmd();
    for (const auto & [dt_name, dts] : dt_sequences) {
      auto scenarios = step_ramp_reverse_scenarios();
      scenarios.pop_back();
      for (const auto & scenario : scenarios) {
        SteerAccelDriver driver(make_isolated_steer_accel_param());
        const auto steps = run_scenario(driver, scenario, speed, dts);
        const auto tag = scenario.name + " " + dt_name + " v=" + std::to_string(speed);
        expect_accels_within(
          stage_accels(scenario.initial_steer, 0.0, steps), limit, tag + " stage");
        expect_accels_within(
          steer_accels(scenario.initial_steer, 0.0, dts, steers_of(steps, false)), limit,
          tag + " out");
      }
    }
  }
}

TEST(SteerAccelLimit, DisabledFlagLeavesCommandUntouched)
{
  std::mt19937 engine(2);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  VehicleCmdFilter filter;
  filter.setParam(make_filter_param());
  for (int i = 0; i < 1000; ++i) {
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
      EXPECT_TRUE(out == cmd) << "i=" << i << " dt=" << dt;
      EXPECT_EQ(clip, 0.0);
      EXPECT_EQ(clip_field, 0.0);
    }
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

  SteerAccelDriver driver(make_isolated_steer_accel_param());
  driver.reset(0.0, 0.3, 0.0);
  for (int k = 0; k < 3; ++k) {
    const double prev_rate = driver.prev_steer_rate();
    const auto s = driver.step_steer(0.002, 0.0, 1.0);
    EXPECT_EQ(s.steer_angle_rate_clip, 0.0) << "short k=" << k;
    EXPECT_EQ(driver.prev_steer_rate(), prev_rate) << "short k=" << k;
  }
  const double prev_steer = driver.prev_out().lateral.steering_tire_angle;
  const double prev_rate = driver.prev_steer_rate();
  const auto s = driver.step_steer(cycle, 0.0, 1.0);
  const double new_rate = (s.stage.lateral.steering_tire_angle - prev_steer) / cycle;
  EXPECT_NEAR(new_rate - prev_rate, 0.8 * cycle, 1e-4);
}

TEST(SteerAccelLimit, CapsLongCycle)
{
  for (const double dt : {0.999, 1.0, 1.0001, 2.5}) {
    const double dt_lim = std::min(dt, 1.0);
    for (const double command_rate : {0.6, 10.0}) {
      VehicleCmdFilter filter;
      filter.setParam(make_steer_accel_param());
      filter.setCurrentSpeed(0.0);
      filter.setPrevCmd(Control());
      filter.setPrevSteerRates(0.0, 0.0);
      Control cmd;
      cmd.lateral.steering_tire_angle = static_cast<float>(command_rate * dt);
      double clip = 0.0;
      double clip_field = 0.0;
      filter.limitLateralSteerAccel(dt, cmd, clip, clip_field);
      const double new_rate = cmd.lateral.steering_tire_angle / dt_lim;
      const auto tag = "dt=" + std::to_string(dt) + " rate=" + std::to_string(command_rate);
      EXPECT_LE(std::abs(new_rate), 0.8 * dt_lim + 1e-6) << tag;
      if (command_rate > 1.0) {
        EXPECT_NEAR(new_rate, 0.8 * dt_lim, 1e-6) << tag;
      }
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

TEST(SteerAccelLimit, LaterStagesExceedLimitOnlyInAllowedDirection)
{
  struct Case
  {
    std::string name;
    VehicleCmdFilterParam param;
    double speed;
    double prev_steer;
    double prev_rate;
    double command;
    double current_steer;
  };
  auto low_rate = make_steer_accel_param();
  low_rate.steer_rate_lim_for_steer_cmd.assign(low_rate.reference_speed_points.size(), 0.2);
  const std::vector<Case> cases{
    {"steer_limit", make_steer_accel_param(), 0.0, 0.99, 0.5, 1.5, 0.99},
    {"steer_rate", low_rate, 0.0, 0.2, 0.6, 0.5, 0.2},
    {"lat_jerk", make_steer_accel_param(), 3.0, 0.3, 0.529, 0.6, 0.3},
    {"lat_acc", make_steer_accel_param(), 10.0, 0.1, 0.0, 0.1, 0.1},
    {"actual_steer_diff", make_steer_accel_param(), 0.0, 0.2, 0.0, 0.2, -1.0},
  };
  for (const auto & c : cases) {
    VehicleCmdFilter filter;
    filter.setParam(c.param);
    filter.setCurrentSpeed(c.speed);
    const double limit = filter.getSteerAccelLimForSteerCmd();
    Control prev;
    prev.lateral.steering_tire_angle = static_cast<float>(c.prev_steer);
    filter.setPrevCmd(prev);
    filter.setPrevSteerRates(c.prev_rate, 0.0);
    Control out;
    out.lateral.steering_tire_angle = static_cast<float>(c.command);
    IsFilterActivated activated;
    double clip = 0.0;
    double clip_field = 0.0;
    filter.filterAll(cycle, c.current_steer, out, activated, true, clip, clip_field);

    const double rate =
      (out.lateral.steering_tire_angle - prev.lateral.steering_tire_angle) / cycle;
    const double accel = (rate - c.prev_rate) / cycle;
    ::testing::Test::RecordProperty(c.name + "_accel", std::to_string(accel));
    ::testing::Test::RecordProperty(c.name + "_rate", std::to_string(rate));
    if (std::abs(accel) <= limit + tolerance_of_accel(1.0, cycle)) {
      continue;
    }
    const double moved = out.lateral.steering_tire_angle - prev.lateral.steering_tire_angle;
    if (c.name == "lat_acc") {
      EXPECT_LT(moved * prev.lateral.steering_tire_angle, 0.0) << c.name << " accel=" << accel;
    } else if (c.name == "actual_steer_diff") {
      EXPECT_LT(
        std::abs(out.lateral.steering_tire_angle - c.current_steer),
        std::abs(prev.lateral.steering_tire_angle - c.current_steer))
        << c.name << " accel=" << accel;
    } else {
      EXPECT_LT(std::abs(rate), std::abs(c.prev_rate)) << c.name << " accel=" << accel;
    }
  }
}

TEST(SteerAccelLimit, CapsOnlySteerAccelStageForLongCycle)
{
  constexpr double dt = 2.5;
  constexpr double speed = 10.0;
  VehicleCmdFilter filter;
  filter.setParam(make_steer_accel_param());
  filter.setCurrentSpeed(speed);
  filter.setPrevCmd(Control());
  filter.setPrevSteerRates(0.0, 0.0);

  Control acc_cmd;
  acc_cmd.longitudinal.velocity = 100.0f;
  filter.limitLongitudinalWithAcc(dt, acc_cmd);
  EXPECT_NEAR(acc_cmd.longitudinal.velocity, 5.0 * dt, 1e-5);

  Control jerk_cmd;
  jerk_cmd.longitudinal.acceleration = 100.0f;
  filter.limitLongitudinalWithJerk(dt, jerk_cmd);
  EXPECT_NEAR(jerk_cmd.longitudinal.acceleration, 5.0 * dt, 1e-5);

  Control lat_jerk_cmd;
  lat_jerk_cmd.lateral.steering_tire_angle = 0.5f;
  filter.limitLateralWithLatJerk(dt, lat_jerk_cmd);
  EXPECT_NEAR(
    lat_jerk_cmd.lateral.steering_tire_angle, std::atan(1.0 * dt * wheel_base / 100.0), 1e-5);

  Control rate_cmd;
  rate_cmd.lateral.steering_tire_angle = 0.5f;
  filter.limitLateralSteerRate(dt, rate_cmd);
  EXPECT_NEAR(rate_cmd.lateral.steering_tire_angle, steer_rate_limit(speed) * dt, 1e-5);

  Control accel_cmd;
  accel_cmd.lateral.steering_tire_angle = 10.0f;
  double clip = 0.0;
  double clip_field = 0.0;
  filter.limitLateralSteerAccel(dt, accel_cmd, clip, clip_field);
  EXPECT_NEAR(accel_cmd.lateral.steering_tire_angle, 0.3 * 1.0 * 1.0, 1e-6);
}

TEST(SteerAccelLimit, ReportsClipAmounts)
{
  struct Case
  {
    std::string name;
    double dt;
    float command_steer;
    float command_rotation_rate;
  };
  const std::vector<Case> cases{
    {"steer_only", cycle, 0.03f, 0.0f},
    {"rotation_only", cycle, 0.0f, 0.5f},
    {"both", cycle, 0.03f, 0.2f},
    {"long_cycle", 2.5, 3.0f, 0.0f},
  };
  for (const auto & c : cases) {
    VehicleCmdFilter filter;
    filter.setParam(make_steer_accel_param());
    filter.setCurrentSpeed(0.0);
    filter.setPrevCmd(Control());
    filter.setPrevSteerRates(0.0, 0.0);
    const double dt_lim = std::min(c.dt, 1.0);

    Control cmd;
    cmd.lateral.steering_tire_angle = c.command_steer;
    cmd.lateral.steering_tire_rotation_rate = c.command_rotation_rate;
    auto stage = cmd;
    double clip = -1.0;
    double clip_field = -1.0;
    filter.limitLateralSteerAccel(c.dt, stage, clip, clip_field);
    const double raw_rate = cmd.lateral.steering_tire_angle / dt_lim;
    const double new_rate = stage.lateral.steering_tire_angle / dt_lim;
    EXPECT_NEAR(clip, std::abs(raw_rate - new_rate) / dt_lim, 1e-4) << c.name;
    EXPECT_NEAR(
      clip_field,
      std::abs(
        cmd.lateral.steering_tire_rotation_rate - stage.lateral.steering_tire_rotation_rate) /
        dt_lim,
      1e-4)
      << c.name;
    EXPECT_EQ(clip > 0.0, c.command_steer > 0.0f) << c.name;
    EXPECT_EQ(clip_field > 0.0, c.command_rotation_rate > 0.0f) << c.name;

    auto out = cmd;
    IsFilterActivated activated;
    double relayed_clip = -1.0;
    double relayed_clip_field = -1.0;
    filter.filterAll(c.dt, 0.0, out, activated, true, relayed_clip, relayed_clip_field);
    EXPECT_EQ(relayed_clip, clip) << c.name;
    EXPECT_EQ(relayed_clip_field, clip_field) << c.name;
  }
}

TEST(SteerAccelLimit, ReportsClipCausedByBrakingBoundOnly)
{
  VehicleCmdFilter filter;
  filter.setParam(make_steer_accel_param());
  filter.setCurrentSpeed(0.0);
  filter.setPrevCmd(Control());
  filter.setPrevSteerRates(0.05, 0.0);
  Control cmd;
  cmd.lateral.steering_tire_angle = 0.003f;
  auto stage = cmd;
  double clip = -1.0;
  double clip_field = -1.0;
  filter.limitLateralSteerAccel(cycle, stage, clip, clip_field);
  const double raw_rate = cmd.lateral.steering_tire_angle / cycle;
  const double new_rate = stage.lateral.steering_tire_angle / cycle;
  EXPECT_LT(new_rate, raw_rate);
  EXPECT_LT(std::abs(new_rate - 0.05) / cycle, 0.5 * 0.8);
  EXPECT_GT(clip, 0.0);
  EXPECT_NEAR(clip, std::abs(raw_rate - new_rate) / cycle, 1e-4);
  EXPECT_EQ(clip_field, 0.0);
}

TEST(SteerAccelLimit, ExistingRateLimitFixesSteerInShortCycle)
{
  for (const double dt : {0.0, 0.001, 0.0049}) {
    VehicleCmdFilter filter;
    filter.setParam(make_steer_accel_param());
    filter.setCurrentSpeed(0.0);
    Control prev;
    prev.lateral.steering_tire_angle = 0.1f;
    filter.setPrevCmd(prev);
    filter.setPrevSteerRates(0.3, 0.0);
    Control out;
    out.lateral.steering_tire_angle = 0.5f;
    IsFilterActivated activated;
    double clip = 0.0;
    double clip_field = 0.0;
    filter.filterAll(dt, 0.1, out, activated, true, clip, clip_field);
    EXPECT_LE(std::abs(out.lateral.steering_tire_angle - 0.1f), 0.6 * dt + 1e-7) << "dt=" << dt;
  }
}

namespace
{

VehicleCmdFilterParam make_open_loop_param()
{
  auto p = make_steer_accel_param();
  p.lat_acc_lim_for_steer_cmd.assign(p.reference_speed_points.size(), 1000.0);
  p.lat_jerk_lim_for_steer_cmd.assign(p.reference_speed_points.size(), 1000.0);
  return p;
}

}  // namespace

TEST(SteerAccelLimit, HoldRequestOvershootMatchesTheory)
{
  const std::vector<std::pair<double, double>> expected_overshoot{
    {0.0, 0.216}, {1.0, 0.272}, {3.0, 0.351}, {5.0, 0.058}, {10.0, 0.003}};
  for (const auto & [speed, overshoot] : expected_overshoot) {
    const double r_lim = steer_rate_limit(speed);
    {
      SteerAccelDriver driver(make_open_loop_param());
      driver.reset(0.0, r_lim, 0.0);
      double max_steer = 0.0;
      for (int k = 0; k < 100; ++k) {
        const auto s =
          driver.step_steer(cycle, speed, driver.prev_out().lateral.steering_tire_angle, false);
        max_steer = std::max(max_steer, static_cast<double>(s.out.lateral.steering_tire_angle));
      }
      EXPECT_EQ(max_steer, 0.0) << "builtin v=" << speed;
    }
    {
      SteerAccelDriver driver(make_open_loop_param());
      driver.reset(0.0, r_lim, 0.0);
      double max_steer = 0.0;
      double stop_time = -1.0;
      double prev_steer = 0.0;
      for (int k = 0; k < 300; ++k) {
        const auto s = driver.step_steer(cycle, speed, 0.0);
        const double steer = s.out.lateral.steering_tire_angle;
        max_steer = std::max(max_steer, steer);
        if (stop_time < 0.0 && steer <= prev_steer) {
          stop_time = k * cycle;
        }
        prev_steer = steer;
      }
      ::testing::Test::RecordProperty(
        "hold_stop_time_v" + std::to_string(static_cast<int>(speed)), std::to_string(stop_time));
      EXPECT_NEAR(max_steer, overshoot, 1e-3) << "stop v=" << speed;
    }
  }
}

TEST(SteerAccelLimit, StepResponseMatchesTheory)
{
  struct Expected
  {
    double speed;
    double rise_time;
    double settle_time;
    double peak;
  };
  const std::vector<Expected> table{
    {0.0, 0.78, 0.99, 0.2001},
    {1.0, 0.87, 1.11, 0.2001},
    {3.0, 1.11, 1.41, 0.2000},
    {5.0, 1.32, 1.68, 0.2000}};
  for (const auto & e : table) {
    SteerAccelDriver driver(make_open_loop_param());
    driver.reset(0.0, 0.0, 0.0);
    double rise_time = -1.0;
    double settle_time = -1.0;
    double peak = 0.0;
    for (int k = 0; k < 400; ++k) {
      const auto s = driver.step_steer(cycle, e.speed, 0.2);
      const double steer = s.out.lateral.steering_tire_angle;
      if (rise_time < 0.0 && steer >= 0.9 * 0.2) {
        rise_time = (k + 1) * cycle;
      }
      if (settle_time < 0.0 && steer >= 0.2 - 1e-6) {
        settle_time = (k + 1) * cycle;
      }
      peak = std::max(peak, steer);
    }
    EXPECT_NEAR(rise_time, e.rise_time, cycle + 1e-9) << "v=" << e.speed;
    EXPECT_NEAR(settle_time, e.settle_time, cycle + 1e-9) << "v=" << e.speed;
    EXPECT_NEAR(peak, e.peak, 1e-3) << "v=" << e.speed;
    EXPECT_LE(peak - 0.2, 1e-4) << "v=" << e.speed;
  }
}

TEST(SteerAccelLimit, LongGapLeavesShortfallAboveMaxCycle)
{
  for (const double dt : {0.8, 1.0, 2.5}) {
    VehicleCmdFilter filter;
    filter.setParam(make_steer_accel_param());
    filter.setCurrentSpeed(0.0);
    filter.setPrevCmd(Control());
    filter.setPrevSteerRates(0.0, 0.0);
    Control cmd;
    cmd.lateral.steering_tire_angle = 10.0f;
    double clip = 0.0;
    double clip_field = 0.0;
    filter.limitLateralSteerAccel(dt, cmd, clip, clip_field);
    const double dt_lim = std::min(dt, 1.0);
    const double achieved_rate = cmd.lateral.steering_tire_angle / dt_lim;
    const double shortfall = 0.8 * dt - achieved_rate;
    EXPECT_NEAR(shortfall, dt <= 1.0 ? 0.0 : 0.8 * (dt - 1.0), 1e-6) << "dt=" << dt;
  }
}

namespace
{

double braking_rate_limit(const double accel_lim, const double dt, const double steer_error)
{
  const double band = accel_lim * dt;
  return (-band + std::sqrt(band * band + 8.0 * accel_lim * std::abs(steer_error))) / 2.0;
}

double accel_limit_of(const VehicleCmdFilterParam & p, const double speed)
{
  VehicleCmdFilter filter;
  filter.setParam(p);
  filter.setCurrentSpeed(speed);
  return filter.getSteerAccelLimForSteerCmd();
}

}  // namespace

TEST(SteerAccelLimit, StepDoesNotOvershootAndFollowsBrakingBound)
{
  const auto p = make_isolated_steer_accel_param();
  for (const double target : {0.05, 0.2, 0.5, -0.05, -0.2, -0.5}) {
    for (const double speed : {0.0, 1.0, 3.0, 5.0, 10.0}) {
      for (const double dt : {0.03, 0.1}) {
        const double accel_lim = accel_limit_of(p, speed);
        SteerAccelDriver driver(p);
        driver.reset(0.0, 0.0, 0.0);
        double prev_steer = 0.0;
        double prev_rate = 0.0;
        const int count = static_cast<int>(std::lround(15.0 / dt));
        const auto tag = "target=" + std::to_string(target) + " v=" + std::to_string(speed) +
                         " dt=" + std::to_string(dt);
        for (int k = 0; k < count; ++k) {
          const auto s = driver.step_steer(dt, speed, target);
          const double stage = s.stage.lateral.steering_tire_angle;
          const double rate = (stage - prev_steer) / dt;
          const double allow = braking_rate_limit(accel_lim, dt, target - prev_steer);
          const double tol_rate =
            tolerance_of_accel(std::max(std::abs(stage), std::abs(prev_steer)), dt) * dt;
          EXPECT_LE((target > 0.0 ? 1.0 : -1.0) * (stage - target), 1e-4) << tag << " k=" << k;
          EXPECT_LE(std::abs(rate), allow + tol_rate) << tag << " k=" << k;
          if (std::abs(rate) < std::abs(prev_rate)) {
            EXPECT_GE(std::abs(rate), allow - accel_lim * dt - tol_rate) << tag << " k=" << k;
          }
          prev_rate = (s.out.lateral.steering_tire_angle - prev_steer) / dt;
          prev_steer = s.out.lateral.steering_tire_angle;
        }
        EXPECT_LT(std::abs(prev_steer - target), 1e-6) << tag;
      }
    }
  }
}

TEST(SteerAccelLimit, DiscreteBrakingBoundHasNoResidualOvershootOrChattering)
{
  const auto p = make_isolated_steer_accel_param();
  const double accel_lim = accel_limit_of(p, 0.0);
  {
    SteerAccelDriver driver(p);
    driver.reset(0.0, 0.0, 0.0);
    double peak = 0.0;
    for (int k = 0; k < 150; ++k) {
      peak = std::max(
        peak,
        static_cast<double>(driver.step_steer(0.1, 0.0, 0.2).stage.lateral.steering_tire_angle));
    }
    EXPECT_LE(peak - 0.2, 1e-4) << "coarse cycle";
  }
  for (const double offset : {0.5 * accel_lim * cycle * cycle, 0.0}) {
    SteerAccelDriver driver(p);
    driver.reset(0.1, 0.0, 0.0);
    const double target = 0.1 + offset;
    int sign_changes = 0;
    double prev_delta = 0.0;
    double prev_steer = 0.1;
    for (int k = 0; k < 500; ++k) {
      const double steer = driver.step_steer(cycle, 0.0, target).out.lateral.steering_tire_angle;
      const double delta = steer - prev_steer;
      if (delta * prev_delta < 0.0) {
        ++sign_changes;
      }
      if (delta != 0.0) {
        prev_delta = delta;
      }
      prev_steer = steer;
    }
    EXPECT_LT(std::abs(prev_steer - target), 1e-6) << "offset=" << offset;
    EXPECT_EQ(sign_changes, 0) << "offset=" << offset;
  }
}

TEST(SteerAccelLimit, ReversalDuringBrakingStaysWithinLimit)
{
  const auto p = make_isolated_steer_accel_param();
  constexpr double speed = 3.0;
  const double accel_lim = accel_limit_of(p, speed);

  SteerAccelDriver reference(p);
  reference.reset(0.0, 0.0, 0.0);
  int braking_begin = -1;
  int braking_end = -1;
  double prev_rate = 0.0;
  for (int k = 0; k < 300; ++k) {
    const double prev_steer = reference.prev_out().lateral.steering_tire_angle;
    const auto s = reference.step_steer(cycle, speed, 0.3);
    const double rate = (s.out.lateral.steering_tire_angle - prev_steer) / cycle;
    if (braking_begin < 0 && rate < prev_rate) {
      braking_begin = k;
    }
    if (
      braking_begin >= 0 && braking_end < 0 &&
      std::abs(s.out.lateral.steering_tire_angle - 0.3) < 1e-6) {
      braking_end = k;
    }
    prev_rate = rate;
  }
  ASSERT_GT(braking_begin, 0);
  ASSERT_GT(braking_end, braking_begin + 4);

  for (const int reversal :
       {braking_begin + 1, (braking_begin + braking_end) / 2, braking_end - 2}) {
    SteerAccelDriver driver(p);
    driver.reset(0.0, 0.0, 0.0);
    std::vector<SteerAccelStep> steps;
    for (int k = 0; k < reversal + 300; ++k) {
      steps.push_back(driver.step_steer(cycle, speed, k < reversal ? 0.3 : -0.3));
    }
    const auto tag = "reversal=" + std::to_string(reversal);
    expect_accels_within(stage_accels(0.0, 0.0, steps), accel_lim, tag);
    for (size_t k = static_cast<size_t>(reversal); k < steps.size(); ++k) {
      EXPECT_GE(steps.at(k).stage.lateral.steering_tire_angle, -0.3 - 1e-4) << tag << " k=" << k;
    }
  }
}

TEST(SteerAccelLimit, AccelBandIsAppliedAfterBrakingBound)
{
  const auto p = make_steer_accel_param();
  const double accel_lim = accel_limit_of(p, 0.0);
  struct Case
  {
    std::string name;
    double prev_rate;
    double command;
  };
  const std::vector<Case> cases{
    {"braking_only", 0.05, 0.003},
    {"accel_only", 0.0, 1.0},
    {"both", 0.3, 0.003},
  };
  for (const auto & c : cases) {
    VehicleCmdFilter filter;
    filter.setParam(p);
    filter.setCurrentSpeed(0.0);
    filter.setPrevCmd(Control());
    filter.setPrevSteerRates(c.prev_rate, 0.0);
    Control stage;
    stage.lateral.steering_tire_angle = static_cast<float>(c.command);
    double clip = 0.0;
    double clip_field = 0.0;
    filter.limitLateralSteerAccel(cycle, stage, clip, clip_field);
    const double new_rate = stage.lateral.steering_tire_angle / cycle;
    const double accel = (new_rate - c.prev_rate) / cycle;
    EXPECT_LE(std::abs(accel), accel_lim * (1.0 + 1e-9) + tolerance_of_accel(1.0, cycle)) << c.name;
    if (c.name == "braking_only") {
      EXPECT_LE(new_rate, braking_rate_limit(accel_lim, cycle, c.command) + 1e-5) << c.name;
      EXPECT_LT(std::abs(accel), accel_lim) << c.name;
    }
    if (c.name == "accel_only") {
      EXPECT_NEAR(accel, accel_lim, tolerance_of_accel(1.0, cycle)) << c.name;
    }
  }

  SteerAccelDriver driver(p);
  driver.reset(0.0, 0.0, 0.0);
  double max_out = 0.0;
  for (int k = 0; k < 100; ++k) {
    const double prev_steer = driver.prev_out().lateral.steering_tire_angle;
    Control cmd;
    cmd.lateral.steering_tire_angle = 0.2f;
    const auto s = driver.step(cycle, 0.0, cmd, 1.5);
    const double side = 0.2 >= prev_steer ? 1.0 : -1.0;
    EXPECT_GE(side * (0.2 - s.stage.lateral.steering_tire_angle), -1e-4) << "k=" << k;
    max_out = std::max(max_out, static_cast<double>(s.out.lateral.steering_tire_angle));
  }
  ::testing::Test::RecordProperty("actual_steer_diff_output_excess", std::to_string(max_out - 0.2));
}

}  // namespace autoware::control_command_gate::test
