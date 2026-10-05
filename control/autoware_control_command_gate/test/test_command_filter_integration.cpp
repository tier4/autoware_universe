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

#include "test_fixtures.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace autoware::control_command_gate::test
{

namespace
{

constexpr double cycle = 0.03;
constexpr double start_time = 1.0;

VehicleCmdFilterParam make_node_param()
{
  return make_isolated_steer_accel_param();
}

VehicleCmdFilterParam make_node_transition_param()
{
  auto p = make_isolated_steer_accel_param();
  p.steer_rate_lim_for_steer_cmd = {0.4, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  return p;
}

double accel_limit(const VehicleCmdFilterParam & p, const double speed)
{
  VehicleCmdFilter filter;
  filter.setParam(p);
  filter.setCurrentSpeed(speed);
  return filter.getSteerAccelLimForSteerCmd();
}

Control steer_command(const double steer, const double rotation_rate = 0.0)
{
  Control cmd;
  cmd.lateral.steering_tire_angle = static_cast<float>(steer);
  cmd.lateral.steering_tire_rotation_rate = static_cast<float>(rotation_rate);
  return cmd;
}

struct Event
{
  double t;
  uint16_t source_id;
  Control cmd;
  std::optional<VehicleState> state;
  std::optional<bool> transition;
};

std::vector<Control> run_events(
  FilterFixture & fixture, const std::vector<Event> & events, const size_t count)
{
  std::vector<Control> outputs;
  for (size_t i = 0; i < count; ++i) {
    const auto & e = events.at(i);
    if (e.state) {
      fixture.set_state(*e.state);
    }
    if (e.transition) {
      fixture.filter().set_transition_flag(*e.transition);
    }
    outputs.push_back(fixture.step(e.source_id, e.t, e.cmd));
  }
  return outputs;
}

struct ProbeSpec
{
  double base_steer;
  double steer_direction;
  double rotation_direction;
  VehicleState state;
  bool transition = false;
};

struct ProbedRates
{
  double steer_rate;
  double rotation_rate;
};

ProbedRates probe_rates(
  const VehicleCmdFilterParam & nominal, const VehicleCmdFilterParam & transition,
  const std::vector<Event> & events, const size_t count, const ProbeSpec & spec)
{
  FilterFixture fixture(nominal, transition);
  run_events(fixture, events, count);
  fixture.set_state(spec.state);
  fixture.filter().set_transition_flag(spec.transition);
  const auto cmd =
    steer_command(spec.base_steer + spec.steer_direction * 10.0, spec.rotation_direction * 100.0);
  const auto out = fixture.step(main_id, events.at(count - 1).t + cycle, cmd);
  const double limit = accel_limit(spec.transition ? transition : nominal, spec.state.speed);
  return {
    (out.lateral.steering_tire_angle - spec.base_steer) / cycle -
      spec.steer_direction * limit * cycle,
    out.lateral.steering_tire_rotation_rate - spec.rotation_direction * limit * cycle};
}

double direction_reducing(const double value)
{
  return value > 0.0 ? -1.0 : 1.0;
}

ProbeSpec auto_probe(const Control & last, const double speed, const double steer_rate)
{
  return {
    last.lateral.steering_tire_angle,
    direction_reducing(steer_rate),
    direction_reducing(last.lateral.steering_tire_rotation_rate),
    {speed, last.lateral.steering_tire_angle, ControlModeReport::AUTONOMOUS}};
}

std::vector<Event> ramp_events(
  const size_t count, const double rate, const double speed, const double steer0 = 0.0)
{
  std::vector<Event> events;
  for (size_t k = 0; k < count; ++k) {
    const double t = start_time + k * cycle;
    Event e{t, main_id, steer_command(steer0 + rate * k * cycle, rate), std::nullopt, std::nullopt};
    if (k == 0) {
      e.state = VehicleState{speed, steer0, ControlModeReport::AUTONOMOUS};
    }
    events.push_back(e);
  }
  return events;
}

void expect_steer_accels_within(
  const std::vector<Control> & outputs, const size_t begin, const double steer0, const double rate0,
  const std::vector<double> & dts, const double limit, const std::string & tag)
{
  double prev_steer = steer0;
  double prev_rate = rate0;
  for (size_t k = begin; k < outputs.size(); ++k) {
    const double steer = outputs.at(k).lateral.steering_tire_angle;
    const double dt = dts.at(k);
    const double rate = (steer - prev_steer) / dt;
    const double accel = (rate - prev_rate) / dt;
    const double tol = tolerance_of_accel(std::max(std::abs(steer), std::abs(prev_steer)), dt);
    EXPECT_LE(std::abs(accel), limit * (1.0 + 1e-9) + tol) << tag << " k=" << k;
    prev_steer = steer;
    prev_rate = rate;
  }
}

void expect_rotation_accels_within(
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

std::vector<double> event_dts(const std::vector<Event> & events)
{
  std::vector<double> dts;
  for (size_t k = 0; k < events.size(); ++k) {
    dts.push_back(k == 0 ? cycle : events.at(k).t - events.at(k - 1).t);
  }
  return dts;
}

constexpr double probe_tolerance = 1e-5;

}  // namespace

TEST(CommandFilterIntegration, TransitionSwitchKeepsContinuity)
{
  auto events = ramp_events(150, 0.15, 1.0);
  auto switched = events;
  switched.at(50).transition = true;
  switched.at(100).transition = false;

  FilterFixture plain_fixture(make_node_param(), make_node_transition_param());
  FilterFixture switched_fixture(make_node_param(), make_node_transition_param());
  const auto plain = run_events(plain_fixture, events, events.size());
  const auto outputs = run_events(switched_fixture, switched, switched.size());
  for (size_t k = 0; k < outputs.size(); ++k) {
    EXPECT_FLOAT_EQ(
      outputs.at(k).lateral.steering_tire_angle, plain.at(k).lateral.steering_tire_angle)
      << "k=" << k;
    EXPECT_FLOAT_EQ(
      outputs.at(k).lateral.steering_tire_rotation_rate,
      plain.at(k).lateral.steering_tire_rotation_rate)
      << "k=" << k;
  }
  const double limit = accel_limit(make_node_param(), 1.0);
  expect_steer_accels_within(
    outputs, 1, outputs.at(0).lateral.steering_tire_angle, 0.0, event_dts(switched), limit,
    "switched");
  expect_rotation_accels_within(
    outputs, 1, outputs.at(0).lateral.steering_tire_rotation_rate, event_dts(switched), limit,
    "switched");
}

TEST(CommandFilterIntegration, EngageStartsFromActualSteerWithZeroRate)
{
  std::vector<Event> events;
  for (size_t k = 0; k < 100; ++k) {
    Event e{start_time + k * cycle, main_id, steer_command(0.0), std::nullopt, std::nullopt};
    if (k == 0) e.state = VehicleState{1.0, 0.3, ControlModeReport::MANUAL};
    if (k == 50) e.state = VehicleState{1.0, 0.3, ControlModeReport::AUTONOMOUS};
    events.push_back(e);
  }
  const auto p = make_node_param();
  const double limit = accel_limit(p, 1.0);

  const auto before =
    probe_rates(p, p, events, 50, {0.3, -1.0, -1.0, {1.0, 0.3, ControlModeReport::AUTONOMOUS}});
  EXPECT_NEAR(before.steer_rate, 0.0, probe_tolerance);
  EXPECT_NEAR(before.rotation_rate, 0.0, probe_tolerance);

  FilterFixture fixture(p, p);
  const auto outputs = run_events(fixture, events, events.size());
  const double engage_rate = (outputs.at(50).lateral.steering_tire_angle - 0.3) / cycle;
  EXPECT_LE(std::abs(engage_rate), limit * cycle + 1e-5);
  EXPECT_LE(std::abs(outputs.at(50).lateral.steering_tire_rotation_rate), limit * cycle + 1e-6);
  expect_steer_accels_within(outputs, 50, 0.3, 0.0, event_dts(events), limit, "after engage");
}

namespace
{

struct BuiltinScenario
{
  std::vector<Event> events;
  size_t builtin_begin;
  size_t builtin_end;
};

BuiltinScenario make_builtin_scenario(
  FilterFixture & fixture, const double speed, const size_t builtin_cycles,
  const std::optional<double> actual_steer_offset = std::nullopt)
{
  BuiltinScenario s;
  s.events.push_back(
    {start_time, main_id, steer_command(-0.8), VehicleState{speed, -0.8, ControlModeReport::MANUAL},
     std::nullopt});
  for (size_t k = 1; k <= 67; ++k) {
    Event e{
      start_time + k * cycle, main_id, steer_command(-0.8 + 0.6 * k * cycle, 0.6), std::nullopt,
      std::nullopt};
    if (k == 1) e.state = VehicleState{speed, -0.8, ControlModeReport::AUTONOMOUS};
    s.events.push_back(e);
  }
  auto outputs = run_events(fixture, s.events, s.events.size());
  s.builtin_begin = s.events.size();
  double t = s.events.back().t;
  Control held = outputs.back();
  for (size_t i = 0; i < builtin_cycles; ++i) {
    t += 0.1;
    Control cmd =
      steer_command(held.lateral.steering_tire_angle, held.lateral.steering_tire_rotation_rate);
    cmd.longitudinal.velocity = 0.0f;
    cmd.longitudinal.acceleration = -2.4f;
    Event e{t, builtin_id, cmd, std::nullopt, std::nullopt};
    if (i == 0 && actual_steer_offset) {
      e.state = VehicleState{
        speed, held.lateral.steering_tire_angle + *actual_steer_offset,
        ControlModeReport::AUTONOMOUS};
    }
    s.events.push_back(e);
    if (e.state) fixture.set_state(*e.state);
    held = fixture.step(e.source_id, e.t, e.cmd);
  }
  s.builtin_end = s.events.size();
  return s;
}

}  // namespace

TEST(CommandFilterIntegration, BuiltinSkipsSteerAccelLimitOnly)
{
  const auto p = make_node_param();
  for (const double speed : {1.0, 3.0}) {
    FilterFixture fixture(p, p);
    const auto s = make_builtin_scenario(fixture, speed, 20);
    const auto & outputs = fixture.output().controls;
    const auto hold = outputs.at(s.builtin_begin - 1);
    for (size_t k = s.builtin_begin; k < s.builtin_end; ++k) {
      EXPECT_FLOAT_EQ(outputs.at(k).lateral.steering_tire_angle, hold.lateral.steering_tire_angle)
        << "v=" << speed << " k=" << k;
    }
    EXPECT_GT(outputs.at(s.builtin_begin).longitudinal.acceleration, -2.4f) << "v=" << speed;
    for (const size_t k : {s.builtin_begin + 1, s.builtin_begin + 5, s.builtin_end}) {
      const auto & last = outputs.at(k - 1);
      const auto rates = probe_rates(p, p, s.events, k, auto_probe(last, speed, 1.0));
      EXPECT_NEAR(rates.steer_rate, 0.0, probe_tolerance) << "v=" << speed << " k=" << k;
      EXPECT_NEAR(rates.rotation_rate, last.lateral.steering_tire_rotation_rate, probe_tolerance)
        << "v=" << speed << " k=" << k;
    }
  }

  FilterFixture fixture(p, p);
  const auto s = make_builtin_scenario(fixture, 1.0, 10, -1.2);
  const auto & outputs = fixture.output().controls;
  const auto moved = outputs.at(s.builtin_begin);
  EXPECT_NE(
    moved.lateral.steering_tire_angle, outputs.at(s.builtin_begin - 1).lateral.steering_tire_angle);
  const auto rates = probe_rates(p, p, s.events, s.builtin_begin + 1, auto_probe(moved, 1.0, 1.0));
  EXPECT_NEAR(rates.steer_rate, 0.0, probe_tolerance);
}

TEST(CommandFilterIntegration, RestartsLimitAfterBuiltin)
{
  const auto p = make_node_param();
  const double limit = accel_limit(p, 1.0);
  std::vector<std::vector<double>> relative_paths;
  for (const size_t builtin_cycles : {1u, 5u, 10u, 50u}) {
    for (const uint16_t resume_id : {main_id, in_lane_stop_id}) {
      FilterFixture fixture(p, p);
      const auto s = make_builtin_scenario(fixture, 1.0, builtin_cycles);
      const double hold = fixture.output().controls.back().lateral.steering_tire_angle;
      double t = s.events.back().t;
      std::vector<Control> resumed;
      std::vector<double> dts;
      for (int i = 0; i < 10; ++i) {
        t += cycle;
        resumed.push_back(fixture.step(resume_id, t, steer_command(hold + 0.3)));
        dts.push_back(cycle);
      }
      const double first_rate = (resumed.front().lateral.steering_tire_angle - hold) / cycle;
      EXPECT_LE(std::abs(first_rate), limit * cycle + 1e-5) << builtin_cycles;
      expect_steer_accels_within(
        resumed, 0, hold, 0.0, dts, limit, "resume " + std::to_string(builtin_cycles));
      std::vector<double> path;
      for (const auto & r : resumed) path.push_back(r.lateral.steering_tire_angle - hold);
      relative_paths.push_back(path);
    }
  }
  for (const auto & path : relative_paths) {
    for (size_t i = 0; i < path.size(); ++i) {
      EXPECT_NEAR(path.at(i), relative_paths.front().at(i), 1e-6) << "i=" << i;
    }
  }
}

TEST(CommandFilterIntegration, ShortCycleKeepsState)
{
  const auto p = make_node_param();
  auto events = ramp_events(30, 0.15, 1.0);
  double t = events.back().t;
  for (int i = 0; i < 5; ++i) {
    t += 0.001;
    events.push_back({t, main_id, steer_command(0.5, 0.5), std::nullopt, std::nullopt});
  }
  FilterFixture fixture(p, p);
  const auto outputs = run_events(fixture, events, events.size());
  const auto before_burst = outputs.at(29);
  for (size_t k = 30; k < 35; ++k) {
    EXPECT_EQ(
      outputs.at(k).lateral.steering_tire_rotation_rate,
      before_burst.lateral.steering_tire_rotation_rate)
      << k;
    EXPECT_LE(
      std::abs(
        outputs.at(k).lateral.steering_tire_angle - outputs.at(k - 1).lateral.steering_tire_angle),
      0.6 * 0.001 + 1e-7)
      << k;
  }
  const double rate_before =
    (outputs.at(29).lateral.steering_tire_angle - outputs.at(28).lateral.steering_tire_angle) /
    cycle;
  const auto probe_before =
    probe_rates(p, p, events, 30, auto_probe(outputs.at(29), 1.0, rate_before));
  const auto probe_after =
    probe_rates(p, p, events, 35, auto_probe(outputs.at(34), 1.0, rate_before));
  EXPECT_NEAR(probe_after.steer_rate, probe_before.steer_rate, probe_tolerance);
  EXPECT_NEAR(probe_after.rotation_rate, probe_before.rotation_rate, probe_tolerance);

  std::vector<Event> manual;
  for (int k = 0; k < 10; ++k) {
    Event e{start_time + k * 0.002, main_id, steer_command(0.5, 0.5), std::nullopt, std::nullopt};
    if (k == 0) e.state = VehicleState{1.0, 0.1, ControlModeReport::MANUAL};
    manual.push_back(e);
  }
  const auto manual_rates = probe_rates(
    p, p, manual, manual.size(), {0.1, -1.0, -1.0, {1.0, 0.1, ControlModeReport::AUTONOMOUS}});
  EXPECT_NEAR(manual_rates.steer_rate, 0.0, probe_tolerance);
  EXPECT_NEAR(manual_rates.rotation_rate, 0.0, probe_tolerance);
}

TEST(CommandFilterIntegration, NoVehicleStatusTreatedAsManualAtStandstill)
{
  const auto p = make_node_param();
  std::vector<Event> events;
  for (int k = 0; k < 30; ++k) {
    events.push_back(
      {start_time + k * cycle, main_id, steer_command(0.2, 0.3), std::nullopt, std::nullopt});
  }
  FilterFixture fixture(p, p);
  const auto outputs = run_events(fixture, events, events.size());
  for (const auto & out : outputs) {
    EXPECT_TRUE(std::isfinite(out.lateral.steering_tire_angle));
    EXPECT_TRUE(std::isfinite(out.lateral.steering_tire_rotation_rate));
    EXPECT_LE(std::abs(out.lateral.steering_tire_angle), 0.8 * cycle * cycle + 1e-6);
  }
  const auto rates = probe_rates(
    p, p, events, events.size(), {0.0, -1.0, -1.0, {0.0, 0.0, ControlModeReport::AUTONOMOUS}});
  EXPECT_NEAR(rates.steer_rate, 0.0, probe_tolerance);
  EXPECT_NEAR(rates.rotation_rate, 0.0, probe_tolerance);
}

TEST(CommandFilterIntegration, RecomputesRateFromOutput)
{
  const auto p = make_node_param();
  {
    auto events = ramp_events(40, 0.15, 1.0);
    events.push_back(
      {events.back().t + 2.5, main_id, steer_command(0.5, 0.15), std::nullopt, std::nullopt});
    FilterFixture fixture(p, p);
    const auto outputs = run_events(fixture, events, events.size());
    const double expected =
      (outputs.at(40).lateral.steering_tire_angle - outputs.at(39).lateral.steering_tire_angle) /
      1.0;
    const auto rates =
      probe_rates(p, p, events, events.size(), auto_probe(outputs.back(), 1.0, expected));
    EXPECT_NEAR(rates.steer_rate, expected, probe_tolerance) << "long cycle";
  }
  {
    auto events = ramp_events(40, 0.15, 1.0);
    events.push_back(
      {events.back().t + 0.004, main_id, steer_command(0.5, 0.15), std::nullopt, std::nullopt});
    events.push_back(
      {events.back().t + cycle, main_id, steer_command(0.5, 0.15), std::nullopt, std::nullopt});
    FilterFixture fixture(p, p);
    const auto outputs = run_events(fixture, events, events.size());
    const double expected =
      (outputs.at(41).lateral.steering_tire_angle - outputs.at(40).lateral.steering_tire_angle) /
      cycle;
    const auto rates =
      probe_rates(p, p, events, events.size(), auto_probe(outputs.back(), 1.0, expected));
    EXPECT_NEAR(rates.steer_rate, expected, probe_tolerance) << "after short cycle";
  }
  {
    auto events = ramp_events(40, 0.15, 1.0);
    FilterFixture first(p, p);
    const auto ramp = run_events(first, events, events.size());
    events.push_back({events.back().t + 0.1, builtin_id, ramp.back(), std::nullopt, std::nullopt});
    FilterFixture fixture(p, p);
    auto outputs = run_events(fixture, events, events.size());
    const auto after_builtin =
      probe_rates(p, p, events, events.size(), auto_probe(outputs.back(), 1.0, 0.0));
    EXPECT_NEAR(after_builtin.steer_rate, 0.0, probe_tolerance) << "after builtin";
    events.push_back(
      {events.back().t + cycle, main_id, steer_command(0.5, 0.15), std::nullopt, std::nullopt});
    outputs.push_back(fixture.step(main_id, events.back().t, events.back().cmd));
    const double expected =
      (outputs.at(41).lateral.steering_tire_angle - outputs.at(40).lateral.steering_tire_angle) /
      cycle;
    const auto after_main =
      probe_rates(p, p, events, events.size(), auto_probe(outputs.back(), 1.0, expected));
    EXPECT_NEAR(after_main.steer_rate, expected, probe_tolerance) << "after main";
  }
}

TEST(CommandFilterIntegration, ZeroCycleKeepsState)
{
  const auto p = make_node_param();
  auto events = ramp_events(30, 0.15, 1.0);
  events.push_back({events.back().t, main_id, steer_command(0.5, 0.5), std::nullopt, std::nullopt});
  FilterFixture fixture(p, p);
  const auto outputs = run_events(fixture, events, events.size());
  EXPECT_EQ(
    outputs.at(30).lateral.steering_tire_rotation_rate,
    outputs.at(29).lateral.steering_tire_rotation_rate);
  EXPECT_EQ(outputs.at(30).lateral.steering_tire_angle, outputs.at(29).lateral.steering_tire_angle);
  const double rate =
    (outputs.at(29).lateral.steering_tire_angle - outputs.at(28).lateral.steering_tire_angle) /
    cycle;
  const auto before = probe_rates(p, p, events, 30, auto_probe(outputs.at(29), 1.0, rate));
  const auto after = probe_rates(p, p, events, 31, auto_probe(outputs.at(30), 1.0, rate));
  EXPECT_NEAR(after.steer_rate, before.steer_rate, probe_tolerance);
  EXPECT_NEAR(after.rotation_rate, before.rotation_rate, probe_tolerance);
}

TEST(CommandFilterIntegration, SimultaneousStateChangesStayWithinLimit)
{
  const auto p = make_node_param();
  const auto tp = make_node_transition_param();
  const double limit = accel_limit(p, 1.0);
  {
    auto events = ramp_events(100, 0.15, 1.0);
    for (size_t k = 50; k < events.size(); ++k) events.at(k).source_id = in_lane_stop_id;
    events.at(50).transition = true;
    FilterFixture fixture(p, tp);
    const auto outputs = run_events(fixture, events, events.size());
    expect_steer_accels_within(
      outputs, 1, outputs.at(0).lateral.steering_tire_angle, 0.0, event_dts(events), limit,
      "source+transition");
    expect_rotation_accels_within(
      outputs, 1, outputs.at(0).lateral.steering_tire_rotation_rate, event_dts(events), limit,
      "source+transition");
  }
  {
    auto events = ramp_events(100, 0.15, 1.0, 0.2);
    events.at(0).state = VehicleState{1.0, 0.2, ControlModeReport::MANUAL};
    for (size_t k = 50; k < events.size(); ++k) events.at(k).source_id = in_lane_stop_id;
    events.at(50).transition = true;
    events.at(50).state = VehicleState{1.0, 0.2, ControlModeReport::AUTONOMOUS};
    FilterFixture fixture(p, tp);
    const auto outputs = run_events(fixture, events, events.size());
    EXPECT_LE(
      std::abs((outputs.at(50).lateral.steering_tire_angle - 0.2) / cycle), limit * cycle + 1e-5);
    expect_steer_accels_within(outputs, 50, 0.2, 0.0, event_dts(events), limit, "with engage");
  }
  {
    auto events = ramp_events(100, 0.15, 1.0);
    for (size_t k = 1; k < events.size(); ++k) events.at(k).transition = (k % 2 == 1);
    FilterFixture fixture(p, tp);
    const auto outputs = run_events(fixture, events, events.size());
    expect_steer_accels_within(
      outputs, 1, outputs.at(0).lateral.steering_tire_angle, 0.0, event_dts(events), limit,
      "toggle");
    expect_rotation_accels_within(
      outputs, 1, outputs.at(0).lateral.steering_tire_rotation_rate, event_dts(events), limit,
      "toggle");
  }
}

TEST(CommandFilterIntegration, NonAutonomousModesAreManual)
{
  const auto p = make_node_param();
  for (const uint8_t mode :
       {ControlModeReport::NO_COMMAND, ControlModeReport::AUTONOMOUS_STEER_ONLY,
        ControlModeReport::AUTONOMOUS_VELOCITY_ONLY, ControlModeReport::MANUAL,
        ControlModeReport::DISENGAGED, ControlModeReport::NOT_READY, static_cast<uint8_t>(255)}) {
    std::vector<Event> events;
    for (int k = 0; k < 10; ++k) {
      Event e{start_time + k * cycle, main_id, steer_command(0.3, 0.3), std::nullopt, std::nullopt};
      if (k == 0) e.state = VehicleState{1.0, 0.1, mode};
      events.push_back(e);
    }
    const auto rates = probe_rates(
      p, p, events, events.size(), {0.1, -1.0, -1.0, {1.0, 0.1, ControlModeReport::AUTONOMOUS}});
    EXPECT_NEAR(rates.steer_rate, 0.0, probe_tolerance) << "mode=" << static_cast<int>(mode);
    EXPECT_NEAR(rates.rotation_rate, 0.0, probe_tolerance) << "mode=" << static_cast<int>(mode);
  }
  ::testing::Test::RecordProperty("autonomous_steer_only_is_manual", "true");
}

TEST(CommandFilterIntegration, MismatchedEnableFlagsKeepStateFromOutput)
{
  for (const bool nominal_enabled : {true, false}) {
    auto nominal = make_node_param();
    auto transition = make_node_param();
    nominal.enable_steer_accel_limit = nominal_enabled;
    transition.enable_steer_accel_limit = !nominal_enabled;
    auto events = ramp_events(130, 0.15, 1.0);
    events.at(50).transition = true;
    events.at(100).transition = false;
    const size_t disabled_begin = nominal_enabled ? 50 : 0;
    const size_t step_cycle = disabled_begin + 10;
    for (size_t k = step_cycle; k < events.size(); ++k) {
      events.at(k).cmd.lateral.steering_tire_angle += 0.1f;
    }
    FilterFixture fixture(nominal, transition);
    const auto outputs = run_events(fixture, events, events.size());
    const auto tag = std::string("nominal_enabled=") + (nominal_enabled ? "true" : "false");

    const size_t probe_cycle = step_cycle + 10;
    const double expected = (outputs.at(probe_cycle - 1).lateral.steering_tire_angle -
                             outputs.at(probe_cycle - 2).lateral.steering_tire_angle) /
                            cycle;
    const auto spec = auto_probe(outputs.at(probe_cycle - 1), 1.0, expected);
    FilterFixture replay(nominal, transition);
    run_events(replay, events, probe_cycle);
    replay.set_state(spec.state);
    replay.filter().set_transition_flag(!nominal_enabled);
    const double limit = accel_limit(make_node_param(), 1.0);
    const auto probe_out = replay.step(
      main_id, events.at(probe_cycle - 1).t + cycle,
      steer_command(spec.base_steer + spec.steer_direction * 10.0, 0.0));
    const double probed = (probe_out.lateral.steering_tire_angle - spec.base_steer) / cycle -
                          spec.steer_direction * limit * cycle;
    EXPECT_NEAR(probed, expected, probe_tolerance) << tag;

    const size_t first_enabled = nominal_enabled ? 100 : 50;
    const double r_prev = (outputs.at(first_enabled - 1).lateral.steering_tire_angle -
                           outputs.at(first_enabled - 2).lateral.steering_tire_angle) /
                          cycle;
    const double r_now = (outputs.at(first_enabled).lateral.steering_tire_angle -
                          outputs.at(first_enabled - 1).lateral.steering_tire_angle) /
                         cycle;
    EXPECT_LE(
      std::abs(r_now - r_prev) / cycle, limit * (1.0 + 1e-9) + tolerance_of_accel(1.0, cycle))
      << tag;
  }
}

TEST(CommandFilterIntegration, PreventsWindupDuringLaterStageClamp)
{
  auto p = make_node_param();
  p.steer_accel_lim_for_steer_cmd.assign(p.reference_speed_points.size(), 0.8);
  p.lat_acc_lim_for_steer_cmd.assign(p.reference_speed_points.size(), 1.5);
  const double steer_max = std::atan(1.5 * 4.76 / 100.0);
  for (const bool reverse : {false, true}) {
    std::vector<Event> events;
    for (size_t k = 0; k < 45; ++k) {
      Event e{start_time + k * cycle, main_id, steer_command(0.2), std::nullopt, std::nullopt};
      if (k == 0) e.state = VehicleState{10.0, 0.0, ControlModeReport::AUTONOMOUS};
      events.push_back(e);
    }
    for (size_t k = 45; k < 75; ++k) {
      Event e{
        start_time + k * cycle, main_id, steer_command(reverse ? 0.0 : 0.2), std::nullopt,
        std::nullopt};
      if (k == 45 && !reverse) e.state = VehicleState{5.0, 0.0, ControlModeReport::AUTONOMOUS};
      events.push_back(e);
    }
    FilterFixture fixture(p, p);
    const auto outputs = run_events(fixture, events, events.size());
    ASSERT_NEAR(outputs.at(44).lateral.steering_tire_angle, steer_max, 1e-5);
    const auto rates = probe_rates(
      p, p, events, 45,
      {static_cast<double>(outputs.at(44).lateral.steering_tire_angle),
       -1.0,
       -1.0,
       {10.0, outputs.at(44).lateral.steering_tire_angle, ControlModeReport::AUTONOMOUS}});
    EXPECT_NEAR(rates.steer_rate, 0.0, probe_tolerance);
    if (!reverse) {
      const double release_rate =
        (outputs.at(45).lateral.steering_tire_angle - outputs.at(44).lateral.steering_tire_angle) /
        cycle;
      EXPECT_LE(std::abs(release_rate) / cycle, 0.8 + tolerance_of_accel(0.3, cycle));
    } else {
      for (size_t k = 45;
           k < outputs.size() && outputs.at(k - 1).lateral.steering_tire_angle > 0.0f; ++k) {
        EXPECT_LE(
          outputs.at(k).lateral.steering_tire_angle,
          outputs.at(k - 1).lateral.steering_tire_angle + 1e-7)
          << k;
      }
    }
  }

  const auto np = make_node_param();
  const auto tp = make_node_transition_param();
  std::vector<Event> events;
  for (size_t k = 0; k < 55; ++k) {
    Event e{start_time + k * cycle, main_id, steer_command(0.0, 0.5), std::nullopt, std::nullopt};
    if (k == 0) {
      e.state = VehicleState{1.0, 0.0, ControlModeReport::AUTONOMOUS};
      e.transition = true;
    }
    if (k == 30) e.transition = false;
    events.push_back(e);
  }
  FilterFixture fixture(np, tp);
  const auto outputs = run_events(fixture, events, events.size());
  ASSERT_NEAR(outputs.at(29).lateral.steering_tire_rotation_rate, 0.2, 1e-6);
  auto spec = auto_probe(outputs.at(29), 1.0, 0.0);
  spec.transition = true;
  const auto rates = probe_rates(np, tp, events, 30, spec);
  EXPECT_NEAR(rates.rotation_rate, 0.2, probe_tolerance);
  expect_rotation_accels_within(
    outputs, 30, 0.2, event_dts(events), accel_limit(np, 1.0), "field release");
}

TEST(CommandFilterIntegration, SharesStateBetweenFilters)
{
  const auto p = make_node_param();
  const auto tp = make_node_transition_param();
  std::vector<Event> events = ramp_events(40, 0.1, 1.0);
  events.at(20).transition = true;
  events.insert(
    events.begin() + 31,
    Event{events.at(30).t + 0.002, main_id, events.at(30).cmd, std::nullopt, std::nullopt});
  for (size_t k = 0; k < events.size(); ++k) {
    if (k > 31) events.at(k).t += 0.002;
  }
  const double resume_t = events.back().t;
  for (int k = 1; k <= 20; ++k) {
    Event e{resume_t + k * cycle, main_id, steer_command(0.1, 0.1), std::nullopt, std::nullopt};
    if (k == 1) e.state = VehicleState{1.0, 0.25, ControlModeReport::MANUAL};
    events.push_back(e);
  }
  const size_t manual_end = events.size();
  for (int k = 1; k <= 20; ++k) {
    Event e{
      resume_t + (20 + k) * cycle, main_id, steer_command(0.1, 0.1), std::nullopt, std::nullopt};
    if (k == 1) e.state = VehicleState{1.0, 0.25, ControlModeReport::AUTONOMOUS};
    events.push_back(e);
  }
  FilterFixture fixture(p, tp);
  const auto outputs = run_events(fixture, events, events.size());

  for (const size_t k : {25ul, 32ul, manual_end - 5, manual_end + 1, manual_end + 10}) {
    const auto & last = outputs.at(k - 1);
    const bool manual = (k > 41 && k <= manual_end);
    const double observed_rate =
      (last.lateral.steering_tire_angle - outputs.at(k - 2).lateral.steering_tire_angle) /
      (events.at(k - 1).t - events.at(k - 2).t);
    ProbeSpec spec = manual
                       ? ProbeSpec{0.25, -1.0, -1.0, {1.0, 0.25, ControlModeReport::AUTONOMOUS}}
                       : auto_probe(last, 1.0, observed_rate);
    spec.transition = false;
    const auto by_nominal = probe_rates(p, tp, events, k, spec);
    spec.transition = true;
    const auto by_transition = probe_rates(p, tp, events, k, spec);
    EXPECT_NEAR(by_nominal.steer_rate, by_transition.steer_rate, probe_tolerance) << "k=" << k;
    EXPECT_NEAR(by_nominal.rotation_rate, by_transition.rotation_rate, probe_tolerance)
      << "k=" << k;
    if (manual) {
      EXPECT_NEAR(by_nominal.steer_rate, 0.0, probe_tolerance) << "k=" << k;
      EXPECT_NEAR(by_nominal.rotation_rate, 0.0, probe_tolerance) << "k=" << k;
    } else if (k == manual_end + 1) {
      const double expected = (last.lateral.steering_tire_angle - 0.25) / cycle;
      EXPECT_NEAR(by_nominal.steer_rate, expected, probe_tolerance) << "k=" << k;
    } else {
      EXPECT_NEAR(
        by_nominal.rotation_rate, last.lateral.steering_tire_rotation_rate, probe_tolerance)
        << "k=" << k;
    }
  }
}

TEST(CommandFilterIntegration, DisabledCommandLimitFilterPassesThrough)
{
  const auto p = make_node_param();
  FilterFixture fixture(p, p, false);
  fixture.set_state({1.0, 0.0, ControlModeReport::AUTONOMOUS});
  for (int k = 0; k < 20; ++k) {
    const auto cmd = steer_command(0.2 * (k % 2), 0.5);
    const auto out = fixture.step(main_id, start_time + k * cycle, cmd);
    EXPECT_TRUE(out == cmd) << k;
  }
}

TEST(CommandFilterIntegration, SourceIdSwitchesLimitWithoutDelay)
{
  const auto p = make_node_param();
  const double limit = accel_limit(p, 1.0);
  FilterFixture fixture(p, p);
  fixture.set_state({1.0, 0.0, ControlModeReport::AUTONOMOUS});
  fixture.step(main_id, start_time, steer_command(0.0));
  double prev_steer = fixture.output().controls.back().lateral.steering_tire_angle;
  for (int k = 1; k <= 20; ++k) {
    const uint16_t id = (k % 2 == 0) ? main_id : builtin_id;
    const auto out = fixture.step(id, start_time + k * cycle, steer_command(0.5));
    const double rate = (out.lateral.steering_tire_angle - prev_steer) / cycle;
    if (id == builtin_id) {
      EXPECT_NEAR(rate, 0.6, 1e-4) << "builtin k=" << k;
    } else {
      EXPECT_LE(std::abs(rate), limit * cycle + 1e-4) << "main k=" << k;
    }
    prev_steer = out.lateral.steering_tire_angle;
  }
}

namespace
{

std::vector<rclcpp::Parameter> gate_overrides()
{
  return {
    rclcpp::Parameter("inputs", std::vector<int64_t>{11, 12, 13, 14, 31}),
    rclcpp::Parameter("inputs_names.31", std::string("in_lane_stop")),
    rclcpp::Parameter("nominal_filter.enable_steer_accel_limit", true),
    rclcpp::Parameter("transition_filter.enable_steer_accel_limit", true),
  };
}

const std::vector<uint16_t> & subscription_sources()
{
  static const std::vector<uint16_t> ids{stop_id, main_id, local_id, remote_id, in_lane_stop_id};
  return ids;
}

class GateDriver
{
public:
  explicit GateDriver(GateFixture & fixture) : fixture_(fixture) {}

  void cycle_once(const std::map<uint16_t, Control> & commands, const uint16_t selected)
  {
    t_ += cycle;
    ASSERT_TRUE(fixture_.set_time(t_));
    const auto before = fixture_.outputs().size();
    for (const auto & [id, cmd] : commands) {
      fixture_.publish(id, cmd);
    }
    if (selected != builtin_id && commands.count(selected)) {
      ASSERT_TRUE(fixture_.wait_outputs(before + 1)) << "t=" << t_;
    } else {
      fixture_.spin_for_a_while();
    }
  }

  double t() const { return t_; }

private:
  GateFixture & fixture_;
  double t_ = start_time;
};

bool is_builtin_output(const TimedControl & output)
{
  return output.control.stamp.sec != 0 || output.control.stamp.nanosec != 0;
}

void expect_gate_outputs_within(
  const std::vector<TimedControl> & outputs, const double limit, const std::string & tag)
{
  for (size_t k = 2; k < outputs.size(); ++k) {
    if (is_builtin_output(outputs.at(k)) || is_builtin_output(outputs.at(k - 1))) {
      continue;
    }
    const double dt1 = outputs.at(k).t - outputs.at(k - 1).t;
    const double dt0 = outputs.at(k - 1).t - outputs.at(k - 2).t;
    if (
      dt1 < VehicleCmdFilter::DT_MIN_STEER_ACCEL_LIMIT ||
      dt0 < VehicleCmdFilter::DT_MIN_STEER_ACCEL_LIMIT) {
      continue;
    }
    const double s0 = outputs.at(k - 2).control.lateral.steering_tire_angle;
    const double s1 = outputs.at(k - 1).control.lateral.steering_tire_angle;
    const double s2 = outputs.at(k).control.lateral.steering_tire_angle;
    const double r1 = (s1 - s0) / dt0;
    const double r2 = (s2 - s1) / dt1;
    const double accel = (r2 - r1) / dt1;
    EXPECT_LE(std::abs(accel), limit * (1.0 + 1e-9) + tolerance_of_accel(1.0, std::min(dt0, dt1)))
      << tag << " k=" << k << " t=" << outputs.at(k).t;
    const double f1 = outputs.at(k - 1).control.lateral.steering_tire_rotation_rate;
    const double f2 = outputs.at(k).control.lateral.steering_tire_rotation_rate;
    EXPECT_LE(std::abs(f2 - f1) / dt1, limit * (1.0 + 1e-9) + 1e-4) << tag << " field k=" << k;
  }
}

}  // namespace

TEST(ControlCmdGateIntegration, AllSourcePairsStayWithinLimit)
{
  std::vector<uint16_t> sources = subscription_sources();
  sources.push_back(builtin_id);
  const double limit = 0.64;
  const auto run =
    [&](const uint16_t from, const uint16_t to, const bool transition, const bool fallback) {
      GateFixture fixture(gate_overrides());
      ASSERT_TRUE(fixture.set_state({1.0, 0.0, ControlModeReport::AUTONOMOUS}));
      GateDriver driver(fixture);
      std::map<uint16_t, Control> idle;
      for (const auto id : subscription_sources()) idle[id] = Control();
      for (int i = 0; i < 25; ++i) driver.cycle_once(idle, builtin_id);
      if (from != builtin_id) {
        ASSERT_TRUE(fixture.select(from, false));
      }
      const double from_begin = driver.t();
      for (int i = 0; i < 33; ++i) {
        auto commands = idle;
        commands[from] = steer_command(0.15 * (driver.t() + cycle - from_begin), 0.15);
        if (from == builtin_id) commands.erase(builtin_id);
        driver.cycle_once(commands, from);
      }
      const double held = fixture.outputs().back().control.lateral.steering_tire_angle;
      const double switch_t = driver.t();
      uint16_t selected = to;
      if (fallback) {
        selected = from;
        for (int i = 0; i < 200 && fixture.source() != builtin_id; ++i) {
          auto commands = idle;
          commands.erase(from);
          driver.cycle_once(commands, builtin_id);
        }
        ASSERT_EQ(fixture.source(), builtin_id);
        selected = builtin_id;
      } else if (to != from) {
        ASSERT_TRUE(fixture.select(to, transition)) << from << "->" << to;
      }
      for (int i = 0; i < 66; ++i) {
        auto commands = idle;
        if (selected != builtin_id) {
          commands[selected] = steer_command(held - 0.15 * (driver.t() + cycle - switch_t), -0.15);
        }
        driver.cycle_once(commands, selected);
      }
      const auto tag = std::to_string(from) + "->" + std::to_string(to) +
                       (transition ? " transition" : "") + (fallback ? " fallback" : "");
      expect_gate_outputs_within(fixture.outputs(), limit, tag);
    };
  for (const auto from : sources) {
    for (const auto to : sources) {
      if (from == to) continue;
      for (const bool transition : {false, true}) {
        run(from, to, transition, false);
      }
    }
  }
  run(main_id, builtin_id, false, true);
}

TEST(ControlCmdGateIntegration, DiscardsNonFiniteCommands)
{
  GateFixture fixture(gate_overrides());
  ASSERT_TRUE(fixture.set_state({1.0, 0.0, ControlModeReport::AUTONOMOUS}));
  GateDriver driver(fixture);
  std::map<uint16_t, Control> idle;
  for (const auto id : subscription_sources()) idle[id] = Control();
  for (int i = 0; i < 25; ++i) driver.cycle_once(idle, builtin_id);
  ASSERT_TRUE(fixture.select(main_id, false));
  const double begin = driver.t();
  const double nan = std::numeric_limits<float>::quiet_NaN();
  const double inf = std::numeric_limits<float>::infinity();
  for (int i = 0; i < 60; ++i) {
    auto commands = idle;
    auto cmd = steer_command(0.15 * (driver.t() + cycle - begin), 0.15);
    bool invalid = true;
    if (i == 20)
      cmd.lateral.steering_tire_angle = static_cast<float>(nan);
    else if (i == 25)
      cmd.lateral.steering_tire_angle = static_cast<float>(inf);
    else if (i == 30)
      cmd.lateral.steering_tire_rotation_rate = static_cast<float>(nan);
    else if (i == 35)
      cmd.longitudinal.velocity = static_cast<float>(nan);
    else
      invalid = false;
    commands[main_id] = cmd;
    if (invalid) {
      auto without_main = idle;
      without_main.erase(main_id);
      driver.cycle_once(without_main, main_id);
      const auto before = fixture.outputs().size();
      fixture.publish(main_id, cmd);
      fixture.spin_for_a_while();
      EXPECT_EQ(fixture.outputs().size(), before) << "i=" << i;
    } else {
      driver.cycle_once(commands, main_id);
    }
  }
  for (const auto & out : fixture.outputs()) {
    EXPECT_TRUE(std::isfinite(out.control.lateral.steering_tire_angle));
    EXPECT_TRUE(std::isfinite(out.control.lateral.steering_tire_rotation_rate));
    EXPECT_TRUE(std::isfinite(out.control.longitudinal.velocity));
  }
  expect_gate_outputs_within(fixture.outputs(), 0.64, "non-finite");
}

TEST(ControlCmdGateIntegration, FirstCommandsAfterStartup)
{
  GateFixture fixture(gate_overrides());
  ASSERT_TRUE(fixture.set_state({1.0, 0.0, ControlModeReport::AUTONOMOUS}));
  GateDriver driver(fixture);
  std::map<uint16_t, Control> idle;
  for (const auto id : subscription_sources()) idle[id] = Control();
  for (int i = 0; i < 25; ++i) driver.cycle_once(idle, builtin_id);
  EXPECT_GE(fixture.outputs().size(), 5u);
  ASSERT_TRUE(fixture.select(main_id, false));
  auto commands = idle;
  commands[main_id] = steer_command(0.3);
  for (int i = 0; i < 3; ++i) driver.cycle_once(commands, main_id);
  expect_gate_outputs_within(fixture.outputs(), 0.64, "startup");
  const auto & outputs = fixture.outputs();
  const double dt = outputs.back().t - outputs.at(outputs.size() - 2).t;
  EXPECT_GE(dt, VehicleCmdFilter::DT_MIN_STEER_ACCEL_LIMIT);
}

TEST(ControlCmdGateIntegration, ResumesAfterShortSourceGap)
{
  for (const double gap : {0.3, 0.5, 0.8}) {
    GateFixture fixture(gate_overrides());
    ASSERT_TRUE(fixture.set_state({1.0, 0.0, ControlModeReport::AUTONOMOUS}));
    GateDriver driver(fixture);
    std::map<uint16_t, Control> idle;
    for (const auto id : subscription_sources()) idle[id] = Control();
    for (int i = 0; i < 25; ++i) driver.cycle_once(idle, builtin_id);
    ASSERT_TRUE(fixture.select(main_id, false));
    const double begin = driver.t();
    const auto ramp = [&]() {
      auto commands = idle;
      commands[main_id] = steer_command(0.15 * (driver.t() + cycle - begin), 0.15);
      return commands;
    };
    for (int i = 0; i < 40; ++i) driver.cycle_once(ramp(), main_id);
    const int gap_cycles = static_cast<int>(std::lround(gap / cycle));
    for (int i = 0; i < gap_cycles; ++i) {
      auto commands = idle;
      commands.erase(main_id);
      driver.cycle_once(commands, main_id);
    }
    ASSERT_EQ(fixture.source(), main_id) << "gap=" << gap;
    for (int i = 0; i < 33; ++i) driver.cycle_once(ramp(), main_id);
    expect_gate_outputs_within(fixture.outputs(), 0.64, "gap=" + std::to_string(gap));
  }
}

TEST(ControlCmdGateIntegration, NonSelectedBuiltinDoesNotOutput)
{
  GateFixture fixture(gate_overrides());
  ASSERT_TRUE(fixture.set_state({1.0, 0.0, ControlModeReport::AUTONOMOUS}));
  GateDriver driver(fixture);
  std::map<uint16_t, Control> idle;
  for (const auto id : subscription_sources()) idle[id] = Control();
  for (int i = 0; i < 25; ++i) driver.cycle_once(idle, builtin_id);
  ASSERT_TRUE(fixture.select(main_id, false));
  const auto before = fixture.outputs().size();
  for (int i = 0; i < 67; ++i) driver.cycle_once(idle, main_id);
  EXPECT_EQ(fixture.outputs().size() - before, 67u);
}

}  // namespace autoware::control_command_gate::test
