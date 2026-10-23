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
#include <optional>
#include <string>
#include <vector>

namespace autoware::control_command_gate::test
{

namespace
{

constexpr double start_time = 1.0;

VehicleCmdFilterParam make_node_transition_param()
{
  auto p = make_isolated_steer_accel_param();
  p.steer_rate_lim_for_steer_cmd = {0.4, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  return p;
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
  std::optional<VehicleState> state{};
  std::optional<bool> transition{};
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
  const double limit = accel_limit_of(spec.transition ? transition : nominal, spec.state.speed);
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

TEST(CommandFilterIntegration, EngageStartsFromActualSteerWithZeroRate)
{
  std::vector<Event> events;
  for (size_t k = 0; k < 100; ++k) {
    Event e{start_time + k * cycle, main_id, steer_command(0.0)};
    if (k == 0) e.state = VehicleState{1.0, 0.3, ControlModeReport::MANUAL};
    if (k == 50) e.state = VehicleState{1.0, 0.3, ControlModeReport::AUTONOMOUS};
    events.push_back(e);
  }
  const auto p = make_isolated_steer_accel_param();
  const double limit = accel_limit_of(p, 1.0);

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
    {start_time, main_id, steer_command(-0.8),
     VehicleState{speed, -0.8, ControlModeReport::MANUAL}});
  for (size_t k = 1; k <= 67; ++k) {
    Event e{start_time + k * cycle, main_id, steer_command(-0.8 + 0.6 * k * cycle, 0.6)};
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
    Event e{t, builtin_id, cmd};
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
  const auto p = make_isolated_steer_accel_param();
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
  const auto p = make_isolated_steer_accel_param();
  const double limit = accel_limit_of(p, 1.0);
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

TEST(CommandFilterIntegration, PreventsWindupDuringLaterStageClamp)
{
  auto p = make_isolated_steer_accel_param();
  p.steer_accel_lim_for_steer_cmd.assign(p.reference_speed_points.size(), 0.8);
  p.lat_acc_lim_for_steer_cmd.assign(p.reference_speed_points.size(), 1.5);
  const double steer_max = std::atan(1.5 * 4.76 / 100.0);
  for (const bool reverse : {false, true}) {
    std::vector<Event> events;
    for (size_t k = 0; k < 45; ++k) {
      Event e{start_time + k * cycle, main_id, steer_command(0.2)};
      if (k == 0) e.state = VehicleState{10.0, 0.0, ControlModeReport::AUTONOMOUS};
      events.push_back(e);
    }
    for (size_t k = 45; k < 75; ++k) {
      Event e{start_time + k * cycle, main_id, steer_command(reverse ? 0.0 : 0.2)};
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

  const auto np = make_isolated_steer_accel_param();
  const auto tp = make_node_transition_param();
  std::vector<Event> events;
  for (size_t k = 0; k < 55; ++k) {
    Event e{start_time + k * cycle, main_id, steer_command(0.0, 0.5)};
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
  expect_rotation_rate_accels_within(
    outputs, 30, 0.2, event_dts(events), accel_limit_of(np, 1.0), "field release");
}

}  // namespace autoware::control_command_gate::test
