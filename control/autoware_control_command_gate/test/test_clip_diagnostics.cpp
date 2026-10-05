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

#include "common/clip_diagnostics.hpp"
#include "test_fixtures.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace autoware::control_command_gate::test
{

namespace
{

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

constexpr double cycle = 0.03;
constexpr double speed = 1.0;

uint8_t run_task(ClipDiag & diag)
{
  diagnostic_updater::DiagnosticStatusWrapper stat;
  static_cast<diagnostic_updater::DiagnosticTask &>(diag).run(stat);
  return stat.level;
}

double accel_limit_at(const VehicleCmdFilterParam & p)
{
  VehicleCmdFilter filter;
  filter.setParam(p);
  filter.setCurrentSpeed(speed);
  return filter.getSteerAccelLimForSteerCmd();
}

struct ClipRequest
{
  std::optional<double> steer_clip;
  std::optional<double> rotation_clip;
  double dt = cycle;
  uint16_t source_id = main_id;
  std::optional<bool> transition;
};

class ClipDriver
{
public:
  ClipDriver(
    const VehicleCmdFilterParam & nominal, const VehicleCmdFilterParam & transition,
    const bool enable_command_limit_filter = true)
  : fixture_(nominal, transition, enable_command_limit_filter),
    nominal_(nominal),
    transition_(transition)
  {
    fixture_.enable_diag();
    fixture_.set_state({speed, 0.0, ControlModeReport::AUTONOMOUS});
    for (int i = 0; i < 3; ++i) {
      step({});
    }
  }

  uint8_t step(const ClipRequest & request)
  {
    if (request.transition) {
      fixture_.filter().set_transition_flag(*request.transition);
      transition_flag_ = *request.transition;
    }
    const double dt_lim = std::min(request.dt, VehicleCmdFilter::DT_MAX_STEER_ACCEL_LIMIT);
    const double limit = accel_limit_at(transition_flag_ ? transition_ : nominal_);
    Control cmd;
    const double steer_accel = request.steer_clip ? limit + *request.steer_clip : 0.0;
    cmd.lateral.steering_tire_angle =
      static_cast<float>(steer_ + (rate_ + steer_accel * dt_lim) * dt_lim);
    const double rotation_accel = request.rotation_clip ? limit + *request.rotation_clip : 0.0;
    cmd.lateral.steering_tire_rotation_rate =
      static_cast<float>(rotation_rate_ + rotation_accel * dt_lim);

    t_ += request.dt;
    const auto out = fixture_.step(request.source_id, t_, cmd);
    if (request.source_id == builtin_id) {
      rate_ = 0.0;
    } else if (request.dt >= VehicleCmdFilter::DT_MIN_STEER_ACCEL_LIMIT) {
      rate_ = (out.lateral.steering_tire_angle - steer_) / dt_lim;
    }
    steer_ = out.lateral.steering_tire_angle;
    rotation_rate_ = out.lateral.steering_tire_rotation_rate;
    return fixture_.run_diag();
  }

  std::vector<size_t> warn_cycles(const std::vector<ClipRequest> & requests)
  {
    std::vector<size_t> result;
    for (size_t i = 0; i < requests.size(); ++i) {
      if (step(requests.at(i)) == DiagnosticStatus::WARN) {
        result.push_back(i + 1);
      }
    }
    return result;
  }

private:
  FilterFixture fixture_;
  VehicleCmdFilterParam nominal_;
  VehicleCmdFilterParam transition_;
  bool transition_flag_ = false;
  double t_ = 1.0;
  double steer_ = 0.0;
  double rate_ = 0.0;
  double rotation_rate_ = 0.0;
};

std::vector<ClipRequest> repeat(const ClipRequest & request, const size_t count)
{
  return std::vector<ClipRequest>(count, request);
}

std::vector<ClipRequest> concat(std::initializer_list<std::vector<ClipRequest>> parts)
{
  std::vector<ClipRequest> result;
  for (const auto & part : parts) {
    result.insert(result.end(), part.begin(), part.end());
  }
  return result;
}

ClipRequest steer_clip(const double c)
{
  ClipRequest r;
  r.steer_clip = c;
  return r;
}

ClipRequest rotation_clip(const double c)
{
  ClipRequest r;
  r.rotation_clip = c;
  return r;
}

}  // namespace

TEST(ClipDiag, LatchesNotificationUntilNextRun)
{
  ClipDiag diag("steer_accel_limit");
  EXPECT_EQ(run_task(diag), DiagnosticStatus::OK);
  diag.notify();
  EXPECT_EQ(run_task(diag), DiagnosticStatus::WARN);
  EXPECT_EQ(run_task(diag), DiagnosticStatus::OK);
  diag.notify();
  diag.notify();
  diag.notify();
  EXPECT_EQ(run_task(diag), DiagnosticStatus::WARN);
  EXPECT_EQ(run_task(diag), DiagnosticStatus::OK);
}

TEST(ClipDiag, NotifiesWhenSteerClipIntegralExceedsThreshold)
{
  auto p = make_isolated_steer_accel_param();
  p.steer_accel_clip_integral_th_diag = 40.0;
  ClipDriver driver(p, p);
  EXPECT_EQ(driver.warn_cycles(repeat(steer_clip(200.0), 21)), (std::vector<size_t>{7, 14, 21}));
}

TEST(ClipDiag, NotifiesWhenRotationClipIntegralExceedsThreshold)
{
  const auto p = make_isolated_steer_accel_param();
  ClipDriver driver(p, p);
  EXPECT_EQ(driver.warn_cycles(repeat(rotation_clip(1.0), 21)), (std::vector<size_t>{7, 14, 21}));
}

TEST(ClipDiag, IntegratesEachSideSeparately)
{
  auto p = make_isolated_steer_accel_param();
  p.steer_accel_clip_integral_th_diag = 40.0;
  ClipDriver driver(p, p);
  ClipRequest both;
  both.steer_clip = 200.0;
  both.rotation_clip = 400.0;
  EXPECT_EQ(driver.warn_cycles(repeat(both, 12)), (std::vector<size_t>{4, 8, 12}));
}

TEST(ClipDiag, ResetsIntegralOnNonClipCycle)
{
  const auto p = make_isolated_steer_accel_param();
  ClipDriver driver(p, p);
  const auto requests = concat(
    {repeat(rotation_clip(1.0), 3), repeat(ClipRequest{}, 1), repeat(rotation_clip(1.0), 7)});
  EXPECT_EQ(driver.warn_cycles(requests), (std::vector<size_t>{11}));
}

TEST(ClipDiag, IntegratesLongCycleWithCappedDt)
{
  const auto p = make_isolated_steer_accel_param();
  ClipDriver driver(p, p);
  auto long_cycle = rotation_clip(0.15);
  long_cycle.dt = 2.5;
  const auto requests = concat({repeat(long_cycle, 1), repeat(rotation_clip(1.0), 2)});
  EXPECT_EQ(driver.warn_cycles(requests), (std::vector<size_t>{3}));
}

TEST(ClipDiag, KeepsIntegralOnShortCycle)
{
  const auto p = make_isolated_steer_accel_param();
  ClipDriver driver(p, p);
  ClipRequest short_cycle;
  short_cycle.dt = 0.002;
  const auto requests =
    concat({repeat(rotation_clip(1.0), 3), repeat(short_cycle, 1), repeat(rotation_clip(1.0), 4)});
  EXPECT_EQ(driver.warn_cycles(requests), (std::vector<size_t>{8}));
}

TEST(ClipDiag, ResetsIntegralOnBuiltinCycle)
{
  const auto p = make_isolated_steer_accel_param();
  ClipDriver driver(p, p);
  ClipRequest builtin;
  builtin.source_id = builtin_id;
  const auto requests =
    concat({repeat(rotation_clip(1.0), 5), repeat(builtin, 1), repeat(rotation_clip(1.0), 7)});
  EXPECT_EQ(driver.warn_cycles(requests), (std::vector<size_t>{13}));
}

TEST(ClipDiag, ResetsIntegralOnDisabledFilterCycle)
{
  const auto nominal = make_isolated_steer_accel_param();
  auto transition = make_isolated_steer_accel_param();
  transition.enable_steer_accel_limit = false;
  ClipDriver driver(nominal, transition);
  auto to_disabled = rotation_clip(1.0);
  to_disabled.transition = true;
  auto to_enabled = rotation_clip(1.0);
  to_enabled.transition = false;
  const auto requests = concat(
    {repeat(rotation_clip(1.0), 5), repeat(to_disabled, 1), repeat(to_enabled, 1),
     repeat(rotation_clip(1.0), 6)});
  EXPECT_EQ(driver.warn_cycles(requests), (std::vector<size_t>{13}));
}

TEST(ClipDiag, UsesThresholdOfActiveFilterAndKeepsIntegralAcrossSwitch)
{
  const auto nominal = make_isolated_steer_accel_param();
  auto transition = make_isolated_steer_accel_param();
  transition.steer_accel_clip_integral_th_diag = 0.4;
  ClipDriver driver(nominal, transition);
  auto to_transition = rotation_clip(1.0);
  to_transition.transition = true;
  const auto requests = concat(
    {repeat(rotation_clip(1.0), 5), repeat(to_transition, 1), repeat(rotation_clip(1.0), 10)});
  EXPECT_EQ(driver.warn_cycles(requests), (std::vector<size_t>{14}));
}

TEST(ClipDiag, StaysOkWhenCommandLimitFilterIsDisabled)
{
  const auto p = make_isolated_steer_accel_param();
  ClipDriver driver(p, p, false);
  ClipRequest both;
  both.steer_clip = 5.0;
  both.rotation_clip = 5.0;
  EXPECT_TRUE(driver.warn_cycles(repeat(both, 30)).empty());
}

TEST(ClipDiag, PublishedByGate)
{
  GateFixture fixture({
    rclcpp::Parameter("nominal_filter.enable_steer_accel_limit", true),
    rclcpp::Parameter("transition_filter.enable_steer_accel_limit", true),
  });
  ASSERT_TRUE(fixture.set_state({speed, 0.0, ControlModeReport::AUTONOMOUS}));
  double t = 1.0;
  for (int i = 0; i < 25; ++i, t += cycle) {
    ASSERT_TRUE(fixture.set_time(t));
    fixture.publish(main_id, Control());
    fixture.spin_for_a_while();
  }
  const std::string name = "control_command_gate: steer_accel_limit";
  EXPECT_TRUE(
    fixture.wait_diagnostic_level(name, DiagnosticStatus::OK, std::chrono::milliseconds(3000)));
  ASSERT_TRUE(fixture.select(main_id, false));
  for (int i = 0; i < 30; ++i, t += cycle) {
    ASSERT_TRUE(fixture.set_time(t));
    Control cmd;
    cmd.lateral.steering_tire_angle = (i % 2 == 0) ? 0.5f : -0.5f;
    fixture.publish(main_id, cmd);
    const auto before = fixture.outputs().size();
    ASSERT_TRUE(fixture.wait_outputs(before + 1));
  }
  EXPECT_TRUE(
    fixture.wait_diagnostic_level(name, DiagnosticStatus::WARN, std::chrono::milliseconds(3000)));
}

}  // namespace autoware::control_command_gate::test
