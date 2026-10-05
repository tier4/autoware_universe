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

namespace autoware::control_command_gate::test
{

TEST(ClipDiag, WarnsOnContinuousClipAndStaysOkWithoutClip)
{
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  constexpr double speed = 1.0;
  const auto p = make_isolated_steer_accel_param();
  FilterFixture fixture(p, p);
  fixture.enable_diag();
  fixture.set_state({speed, 0.0, ControlModeReport::AUTONOMOUS});

  double t = 1.0;
  double rotation_rate = 0.0;
  const auto step = [&](const double rotation_accel) {
    t += cycle;
    Control cmd;
    cmd.lateral.steering_tire_rotation_rate =
      static_cast<float>(rotation_rate + rotation_accel * cycle);
    rotation_rate = fixture.step(main_id, t, cmd).lateral.steering_tire_rotation_rate;
    return fixture.run_diag();
  };

  for (int k = 0; k < 3; ++k) {
    step(0.0);
  }
  const double clip_accel = accel_limit_of(p, speed) + 1.0;
  for (int k = 1; k <= 7; ++k) {
    EXPECT_EQ(step(clip_accel), k == 7 ? DiagnosticStatus::WARN : DiagnosticStatus::OK)
      << "clip k=" << k;
  }
  for (int k = 1; k <= 3; ++k) {
    EXPECT_EQ(step(0.0), DiagnosticStatus::OK) << "no clip k=" << k;
  }
}

}  // namespace autoware::control_command_gate::test
