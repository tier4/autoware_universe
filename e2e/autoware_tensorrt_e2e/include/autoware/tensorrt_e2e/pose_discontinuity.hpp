// Copyright 2026 TIER IV, Inc.
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

#pragma once
#include <array>
#include <cmath>
namespace autoware::tensorrt_e2e {
struct PoseContinuityLimits {
  // Slack plus physically possible motion; tune for the vehicle/localizer.
  double translation_slack_m{2.0};
  double max_speed_mps{60.0};
  double yaw_slack_rad{0.35};
  double max_yaw_rate_rps{2.0};
};
inline bool pose_discontinuous(const std::array<double, 4> &a,
                               const std::array<double, 4> &b, double dt,
                               const PoseContinuityLimits &limits) {
  const double yaw = std::abs(
      std::atan2(b[3] * a[2] - b[2] * a[3], b[2] * a[2] + b[3] * a[3]));
  return std::hypot(b[0] - a[0], b[1] - a[1]) >
             limits.translation_slack_m + limits.max_speed_mps * dt ||
         yaw > limits.yaw_slack_rad + limits.max_yaw_rate_rps * dt;
}
} // namespace autoware::tensorrt_e2e
