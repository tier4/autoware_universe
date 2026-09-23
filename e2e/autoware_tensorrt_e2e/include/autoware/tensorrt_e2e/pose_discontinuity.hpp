// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
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
