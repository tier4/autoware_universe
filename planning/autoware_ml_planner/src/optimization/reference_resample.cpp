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

#include "autoware/ml_planner/optimization/reference_resample.hpp"

#include <autoware_utils/math/normalization.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace autoware::ml_planner::optimization
{
namespace
{
constexpr double curvature_epsilon_inv_m = 1.0e-6;
constexpr double steer_curvature_limit_rad = 0.7;

std::array<double, opt_horizon + 1> unwrap_yaw(
  const std::array<PoseSample, opt_horizon + 1> & poses)
{
  std::array<double, opt_horizon + 1> yaw{};
  yaw[0] = poses[0].yaw;
  for (size_t index = 1; index < yaw.size(); ++index) {
    yaw[index] =
      yaw[index - 1] + autoware_utils::normalize_radian(poses[index].yaw - yaw[index - 1]);
  }
  return yaw;
}

std::array<double, opt_horizon + 1> curvature_from_steering(
  const std::array<PoseSample, opt_horizon + 1> & poses, const double wheelbase_m)
{
  const double kappa_limit = std::tan(steer_curvature_limit_rad) / wheelbase_m;
  std::array<double, opt_horizon + 1> kappa{};
  for (size_t index = 0; index < poses.size(); ++index) {
    const double steering =
      std::clamp(poses[index].steering_rad, -steer_curvature_limit_rad, steer_curvature_limit_rad);
    kappa[index] = std::clamp(std::tan(steering) / wheelbase_m, -kappa_limit, kappa_limit);
  }
  return kappa;
}

double interpolate(
  const std::array<double, opt_horizon + 1> & arc,
  const std::array<double, opt_horizon + 1> & values, const double distance)
{
  if (distance <= arc.front()) {
    return values.front();
  }
  if (distance >= arc.back()) {
    return values.back();
  }
  const auto upper = std::upper_bound(arc.begin(), arc.end(), distance);
  const size_t index = static_cast<size_t>(std::distance(arc.begin(), upper));
  const double span = arc[index] - arc[index - 1];
  if (span < 1.0e-9) {
    return values[index];
  }
  const double ratio = (distance - arc[index - 1]) / span;
  return values[index - 1] + ratio * (values[index] - values[index - 1]);
}
}  // namespace

ResampledSequence resample_pose_sequence(
  const std::array<PoseSample, opt_horizon + 1> & poses, const double ego_speed_mps,
  const double lateral_accel_limit_mps2, const double wheelbase_m,
  const double min_acceleration_mps2, const double max_acceleration_mps2)
{
  if (wheelbase_m <= 0.0) {
    throw std::invalid_argument("resample_pose_sequence: wheelbase must be positive");
  }
  if (min_acceleration_mps2 > max_acceleration_mps2) {
    throw std::invalid_argument("resample_pose_sequence: acceleration bounds are inverted");
  }

  std::array<double, opt_horizon + 1> arc{};
  std::array<double, opt_horizon + 1> yaw = unwrap_yaw(poses);
  std::array<double, opt_horizon + 1> x{};
  std::array<double, opt_horizon + 1> y{};
  std::array<double, opt_horizon + 1> desired_speed{};
  for (size_t index = 0; index < poses.size(); ++index) {
    x[index] = poses[index].x;
    y[index] = poses[index].y;
    desired_speed[index] = std::max(0.0, poses[index].speed_mps);
    if (index > 0) {
      arc[index] = arc[index - 1] + std::hypot(x[index] - x[index - 1], y[index] - y[index - 1]);
    }
  }
  const auto curvature = curvature_from_steering(poses, wheelbase_m);
  const double arc_end = arc.back();
  const double speed0 = std::max(0.0, ego_speed_mps);

  ResampledSequence output;
  output.curvature_inv_m = curvature;
  output.poses[0] = poses[0];
  output.poses[0].yaw = yaw[0];
  output.speed_mps[0] = speed0;

  double speed = speed0;
  double distance = 0.0;
  for (size_t step = 0; step < opt_horizon; ++step) {
    const double kappa = interpolate(arc, curvature, distance);
    double speed_cap = std::numeric_limits<double>::infinity();
    if (std::abs(kappa) > curvature_epsilon_inv_m) {
      speed_cap = std::sqrt(std::max(lateral_accel_limit_mps2, 0.0) / std::abs(kappa));
    }
    const double target = std::min(speed_cap, interpolate(arc, desired_speed, distance));
    const double acceleration =
      std::clamp((target - speed) / opt_dt_s, min_acceleration_mps2, max_acceleration_mps2);
    speed = std::max(0.0, speed + acceleration * opt_dt_s);
    distance = std::min(arc_end, distance + speed * opt_dt_s);
    const double kappa_at_station = interpolate(arc, curvature, distance);

    const size_t index = step + 1;
    output.poses[index].x = interpolate(arc, x, distance);
    output.poses[index].y = interpolate(arc, y, distance);
    output.poses[index].yaw = interpolate(arc, yaw, distance);
    output.speed_mps[index] = speed;
    output.steering_rad[index] = std::atan(kappa_at_station * wheelbase_m);
    output.acceleration_mps2[index] = acceleration;
  }
  return output;
}

}  // namespace autoware::ml_planner::optimization
