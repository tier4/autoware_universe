// Copyright 2025 TIER IV, Inc.
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

#include "slow_down_interpolator.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::experimental::utils
{
tl::expected<SlowDownInterpolator::SlowDownPlan, std::string>
SlowDownInterpolator::get_interp_to_point(
  const double curr_vel, const double lon_dist_to_bound_m, double lat_dist_to_bound_m,
  const SideKey side_key) const
{
  const auto interp_vel_mps = interp_velocity(curr_vel, lat_dist_to_bound_m, side_key);
  const auto exp_decel_mps2 =
    calc_expected_deceleration(curr_vel, interp_vel_mps, lon_dist_to_bound_m);

  const auto exp_jerk = interp_jerk(lat_dist_to_bound_m, exp_decel_mps2, side_key);
  const auto exp_vel = calc_new_velocity(curr_vel, exp_decel_mps2, exp_jerk, lon_dist_to_bound_m);

  const auto rel_dist_m =
    std::max((interp_vel_mps * interp_vel_mps - curr_vel * curr_vel) / (2 * exp_decel_mps2), 0.0);

  return SlowDownPlan{rel_dist_m, exp_vel, exp_decel_mps2};
}

double SlowDownInterpolator::calc_expected_deceleration(
  const double curr_vel, const double target_vel, const double arclength_to_point_m) const
{
  const auto & th_acc_mps2 = th_trigger_.th_acc_mps2;
  if (arclength_to_point_m < std::numeric_limits<double>::epsilon()) {
    return th_acc_mps2.max;
  }
  // check if lon dist to point needs hard braking
  // v^2 = v_0^2 + 2as
  const auto clamped_vel = std::max(curr_vel, th_trigger_.th_vel_mps.min);
  const auto target_acc_mp2 =
    (target_vel * target_vel - clamped_vel * clamped_vel) / (2.0 * arclength_to_point_m);

  return std::clamp(target_acc_mp2, th_acc_mps2.max, th_acc_mps2.min);
}

double SlowDownInterpolator::calc_new_velocity(
  const double curr_vel, const double a_target, const double jerk,
  const double lon_dist_to_point_m) const
{
  const auto th_slow_down_vel = th_trigger_.th_vel_mps.min;

  if (curr_vel < th_slow_down_vel) {
    return th_slow_down_vel;
  }

  const double t = lon_dist_to_point_m / std::max(curr_vel, 1e-6);  // avoid div by zero
  const double t_j = std::abs(a_target / std::max(jerk, 1e-6));

  if (t <= t_j) {
    return std::clamp(
      curr_vel + 0.5 * jerk * t * t, th_slow_down_vel,
      std::max(th_slow_down_vel + std::numeric_limits<double>::epsilon(), curr_vel));
  }

  const double delta_v1 = 0.5 * a_target * a_target / jerk;
  const double delta_v2 = a_target * (t - t_j);

  return std::clamp(
    curr_vel + delta_v1 + delta_v2, th_slow_down_vel,
    std::max(th_slow_down_vel + std::numeric_limits<double>::epsilon(), curr_vel));
};

double SlowDownInterpolator::interp_velocity(
  const double curr_vel, const double lat_dist, const SideKey side_key) const
{
  if (curr_vel <= th_trigger_.th_vel_mps.min) {
    return th_trigger_.th_vel_mps.min;
  }

  const auto min_dist = th_trigger_.th_dist_to_boundary_m[side_key].min;
  const auto max_dist = th_trigger_.th_dist_to_boundary_m[side_key].max;
  const auto lat_dist_axis = get_lat_dist_axis(side_key);

  const auto vel_axis = get_vel_axis();
  if (lat_dist >= max_dist) {
    return vel_axis.back();
  }

  if (lat_dist <= min_dist) {
    return vel_axis.front();
  }

  return autoware::interpolation::lerp(lat_dist_axis, vel_axis, lat_dist);
}

std::vector<double> SlowDownInterpolator::get_lat_dist_axis(const SideKey side_key) const
{
  return {
    th_trigger_.th_dist_to_boundary_m[side_key].min,
    th_trigger_.th_dist_to_boundary_m[side_key].max};
}

std::vector<double> SlowDownInterpolator::get_vel_axis() const
{
  return {th_trigger_.th_vel_mps.min, th_trigger_.th_vel_mps.max};
}

double SlowDownInterpolator::interp_jerk(
  const double lat_dist_to_bound_m, const double expected_decel_mps2, const SideKey side_key) const
{
  const auto th_dist_to_boundary_m = th_trigger_.th_dist_to_boundary_m[side_key];
  const auto lat_ratio = (th_dist_to_boundary_m.max - lat_dist_to_bound_m) /
                         (th_dist_to_boundary_m.max - th_dist_to_boundary_m.min);

  const auto th_decel_mps2 = th_trigger_.th_acc_mps2;
  const auto max_decel = std::abs(th_decel_mps2.max);
  const auto min_decel = std::abs(th_decel_mps2.min);
  const auto lon_ratio = (std::abs(expected_decel_mps2) - min_decel) / (max_decel - min_decel);

  const auto combined_ratio = 0.5 * lat_ratio + 0.5 * lon_ratio;

  const auto min_jerk = th_trigger_.th_jerk_mps3.min;
  const auto max_jerk = th_trigger_.th_jerk_mps3.max;
  if (combined_ratio <= 0.0) return min_jerk;
  if (combined_ratio >= 1.0) return max_jerk;

  return autoware::interpolation::lerp(min_jerk, max_jerk, combined_ratio);
}

tl::expected<double, std::string> SlowDownInterpolator::calc_expected_min_velocity() const
{
  const auto t_e = time_jerk_limited_s_;
  const auto a_c = th_trigger_.th_acc_mps2.min;
  auto min_vel = std::min(-a_c * t_e, th_trigger_.th_vel_mps.min);

  if (min_vel < 0.0) {
    return tl::make_unexpected("Invalid, expected min vel less than zero.");
  }

  return min_vel;
}

tl::expected<double, std::string> SlowDownInterpolator::calc_expected_stop_distance(
  const double curr_vel_mps) const
{
  /*
    Derivation of jerk-limited stopping distance (no constant accel phase):

    Assume:
      - Jerk j is applied to ramp acceleration from 0 to a_e
      - Time to reach a_e: t_e = a_e / j
      - Velocity during ramp: v(t) = ∫ a(t) dt = ∫ jt dt = 0.5 * j * t^2
      - Distance during ramp: s(t) = ∫ v(t) dt = ∫ (0.5 * j * t^2) dt = (1/6) * j * t^3

    Plug in t_e = a_e / j:
      s_half = (1/6) * j * (a_e / j)^3
             = (1/6) * a_e^3 / j^2

    Full stop has symmetric ramp-in and ramp-out:
      s_total = 2 * s_half
              = (1/3) * a_e^3 / j^2

    Let t_e = a_e / j  →  j = a_e / t_e
    Substitute into equation:
      s_total = (1/3) * a_e * t_e^2
  */

  const auto t_delay = th_trigger_.brake_delay_s;
  const auto t_e = time_jerk_limited_s_;
  const auto error_margin = th_trigger_.dist_error_m;
  const auto a_e = th_trigger_.th_acc_mps2.max;

  const auto delay_dist = std::max(curr_vel_mps, th_trigger_.th_vel_mps.min) * t_delay;
  const auto jerk_stop_dist = -a_e * t_e * t_e / 6.0;

  if (delay_dist < 0.0 || jerk_stop_dist < 0.0 || error_margin < 0.0) {
    return tl::make_unexpected("Invalid, expected values less than zero.");
  }

  return delay_dist + jerk_stop_dist + error_margin;
}
double SlowDownInterpolator::calc_time_to_departure(
  const double vel_mps, const double lon_dist_to_bound_m)
{
  return vel_mps / lon_dist_to_bound_m;
}

bool SlowDownInterpolator::is_exceeded_ttd_th(
  const double vel_mps, const double lon_dist_to_bound_m) const
{
  const auto time_to_departure_s = calc_time_to_departure(vel_mps, lon_dist_to_bound_m);
  return time_to_departure_s > th_trigger_.th_time_to_departure_s;
}
}  // namespace autoware::motion_velocity_planner::experimental::utils
