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

#ifndef SLOW_DOWN_INTERPOLATOR_HPP_
#define SLOW_DOWN_INTERPOLATOR_HPP_

#include "type_alias.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <tl_expected/expected.hpp>

#include <string>
#include <vector>
namespace autoware::motion_velocity_planner::experimental::utils
{

class SlowDownInterpolator
{
public:
  struct SlowDownPlan
  {
    double rel_dist_m;
    double target_vel_mps;
    double expected_decel;
  };

  explicit SlowDownInterpolator(const TriggerThreshold & th_trigger)
  : th_trigger_(th_trigger),
    time_jerk_limited_s_(th_trigger_.th_acc_mps2.max / th_trigger_.th_jerk_mps3.max)
  {
  }

  /**
   * @brief Calculates a deceleration plan for the ego vehicle toward the boundary.
   *
   * This function computes how far in advance the vehicle should start slowing down,
   * what velocity it should aim for at the slow-down point, and what deceleration value
   * would achieve that, based on the current velocity, lateral and longitudinal distances,
   * and the side of the vehicle approaching the boundary.
   *
   * @param curr_vel Current velocity of the vehicle in meters per second.
   * @param lon_dist_to_bound_m Distance along the trajectory to the boundary point.
   * @param lat_dist_to_bound_m Lateral distance from the ego to the boundary (positive or
   * negative).
   * @param side_key Specifies whether the left or right boundary is used for threshold lookup.
   * @return A tuple containing:
   *         - distance to start slowing down,
   *         - target velocity at the boundary,
   *         - expected deceleration.
   *         Returns an error message if the deceleration exceeds the threshold.
   */
  [[nodiscard]] tl::expected<SlowDownPlan, std::string> get_interp_to_point(
    const double curr_vel, const double lon_dist_to_bound_m, double lat_dist_to_bound_m,
    const SideKey side_key) const;

private:
  /**
   * @brief Estimates the deceleration required to reach the target velocity over a given distance.
   *
   * Uses the standard kinematic formula and clamps the result between the minimum and maximum
   * allowed deceleration values.
   *
   * @param curr_vel Current velocity.
   * @param target_vel Desired velocity.
   * @param arclength_to_point_m Distance available for deceleration.
   * @return Estimated deceleration, clamped within configured thresholds.
   */
  [[nodiscard]] double calc_expected_deceleration(
    const double curr_vel, const double target_vel, const double arclength_to_point_m) const;

  /**
   * @brief Calculates the velocity at the slow-down point considering acceleration and jerk.
   *
   * This method simulates velocity change under constant jerk followed by constant acceleration,
   * and clamps the result to ensure it does not drop below the configured minimum velocity.
   *
   * @param curr_vel Current velocity of the ego vehicle.
   * @param a_target Target acceleration after the jerk phase.
   * @param jerk Applied jerk value.
   * @param lon_dist_to_point_m Distance available for deceleration.
   * @return Estimated target velocity at the end of the slow-down interval.
   */
  [[nodiscard]] double calc_new_velocity(
    const double curr_vel, const double a_target, const double jerk,
    const double lon_dist_to_point_m) const;

  [[nodiscard]] double interp_velocity(
    const double curr_vel, const double lat_dist, const SideKey side_key) const;

  [[nodiscard]] std::vector<double> get_lat_dist_axis(const SideKey side_key) const;

  [[nodiscard]] std::vector<double> get_vel_axis() const;

  [[nodiscard]] double interp_jerk(
    const double lat_dist_to_bound_m, const double expected_decel_mps2,
    const SideKey side_key) const;

  /**
   * @brief Calculate the minimum velocity where emergency jerk-limited stopping
   *        takes as long as a comfortable constant deceleration stop.
   *
   * This function finds the velocity at which the total stopping time using
   * emergency deceleration with jerk ramp-in (i.e., time = a_e / j_e) matches
   * the time required to stop using comfortable constant deceleration
   * (i.e., time = v / a_c). This represents the velocity at which switching to
   * emergency braking will preserve the same time-to-stop experience as a
   * comfortable stop.
   *
   * @return Expected minimum velocity [m/s] at which the ego vehicle can begin
   *         emergency jerk-limited braking and still match the time duration
   *         of a comfortable deceleration profile.
   */
  [[nodiscard]] tl::expected<double, std::string> calc_expected_min_velocity() const;

  /**
   * @brief Estimate the total stopping distance using jerk-limited braking with delay and error
   * margin.
   *
   * This function computes the expected stopping distance by accounting for:
   * - The forward distance traveled during the brake delay period (at current velocity or a minimum
   * threshold).
   * - The theoretical stopping distance under a jerk-limited deceleration profile (s = 1/3 * a *
   * t²).
   * - An additional safety margin to account for braking error or uncertainty.
   *
   * The jerk-limited stop is modeled as a symmetric profile that ramps acceleration from 0 to max
   * (a_e), then back to 0, with time t_e = a_e / j_e. No constant acceleration plateau is assumed.
   *
   * @param curr_vel The current ego vehicle velocity [m/s] at the time of evaluation.
   * @return Total expected stopping distance [m]. Returns tl::unexpected if any component is
   * invalid (negative).
   */
  [[nodiscard]] tl::expected<double, std::string> calc_expected_stop_distance(
    const double curr_vel) const;

  [[nodiscard]] double calc_max_accel_jerk_time() const;

  static double calc_time_to_departure(const double vel_mps, const double lon_dist_to_bound_m);

  [[nodiscard]] bool is_exceeded_ttd_th(
    const double vel_mps, const double lon_dist_to_bound_m) const;

  TriggerThreshold th_trigger_;
  double time_jerk_limited_s_{std::numeric_limits<double>::max()};
};

}  // namespace autoware::motion_velocity_planner::experimental::utils

#endif  // SLOW_DOWN_INTERPOLATOR_HPP_
