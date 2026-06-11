// Copyright 2026 Autoware Foundation
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

#include "autoware/avoidance_target_detector/two_class_filter.hpp"

#include "autoware/avoidance_target_detector/impl.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace autoware::avoidance_target_detector
{

namespace
{

constexpr double k_high_likelihood = 0.95;
constexpr double k_low_likelihood = 0.05;
constexpr double k_neutral_likelihood = 0.5;
bool is_object_of_interest(const PredictedObject & object)
{
  constexpr float probability_threshold = 0.1f;
  return std::any_of(
    object.classification.begin(), object.classification.end(),
    [](const ObjectClassification & classification) {
      if (classification.probability <= probability_threshold) {
        return false;
      }
      return std::find(
               labels_of_interest.begin(), labels_of_interest.end(), classification.label) !=
             labels_of_interest.end();
    });
}

double linear_velocity_norm(const PredictedObject & object)
{
  const auto & linear = object.kinematics.initial_twist_with_covariance.twist.linear;
  return std::hypot(linear.x, linear.y, linear.z);
}

}  // namespace

TwoClassFilter::TwoClassFilter(
  [[maybe_unused]] const PredictedObject & object, const rclcpp::Time & last_update_time)
: prior_{k_neutral_likelihood},
  posterior_{k_neutral_likelihood},
  is_initialized_{false},
  last_update_time_{last_update_time}
{
}

void TwoClassFilter::observe_and_update(
  const rclcpp::Time & current_time, const PredictedObject & object, const Trajectory & trajectory)
{
  calculate_likelihood(object, trajectory);

  if (!is_initialized_) {
    posterior_ = target_likelihood_;
    prior_ = posterior_;
    target_likelihood_ = 1.0;
    non_target_likelihood_ = 1.0;
    is_initialized_ = true;
    last_update_time_ = current_time;
    return;
  }

  apply_transition_to_prior(object, trajectory);
  apply_bayesian_update();
  last_update_time_ = current_time;

  target_likelihood_ = 1.0;
  non_target_likelihood_ = 1.0;
  prior_ = posterior_;
}

void TwoClassFilter::apply_transition_to_prior(
  const PredictedObject & object, const Trajectory & trajectory)
{
  const auto transition = transition_matrix(object, trajectory);
  const double non_target_prior = 1.0 - prior_;
  prior_ = transition[0][0] * prior_ + transition[1][0] * non_target_prior;
}

void TwoClassFilter::apply_bayesian_update()
{
  const double numerator = prior_ * target_likelihood_;
  const double denominator = numerator + (1.0 - prior_) * non_target_likelihood_;
  posterior_ = denominator > 0.0 ? numerator / denominator : prior_;
}

/** TargetFilter implementation */

void TargetFilter::calculate_likelihood(
  const PredictedObject & object, [[maybe_unused]] const Trajectory & trajectory)
{
  target_likelihood_ = is_object_of_interest(object) ? k_high_likelihood : k_low_likelihood;
  non_target_likelihood_ = 1.0 - target_likelihood_;
}

TwoClassFilter::Matrix2x2 TargetFilter::transition_matrix(
  [[maybe_unused]] const PredictedObject & object,
  [[maybe_unused]] const Trajectory & trajectory) const
{
  constexpr double state_persistence = 0.95;
  constexpr double switch_probability = 1.0 - state_persistence;
  return {{
    {state_persistence, switch_probability},
    {switch_probability, state_persistence},
  }};
}

/** StationaryFilter implementation */

void StationaryFilter::calculate_likelihood(
  const PredictedObject & object, [[maybe_unused]] const Trajectory & trajectory)
{
  const double pre_clamp_probability =
    -(linear_velocity_norm(object) - MovingObjectFilterParams::max_linear_velocity_mps) /
    (MovingObjectFilterParams::max_linear_velocity_mps -
     MovingObjectFilterParams::promising_stop_velocity_mps);
  target_likelihood_ = std::clamp(pre_clamp_probability, 0.1, 0.99);
  non_target_likelihood_ = 1.0 - target_likelihood_;
}

TwoClassFilter::Matrix2x2 StationaryFilter::transition_matrix(
  [[maybe_unused]] const PredictedObject & object,
  [[maybe_unused]] const Trajectory & trajectory) const
{
  const double velocity_norm = linear_velocity_norm(object);

  if (velocity_norm < MovingObjectFilterParams::max_linear_velocity_mps) {
    return {{
      {0.9, 0.1},
      {0.5, 0.5},
    }};
  }

  return {{
    {0.05, 0.95},
    {0.95, 0.05},
  }};
}

/** DeviationFilter implementation */

void DeviationFilter::calculate_likelihood(
  const PredictedObject & object, [[maybe_unused]] const Trajectory & trajectory)
{
  target_likelihood_ = should_filter_out_on_trajectory_object(trajectory, object)
                         ? k_low_likelihood
                         : k_high_likelihood;
  non_target_likelihood_ = 1.0 - target_likelihood_;
}

TwoClassFilter::Matrix2x2 DeviationFilter::transition_matrix(
  const PredictedObject & object, const Trajectory & trajectory) const
{
  if (is_object_beyond_trajectory_end(trajectory, object)) {
    return {{
      {0.8, 0.2},
      {0.4, 0.6},
    }};
  }

  return {{
    {0.95, 0.05},
    {0.2, 0.8},
  }};
}

FilterManager::FilterManager(const PredictedObject & object, const rclcpp::Time & last_update_time)
: target_filter_{std::make_unique<TargetFilter>(object, last_update_time)},
  stationary_filter_{std::make_unique<StationaryFilter>(object, last_update_time)},
  deviation_filter_{std::make_unique<DeviationFilter>(object, last_update_time)},
  is_target_stamped_{last_update_time, false},
  stale_check_time_{last_update_time}
{
}

void FilterManager::observe_and_update_all(
  const rclcpp::Time & current_time, const PredictedObject & object, const Trajectory & trajectory)
{
  stale_check_time_ = current_time;

  target_filter_->observe_and_update(current_time, object, trajectory);
  stationary_filter_->observe_and_update(current_time, object, trajectory);
  deviation_filter_->observe_and_update(current_time, object, trajectory);

  const bool is_target_now = is_object_of_interest() && is_stationary() && is_deviated();

  if (!is_initialized_) {
    is_target_stamped_.first = current_time;
    is_target_stamped_.second = is_target_now;
    is_initialized_ = true;
    return;
  }

  state_change_count_ = (is_target_now != is_target_stamped_.second) ? state_change_count_ + 1 : 0;

  if (
    rclcpp::Time(current_time) - rclcpp::Time(is_target_stamped_.first) <
    rclcpp::Duration::from_seconds(hysteresis_seconds)) {
    return;
  }

  if (state_change_count_ < count_threshold) {
    return;
  }

  if (is_target_now && !is_target_stamped_.second) {
    debug_log_ = std::string("Turned to target on due to \n") +
                 "is_object_of_interest: " + std::to_string(get_is_target_probability()) + "\n" +
                 "is_stationary: " + std::to_string(get_is_stationary_probability()) + "\n" +
                 "is_deviated: " + std::to_string(get_is_deviated_probability());
  } else if (!is_target_now && is_target_stamped_.second) {
    debug_log_ = std::string("Turned to non-target on due to \n") +
                 "is_object_of_interest: " + std::to_string(get_is_target_probability()) + "\n" +
                 "is_stationary: " + std::to_string(get_is_stationary_probability()) + "\n" +
                 "is_deviated: " + std::to_string(get_is_deviated_probability());
  }

  is_target_stamped_.first = current_time;
  is_target_stamped_.second = is_target_now;
}

}  // namespace autoware::avoidance_target_detector
