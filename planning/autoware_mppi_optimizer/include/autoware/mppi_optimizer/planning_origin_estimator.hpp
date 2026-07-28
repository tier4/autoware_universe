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

#ifndef AUTOWARE__MPPI_OPTIMIZER__PLANNING_ORIGIN_ESTIMATOR_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__PLANNING_ORIGIN_ESTIMATOR_HPP_

#include <rclcpp/time.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>

namespace autoware::mppi_optimizer
{

struct PlanningOriginState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double velocity{0.0};
};

struct PlanningOriginEstimatorParams
{
  double max_position_error{0.1};
  double max_yaw_error{0.05};
};

struct PlanningOriginEstimate
{
  PlanningOriginState state;
  bool used_stitched_state{false};
  double position_error{0.0};
  double yaw_error{0.0};
};

/**
 * @brief Select a continuous MPPI planning origin from the previous optimal state trajectory.
 *
 * The estimator first interpolates the retained trajectory at the current time and compares it
 * with raw ego odometry. If both divergence checks pass, it returns the retained trajectory
 * interpolated at current_time + lookahead. Otherwise it clears its history and returns raw_ego.
 */
class PlanningOriginEstimator
{
public:
  using Trajectory = autoware_planning_msgs::msg::Trajectory;

  explicit PlanningOriginEstimator(const PlanningOriginEstimatorParams & params = {});

  void set_params(const PlanningOriginEstimatorParams & params);
  [[nodiscard]] const PlanningOriginEstimatorParams & params() const noexcept;

  /**
   * @brief Retain an accepted MPPI state trajectory and the time at which it was generated.
   */
  void store_trajectory(const Trajectory & trajectory, const rclcpp::Time & generation_time);

  /**
   * @brief Select raw ego or a time-interpolated state from the previous MPPI trajectory.
   *
   * @param raw_ego Current state from localization.
   * @param current_time Timestamp corresponding to raw_ego.
   * @param lookahead_seconds Optional system-delay lookahead applied only after divergence is
   *        checked at current_time.
   */
  PlanningOriginEstimate estimate(
    const PlanningOriginState & raw_ego, const rclcpp::Time & current_time,
    double lookahead_seconds = 0.0);

  /**
   * @brief Overwrite an output prefix with the time-aligned previous optimal trajectory.
   *
   * The output time_from_start values and message header are preserved. All other trajectory-point
   * fields are copied or interpolated from the retained trajectory.
   *
   * @return Number of output points overwritten.
   */
  std::size_t copy_prefix(
    Trajectory & trajectory, const rclcpp::Time & current_time, double prefix_duration) const;

  void reset() noexcept;
  [[nodiscard]] bool has_trajectory() const noexcept;

private:
  [[nodiscard]] std::optional<PlanningOriginState> interpolate(double relative_time) const;
  [[nodiscard]] std::optional<autoware_planning_msgs::msg::TrajectoryPoint> interpolate_point(
    double relative_time) const;

  PlanningOriginEstimatorParams params_;
  std::optional<Trajectory> previous_optimal_trajectory_;
  std::int64_t generation_time_ns_{0};
};

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__PLANNING_ORIGIN_ESTIMATOR_HPP_
