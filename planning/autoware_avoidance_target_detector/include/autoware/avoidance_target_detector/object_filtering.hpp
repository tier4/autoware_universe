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

#ifndef AUTOWARE__AVOIDANCE_TARGET_DETECTOR__OBJECT_FILTERING_HPP_
#define AUTOWARE__AVOIDANCE_TARGET_DETECTOR__OBJECT_FILTERING_HPP_

#include "autoware/avoidance_target_detector/boundary.hpp"
#include "autoware/avoidance_target_detector/parameter.hpp"

#include <autoware/trajectory/trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detail/predicted_object__struct.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::avoidance_target_detector
{

using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

/** Two-class Bayesian filter with a configurable transition matrix and observation model. */
class TwoClassFilter
{
public:
  using Matrix2x2 = std::array<std::array<double, 2>, 2>;

  explicit TwoClassFilter(const PredictedObject & object, const rclcpp::Time & last_update_time);
  TwoClassFilter(const TwoClassFilter &) = default;
  TwoClassFilter(TwoClassFilter &&) noexcept = default;
  TwoClassFilter & operator=(const TwoClassFilter &) = default;
  TwoClassFilter & operator=(TwoClassFilter &&) noexcept = default;
  virtual ~TwoClassFilter() = default;

  [[nodiscard]] double get_posterior() const { return posterior_; }
  [[nodiscard]] bool is_initialized() const { return is_initialized_; }
  [[nodiscard]] bool is_stale(const rclcpp::Time & current_time) const
  {
    return current_time - last_update_time_ > rclcpp::Duration::from_seconds(1.0);
  }
  void observe_and_update(
    const rclcpp::Time & current_time, const PredictedObject & object,
    const Trajectory & trajectory);

protected:
  void apply_transition_to_prior(const PredictedObject & object, const Trajectory & trajectory);
  void apply_bayesian_update();

  virtual void calculate_likelihood(
    const PredictedObject & object, [[maybe_unused]] const Trajectory & trajectory) = 0;

  double target_likelihood_{1.0};
  double non_target_likelihood_{1.0};

private:
  [[nodiscard]] virtual Matrix2x2 transition_matrix(
    [[maybe_unused]] const PredictedObject & object,
    [[maybe_unused]] const Trajectory & trajectory) const = 0;

  double prior_;
  double posterior_;
  bool is_initialized_;
  rclcpp::Time last_update_time_;
};

/** Filters objects by classification label and probability. */
class TargetFilter final : public TwoClassFilter
{
public:
  using TwoClassFilter::TwoClassFilter;

  void calculate_likelihood(
    const PredictedObject & object, [[maybe_unused]] const Trajectory & trajectory) override;

private:
  [[nodiscard]] Matrix2x2 transition_matrix(
    [[maybe_unused]] const PredictedObject & object,
    [[maybe_unused]] const Trajectory & trajectory) const override;
};

/** Filters objects by linear velocity (stationary vs moving). */
class StationaryFilter final : public TwoClassFilter
{
public:
  using TwoClassFilter::TwoClassFilter;

  void calculate_likelihood(
    const PredictedObject & object, [[maybe_unused]] const Trajectory & trajectory) override;

private:
  [[nodiscard]] Matrix2x2 transition_matrix(
    [[maybe_unused]] const PredictedObject & object,
    [[maybe_unused]] const Trajectory & trajectory) const override;
};

/** Filters objects by on-trajectory lateral deviation pattern. */
class DeviationFilter final : public TwoClassFilter
{
public:
  using TwoClassFilter::TwoClassFilter;

  void calculate_likelihood(
    const PredictedObject & object, [[maybe_unused]] const Trajectory & trajectory) override;

private:
  [[nodiscard]] Matrix2x2 transition_matrix(
    [[maybe_unused]] const PredictedObject & object,
    [[maybe_unused]] const Trajectory & trajectory) const override;
};

/** Per-object avoidance target detector using Bayesian filters and hysteresis. */
class ObjectSelector;

class AvoidanceTargetDetector
{
public:
  explicit AvoidanceTargetDetector(
    const PredictedObject & object, const rclcpp::Time & last_update_time);

  void observe_and_update_all(
    const rclcpp::Time & current_time, const PredictedObject & object,
    const Trajectory & trajectory, const lanelet::LaneletMapPtr & route_map,
    const lanelet::routing::RoutingGraphConstPtr & routing_graph,
    const lanelet::BasicPolygon2d & near_segment_polygon,
    const std::vector<lanelet::ConstLanelet> & ego_lanelets);
  [[nodiscard]] double get_is_target_probability() const { return target_filter_->get_posterior(); }
  [[nodiscard]] double get_is_stationary_probability() const
  {
    return stationary_filter_->get_posterior();
  }
  [[nodiscard]] double get_is_deviated_probability() const
  {
    return deviation_filter_->get_posterior();
  }

  [[nodiscard]] bool is_object_of_interest() const { return target_filter_->get_posterior() > 0.5; }
  [[nodiscard]] bool is_stationary() const { return stationary_filter_->get_posterior() > 0.5; }
  [[nodiscard]] bool is_deviated() const { return deviation_filter_->get_posterior() > 0.5; }

  [[nodiscard]] bool is_stationary_avoidance_target() const
  {
    return is_stationary_avoidance_target_stamped_.second;
  }

  [[nodiscard]] bool is_moving_vehicle() const { return is_moving_vehicle_stamped_.second; }

  [[nodiscard]] bool is_stale(const rclcpp::Time & current_time) const
  {
    return rclcpp::Time(current_time) - rclcpp::Time(stale_check_time_) >
           rclcpp::Duration::from_seconds(FilterManagerParams::stale_threshold_seconds);
  }

private:
  uint8_t avoidance_state_change_count_{0};
  uint8_t moving_vehicle_state_change_count_{0};

  std::unique_ptr<TargetFilter> target_filter_;
  std::unique_ptr<StationaryFilter> stationary_filter_;
  std::unique_ptr<DeviationFilter> deviation_filter_;

  std::pair<rclcpp::Time, bool> is_stationary_avoidance_target_stamped_;
  std::pair<rclcpp::Time, bool> is_moving_vehicle_stamped_;
  rclcpp::Time stale_check_time_;

  bool is_driving_along_candidate_now_{false};
  bool is_on_polygon_now_{false};

  bool is_avoidance_tracking_initialized_{false};
  bool is_moving_vehicle_tracking_initialized_{false};

  void track_avoidance_targets();
  void track_driving_along_vehicles();

  friend class ObjectSelector;
};

using AvoidanceTargetDetectorMap = std::map<std::string, AvoidanceTargetDetector>;

/** Selects avoidance targets from predicted objects using per-object Bayesian filters. */
class ObjectSelector
{
public:
  /** @brief Update per-object Bayesian filters and prune stale entries. */
  void update_objects(
    const rclcpp::Time & current_time, const PredictedObjects & objects,
    const Trajectory & trajectory, const ExtendedRouteHandler & extended_route_handler,
    const autoware::experimental::trajectory::Trajectory<TrajectoryPoint> & ego_trajectory,
    bool ego_trajectory_built);

  /**
   * @brief Select avoidance targets from predicted objects using updated filter state.
   * @details Call update_objects() first in the same cycle. Runs avoidance-target tracking,
   *          then removes non-targets and objects outside longitudinal and lateral distance bounds.
   */
  [[nodiscard]] PredictedObjects get_avoidance_targets(
    const PredictedObjects & objects, const Trajectory & trajectory,
    const RouteBounds & route_bounds);

  [[nodiscard]] PredictedObjects get_driving_along_vehicles(
    const PredictedObjects & objects, const ExtendedRouteHandler & extended_route_handler,
    const autoware::experimental::trajectory::Trajectory<TrajectoryPoint> & ego_trajectory,
    const Trajectory & trajectory);

private:
  void track_avoidance_targets();
  void track_driving_along_vehicles();

  AvoidanceTargetDetectorMap object_filters_;
};

/**
 * @brief Check whether the object footprint lies beyond the trajectory end in arc-length.
 * @param trajectory Reference trajectory.
 * @param object Predicted object.
 * @return True if the minimum footprint s exceeds trajectory.length().
 */
[[nodiscard]] bool is_object_beyond_trajectory_end(
  const Trajectory & trajectory, const PredictedObject & object);

/**
 * @brief Check whether an object should be filtered out as on-trajectory corridor alignment.
 * @param trajectory Reference trajectory.
 * @param object Predicted object.
 * @return True if the object aligns with the trajectory corridor and should be filtered out.
 */
[[nodiscard]] bool should_filter_out_on_trajectory_object(
  const Trajectory & trajectory, const PredictedObject & object);

/**
 * @brief Check whether an object should be filtered out by longitudinal distance from trajectory.
 * @details Removes objects whose entire footprint lies before the trajectory start or after the
 *          trajectory end, with tolerance. Uses signed longitudinal deviation from start/end poses
 *          so points beyond the polyline end are detected correctly.
 */
[[nodiscard]] bool should_filter_out_by_longitudinal_distance(
  const Trajectory & trajectory, const PredictedObject & object,
  const LongitudinalDistanceFilterParams & params = {});

/**
 * @brief Check whether an object should be filtered out by lateral distance from drivable bounds.
 * @details Removes objects whose entire footprint lies outside the drivable area corridor, with
 *          tolerance.
 */
[[nodiscard]] bool should_filter_out_by_lateral_distance(
  const RouteBounds & route_bounds, const Trajectory & trajectory, const PredictedObject & object,
  const LateralDistanceFilterParams & params = {});

}  // namespace autoware::avoidance_target_detector

#endif  // AUTOWARE__AVOIDANCE_TARGET_DETECTOR__OBJECT_FILTERING_HPP_
