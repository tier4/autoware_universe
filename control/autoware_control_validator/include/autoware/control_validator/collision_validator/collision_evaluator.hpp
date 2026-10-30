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

// Ported from autoware_trajectory_validator (autoware.universe 3eb3290355):
//   src/filters/safety/collision_check_filter/types.hpp
//   src/filters/safety/collision_check_filter/trajectory_utils.{hpp,cpp}
//   src/filters/safety/collision_check_filter/assessment.cpp (find_collision_timing, TTC part;
//   assess_ego_drac, loop structure)
// Changes from the source: namespace; TrajectoryIdentification / CandidateTrajectory /
// FilterContext removed; PET search, object DRAC, RSS, caches and the constant curvature predictor
// are not ported; find_collision_timing renamed to find_first_collision and returns the indices of
// the first colliding window; the yaw of vehicle predicted paths is replaced by a bicycle model;
// the ego DRAC iterates over braking delays instead of decelerations. This copy does not follow
// later changes of the source.

#ifndef AUTOWARE__CONTROL_VALIDATOR__COLLISION_VALIDATOR__COLLISION_EVALUATOR_HPP_
#define AUTOWARE__CONTROL_VALIDATOR__COLLISION_VALIDATOR__COLLISION_EVALUATOR_HPP_

#include "autoware/control_validator/collision_validator/pointcloud_preprocessor.hpp"
#include "autoware/control_validator/collision_validator/sweep_types.hpp"

#include <Eigen/Geometry>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::control_validator::collision_validator
{
namespace detail
{
template <typename Points>
std::pair<double, double> project_points(const Points & ring, double axis_x, double axis_y)
{
  if (ring.empty()) {
    return {0.0, 0.0};
  }

  const auto project = [&](const auto & point) {
    return boost::geometry::get<0>(point) * axis_x + boost::geometry::get<1>(point) * axis_y;
  };

  auto point_it = ring.begin();
  double min_projection = project(*point_it);
  double max_projection = min_projection;
  for (++point_it; point_it != ring.end(); ++point_it) {
    const double projection = project(*point_it);
    min_projection = std::min(min_projection, projection);
    max_projection = std::max(max_projection, projection);
  }

  return {min_projection, max_projection};
}

template <typename ClosedRing>
bool has_separating_axis(
  const ClosedRing & candidate_axes, const ClosedRing & ring_a, const ClosedRing & ring_b)
{
  auto previous_it = candidate_axes.begin();
  for (auto current_it = std::next(candidate_axes.begin()); current_it != candidate_axes.end();
       ++current_it) {
    const double edge_x =
      boost::geometry::get<0>(*current_it) - boost::geometry::get<0>(*previous_it);
    const double edge_y =
      boost::geometry::get<1>(*current_it) - boost::geometry::get<1>(*previous_it);
    const auto [min_a, max_a] = project_points(ring_a, -edge_y, edge_x);
    const auto [min_b, max_b] = project_points(ring_b, -edge_y, edge_x);
    if (max_a < min_b || max_b < min_a) {
      return true;
    }
    previous_it = current_it;
  }

  return false;
}

template <typename ConvexPolygon>
bool intersects_sat(const ConvexPolygon & poly_a, const ConvexPolygon & poly_b)
{
  const auto & ring_a = poly_a.outer();
  const auto & ring_b = poly_b.outer();

  if (ring_a.empty() || ring_b.empty()) {
    return false;
  }

  return !has_separating_axis(ring_a, ring_a, ring_b) &&
         !has_separating_axis(ring_b, ring_a, ring_b);
}

Eigen::Isometry2d pose_to_isometry(const geometry_msgs::msg::Pose & pose);
Point2d transform_point(const Eigen::Isometry2d & transform, const Point2d & point);

// Vertices of the outline that `shape` defines, in the object local frame, as an open ring.
std::vector<Point2d> create_base_polygon(const autoware_perception_msgs::msg::Shape & shape);

Polygon2d to_polygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape);

void correct_vehicle_yaw(PoseTrajectory & poses, double initial_yaw, double rear_axle_length);

struct EgoDimensions
{
  double front_offset{0.0};
  double rear_overhang{0.0};
  double vehicle_width{0.0};
};

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const std::vector<Point2d> & base_poly);

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory,
  const autoware_perception_msgs::msg::Shape & object_shape);

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const EgoDimensions & ego_dimensions);

TimeTrajectory serialize_times(const autoware_planning_msgs::msg::Trajectory & trajectory);
TravelDistanceTrajectory serialize_distances(
  const autoware_planning_msgs::msg::Trajectory & trajectory);

struct InterpolatedState
{
  double distance;
  geometry_msgs::msg::Pose pose;
  double longitudinal_velocity;
};

class TrajectoryInterpolator
{
public:
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory_points_{};
  const rclcpp::Time reference_time_{};
  const std::vector<double> time_from_refs_{};
  const std::vector<double> dist_from_fronts_{};

  explicit TrajectoryInterpolator(const autoware_planning_msgs::msg::Trajectory & trajectory);
  InterpolatedState interpolate_state_from_time(const rclcpp::Time & target_time) const;
  InterpolatedState interpolate_state_from_dist(double target_dist) const;
};

/**
 * @brief Expand the predicted path of an object at constant velocity.
 * @param time_offset added to every sample time (object stamp minus ego reference stamp) [s]
 */
TrajectoryData generate_predicted_path_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  const autoware_perception_msgs::msg::PredictedPath & predicted_path, double max_time,
  double time_offset, double time_resolution, std::optional<double> rear_axle_length);

/**
 * @brief Trajectory of a static polygon: the same footprint at every sample time.
 */
TrajectoryData generate_static_trajectory(const Polygon2d & polygon, const TimeTrajectory & times);

struct CollisionStep
{
  size_t ref_index;
  IndexRange test_index_range;
};

/**
 * @brief First time window [t_{k-1}, t_k] of `ref` (k >= 1) in which the swept footprints of both
 * trajectories overlap. Windows whose end step is stopped are skipped.
 */
std::optional<CollisionStep> find_first_collision(
  const TrajectoryData & ref, const TrajectoryData & test,
  const std::vector<bool> & ref_stopped = {});

/**
 * @brief First time window [t_{k-1}, t_k] of `ref` (k >= 1) whose swept footprint contains the
 * point. Windows whose end step is stopped are skipped.
 * @return k
 */
std::optional<size_t> find_first_collision(
  const TrajectoryData & ref, const Point2d & point, const std::vector<bool> & ref_stopped = {});
}  // namespace detail

struct CollisionCandidate
{
  Polygon2d polygon_now;  // map; a single vertex for a point
  double distance_now{0.0};
  bool near{false};
  uint8_t brake_level{0};
};

/**
 * @brief Cosine of the angle between the ego heading at the collision time and the direction from
 * the ego base_link origin to the collision point.
 * @param ego_pose_at_hit resampled ego pose at t_k
 * @param forward false when the ego is reversing (the heading vector is flipped)
 */
double compute_cos_collision(
  const geometry_msgs::msg::Pose & ego_pose_at_hit, bool forward, const Point2d & hit_point);

struct CollisionEvaluatorParam
{
  double max_time{5.0};
  double time_resolution{0.1};
  size_t n_eval_object{10};
  size_t n_eval_pointcloud{10};
  std::vector<uint8_t> enabled_labels;
  double max_objects_stamp_gap_s{0.5};
  double stop_velocity{0.1};
  double proximity_distance{0.3};
  std::array<double, 3> reaction_times{2.0, 1.5, 1.0};
  double braking_deceleration{6.0};
  double cos_threshold{-1.0};
  double rear_axle_ratio{0.3};
};

struct BrakingParam
{
  double reaction_time;
  double deceleration;
};

class SweepsWithBraking;

class CollisionEvaluator
{
public:
  explicit CollisionEvaluator(const CollisionEvaluatorParam & param);

  void update_parameters(const CollisionEvaluatorParam & param);

  /**
   * @brief Sweep the ego footprint along the predicted trajectory and judge the proximity and the
   * collision with objects (footprint sweep) and static points. Only the objects and points that
   * are near or have a brake_level of 1 or more are returned.
   * @param objects nullptr evaluates no object
   * @param point_clusters clusters in the map frame; nullopt evaluates no point
   */
  std::vector<CollisionCandidate> evaluate(
    const autoware_planning_msgs::msg::Trajectory & predicted_trajectory,
    const nav_msgs::msg::Odometry & odom, const vehicle_info_utils::VehicleInfo & vehicle_info,
    const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects,
    const std::optional<PointCloudClusters> & point_clusters) const;

private:
  friend class SweepsWithBraking;

  EgoSweep build_ego_sweep(
    const autoware_planning_msgs::msg::Trajectory & predicted_trajectory,
    const nav_msgs::msg::Odometry & odom, const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::optional<BrakingParam> & braking = std::nullopt) const;

  detail::TrajectoryData make_object_trajectory(
    const autoware_perception_msgs::msg::PredictedObject & object, double time_offset,
    const detail::TimeTrajectory & static_times) const;

  CollisionCandidate evaluate_object(
    const EgoSweep & ego_sweep, SweepsWithBraking & braked_sweeps, CollisionCandidate candidate,
    const detail::TrajectoryData & object_trajectory) const;

  void evaluate_points(
    const EgoSweep & ego_sweep, SweepsWithBraking & braked_sweeps,
    const std::vector<Point2d> & points, std::vector<CollisionCandidate> & out) const;

  CollisionEvaluatorParam param_;
};
}  // namespace autoware::control_validator::collision_validator

#endif  // AUTOWARE__CONTROL_VALIDATOR__COLLISION_VALIDATOR__COLLISION_EVALUATOR_HPP_
