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
//   src/filters/safety/collision_check_filter/assessment.cpp (find_collision_timing, TTC part)
// Changes from the source: namespace; TrajectoryIdentification / CandidateTrajectory /
// FilterContext removed; braking profile, PET search, DRAC / RSS, caches and the constant
// curvature predictor are not ported. This copy does not follow later changes of the source.

#ifndef HAZARD__COLLISION_SWEEP_HPP_
#define HAZARD__COLLISION_SWEEP_HPP_

#include "hazard/pointcloud_preprocessor.hpp"

#include <Eigen/Geometry>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::control_validator::hazard
{
using autoware_utils_geometry::Box2d;
using autoware_utils_geometry::MultiPoint2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

static constexpr double TIME_INDEX_EPSILON = 1e-3;

using IndexRange = std::pair<size_t, size_t>;
using TimeRange = std::pair<double, double>;

using TimeTrajectory = std::vector<double>;
using TravelDistanceTrajectory = std::vector<double>;
using PoseTrajectory = std::vector<geometry_msgs::msg::Pose>;
using QuadTrajectory = std::vector<std::array<Point2d, 4>>;
using NgonTrajectory = std::vector<std::vector<Point2d>>;
using FootprintTrajectory = std::variant<QuadTrajectory, NgonTrajectory>;

struct CollisionTiming
{
  double ttc;
};

class TrajectoryData
{
private:
  TimeTrajectory times_;
  PoseTrajectory poses_;
  FootprintTrajectory footprints_;
  mutable std::map<IndexRange, Box2d> envelope_cache_;
  mutable std::map<IndexRange, Polygon2d> convex_cache_;

  size_t get_same_or_earlier_time_index(const double t) const
  {
    const auto it = std::upper_bound(times_.begin(), times_.end(), t + TIME_INDEX_EPSILON);
    if (it == times_.begin()) return 0;
    return std::distance(times_.begin(), it - 1);
  }

  size_t get_same_or_later_time_index(const double t) const
  {
    const auto it = std::lower_bound(times_.begin(), times_.end(), t - TIME_INDEX_EPSILON);
    if (it == times_.end()) return times_.size() - 1;
    return std::distance(times_.begin(), it);
  }

  Box2d compute_envelope(const IndexRange & key) const
  {
    Box2d box;
    boost::geometry::assign_inverse(box);
    std::visit(
      [&](const auto & polygons) {
        for (size_t i = key.first; i <= key.second; ++i) {
          for (const auto & pt : polygons[i]) {
            boost::geometry::expand(box, pt);
          }
        }
      },
      footprints_);
    return box;
  }

  Polygon2d compute_convex(const IndexRange & key) const
  {
    MultiPoint2d all_points;
    std::visit(
      [&](const auto & polygons) {
        size_t total_point_count = (key.second - key.first + 1U) * polygons[key.first].size();
        all_points.reserve(total_point_count);
        for (size_t i = key.first; i <= key.second; ++i) {
          for (const auto & pt : polygons[i]) {
            all_points.push_back(pt);
          }
        }
      },
      footprints_);

    Polygon2d hull;
    hull.outer().reserve(all_points.size());
    boost::geometry::convex_hull(all_points, hull);
    return hull;
  }

public:
  TrajectoryData(TimeTrajectory times, PoseTrajectory poses, FootprintTrajectory footprints)
  : times_(std::move(times)), poses_(std::move(poses)), footprints_(std::move(footprints))
  {
    if (times_.empty()) {
      throw std::invalid_argument("Trajectory must not be empty");
    }
    if (times_.size() != poses_.size()) {
      throw std::invalid_argument("Trajectory sizes mismatch (times vs poses)");
    }
    const auto footprint_size =
      std::visit([](const auto & polygons) { return polygons.size(); }, footprints_);
    if (times_.size() != footprint_size) {
      throw std::invalid_argument("Trajectory sizes mismatch (times vs footprints)");
    }
    if (!std::is_sorted(times_.begin(), times_.end())) {
      throw std::invalid_argument("Trajectory times must be in ascending order");
    }
  }

  TrajectoryData() = delete;

  const TimeTrajectory & getTimes() const { return times_; }
  const PoseTrajectory & getPoses() const { return poses_; }
  const FootprintTrajectory & getFootprints() const { return footprints_; }

  size_t size() const { return times_.size(); }

  std::optional<IndexRange> resolve_covering_index_range(const TimeRange & time_range) const
  {
    const bool has_no_overlap =
      time_range.second < times_.front() || time_range.first > times_.back();
    if (has_no_overlap) {
      return std::nullopt;
    }

    const auto start_index = get_same_or_earlier_time_index(time_range.first);
    const auto end_index = get_same_or_later_time_index(time_range.second);
    return IndexRange{start_index, end_index};
  }

  const Box2d & get_or_compute_envelope(const IndexRange & key) const
  {
    auto [it, inserted] = envelope_cache_.try_emplace(key);
    if (inserted) {
      it->second = compute_envelope(key);
    }
    return it->second;
  }

  const Box2d & get_or_compute_overall_envelope() const
  {
    return get_or_compute_envelope(IndexRange{0U, times_.size() - 1});
  }

  const Polygon2d & get_or_compute_convex(const IndexRange & key) const
  {
    auto [it, inserted] = convex_cache_.try_emplace(key);
    if (inserted) {
      it->second = compute_convex(key);
    }
    return it->second;
  }
};

namespace geometry
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
}  // namespace detail

template <typename ConvexPolygon>
bool intersects_sat(const ConvexPolygon & poly_a, const ConvexPolygon & poly_b)
{
  const auto & ring_a = poly_a.outer();
  const auto & ring_b = poly_b.outer();

  if (ring_a.empty() || ring_b.empty()) {
    return false;
  }

  return !detail::has_separating_axis(ring_a, ring_a, ring_b) &&
         !detail::has_separating_axis(ring_b, ring_a, ring_b);
}

Eigen::Isometry2d pose_to_isometry(const geometry_msgs::msg::Pose & pose);
Point2d transform_point(const Eigen::Isometry2d & transform, const Point2d & point);

// Vertices of the outline that `shape` defines, in the object local frame, as an open ring.
std::vector<Point2d> create_base_polygon(const autoware_perception_msgs::msg::Shape & shape);

Polygon2d to_polygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape);

// Closed polygon of one footprint sample.
Polygon2d to_polygon2d(const std::array<Point2d, 4> & quad);
Polygon2d to_polygon2d(const std::vector<Point2d> & ring);
}  // namespace geometry

namespace trajectory
{
namespace footprint
{
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
}  // namespace footprint

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
};

/**
 * @brief Expand the predicted path of an object at constant velocity.
 * @param time_offset added to every sample time (object stamp minus ego reference stamp) [s]
 */
TrajectoryData generate_predicted_path_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  const autoware_perception_msgs::msg::PredictedPath & predicted_path, double max_time,
  double time_offset, double time_resolution);

/**
 * @brief Trajectory of a static polygon: the same footprint at every sample time.
 */
TrajectoryData generate_static_trajectory(const Polygon2d & polygon, const TimeTrajectory & times);
}  // namespace trajectory

namespace collision_timing_assessment
{
/**
 * @brief First time window [t_{k-1}, t_k] of `ref_trajectory` (k >= 1) in which the swept
 * footprints of both trajectories overlap. The TTC is t_k.
 */
std::optional<CollisionTiming> find_collision_timing(
  const TrajectoryData & ref_trajectory, const TrajectoryData & test_trajectory,
  const std::vector<bool> & ref_stopped = {});
}  // namespace collision_timing_assessment

struct SweepCandidate
{
  enum class Kind { OBJECT, POINTCLOUD };
  Kind kind{Kind::OBJECT};
  uint8_t label{0};
  Polygon2d polygon_now;  // map; a single-vertex degenerate polygon for a point
  double distance_now{0.0};
  std::optional<double> ttc;
  std::optional<double> cos_collision;  // cosine of the collision angle at the TTC (set with ttc)
  builtin_interfaces::msg::Time source_stamp;  // header.stamp of the objects / pointcloud message
};

/**
 * @brief Cosine of the angle between the ego heading at the collision time and the direction from
 * the ego base_link origin to the collision point.
 * @param ego_pose_at_hit resampled ego pose at t_k
 * @param forward false when the ego is reversing (the heading vector is flipped)
 */
double compute_cos_collision(
  const geometry_msgs::msg::Pose & ego_pose_at_hit, bool forward, const Point2d & hit_point);

struct SweepLimits
{
  double max_time{5.0};
  size_t n_eval_object{10};
  size_t n_eval_pointcloud{10};
  std::vector<uint8_t> enabled_labels;
  bool enable_pointcloud{true};
  double max_objects_stamp_gap_s{0.5};
  double stop_velocity{0.1};
};

class CollisionSweep
{
public:
  CollisionSweep(const vehicle_info_utils::VehicleInfo & vehicle_info, double time_resolution);

  void set_time_resolution(double time_resolution);

  /**
   * @brief Sweep the ego footprint along the predicted trajectory and compute the TTC and the
   * collision angle against objects (footprint sweep) and pointcloud clusters (point in
   * footprint). Object candidates are returned whether or not they collide; point candidates are
   * returned only for points that collide.
   * @param point_clusters clusters already filtered by the caller (D3); the closest
   *   n_eval_pointcloud clusters are evaluated with all their points
   * @param pointcloud_stamp header.stamp of the pointcloud the clusters came from
   * @throws std::invalid_argument when the predicted trajectory cannot be interpolated
   */
  std::vector<SweepCandidate> evaluate(
    const autoware_planning_msgs::msg::Trajectory & predicted, const nav_msgs::msg::Odometry & odom,
    const autoware_perception_msgs::msg::PredictedObjects & objects,
    const std::vector<PointCluster> & point_clusters,
    const builtin_interfaces::msg::Time & pointcloud_stamp, const SweepLimits & limits) const;

  Polygon2d ego_polygon(const geometry_msgs::msg::Pose & pose) const;

  static double distance_now(const Polygon2d & ego_now, const Polygon2d & candidate);

private:
  trajectory::footprint::EgoDimensions ego_dimensions_;
  double time_resolution_;
};
}  // namespace autoware::control_validator::hazard

#endif  // HAZARD__COLLISION_SWEEP_HPP_
