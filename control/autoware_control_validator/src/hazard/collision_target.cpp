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

#include "hazard/collision_target.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::control_validator::hazard
{
namespace
{
// Midpoint of the closest pair of points between the boundaries of two polygons, used when the
// footprints touch only along an edge and the intersection area is empty.
Point2d closest_boundary_midpoint(const Polygon2d & a, const Polygon2d & b)
{
  double best = std::numeric_limits<double>::infinity();
  Point2d best_a;
  Point2d best_b;
  const auto update = [&](const Point2d & pa, const Point2d & pb) {
    const double d = boost::geometry::distance(pa, pb);
    if (d < best) {
      best = d;
      best_a = pa;
      best_b = pb;
    }
  };
  const auto project = [](const Point2d & p, const Point2d & s0, const Point2d & s1) {
    const Eigen::Vector2d d = s1 - s0;
    const double len2 = d.squaredNorm();
    if (len2 < 1e-12) return s0;
    const double t = std::clamp(static_cast<double>((p - s0).dot(d)) / len2, 0.0, 1.0);
    return Point2d{s0.x() + t * d.x(), s0.y() + t * d.y()};
  };
  const auto & ra = a.outer();
  const auto & rb = b.outer();
  for (size_t i = 0; i + 1 < rb.size(); ++i) {
    for (const auto & pa : ra) {
      update(pa, project(pa, rb[i], rb[i + 1]));
    }
  }
  for (size_t i = 0; i + 1 < ra.size(); ++i) {
    for (const auto & pb : rb) {
      update(project(pb, ra[i], ra[i + 1]), pb);
    }
  }
  return Point2d{0.5 * (best_a.x() + best_b.x()), 0.5 * (best_a.y() + best_b.y())};
}

// Collision point of an object: centroid of ego_footprint(t_k) ∩ object_footprint(t_k).
Point2d object_hit_point(const Polygon2d & ego_footprint, const Polygon2d & object_footprint)
{
  boost::geometry::model::multi_polygon<Polygon2d> overlap;
  boost::geometry::intersection(ego_footprint, object_footprint, overlap);
  if (!overlap.empty() && boost::geometry::area(overlap) > 1e-9) {
    Point2d centroid;
    boost::geometry::centroid(overlap, centroid);
    return centroid;
  }
  return closest_boundary_midpoint(ego_footprint, object_footprint);
}

Polygon2d footprint_polygon(const FootprintTrajectory & footprints, const size_t index)
{
  return std::visit(
    [index](const auto & polygons) { return geometry::to_polygon2d(polygons.at(index)); },
    footprints);
}

Polygon2d point_polygon(const Point2d & point)
{
  Polygon2d polygon;
  polygon.outer().push_back(point);
  polygon.outer().push_back(point);
  return polygon;
}

SweepCandidate make_point_candidate(
  const EgoSweep & ego, const builtin_interfaces::msg::Time & source_stamp,
  const Point2d & point_now, const size_t k, const Point2d & hit_point)
{
  SweepCandidate candidate;
  candidate.kind = SweepCandidate::Kind::POINTCLOUD;
  candidate.label = 0;
  candidate.source_stamp = source_stamp;
  candidate.polygon_now = point_polygon(point_now);
  candidate.distance_now = boost::geometry::distance(ego.now(), point_now);
  candidate.ttc = ego.times[k];
  candidate.cos_collision = compute_cos_collision(ego.poses[k], ego.forward, hit_point);
  return candidate;
}

struct LocalSweep
{
  std::vector<Eigen::Isometry2d> object_to_map;
  std::vector<Polygon2d> footprints;
  std::optional<Box2d> envelope;
};

LocalSweep make_local_sweep(const EgoSweep & ego, const ObjectMotion & motion)
{
  const size_t num_steps = ego.polygons.size();
  LocalSweep local;
  local.object_to_map.resize(num_steps);
  local.footprints.resize(num_steps);
  for (size_t k = 0; k < num_steps; ++k) {
    local.object_to_map[k] = geometry::pose_to_isometry(motion.pose_at(ego.times[k]));
    if (ego.stopped[k]) {
      continue;
    }
    const Eigen::Isometry2d map_to_object = local.object_to_map[k].inverse();
    auto & footprint = local.footprints[k];
    footprint.outer().reserve(ego.polygons[k].outer().size());
    for (const auto & vertex : ego.polygons[k].outer()) {
      footprint.outer().push_back(geometry::transform_point(map_to_object, vertex));
    }
    Box2d box;
    boost::geometry::envelope(footprint, box);
    if (local.envelope) {
      boost::geometry::expand(*local.envelope, box);
    } else {
      local.envelope = box;
    }
  }
  return local;
}

std::optional<size_t> first_hit_step(
  const Point2d & point, const std::vector<Polygon2d> & footprints,
  const std::vector<bool> & stopped, const Box2d & envelope)
{
  if (!boost::geometry::covered_by(point, envelope)) {
    return std::nullopt;
  }
  for (size_t k = 0; k < footprints.size(); ++k) {
    if (!stopped[k] && boost::geometry::within(point, footprints[k])) {
      return k;
    }
  }
  return std::nullopt;
}

template <typename CollidesWhenBraking>
uint8_t brake_level_of(const CollidesWhenBraking & collides_when_braking)
{
  uint8_t level = 0;
  for (size_t i = 1; i <= BrakedSweeps::kNumLevels; ++i) {
    if (!collides_when_braking(i)) {
      break;
    }
    level = static_cast<uint8_t>(i);
  }
  return level;
}
}  // namespace

BrakedSweeps::BrakedSweeps(
  const CollisionSweep & sweep, const autoware_planning_msgs::msg::Trajectory & predicted,
  const nav_msgs::msg::Odometry & odom, const SweepLimits & limits)
: sweep_(sweep), predicted_(predicted), odom_(odom), limits_(limits)
{
}

const EgoSweep & BrakedSweeps::at(const size_t level) const
{
  auto & sweep = sweeps_.at(level - 1);
  if (!sweep) {
    sweep.emplace(sweep_.build_ego_sweep(
      predicted_, odom_, limits_,
      BrakingParam{limits_.reaction_times.at(level - 1), limits_.braking_deceleration}));
  }
  return *sweep;
}

EgoSweep::EgoSweep(
  TimeTrajectory times_in, PoseTrajectory poses_in, const FootprintTrajectory & footprints,
  std::vector<bool> stopped_in, const bool forward_in)
: times(std::move(times_in)),
  poses(std::move(poses_in)),
  stopped(std::move(stopped_in)),
  trajectory(times, poses, footprints),
  forward(forward_in)
{
  polygons.reserve(times.size());
  for (size_t k = 0; k < times.size(); ++k) {
    polygons.push_back(footprint_polygon(footprints, k));
  }
  envelope = trajectory.get_or_compute_overall_envelope();
}

ObjectMotion::ObjectMotion(
  TrajectoryData trajectory, const bool is_static, const geometry_msgs::msg::Pose & initial_pose)
: trajectory_(std::move(trajectory)), is_static_(is_static), initial_pose_(initial_pose)
{
}

geometry_msgs::msg::Pose ObjectMotion::pose_at(const double t) const
{
  if (is_static_) {
    return initial_pose_;
  }
  const auto & times = trajectory_.getTimes();
  const auto & poses = trajectory_.getPoses();
  if (times.size() == 1) {
    return poses.front();
  }
  if (t <= times.front()) {
    const double span = times[1] - times[0];
    if (span <= 0.0) {
      return poses.front();
    }
    const double r = (t - times[0]) / span;
    auto pose = poses[0];
    pose.position.x += r * (poses[1].position.x - poses[0].position.x);
    pose.position.y += r * (poses[1].position.y - poses[0].position.y);
    pose.position.z += r * (poses[1].position.z - poses[0].position.z);
    return pose;
  }
  if (t >= times.back()) {
    return poses.back();
  }
  const auto upper = static_cast<size_t>(
    std::distance(times.begin(), std::upper_bound(times.begin(), times.end(), t)));
  const size_t lower = upper - 1;
  const double span = times[upper] - times[lower];
  const double r = span > 0.0 ? (t - times[lower]) / span : 0.0;
  const auto & p0 = poses[lower];
  const auto & p1 = poses[upper];
  geometry_msgs::msg::Pose pose;
  pose.position.x = p0.position.x + r * (p1.position.x - p0.position.x);
  pose.position.y = p0.position.y + r * (p1.position.y - p0.position.y);
  pose.position.z = p0.position.z + r * (p1.position.z - p0.position.z);
  const double yaw0 = autoware_utils_geometry::get_rpy(p0).z;
  const double yaw1 = autoware_utils_geometry::get_rpy(p1).z;
  const double diff = std::atan2(std::sin(yaw1 - yaw0), std::cos(yaw1 - yaw0));
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw0 + r * diff);
  return pose;
}

ObjectTarget::ObjectTarget(
  SweepCandidate candidate, std::shared_ptr<const ObjectMotion> motion,
  const double time_resolution)
: candidate_(std::move(candidate)), motion_(std::move(motion)), time_resolution_(time_resolution)
{
}

void ObjectTarget::evaluate(
  const EgoSweep & ego, BrakedSweeps & braked, std::vector<SweepCandidate> & out) const
{
  auto candidate = candidate_;
  const auto & object_trajectory = motion_->trajectory();
  const auto timing = collision_timing_assessment::find_collision_timing(
    ego.trajectory, object_trajectory, ego.stopped);
  if (timing) {
    candidate.ttc = timing->ttc;
    const auto k = static_cast<size_t>(std::llround(timing->ttc / time_resolution_));
    const size_t k_ego = std::min(k, ego.times.size() - 1);
    const auto object_range =
      object_trajectory.resolve_covering_index_range({ego.times[k_ego], ego.times[k_ego]});
    const Polygon2d object_at_hit =
      object_range ? object_trajectory.get_or_compute_convex(*object_range) : candidate.polygon_now;
    const auto hit = object_hit_point(ego.polygons[k_ego], object_at_hit);
    candidate.cos_collision = compute_cos_collision(ego.poses[k_ego], ego.forward, hit);
    candidate.brake_level = brake_level_of([&](const size_t level) {
      const auto & braked_ego = braked.at(level);
      return collision_timing_assessment::find_collision_timing(
               braked_ego.trajectory, object_trajectory, braked_ego.stopped)
        .has_value();
    });
  }
  out.push_back(std::move(candidate));
}

PointClusterTarget::PointClusterTarget(
  std::vector<Point2d> points, std::shared_ptr<const ObjectMotion> motion,
  const builtin_interfaces::msg::Time & source_stamp)
: points_(std::move(points)), motion_(std::move(motion)), source_stamp_(source_stamp)
{
}

void PointClusterTarget::evaluate(
  const EgoSweep & ego, BrakedSweeps & braked, std::vector<SweepCandidate> & out) const
{
  if (!motion_ || motion_->is_static()) {
    evaluate_static(ego, braked, out);
  } else {
    evaluate_attached(ego, braked, out);
  }
}

void PointClusterTarget::evaluate_static(
  const EgoSweep & ego, BrakedSweeps & braked, std::vector<SweepCandidate> & out) const
{
  for (const auto & point : points_) {
    const auto k = first_hit_step(point, ego.polygons, ego.stopped, ego.envelope);
    if (!k) {
      continue;
    }
    auto candidate = make_point_candidate(ego, source_stamp_, point, *k, point);
    candidate.brake_level = brake_level_of([&](const size_t level) {
      const auto & braked_ego = braked.at(level);
      return first_hit_step(point, braked_ego.polygons, braked_ego.stopped, braked_ego.envelope)
        .has_value();
    });
    out.push_back(std::move(candidate));
  }
}

void PointClusterTarget::evaluate_attached(
  const EgoSweep & ego, BrakedSweeps & braked, std::vector<SweepCandidate> & out) const
{
  const auto local = make_local_sweep(ego, *motion_);
  if (!local.envelope) {
    return;
  }
  std::array<std::optional<LocalSweep>, BrakedSweeps::kNumLevels> braked_local;
  const auto braked_local_at = [&](const size_t level) -> const LocalSweep & {
    auto & sweep = braked_local.at(level - 1);
    if (!sweep) {
      sweep = make_local_sweep(braked.at(level), *motion_);
    }
    return *sweep;
  };

  const auto object_to_map_now = geometry::pose_to_isometry(motion_->pose_at(ego.times.front()));
  for (const auto & point : points_) {
    const auto k = first_hit_step(point, local.footprints, ego.stopped, *local.envelope);
    if (!k) {
      continue;
    }
    const auto point_now = geometry::transform_point(object_to_map_now, point);
    const auto hit_point = geometry::transform_point(local.object_to_map[*k], point);
    auto candidate = make_point_candidate(ego, source_stamp_, point_now, *k, hit_point);
    candidate.brake_level = brake_level_of([&](const size_t level) {
      const auto & braked_ego = braked.at(level);
      const auto & braked_sweep = braked_local_at(level);
      return braked_sweep.envelope &&
             first_hit_step(
               point, braked_sweep.footprints, braked_ego.stopped, *braked_sweep.envelope)
               .has_value();
    });
    out.push_back(std::move(candidate));
  }
}
}  // namespace autoware::control_validator::hazard
