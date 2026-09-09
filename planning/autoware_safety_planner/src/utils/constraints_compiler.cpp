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

#include "constraints_compiler.hpp"

#include <boost/geometry/algorithms/covered_by.hpp>

#include <algorithm>
#include <cmath>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

//! [m] Step at which the centerline is turned into a polyline and a region is scanned. It sets the
//! error the projected views may carry, which the header allows up to the grid of the DP.
constexpr double CENTERLINE_SAMPLE_INTERVAL_M = 0.5;

//! [m] Conservative padding for what the polyline approximation of the centerline loses. Added
//! only to the views carrying an arc length box: the occupancies and the ScalarBound with a
//! region.
constexpr double PROJECTION_PAD_M = 0.5 * CENTERLINE_SAMPLE_INTERVAL_M;

//! [m] arc lengths within this are the same
constexpr double S_EPS = 1e-6;

double cross2d(const double ax, const double ay, const double bx, const double by)
{
  return ax * by - ay * bx;
}

//! The centerline as a polyline. Projecting onto it keeps the number of Trajectory::compute calls
//! down.
class Centerline
{
public:
  explicit Centerline(const PathPointTrajectory & path)
  {
    const double length = path.length();
    if (!(length > 0.0)) {
      return;
    }
    const auto n = static_cast<std::size_t>(std::ceil(length / CENTERLINE_SAMPLE_INTERVAL_M));
    s_.reserve(n + 1);
    points_.reserve(n + 1);
    for (std::size_t i = 0; i <= n; ++i) {
      const double s = std::min(static_cast<double>(i) * CENTERLINE_SAMPLE_INTERVAL_M, length);
      if (!s_.empty() && s - s_.back() < S_EPS) {
        continue;
      }
      const auto p = path.compute(s).point.pose.position;
      s_.push_back(s);
      points_.emplace_back(p.x, p.y);
    }
    length_ = length;
    curvature_ = path.curvature(s_);
  }

  bool valid() const { return points_.size() >= 2; }
  double length() const { return length_; }
  const std::vector<Point2d> & points() const { return points_; }
  const std::vector<double> & arc_lengths() const { return s_; }

  //! Projects a world point to (s, l), returning nullopt for a point that falls outside the
  //! longitudinal range of the centerline, and for one outside the radius of curvature
  //! (|l*k| >= 1), where the projection is ambiguous.
  std::optional<SlPoint> project(const Point2d & q) const
  {
    if (!valid()) {
      return std::nullopt;
    }

    double best_dist2 = INF;
    std::optional<SlPoint> best;
    for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
      const double dx = points_[i + 1].x() - points_[i].x();
      const double dy = points_[i + 1].y() - points_[i].y();
      const double seg_len2 = dx * dx + dy * dy;
      if (seg_len2 < S_EPS * S_EPS) {
        continue;
      }
      const double qx = q.x() - points_[i].x();
      const double qy = q.y() - points_[i].y();
      double t = (qx * dx + qy * dy) / seg_len2;
      // Only the first and the last segment may extrapolate, so that a point just beyond an end
      // of the centerline is not lost. Inside, the parameter is clamped to [0, 1] rather than
      // rejected: at a kink the perpendicular bands of the neighboring segments do not meet and
      // leave a wedge of width |l*dtheta| on the outside of the turn. Treating a point in that
      // wedge as belonging to no segment would, for a generator whose boundary vertices sit at the
      // centerline samples (simple_drivable_area), drop the outer vertices entirely and remove the
      // lateral constraint over that s range, i.e. fail open
      const bool first = (i == 0);
      const bool last = (i + 2 == points_.size());
      if (!first) {
        t = std::max(t, 0.0);
      }
      if (!last) {
        t = std::min(t, 1.0);
      }
      const double foot_x = points_[i].x() + t * dx;
      const double foot_y = points_[i].y() + t * dy;
      const double dist2 =
        (q.x() - foot_x) * (q.x() - foot_x) + (q.y() - foot_y) * (q.y() - foot_y);
      if (dist2 >= best_dist2) {
        continue;
      }
      const double seg_len = std::sqrt(seg_len2);
      SlPoint sl;
      sl.s = s_[i] + t * (s_[i + 1] - s_[i]);
      sl.l = cross2d(dx / seg_len, dy / seg_len, qx, qy);
      best_dist2 = dist2;
      best = sl;
    }

    if (!best) {
      return std::nullopt;
    }
    // Beyond the ends there is no meaningful s
    if (best->s < -S_EPS || best->s > length_ + S_EPS) {
      return std::nullopt;
    }
    best->s = std::clamp(best->s, 0.0, length_);
    if (std::abs(best->l) * std::abs(curvature_at(best->s)) >= 1.0) {
      return std::nullopt;  // beyond the center of curvature the projection is ambiguous
    }
    return best;
  }

private:
  double curvature_at(const double s) const
  {
    if (curvature_.empty()) {
      return 0.0;
    }
    const auto it = std::lower_bound(s_.begin(), s_.end(), s);
    const auto idx = static_cast<std::size_t>(std::distance(s_.begin(), it));
    return curvature_[std::min(idx, curvature_.size() - 1)];
  }

  double length_{0.0};
  std::vector<double> s_;
  std::vector<Point2d> points_;
  std::vector<double> curvature_;
};

//! Subdivides a polyline to steps of at most CENTERLINE_SAMPLE_INTERVAL_M. Projecting only the
//! vertices of a long edge would miss part of its (s, l) range on a curve.
std::vector<Point2d> densify(const std::vector<Point2d> & points, const bool closed)
{
  std::vector<Point2d> out;
  if (points.empty()) {
    return out;
  }
  const std::size_t n = closed ? points.size() : points.size() - 1;
  for (std::size_t i = 0; i < n; ++i) {
    const Point2d & a = points[i];
    const Point2d & b = points[(i + 1) % points.size()];
    out.push_back(a);
    const double dx = b.x() - a.x();
    const double dy = b.y() - a.y();
    const double len = std::hypot(dx, dy);
    const auto division = static_cast<std::size_t>(len / CENTERLINE_SAMPLE_INTERVAL_M);
    for (std::size_t k = 1; k < division; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(division);
      out.emplace_back(a.x() + t * dx, a.y() + t * dy);
    }
  }
  if (!closed) {
    out.push_back(points.back());
  }
  return out;
}

std::vector<Point2d> ring_points(const Polygon2d & polygon)
{
  std::vector<Point2d> points;
  const auto & ring = polygon.outer();
  points.reserve(ring.size());
  for (const auto & p : ring) {
    points.push_back(p);
  }
  // A Polygon2d is closed; drop the duplicated vertex and treat it as an open polygon
  if (points.size() >= 2) {
    const auto & f = points.front();
    const auto & b = points.back();
    if (std::hypot(f.x() - b.x(), f.y() - b.y()) < S_EPS) {
      points.pop_back();
    }
  }
  return points;
}

// -----------------------------------------------------------------------------------------------
// projection, per payload
// -----------------------------------------------------------------------------------------------

//! The arc length intervals where the region meets the centerline, one per interval
std::vector<std::pair<double, double>> intersect_region(
  const Centerline & centerline, const Polygon2d & region)
{
  std::vector<std::pair<double, double>> intervals;
  const auto & s = centerline.arc_lengths();
  const auto & p = centerline.points();
  bool inside = false;
  double s_begin = 0.0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    const bool contained = boost::geometry::covered_by(p[i], region);
    if (contained && !inside) {
      inside = true;
      s_begin = s[i];
    } else if (!contained && inside) {
      inside = false;
      intervals.emplace_back(s_begin, s[i - 1]);
    }
  }
  if (inside) {
    intervals.emplace_back(s_begin, s.back());
  }
  // Give back what the scanning step cut off, by widening the intervals
  for (auto & interval : intervals) {
    interval.first = std::max(0.0, interval.first - PROJECTION_PAD_M);
    interval.second = std::min(centerline.length(), interval.second + PROJECTION_PAD_M);
  }
  return intervals;
}

bool project_scalar_bound(
  const Centerline & centerline, const ScalarBound & bound, const std::size_t raw_index,
  std::vector<ScalarBoundEntry> & out)
{
  ScalarBoundEntry entry;
  entry.quantity = bound.quantity;
  entry.min = bound.min;
  entry.max = bound.max;
  entry.raw_index = raw_index;

  if (!bound.region) {
    out.push_back(entry);  // everywhere; s0 and s1 stay infinite
    return true;
  }

  const auto intervals = intersect_region(centerline, *bound.region);
  if (intervals.empty()) {
    return false;  // a region that misses the centerline does not reach the coarse consumers
  }
  for (const auto & [s0, s1] : intervals) {
    entry.s0 = s0;
    entry.s1 = s1;
    out.push_back(entry);
  }
  return true;
}

bool project_boundary(
  const Centerline & centerline, const Boundary & boundary, const std::size_t raw_index,
  std::vector<LateralBoundEntry> & out)
{
  std::vector<Point2d> vertices;
  vertices.reserve(boundary.polyline.size());
  for (const auto & p : boundary.polyline) {
    vertices.push_back(p);
  }
  if (vertices.size() < 2) {
    return false;
  }

  // Vertices that cannot be projected (outside the centerline, or ambiguous) are dropped; the
  // boundary is still usable as a lateral envelope as long as part of it lands on s
  std::vector<SlPoint> projected;
  for (const auto & v : densify(vertices, false)) {
    if (const auto sl = centerline.project(v)) {
      projected.push_back(*sl);
    }
  }

  // Boundary carries no left/right tag, so the forbidden side is the side of the reference path the
  // polyline lies on. The sign is taken by majority: when the centerline is cut at the goal and the
  // lanelet folds back (hairpin, rotary), a few vertices of the return leg project to the other
  // side, and letting those flip the side would make the whole corridor undrivable
  std::size_t left_count = 0;
  for (const auto & sl : projected) {
    left_count += static_cast<std::size_t>(sl.l > 0.0);
  }
  const bool forbids_left = 2 * left_count >= projected.size();

  LateralBoundEntry entry;
  entry.forbidden_side = forbids_left ? Side::LEFT : Side::RIGHT;
  entry.raw_index = raw_index;
  for (const auto & sl : projected) {
    // Vertices on the other side belong to the folded-back part of the polyline, not to the
    // boundary of the corridor around the reference path
    if (forbids_left == (sl.l > 0.0)) {
      entry.polyline.push_back(sl);
    }
  }
  if (entry.polyline.size() < 2) {
    return false;
  }
  std::stable_sort(
    entry.polyline.begin(), entry.polyline.end(),
    [](const SlPoint & a, const SlPoint & b) { return a.s < b.s; });
  out.push_back(std::move(entry));
  return true;
}

bool project_gate(
  const Centerline & centerline, const Constraint & constraint, const Gate & gate,
  const std::size_t raw_index, std::vector<StopBarEntry> & out)
{
  const auto & p = centerline.points();
  const auto & s = centerline.arc_lengths();
  const Point2d & g0 = gate.line.first;
  const Point2d & g1 = gate.line.second;
  const double gx = g1.x() - g0.x();
  const double gy = g1.y() - g0.y();

  std::optional<double> s_stop;
  for (std::size_t i = 0; i + 1 < p.size(); ++i) {
    const double cx = p[i + 1].x() - p[i].x();
    const double cy = p[i + 1].y() - p[i].y();
    const double denom = cross2d(cx, cy, gx, gy);
    if (std::abs(denom) < 1e-12) {
      continue;  // parallel
    }
    const double ox = g0.x() - p[i].x();
    const double oy = g0.y() - p[i].y();
    const double t = cross2d(ox, oy, gx, gy) / denom;  // along the centerline segment
    const double u = cross2d(ox, oy, cx, cy) / denom;  // along the gate
    if (t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0) {
      continue;
    }
    const double s_cross = s[i] + t * (s[i + 1] - s[i]);
    // A gate crossing more than once takes effect at the first, nearest intersection
    if (!s_stop || s_cross < *s_stop) {
      s_stop = s_cross;
    }
  }
  if (!s_stop) {
    return false;  // a gate that misses the centerline is absent from the view
  }

  StopBarEntry entry;
  entry.s_stop = *s_stop;
  entry.time = constraint.domain.time;
  entry.raw_index = raw_index;
  out.push_back(entry);
  return true;
}

//! Flattens the occupancy of a KeepOut into a sequence of (time, occupied shape)
std::vector<std::pair<double, std::vector<Point2d>>> sample_occupancy(const KeepOut & keep_out)
{
  std::vector<std::pair<double, std::vector<Point2d>>> samples;
  if (const auto * body = std::get_if<RigidBody>(&keep_out.occupancy)) {
    const auto shape = ring_points(body->shape);
    samples.reserve(body->waypoints.size());
    for (const auto & wp : body->waypoints) {
      const double cos_yaw = std::cos(wp.pose.yaw);
      const double sin_yaw = std::sin(wp.pose.yaw);
      std::vector<Point2d> world;
      world.reserve(shape.size());
      for (const auto & v : shape) {
        world.emplace_back(
          wp.pose.position.x() + cos_yaw * v.x() - sin_yaw * v.y(),
          wp.pose.position.y() + sin_yaw * v.x() + cos_yaw * v.y());
      }
      samples.emplace_back(wp.t, std::move(world));
    }
  } else if (const auto * seq = std::get_if<TimedPolygonSequence>(&keep_out.occupancy)) {
    samples.reserve(seq->polygons.size());
    for (const auto & tp : seq->polygons) {
      samples.emplace_back(tp.t, ring_points(tp.polygon));
    }
  }
  return samples;
}

bool project_keep_out(
  const Centerline & centerline, const Constraint & constraint, const KeepOut & keep_out,
  const std::size_t raw_index, std::vector<OccupancyEntry> & out)
{
  const auto samples = sample_occupancy(keep_out);
  if (samples.empty()) {
    return false;
  }

  // A slab spans two neighboring samples, and its occupancy conservatively bounds the union of the
  // shapes at both ends. A static object, with a single sample, becomes one slab covering the whole
  // time window
  OccupancyEntry entry;
  entry.raw_index = raw_index;

  const std::size_t slab_count = std::max<std::size_t>(samples.size() - 1, 1);
  for (std::size_t i = 0; i < slab_count; ++i) {
    const bool single = samples.size() == 1;
    const double t_begin = single ? constraint.domain.time.t0 : samples[i].first;
    const double t_end = single ? constraint.domain.time.t1 : samples[i + 1].first;
    const double t0 = std::max(t_begin, constraint.domain.time.t0);
    const double t1 = std::min(t_end, constraint.domain.time.t1);
    if (t1 < t0) {
      continue;  // outside the time window
    }

    double s0 = +INF;
    double s1 = -INF;
    double l0 = +INF;
    double l1 = -INF;
    bool projectable = true;
    for (std::size_t k = i; k <= (single ? i : i + 1) && projectable; ++k) {
      for (const auto & v : densify(samples[k].second, true)) {
        const auto sl = centerline.project(v);
        if (!sl) {
          // A shape that is only partly projectable would yield a box smaller than the real
          // occupancy, which is the unsafe direction, so it is left out
          projectable = false;
          break;
        }
        s0 = std::min(s0, sl->s);
        s1 = std::max(s1, sl->s);
        l0 = std::min(l0, sl->l);
        l1 = std::max(l1, sl->l);
      }
    }
    if (!projectable || s0 > s1) {
      continue;
    }

    OccupancySlab slab;
    slab.t0 = t0;
    slab.t1 = t1;
    slab.s0 = std::max(0.0, s0 - PROJECTION_PAD_M);
    slab.s1 = std::min(centerline.length(), s1 + PROJECTION_PAD_M);
    slab.l0 = l0 - PROJECTION_PAD_M;
    slab.l1 = l1 + PROJECTION_PAD_M;
    entry.slabs.push_back(slab);
  }

  if (entry.slabs.empty()) {
    return false;
  }
  std::stable_sort(
    entry.slabs.begin(), entry.slabs.end(),
    [](const OccupancySlab & a, const OccupancySlab & b) { return a.t0 < b.t0; });
  out.push_back(std::move(entry));
  return true;
}

}  // namespace

CompiledConstraints compile_constraint_list(
  const PlannerContext & context, const std::vector<Constraint> & constraints)
{
  CompiledConstraints compiled;
  compiled.raw_constraints = constraints;

  const Centerline centerline(context.reference_path);
  if (!centerline.valid()) {
    // Without a centerline there are no projected views; raw is unaffected and still applies in
    // the exact evaluation
    compiled.unprojected.resize(constraints.size());
    for (std::size_t i = 0; i < constraints.size(); ++i) {
      compiled.unprojected[i] = i;
    }
    return compiled;
  }

  for (std::size_t i = 0; i < constraints.size(); ++i) {
    const auto & constraint = constraints[i];
    const bool projected = std::visit(
      [&](const auto & payload) {
        using Payload = std::decay_t<decltype(payload)>;
        if constexpr (std::is_same_v<Payload, ScalarBound>) {
          return project_scalar_bound(centerline, payload, i, compiled.scalar_bounds);
        } else if constexpr (std::is_same_v<Payload, Boundary>) {
          return project_boundary(centerline, payload, i, compiled.lateral_bounds);
        } else if constexpr (std::is_same_v<Payload, Gate>) {
          return project_gate(centerline, constraint, payload, i, compiled.stop_bars);
        } else {
          return project_keep_out(centerline, constraint, payload, i, compiled.occupancies);
        }
      },
      constraint.payload);
    if (!projected) {
      compiled.unprojected.push_back(i);
    }
  }

  return compiled;
}

}  // namespace autoware::safety_planner
