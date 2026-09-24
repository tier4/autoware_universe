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

#include "compiled_constraints_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

KinematicLimits collect_kinematic_limits(const CompiledConstraints & compiled_constraints)
{
  KinematicLimits limits;
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    const bool is_global = bound.s0 == -INF && bound.s1 == INF;
    if (!is_global) {
      continue;  // a bound limited to an interval (a map speed limit, ...) is read per s
    }
    // Every global bound of the IR is a hard limit; the nominal values keep their defaults. In
    // particular the hard LAT_ACCEL is not fed into a_lat_nom, which would loosen the corner
    // deceleration
    switch (bound.quantity) {
      case BoundedQuantity::VELOCITY:
        limits.v_hard = std::min(limits.v_hard, bound.max);
        break;
      case BoundedQuantity::LON_ACCEL:
        limits.a_hard_min = std::max(limits.a_hard_min, bound.min);
        limits.a_hard_max = std::min(limits.a_hard_max, bound.max);
        break;
      default:
        // LAT_ACCEL / LON_JERK / STEER_* are the NLP's job, not the views'
        break;
    }
  }
  return limits;
}

std::vector<SlPoint> make_lateral_envelope(
  const std::vector<std::vector<SlPoint>> & pieces, const Side forbidden_side)
{
  const bool left = forbidden_side == Side::LEFT;
  const auto tighter = [left](const double a, const double b) {
    return left ? std::min(a, b) : std::max(a, b);
  };

  // Every segment with its ends ordered in s; a lone vertex is a segment of zero length
  struct Segment
  {
    SlPoint a;
    SlPoint b;
  };
  std::vector<Segment> segments;
  std::vector<double> breakpoints;
  for (const auto & piece : pieces) {
    for (std::size_t i = 0; i < piece.size(); ++i) {
      breakpoints.push_back(piece[i].s);
      if (piece.size() == 1 || i + 1 < piece.size()) {
        const auto & p0 = piece[i];
        const auto & p1 = piece[std::min(i + 1, piece.size() - 1)];
        segments.push_back(p0.s <= p1.s ? Segment{p0, p1} : Segment{p1, p0});
      }
    }
  }
  std::sort(breakpoints.begin(), breakpoints.end());
  breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end()), breakpoints.end());
  std::sort(segments.begin(), segments.end(), [](const Segment & x, const Segment & y) {
    return x.a.s < y.a.s;
  });

  const auto l_at = [](const Segment & seg, const double s) {
    if (seg.b.s <= seg.a.s) {
      return seg.a.l;
    }
    const double r = std::clamp((s - seg.a.s) / (seg.b.s - seg.a.s), 0.0, 1.0);
    return seg.a.l * (1.0 - r) + seg.b.l * r;
  };

  // Sweep in s, keeping the segments that cover the current breakpoint. Few segments overlap in s
  // (one per leg of the boundary), so the active set stays small
  std::vector<SlPoint> envelope;
  std::vector<const Segment *> active;
  std::size_t next = 0;
  for (std::size_t i = 0; i < breakpoints.size(); ++i) {
    const double s = breakpoints[i];
    while (next < segments.size() && segments[next].a.s <= s) {
      active.push_back(&segments[next++]);
    }
    active.erase(
      std::remove_if(
        active.begin(), active.end(), [s](const Segment * seg) { return seg->b.s < s; }),
      active.end());

    // A zero-length or vertical segment at s counts with both of its ends
    double value = left ? INF : -INF;
    for (const auto * seg : active) {
      value = seg->b.s <= seg->a.s ? tighter(value, tighter(seg->a.l, seg->b.l))
                                   : tighter(value, l_at(*seg, s));
    }
    envelope.push_back({s, value});

    if (i + 1 == breakpoints.size()) {
      break;
    }
    // No vertex lies strictly between two breakpoints, so the segments spanning the interval are
    // straight across it. When the tightest one differs at both ends they cross inside, and the
    // crossing is a vertex of the envelope. Where nothing spans the interval (a gap between
    // pieces) the envelope is bridged linearly, as the consumers interpolate it
    const double s_next = breakpoints[i + 1];
    const Segment * at_lo = nullptr;
    const Segment * at_hi = nullptr;
    for (const auto * seg : active) {
      if (seg->b.s < s_next) {
        continue;
      }
      if (!at_lo || tighter(l_at(*seg, s), l_at(*at_lo, s)) != l_at(*at_lo, s)) {
        at_lo = seg;
      }
      if (!at_hi || tighter(l_at(*seg, s_next), l_at(*at_hi, s_next)) != l_at(*at_hi, s_next)) {
        at_hi = seg;
      }
    }
    if (at_lo == at_hi) {
      continue;
    }
    // d(s) = l_lo(s) - l_hi(s) is linear and changes sign over the interval
    const double d0 = l_at(*at_lo, s) - l_at(*at_hi, s);
    const double d1 = l_at(*at_lo, s_next) - l_at(*at_hi, s_next);
    if (d0 == d1) {
      continue;
    }
    const double s_cross = s + (s_next - s) * d0 / (d0 - d1);
    if (s_cross <= s || s_cross >= s_next) {
      continue;
    }
    double value_cross = left ? INF : -INF;
    for (const auto * seg : active) {
      if (seg->b.s >= s_next) {
        value_cross = tighter(value_cross, l_at(*seg, s_cross));
      }
    }
    envelope.push_back({s_cross, value_cross});
  }
  return envelope;
}

BoundaryProfile make_boundary_profile(
  const std::vector<const LateralBoundEntry *> & bounds, const Pose2d & frame, const double x_min,
  const double x_max, const double s_min, const double s_max, const double bin)
{
  BoundaryProfile profile;
  profile.frame = frame;
  profile.x0 = x_min;
  profile.bin = bin;
  const auto bins = static_cast<std::size_t>(std::ceil((x_max - x_min) / bin));
  profile.left.assign(bins, INF);
  profile.right.assign(bins, -INF);

  const double cos_yaw = std::cos(frame.yaw);
  const double sin_yaw = std::sin(frame.yaw);
  const auto to_local = [&](const Point2d & p) {
    const double dx = p.x() - frame.position.x();
    const double dy = p.y() - frame.position.y();
    return Point2d{cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy};
  };
  const auto in_window = [&](const ProjectedVertex & v) {
    return v.sl.s >= s_min && v.sl.s <= s_max;
  };

  for (const auto * bound : bounds) {
    const bool left = bound->forbidden_side == Side::LEFT;
    auto & target = left ? profile.left : profile.right;
    const auto tighter = [left](const double a, const double b) {
      return left ? std::min(a, b) : std::max(a, b);
    };
    for (const auto & piece : bound->pieces) {
      for (std::size_t i = 0; i < piece.size(); ++i) {
        // A lone vertex is a segment of zero length
        if (i + 1 == piece.size() && piece.size() > 1) {
          break;
        }
        const auto & v0 = piece[i];
        const auto & v1 = piece[std::min(i + 1, piece.size() - 1)];
        if (!in_window(v0) && !in_window(v1)) {
          continue;
        }
        auto a = to_local(v0.position);
        auto b = to_local(v1.position);
        if (a.x() > b.x()) {
          std::swap(a, b);
        }
        if (b.x() < x_min || a.x() > x_max) {
          continue;
        }
        // The segment is straight, so within a bin its extreme y is at an end of the part inside
        const auto y_at = [&](const double x) {
          const double dx = b.x() - a.x();
          return dx > 0.0 ? a.y() + (b.y() - a.y()) * (x - a.x()) / dx : tighter(a.y(), b.y());
        };
        const auto bin_of = [&](const double x) {
          return std::min(
            static_cast<std::size_t>(std::max(0.0, (x - x_min) / bin)), target.size() - 1);
        };
        const double xa = std::max(a.x(), x_min);
        const double xb = std::min(b.x(), x_max);
        for (std::size_t k = bin_of(xa); k <= bin_of(xb); ++k) {
          const double lo = std::max(xa, x_min + static_cast<double>(k) * bin);
          const double hi = std::min(xb, x_min + static_cast<double>(k + 1) * bin);
          target[k] = tighter(target[k], tighter(y_at(lo), y_at(hi)));
        }
      }
    }
  }
  profile.left_min = *std::min_element(profile.left.begin(), profile.left.end());
  profile.right_max = *std::max_element(profile.right.begin(), profile.right.end());
  return profile;
}

bool footprint_hits_boundary(
  const BoundaryProfile & profile, const VehicleInfo & vehicle_info, const Pose2d & rear_axle,
  const double longitudinal_margin)
{
  const double cos_frame = std::cos(profile.frame.yaw);
  const double sin_frame = std::sin(profile.frame.yaw);
  const double dx = rear_axle.position.x() - profile.frame.position.x();
  const double dy = rear_axle.position.y() - profile.frame.position.y();
  const double x = cos_frame * dx + sin_frame * dy;
  const double y = -sin_frame * dx + cos_frame * dy;
  const double cos_yaw = std::cos(rear_axle.yaw - profile.frame.yaw);
  const double sin_yaw = std::sin(rear_axle.yaw - profile.frame.yaw);

  const double front = vehicle_info.max_longitudinal_offset_m + longitudinal_margin;
  const double rear = vehicle_info.min_longitudinal_offset_m - longitudinal_margin;
  const double body[4][2] = {
    {front, vehicle_info.max_lateral_offset_m},
    {front, vehicle_info.min_lateral_offset_m},
    {rear, vehicle_info.min_lateral_offset_m},
    {rear, vehicle_info.max_lateral_offset_m}};
  std::array<Point2d, 4> corners;
  double x_lo = INF;
  double x_hi = -INF;
  double y_lo = INF;
  double y_hi = -INF;
  for (std::size_t i = 0; i < 4; ++i) {
    corners[i] = Point2d{
      x + cos_yaw * body[i][0] - sin_yaw * body[i][1],
      y + sin_yaw * body[i][0] + cos_yaw * body[i][1]};
    x_lo = std::min(x_lo, corners[i].x());
    x_hi = std::max(x_hi, corners[i].x());
    y_lo = std::min(y_lo, corners[i].y());
    y_hi = std::max(y_hi, corners[i].y());
  }
  // Clear of every bin at once, the common case away from the boundaries
  if (y_hi <= profile.left_min && y_lo >= profile.right_max) {
    return false;
  }

  const auto bins = static_cast<std::ptrdiff_t>(profile.left.size());
  const auto k_lo = std::max<std::ptrdiff_t>(
    static_cast<std::ptrdiff_t>(std::floor((x_lo - profile.x0) / profile.bin)), 0);
  const auto k_hi = std::min<std::ptrdiff_t>(
    static_cast<std::ptrdiff_t>(std::floor((x_hi - profile.x0) / profile.bin)), bins - 1);
  for (auto k = k_lo; k <= k_hi; ++k) {
    // The range of y of the rectangle within the bin: at the corners inside it and where the edges
    // cross its two sides
    const double bx0 = profile.x0 + static_cast<double>(k) * profile.bin;
    const double bx1 = bx0 + profile.bin;
    double lo = INF;
    double hi = -INF;
    for (std::size_t i = 0; i < 4; ++i) {
      const auto & p = corners[i];
      const auto & q = corners[(i + 1) % 4];
      if (p.x() >= bx0 && p.x() <= bx1) {
        lo = std::min(lo, p.y());
        hi = std::max(hi, p.y());
      }
      for (const double bx : {bx0, bx1}) {
        if ((p.x() - bx) * (q.x() - bx) < 0.0) {
          const double yc = p.y() + (q.y() - p.y()) * (bx - p.x()) / (q.x() - p.x());
          lo = std::min(lo, yc);
          hi = std::max(hi, yc);
        }
      }
    }
    const auto i = static_cast<std::size_t>(k);
    if (hi > profile.left[i] || lo < profile.right[i]) {
      return true;
    }
  }
  return false;
}

double interpolate_boundary_l(const std::vector<SlPoint> & polyline, const double s)
{
  if (s <= polyline.front().s) {
    return polyline.front().l;
  }
  if (s >= polyline.back().s) {
    return polyline.back().l;
  }
  for (std::size_t seg = 0; seg + 1 < polyline.size(); ++seg) {
    const auto & p0 = polyline[seg];
    const auto & p1 = polyline[seg + 1];
    if (s <= p1.s) {
      const double ratio = (p1.s > p0.s) ? (s - p0.s) / (p1.s - p0.s) : 0.0;
      return p0.l * (1.0 - ratio) + p1.l * ratio;
    }
  }
  return polyline.back().l;
}

bool lateral_bound_extreme_l(
  const LateralBoundEntry & bound, const double s_lo_in, const double s_hi_in, double & extreme_l)
{
  const auto & polyline = bound.polyline;
  if (polyline.size() < 2) {
    return false;
  }
  if (s_hi_in < polyline.front().s || s_lo_in > polyline.back().s) {
    return false;  // the constraint does not apply where the s ranges do not overlap
  }

  // Extremum of the boundary l over the overlap [s_lo, s_hi]. LEFT forbids everything left of the
  // boundary (larger l), so the tightest value is min(l_b); RIGHT is the other way round
  const double s_lo = std::max(s_lo_in, polyline.front().s);
  const double s_hi = std::min(s_hi_in, polyline.back().s);
  double value = interpolate_boundary_l(polyline, s_lo);
  const auto update = [&](const double l_b) {
    value = (bound.forbidden_side == Side::LEFT) ? std::min(value, l_b) : std::max(value, l_b);
  };
  update(interpolate_boundary_l(polyline, s_hi));
  for (const auto & vertex : polyline) {
    if (vertex.s > s_lo && vertex.s < s_hi) {
      update(vertex.l);
    }
  }
  extreme_l = value;
  return true;
}

bool violates_lateral_bound(const LateralBoundEntry & bound, const SlBox & box)
{
  double extreme_l = 0.0;
  if (!lateral_bound_extreme_l(bound, box.s_min, box.s_max, extreme_l)) {
    return false;
  }
  if (bound.forbidden_side == Side::LEFT) {
    return box.l_max > extreme_l;
  }
  return box.l_min < extreme_l;
}

bool violates_occupancy(
  const OccupancyEntry & occupancy, const SlBox & box, const double t0, const double t1)
{
  for (const auto & slab : occupancy.slabs) {
    const bool time_overlaps = slab.t1 >= t0 && slab.t0 <= t1;
    if (!time_overlaps) {
      continue;
    }
    const bool box_overlaps =
      slab.s1 >= box.s_min && slab.s0 <= box.s_max && slab.l1 >= box.l_min && slab.l0 <= box.l_max;
    if (box_overlaps) {
      return true;
    }
  }
  return false;
}

bool violates_stop_bar(
  const StopBarEntry & stop_bar, const SlBox & box, const double t0, const double t1)
{
  const bool time_overlaps = stop_bar.time.t1 >= t0 && stop_bar.time.t0 <= t1;
  if (!time_overlaps) {
    return false;
  }
  return box.s_max > stop_bar.s_stop;
}

}  // namespace autoware::safety_planner
