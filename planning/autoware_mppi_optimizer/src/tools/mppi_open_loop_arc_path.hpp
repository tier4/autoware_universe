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

#pragma once

#include "autoware/mppi_optimizer/mppi_debug_trajectory_io.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::mppi_optimizer::open_loop_sim
{

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

struct PathSample
{
  float s{0.0F};
  float x{0.0F};
  float y{0.0F};
  float yaw{0.0F};
};

/** Piecewise straight / constant-curvature path stored as a dense polyline in arc length. */
class ArcLengthPath
{
public:
  void clear() { samples_.clear(); }

  bool empty() const { return samples_.size() < 2U; }

  float totalLength() const
  {
    if (samples_.empty()) {
      return 0.0F;
    }
    return samples_.back().s;
  }

  const std::vector<PathSample> & samples() const { return samples_; }

  void appendStraight(const float length, const float integration_step)
  {
    if (length <= 0.0F) {
      return;
    }
    if (integration_step <= 0.0F) {
      throw std::runtime_error("path integration step must be positive");
    }
    ensureSeed();
    float s = samples_.back().s;
    float x = samples_.back().x;
    float y = samples_.back().y;
    const float yaw = samples_.back().yaw;
    const float cos_yaw = std::cos(yaw);
    const float sin_yaw = std::sin(yaw);
    const int steps = static_cast<int>(std::ceil(length / integration_step));
    for (int i = 1; i <= steps; ++i) {
      const float ds = (i == steps) ? (length - integration_step * static_cast<float>(steps - 1))
                                    : integration_step;
      s += ds;
      x += cos_yaw * ds;
      y += sin_yaw * ds;
      samples_.push_back(PathSample{s, x, y, yaw});
    }
  }

  /** Positive angle turns left (counter-clockwise); negative turns right. */
  void appendArc(const float radius, const float signed_angle_rad, const float integration_step)
  {
    if (std::abs(signed_angle_rad) <= 1.0e-6F) {
      return;
    }
    if (radius <= 0.0F) {
      throw std::runtime_error("turn radius must be positive");
    }
    if (integration_step <= 0.0F) {
      throw std::runtime_error("path integration step must be positive");
    }
    ensureSeed();
    const float kappa = signed_angle_rad / radius;
    const float arc_length = std::abs(signed_angle_rad) * radius;
    float s = samples_.back().s;
    float x = samples_.back().x;
    float y = samples_.back().y;
    float yaw = samples_.back().yaw;
    const int steps = static_cast<int>(std::ceil(arc_length / integration_step));
    for (int i = 1; i <= steps; ++i) {
      const float ds = (i == steps) ? (arc_length - integration_step * static_cast<float>(steps - 1))
                                    : integration_step;
      const float mid_yaw = yaw + 0.5F * kappa * ds;
      x += std::cos(mid_yaw) * ds;
      y += std::sin(mid_yaw) * ds;
      yaw += kappa * ds;
      s += ds;
      samples_.push_back(PathSample{s, x, y, yaw});
    }
  }

  PathSample sampleAtArcLength(const float s_query) const
  {
    if (samples_.empty()) {
      return {};
    }
    if (s_query <= samples_.front().s) {
      return samples_.front();
    }
    if (s_query >= samples_.back().s) {
      return samples_.back();
    }
    const auto it = std::lower_bound(
      samples_.begin(), samples_.end(), s_query,
      [](const PathSample & a, const float s) { return a.s < s; });
    if (it == samples_.begin()) {
      return *it;
    }
    const PathSample & b = *it;
    const PathSample & a = *(it - 1);
    const float denom = b.s - a.s;
    const float t = (denom > 1.0e-6F) ? (s_query - a.s) / denom : 0.0F;
    PathSample out;
    out.s = s_query;
    out.x = a.x + t * (b.x - a.x);
    out.y = a.y + t * (b.y - a.y);
    out.yaw = a.yaw + t * (b.yaw - a.yaw);
    return out;
  }

  TrajectoryPoint trajectoryPointAt(const float s, const float speed) const
  {
    const PathSample p = sampleAtArcLength(s);
    TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(p.x);
    point.pose.position.y = static_cast<double>(p.y);
    point.pose.orientation = quaternionFromYaw(static_cast<double>(p.yaw));
    point.longitudinal_velocity_mps = speed;
    const int nanos = static_cast<int>(std::lround(static_cast<double>(p.s / speed) * 1.0e9));
    point.time_from_start.sec = nanos / 1000000000;
    point.time_from_start.nanosec = static_cast<std::uint32_t>(nanos % 1000000000);
    return point;
  }

  Trajectory discretizeReference(
    const float extra_margin_s, const float ds, const float speed) const
  {
    if (ds <= 0.0F) {
      throw std::runtime_error("reference spacing must be positive");
    }
    const float path_length = totalLength() + std::max(0.0F, extra_margin_s);
    const std::size_t point_count =
      static_cast<std::size_t>(std::ceil(static_cast<double>(path_length) / static_cast<double>(ds))) +
      1U;
    Trajectory trajectory;
    trajectory.header.frame_id = "map";
    trajectory.points.reserve(point_count);
    for (std::size_t i = 0; i < point_count; ++i) {
      const float s = static_cast<float>(i) * ds;
      trajectory.points.push_back(trajectoryPointAt(s, speed));
    }
    return trajectory;
  }

  Trajectory sliceHorizon(
    const float s_ego, const float ds, const std::size_t count, const float speed) const
  {
    Trajectory trajectory;
    trajectory.header.frame_id = "map";
    if (empty() || count == 0U || ds <= 0.0F) {
      return trajectory;
    }
    const float s_max = totalLength();
    trajectory.points.reserve(count);
    for (std::size_t k = 0; k < count; ++k) {
      const float s = std::clamp(s_ego + static_cast<float>(k) * ds, 0.0F, s_max);
      trajectory.points.push_back(trajectoryPointAt(s, speed));
    }
    return trajectory;
  }

  /** Closest point on the polyline; signed lateral error is positive when ego is left of the path. */
  void projectEgo(
    const float x, const float y, float & s_out, float & signed_lateral_m,
    float & heading_error_rad) const
  {
    if (samples_.size() < 2U) {
      s_out = 0.0F;
      signed_lateral_m = 0.0F;
      heading_error_rad = 0.0F;
      return;
    }
    float best_dist_sq = std::numeric_limits<float>::max();
    PathSample best = samples_.front();
    float best_lateral = 0.0F;

    for (std::size_t i = 0; i + 1U < samples_.size(); ++i) {
      const PathSample & a = samples_[i];
      const PathSample & b = samples_[i + 1U];
      const float vx = b.x - a.x;
      const float vy = b.y - a.y;
      const float seg_len_sq = vx * vx + vy * vy;
      float t = 0.0F;
      if (seg_len_sq > 1.0e-8F) {
        t = std::clamp(((x - a.x) * vx + (y - a.y) * vy) / seg_len_sq, 0.0F, 1.0F);
      }
      const float px = a.x + t * vx;
      const float py = a.y + t * vy;
      const float dx = x - px;
      const float dy = y - py;
      const float dist_sq = dx * dx + dy * dy;
      if (dist_sq < best_dist_sq) {
        best_dist_sq = dist_sq;
        best.s = a.s + t * (b.s - a.s);
        best.x = px;
        best.y = py;
        best.yaw = a.yaw + t * (b.yaw - a.yaw);
        const float nx = -std::sin(best.yaw);
        const float ny = std::cos(best.yaw);
        best_lateral = dx * nx + dy * ny;
      }
    }
    s_out = best.s;
    signed_lateral_m = best_lateral;
    heading_error_rad = 0.0F;  // filled by caller with ego yaw
    (void)heading_error_rad;
  }

private:
  void ensureSeed()
  {
    if (samples_.empty()) {
      samples_.push_back(PathSample{0.0F, 0.0F, 0.0F, 0.0F});
    }
  }

  std::vector<PathSample> samples_;
};

/** Straight, left arc, mid straight, right arc, tail straight (fixed world frame). */
inline ArcLengthPath makeLeftThenRightTurnPath(
  const float straight_before_m, const float turn_radius_m, const float turn_angle_rad,
  const float straight_mid_m, const float straight_after_m, const float integration_step_m)
{
  ArcLengthPath path;
  path.appendStraight(straight_before_m, integration_step_m);
  path.appendArc(turn_radius_m, turn_angle_rad, integration_step_m);
  path.appendStraight(straight_mid_m, integration_step_m);
  path.appendArc(turn_radius_m, -turn_angle_rad, integration_step_m);
  path.appendStraight(straight_after_m, integration_step_m);
  return path;
}

/** Matches `FirstOrderDubinsBicycleCost::kMaxRoadBorderSegments` (left + right polylines share this cap). */
inline constexpr std::size_t kOpenLoopMaxRoadBorderSegments = 256U;

/** Centerline-to-border distance that fits ego half-width + MPPI road-border collision margin. */
inline float minRoadBorderHalfWidthForEgo(
  const float ego_width, const float road_border_collision_margin)
{
  return 0.5F * ego_width + road_border_collision_margin + 0.15F;
}

/** Arc-length spans of the left / right turn sections (matches `makeLeftThenRightTurnPath`). */
struct LrTurnPathLayout
{
  float s_left_arc_start{0.0F};
  float s_left_arc_end{0.0F};
  float s_right_arc_start{0.0F};
  float s_right_arc_end{0.0F};
};

inline LrTurnPathLayout lrTurnPathLayout(
  const float straight_before_m, const float turn_radius_m, const float turn_angle_rad,
  const float straight_mid_m)
{
  const float arc_len = std::abs(turn_angle_rad) * turn_radius_m;
  LrTurnPathLayout layout;
  layout.s_left_arc_start = straight_before_m;
  layout.s_left_arc_end = straight_before_m + arc_len;
  layout.s_right_arc_start = layout.s_left_arc_end + straight_mid_m;
  layout.s_right_arc_end = layout.s_right_arc_start + arc_len;
  return layout;
}

/** Half-widths (+left / +right from centerline). Left turn: narrow the right (outer) edge. */
inline std::pair<float, float> lrTurnRoadBorderHalfWidthsAtS(
  const float s, const LrTurnPathLayout & layout, const float straight_half_width_m,
  const float turn_outer_half_width_m)
{
  if (s >= layout.s_left_arc_start && s <= layout.s_left_arc_end) {
    return {straight_half_width_m, turn_outer_half_width_m};
  }
  if (s >= layout.s_right_arc_start && s <= layout.s_right_arc_end) {
    return {turn_outer_half_width_m, straight_half_width_m};
  }
  return {straight_half_width_m, straight_half_width_m};
}

inline std::vector<Segment> makeRoadBorderPolylines(
  const ArcLengthPath & path, const float sample_step_m,
  const std::function<std::pair<float, float>(float s)> & half_widths_at_s)
{
  if (path.empty() || sample_step_m <= 0.0F) {
    return {};
  }
  const float total = path.totalLength();
  const std::size_t max_intervals_per_edge =
    std::max<std::size_t>(1U, kOpenLoopMaxRoadBorderSegments / 2U);
  const float min_step_for_cap = total / static_cast<float>(max_intervals_per_edge);
  const float step = std::max(sample_step_m, min_step_for_cap);
  std::vector<std::array<float, 2>> left;
  std::vector<std::array<float, 2>> right;
  for (float s = 0.0F; s <= total + 1.0e-4F; s += step) {
    const float s_clamped = std::min(s, total);
    const auto on = path.sampleAtArcLength(s_clamped);
    const auto [half_left, half_right] = half_widths_at_s(s_clamped);
    const float nx = -std::sin(on.yaw);
    const float ny = std::cos(on.yaw);
    left.push_back({on.x + half_left * nx, on.y + half_left * ny});
    right.push_back({on.x - half_right * nx, on.y - half_right * ny});
  }
  std::vector<Segment> segments;
  if (left.size() < 2U) {
    return segments;
  }
  segments.reserve(2U * (left.size() - 1U));
  for (std::size_t i = 0; i + 1U < left.size(); ++i) {
    segments.push_back(
      Segment{left[i][0], left[i][1], left[i + 1U][0], left[i + 1U][1]});
    segments.push_back(
      Segment{right[i][0], right[i][1], right[i + 1U][0], right[i + 1U][1]});
  }
  return segments;
}

/** Uniform corridor (both sides). */
inline std::vector<Segment> makeParallelRoadBorders(
  const ArcLengthPath & path, const float half_width_m, const float sample_step_m)
{
  if (half_width_m <= 0.0F) {
    return {};
  }
  return makeRoadBorderPolylines(
    path, sample_step_m,
    [half_width_m](float) { return std::pair<float, float>{half_width_m, half_width_m}; });
}

/** LR-turn demo: tighter outer lane line on each arc so MPPI must respect road borders in turns. */
inline std::vector<Segment> makeLrTurnRoadBorders(
  const ArcLengthPath & path, const LrTurnPathLayout & layout, const float straight_half_width_m,
  const float turn_outer_half_width_m, const float sample_step_m)
{
  if (straight_half_width_m <= 0.0F || turn_outer_half_width_m <= 0.0F) {
    return {};
  }
  return makeRoadBorderPolylines(
    path, sample_step_m,
    [&](const float s) {
      return lrTurnRoadBorderHalfWidthsAtS(
        s, layout, straight_half_width_m, turn_outer_half_width_m);
    });
}

}  // namespace autoware::mppi_optimizer::open_loop_sim
