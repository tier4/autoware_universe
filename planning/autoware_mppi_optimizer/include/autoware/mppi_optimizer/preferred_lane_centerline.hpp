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

#ifndef AUTOWARE__MPPI_OPTIMIZER__PREFERRED_LANE_CENTERLINE_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__PREFERRED_LANE_CENTERLINE_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::mppi_optimizer
{
/** Device-free route selection, also used by synthetic route tests. */
class PreferredLaneCenterlineSelector
{
public:
  struct Point
  {
    double x{}, y{}, z{};
  };
  struct Lane
  {
    std::vector<Point> centerline;
    std::vector<Point> polygon;
  };
  struct Section
  {
    std::vector<Lane> lanes;  // All route primitives, for ego association in nonpreferred lanes.
    std::vector<Point> preferred;
  };

  void reset(std::vector<Section> sections)
  {
    sections_ = std::move(sections);
    previous_section_.reset();
    ++revision_;
  }

  PreferredLaneCenterlineInput select(
    const Point & ego, double yaw, double forward_distance, double backward_distance = 10.0)
  {
    PreferredLaneCenterlineInput result;
    result.revision = revision_;
    if (sections_.empty()) return result;
    if (
      !finite(ego) || !std::isfinite(yaw) || !std::isfinite(forward_distance) ||
      forward_distance < 0 || !std::isfinite(backward_distance) || backward_distance < 0) {
      result.status = "invalid_geometry";
      return result;
    }
    std::vector<std::size_t> candidates;
    for (std::size_t i = 0; i < sections_.size(); ++i) {
      for (const auto & lane : sections_[i].lanes) {
        if (!inside(ego, lane.polygon)) continue;
        const auto projection = project(ego, lane.centerline);
        if (
          projection.valid && std::abs(projection.z - ego.z) <= 2.0 &&
          projection.dx * std::cos(yaw) + projection.dy * std::sin(yaw) > 0.0) {
          candidates.push_back(i);
          break;
        }
      }
    }
    if (candidates.empty()) {
      previous_section_.reset();
      result.status = "ego_not_associated";
      return result;
    }
    // Consecutive sections may share a boundary. Nonadjacent occurrences need prior progress.
    if (candidates.back() - candidates.front() > 1) {
      if (previous_section_) {
        const auto previous = *previous_section_;
        candidates.erase(
          std::remove_if(
            candidates.begin(), candidates.end(),
            [previous](auto i) { return i + 1 < previous || i > previous + 1; }),
          candidates.end());
      }
      if (candidates.empty() || candidates.back() - candidates.front() > 1) {
        result.status = "ambiguous_route";
        return result;
      }
    }
    const std::size_t anchor = candidates.back();
    previous_section_ = anchor;
    const auto anchor_projection = project(ego, sections_[anchor].preferred);
    if (!anchor_projection.valid) {
      result.status = "invalid_geometry";
      return result;
    }
    // Clip the route interval by arc length, retaining whole segments at its edges.
    std::size_t first = anchor;
    double lower = anchor_projection.s - backward_distance;
    while (lower < 0 && first > 0) {
      --first;
      lower += length(sections_[first].preferred);
    }
    lower = std::max(0.0, lower);
    double upper = anchor_projection.s + forward_distance;
    for (std::size_t i = first; i < anchor; ++i) upper += length(sections_[i].preferred);
    double position = 0.0;
    for (std::size_t i = first; i < sections_.size() && position <= upper; ++i) {
      const auto & points = sections_[i].preferred;
      if (points.size() < 2) {
        result.status = "invalid_geometry";
        result.segments.clear();
        return result;
      }
      for (std::size_t j = 1; j < points.size(); ++j) {
        const auto & a = points[j - 1];
        const auto & b = points[j];
        if (!finite(a) || !finite(b)) {
          result.status = "invalid_geometry";
          result.segments.clear();
          return result;
        }
        const double distance = std::hypot(b.x - a.x, b.y - a.y);
        const double end = position + distance;
        if (distance > 1.0E-6 && end >= lower && position <= upper) {
          // No artificial segment is inserted between lanelets or across preference changes.
          Segment segment{
            static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(b.x),
            static_cast<float>(b.y)};
          if (
            !std::isfinite(segment.x0) || !std::isfinite(segment.y0) ||
            !std::isfinite(segment.x1) || !std::isfinite(segment.y1) ||
            (segment.x0 == segment.x1 && segment.y0 == segment.y1)) {
            result.status = "invalid_geometry";
            result.segments.clear();
            return result;
          }
          result.segments.push_back(segment);
          if (result.segments.size() > kMaxPreferredLaneCenterSegments) {
            result.status = "overflow";
            result.segments.clear();
            return result;
          }
        }
        position = end;
      }
    }
    result.status = result.segments.empty() ? "unavailable" : "active";
    return result;
  }

private:
  struct Projection
  {
    bool valid{false};
    double s{}, z{}, dx{}, dy{};
  };
  static bool finite(const Point & p)
  {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  }
  static double length(const std::vector<Point> & points)
  {
    double value = 0;
    for (std::size_t i = 1; i < points.size(); ++i)
      value += std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    return value;
  }
  static Projection project(const Point & p, const std::vector<Point> & points)
  {
    Projection result;
    double best = std::numeric_limits<double>::infinity(), s = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
      const auto & a = points[i - 1];
      const auto & b = points[i];
      if (!finite(a) || !finite(b)) return {};
      const double dx = b.x - a.x, dy = b.y - a.y;
      const double squared = dx * dx + dy * dy;
      if (squared <= 1.0E-12) continue;
      const double t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / squared, 0.0, 1.0);
      const double d = std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
      if (d < best) {
        best = d;
        result = {true, s + t * std::sqrt(squared), a.z + t * (b.z - a.z), dx, dy};
      }
      s += std::sqrt(squared);
    }
    return result;
  }
  static bool inside(const Point & p, const std::vector<Point> & polygon)
  {
    if (polygon.size() < 3) return false;
    bool included = false;
    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
      const auto & a = polygon[j];
      const auto & b = polygon[i];
      if (!finite(a) || !finite(b)) return false;
      const double dx = b.x - a.x, dy = b.y - a.y;
      const double len = std::hypot(dx, dy);
      if (
        len > 0 && std::abs((p.x - a.x) * dy - (p.y - a.y) * dx) <= 1.0E-6 * len &&
        (p.x - a.x) * (p.x - b.x) + (p.y - a.y) * (p.y - b.y) <= 1.0E-12)
        return true;
      if ((a.y > p.y) != (b.y > p.y) && p.x < a.x + (p.y - a.y) * dx / dy) included = !included;
    }
    return included;
  }
  std::vector<Section> sections_;
  std::optional<std::size_t> previous_section_;
  std::uint64_t revision_{0};
};
}  // namespace autoware::mppi_optimizer
#endif  // AUTOWARE__MPPI_OPTIMIZER__PREFERRED_LANE_CENTERLINE_HPP_
