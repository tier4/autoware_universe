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

// TrajectoryData and the trajectory types are ported from autoware_trajectory_validator
// (autoware.universe 3eb3290355):
//   src/filters/safety/collision_check_filter/types.hpp
//   src/filters/safety/collision_check_filter/trajectory_utils.hpp (TrajectoryData)
// Changes from the source: namespace; TrajectoryIdentification removed. This copy does not follow
// later changes of the source.

#ifndef AUTOWARE__CONTROL_VALIDATOR__COLLISION_VALIDATOR__SWEEP_TYPES_HPP_
#define AUTOWARE__CONTROL_VALIDATOR__COLLISION_VALIDATOR__SWEEP_TYPES_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::control_validator::collision_validator
{
using autoware_utils_geometry::Box2d;
using autoware_utils_geometry::MultiPoint2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

namespace detail
{
static constexpr double TIME_INDEX_EPSILON = 1e-3;

using IndexRange = std::pair<size_t, size_t>;
using TimeRange = std::pair<double, double>;

using TimeTrajectory = std::vector<double>;
using TravelDistanceTrajectory = std::vector<double>;
using PoseTrajectory = std::vector<geometry_msgs::msg::Pose>;
using QuadTrajectory = std::vector<std::array<Point2d, 4>>;
using NgonTrajectory = std::vector<std::vector<Point2d>>;
using FootprintTrajectory = std::variant<QuadTrajectory, NgonTrajectory>;

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

// Closed polygon of one footprint sample.
Polygon2d to_polygon2d(const std::array<Point2d, 4> & quad);
Polygon2d to_polygon2d(const std::vector<Point2d> & ring);
}  // namespace detail

struct EgoSweep
{
  EgoSweep(
    detail::TimeTrajectory times_in, detail::PoseTrajectory poses_in,
    const detail::FootprintTrajectory & footprints, std::vector<bool> stopped_in, bool forward_in);

  detail::TimeTrajectory times;
  detail::PoseTrajectory poses;
  std::vector<Polygon2d> polygons;
  std::vector<bool> stopped;
  detail::TrajectoryData trajectory;
  bool forward;

  const Polygon2d & now() const { return polygons.front(); }
};
}  // namespace autoware::control_validator::collision_validator

#endif  // AUTOWARE__CONTROL_VALIDATOR__COLLISION_VALIDATOR__SWEEP_TYPES_HPP_
