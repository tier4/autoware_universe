// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER_HPP_

#include "autoware/trajectory_validator/validator_interface.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/expand.hpp>
#include <boost/range/iterator_range.hpp>

#include <algorithm>
#include <any>
#include <cassert>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
using autoware_utils_geometry::Box2d;
using autoware_utils_geometry::MultiPoint2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

using TimeTrajectory = std::vector<double>;
using TravelDistanceTrajectory = std::vector<double>;
using PoseTrajectory = std::vector<geometry_msgs::msg::Pose>;
using FootprintTrajectory = std::vector<Polygon2d>;
using StepPolygonTrajectory = std::vector<Polygon2d>;
using IndexRange = std::pair<size_t, size_t>;
using TimeRange = std::pair<double, double>;

static constexpr double TIME_RESOLUTION = 0.1;  // 100ms intervals
static constexpr double TIME_INDEX_EPSILON = 1e-3;

struct ObjectIdentification
{
  std::string classification;
  std::string id;
};

class TrajectoryData
{
private:
  const ObjectIdentification object_identification_;
  const TimeTrajectory times_;
  const TravelDistanceTrajectory distances_;
  const PoseTrajectory poses_;
  const FootprintTrajectory footprints_;
  mutable std::map<IndexRange, Box2d> envelope_cache_;
  mutable std::map<IndexRange, Polygon2d> convex_cache_;

  // obsolete
  size_t getClosestTimeIndex(const double t) const
  {
    const auto it = std::lower_bound(times_.begin(), times_.end(), t);

    if (it == times_.begin()) return 0;
    if (it == times_.end()) return times_.size() - 1;

    const auto prev_it = it - 1;
    const auto closest_it = (std::abs(*it - t) < std::abs(*prev_it - t)) ? it : prev_it;
    return std::distance(times_.begin(), closest_it);
  }

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

  IndexRange resolve_covering_index_range(const TimeRange & key_time) const
  {
    assert(key_time.first <= key_time.second);

    auto start_index = get_same_or_earlier_time_index(key_time.first);
    auto end_index = get_same_or_later_time_index(key_time.second);
    return {start_index, end_index};
  }

  Box2d compute_envelope(const IndexRange & key) const
  {
    assert(key.first <= key.second);

    Box2d box;
    boost::geometry::assign_inverse(box);
    for (size_t i = key.first; i <= key.second; ++i) {
      for (const auto & pt : footprints_[i].outer()) {
        boost::geometry::expand(box, pt);
      }
    }
    return box;
  }

  Polygon2d compute_convex(const IndexRange & key) const
  {
    assert(key.first <= key.second);

    MultiPoint2d all_points;

    all_points.reserve((key.second - key.first + 1) * 4);  // heuristic reserve
    for (size_t i = key.first; i <= key.second; ++i) {
      const auto & poly = footprints_[i];
      for (const auto & pt : poly.outer()) {
        all_points.push_back(pt);
      }
    }

    Polygon2d hull;
    hull.outer().reserve(all_points.size());
    boost::geometry::convex_hull(all_points, hull);
    return hull;
  }

public:
  TrajectoryData(
    ObjectIdentification object_identification, TimeTrajectory times,
    TravelDistanceTrajectory distances, PoseTrajectory poses, FootprintTrajectory footprints)
  : object_identification_(std::move(object_identification)),
    times_(std::move(times)),
    distances_(std::move(distances)),
    poses_(std::move(poses)),
    footprints_(std::move(footprints))
  {
    if (times_.empty()) {
      throw std::invalid_argument(
        "Trajectory must not be empty classification: " + object_identification_.classification +
        ", ID: " + object_identification_.id);
    }
    if (times_.size() != distances_.size()) {
      throw std::invalid_argument(
        "Trajectory sizes mismatch (times vs distances) classification: " +
        object_identification_.classification + ", ID: " + object_identification_.id);
    }
    if (times_.size() != poses_.size()) {
      throw std::invalid_argument(
        "Trajectory sizes mismatch (times vs poses) classification: " +
        object_identification_.classification + ", ID: " + object_identification_.id);
    }
    if (times_.size() != footprints_.size()) {
      throw std::invalid_argument(
        "Trajectory sizes mismatch (times vs footprints) classification: " +
        object_identification_.classification + ", ID: " + object_identification_.id);
    }
  }

  TrajectoryData() = delete;

  const ObjectIdentification & getObjectIdentification() const { return object_identification_; }
  const TimeTrajectory & getTimes() const { return times_; }
  const TravelDistanceTrajectory & getDistances() const { return distances_; }
  const PoseTrajectory & getPoses() const { return poses_; }
  const FootprintTrajectory & getFootprints() const { return footprints_; }

  size_t size() const { return times_.size(); }

  // obsolete
  boost::iterator_range<FootprintTrajectory::const_iterator> getFootprintsInTimeRange(
    double start_time, double end_time) const
  {
    if (start_time > end_time || footprints_.empty()) {
      return boost::make_iterator_range(footprints_.end(), footprints_.end());
    }

    const size_t start_index = getClosestTimeIndex(start_time);
    const size_t end_index = getClosestTimeIndex(end_time);

    if (start_index > end_index) {
      return boost::make_iterator_range(footprints_.end(), footprints_.end());
    }

    const auto start_iter = footprints_.begin() + start_index;
    const auto end_iter = footprints_.begin() + std::min(end_index + 1, footprints_.size());

    return boost::make_iterator_range(start_iter, end_iter);
  }

  const Box2d & get_or_compute_envelope(const IndexRange & key) const
  {
    assert(key.first <= key.second);

    auto [it, inserted] = envelope_cache_.try_emplace(key);
    if (inserted) {
      it->second = compute_envelope(key);
    }
    return it->second;
  }

  const Box2d & get_or_compute_envelope(const TimeRange & key_time) const
  {
    return get_or_compute_envelope(resolve_covering_index_range(key_time));
  }

  Box2d get_or_compute_overall_envelope(void) const
  {
    return get_or_compute_envelope(IndexRange{0U, footprints_.size() - 1});
  }

  const Polygon2d & get_or_compute_convex(const IndexRange & key) const
  {
    assert(key.first <= key.second);

    auto [it, inserted] = convex_cache_.try_emplace(key);
    if (inserted) {
      it->second = compute_convex(key);
    }
    return it->second;
  }

  const Polygon2d & get_or_compute_convex(const TimeRange & key_time) const
  {
    return get_or_compute_convex(resolve_covering_index_range(key_time));
  }
};

class ContinuousDetectionTimes
{
public:
  void clear()
  {
    current_time_.reset();
    detection_start_times_.clear();
  }

  template <typename Detections, typename KeyFunc>
  void update(const rclcpp::Time & current_time, const Detections & detections, KeyFunc key_func)
  {
    current_time_ = current_time;

    std::unordered_set<std::string> active_keys{};
    for (const auto & detection : detections) {
      const auto key = key_func(detection);
      active_keys.insert(key);
      detection_start_times_.try_emplace(key, current_time);
    }

    for (auto it = detection_start_times_.begin(); it != detection_start_times_.end();) {
      if (!active_keys.count(it->first)) {
        it = detection_start_times_.erase(it);
      } else {
        ++it;
      }
    }
  }

  double get_time(const std::string & key) const
  {
    if (!current_time_) {
      return 0.0;
    }

    const auto it = detection_start_times_.find(key);
    if (it == detection_start_times_.end()) {
      return 0.0;
    }

    return (*current_time_ - it->second).seconds();
  }

private:
  std::optional<rclcpp::Time> current_time_;
  std::unordered_map<std::string, rclcpp::Time> detection_start_times_;
};

class CollisionCheckFilter : public plugin::ValidatorInterface
{
public:
  CollisionCheckFilter() : ValidatorInterface("collision_check_filter") {}

  tl::expected<void, std::string> is_feasible(
    const TrajectoryPoints & traj_points, const FilterContext & context) override;

  void update_parameters(const validator::Params & params) final;

private:
  validator::Params::CollisionCheck::PetCollision pet_collision_params_;
  validator::Params::CollisionCheck::Rss rss_params_;
  ContinuousDetectionTimes pet_continuous_times_;
  ContinuousDetectionTimes rss_continuous_times_;
  ContinuousDetectionTimes drac_continuous_times_;

  void add_debug_markers(
    const rclcpp::Time & stamp, const std::string & ns, const Polygon2d & ego_hull,
    const Polygon2d & object_hull);
};

}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER_HPP_
