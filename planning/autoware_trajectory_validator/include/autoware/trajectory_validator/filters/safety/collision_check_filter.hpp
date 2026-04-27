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
#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/expand.hpp>
#include <boost/range/iterator_range.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
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

static constexpr double TIME_INDEX_EPSILON = 1e-3;

struct TrajectoryIdentification
{
  std::string classification;
  builtin_interfaces::msg::Time stamp{};
  unique_identifier_msgs::msg::UUID uuid{};
  std::string trajectory_type{};
  double acceleration{};

  TrajectoryIdentification() = default;
  explicit TrajectoryIdentification(std::string classification)
  : classification(std::move(classification))
  {
  }

  TrajectoryIdentification(
    const autoware_perception_msgs::msg::PredictedObject & object,
    const builtin_interfaces::msg::Time stamp, std::string trajectory_type = {},
    double acceleration = 0.0)
  : classification(autoware::object_recognition_utils::convertLabelToString(
      autoware::object_recognition_utils::getHighestProbLabel(object.classification))),
    stamp(stamp),
    uuid(object.object_id),
    trajectory_type(std::move(trajectory_type)),
    acceleration(acceleration)
  {
  }

  std::string object_id_string() const { return autoware_utils_uuid::to_hex_string(uuid); }
  std::string trajectory_id_string() const
  {
    return object_id_string() + "_" + trajectory_type + " acc: " + std::to_string(acceleration);
  }
};

namespace geometry
{
Polygon2d to_polygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape);

}  // namespace geometry

class TrajectoryData
{
private:
  TrajectoryIdentification identification_;
  TimeTrajectory times_;
  TravelDistanceTrajectory distances_;
  PoseTrajectory poses_;
  FootprintTrajectory footprints_;
  mutable std::map<IndexRange, Box2d> envelope_cache_;
  mutable std::map<IndexRange, Polygon2d> convex_cache_;

  // todo: use for loop search with hint, instead of binary search.
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
      for (const auto & pt : footprints_[i].outer()) {
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
    TrajectoryIdentification trajectory_identification, TimeTrajectory times,
    TravelDistanceTrajectory distances, PoseTrajectory poses, FootprintTrajectory footprints)
  : identification_(std::move(trajectory_identification)),
    times_(std::move(times)),
    distances_(std::move(distances)),
    poses_(std::move(poses)),
    footprints_(std::move(footprints))
  {
    if (times_.empty()) {
      throw std::invalid_argument(
        "Trajectory must not be empty classification: " + identification_.classification);
    }
    if (times_.size() != distances_.size()) {
      throw std::invalid_argument(
        "Trajectory sizes mismatch (times vs distances) classification: " +
        identification_.classification);
    }
    if (times_.size() != poses_.size()) {
      throw std::invalid_argument(
        "Trajectory sizes mismatch (times vs poses) classification: " +
        identification_.classification);
    }
    if (times_.size() != footprints_.size()) {
      throw std::invalid_argument(
        "Trajectory sizes mismatch (times vs footprints) classification: " +
        identification_.classification);
    }
  }

  TrajectoryData() = delete;

  const TrajectoryIdentification & getObjectIdentification() const { return identification_; }
  const TimeTrajectory & getTimes() const { return times_; }
  const TravelDistanceTrajectory & getDistances() const { return distances_; }
  const PoseTrajectory & getPoses() const { return poses_; }
  const FootprintTrajectory & getFootprints() const { return footprints_; }

  size_t size() const { return times_.size(); }

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

  const Box2d & get_or_compute_overall_envelope(void) const
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

namespace detail
{
template <class T, class = void>
struct has_object_class_map : std::false_type
{
};
template <class T>
struct has_object_class_map<T, std::void_t<decltype(std::declval<const T &>().object_class_map)>>
: std::true_type
{
};
}  // namespace detail



// Returns the leaf scalar (Entry::value) for a per-class map lookup with
// "base" fallback. NaN per-class entries are treated as "unset" for
// floating-point Entry::value. Accepts a per-class wrapper struct or a plain
// scalar (returned as-is). Always returns a scalar.
template <class T>
auto resolve_per_class(const T & s, const std::string & cls)
{
  if constexpr (detail::has_object_class_map<T>::value) {
    const auto & m = s.object_class_map;
    using Entry = typename std::decay_t<decltype(m)>::mapped_type;
    using ValueT = decltype(Entry{}.value);
    auto it = m.find(cls);

    auto format_map_keys = [&m]() {
      std::string out = "{";
      bool first = true;
      for (const auto & kv : m) {
        if (!first) out += ", ";
        out += kv.first;
        first = false;
      }
      return out + "}";
    };

    if constexpr (std::is_floating_point_v<ValueT>) {
      // double / float: per-class value if explicitly set (non-NaN); otherwise base.
      if (it != m.end() && !std::isnan(it->second.value)) return it->second.value;
      if (auto base_it = m.find("base"); base_it != m.end()) return base_it->second.value;
      const std::string state = (it == m.end()) ? "class key not present in map"
                                                : "class entry value is NaN (unset)";
      throw std::runtime_error(
        "resolve_per_class: cannot resolve floating-point value for class '" + cls + "' (" +
        state + "), and 'base' is also missing. Map keys: " + format_map_keys());
    } else if constexpr (std::is_same_v<ValueT, bool>) {
      // bool: per-class value only; no base fallback.
      if (it != m.end()) return it->second.value;
      throw std::runtime_error(
        "resolve_per_class: no bool entry for class '" + cls +
        "'; bool maps require an explicit per-class value. Map keys: " + format_map_keys());
    } else {
      // int / string / other non-arithmetic-floating types: per-class value as-is.
      if (it != m.end()) return it->second.value;
      throw std::runtime_error(
        "resolve_per_class: no entry for class '" + cls +
        "'. Map keys: " + format_map_keys());
    }

  } else {
    static_assert(
      std::is_arithmetic_v<T> || std::is_same_v<T, std::string>,
      "resolve_per_class scalar passthrough requires double/float/int/bool/string.");
    return s;
  }
}

class CollisionCheckFilter : public plugin::ValidatorInterface
{
public:
  CollisionCheckFilter() : ValidatorInterface("collision_check_filter") {}

  result_t is_feasible(
    const TrajectoryPoints & traj_points, const FilterContext & context) override;

  void update_parameters(const validator::Params & params) final;

private:
  validator::Params::CollisionCheck::PetCollision pet_collision_params_;
  validator::Params::CollisionCheck::Rss rss_params_;
  validator::Params::CollisionCheck::Drac drac_params_;
  validator::Params::CollisionCheck::GlobalSetting global_setting_;
  ContinuousDetectionTimes pet_continuous_times_;
  ContinuousDetectionTimes rss_continuous_times_;
  ContinuousDetectionTimes drac_continuous_times_;

  void clear_detection_times();
  void add_debug_markers(
    const rclcpp::Time & stamp, const std::string & ns, const std::string & trajectory_id,
    const PoseTrajectory & ego_trajectory, const PoseTrajectory & object_trajectory,
    const Polygon2d & ego_hull, const Polygon2d & object_hull);
  void add_error_text_marker(
    const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & ego_pose,
    const std::string & error_msg);
};

}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER_HPP_
