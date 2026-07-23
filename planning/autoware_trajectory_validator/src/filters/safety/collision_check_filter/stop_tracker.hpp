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

#ifndef FILTERS__SAFETY__COLLISION_CHECK_FILTER__STOP_TRACKER_HPP_
#define FILTERS__SAFETY__COLLISION_CHECK_FILTER__STOP_TRACKER_HPP_

#include "autoware/trajectory_validator/detail/uuid_hash.hpp"
#include "parameter.hpp"

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <unordered_map>

namespace autoware::trajectory_validator::plugin::safety
{
namespace detail
{
struct StopTimeRecord
{
  rclcpp::Time stopped_since;
  rclcpp::Time last_seen;
};
}  // namespace detail

/**
 * @brief Tracks how long each currently stopped object has remained stopped.
 *
 * Only stopped objects are retained. A moving object is removed immediately, and a stopped object
 * that is no longer observed is removed after the configured history timeout. All observation
 * timestamps are stored as absolute rclcpp::Time values.
 */
class ObjectStopTracker
{
public:
  explicit ObjectStopTracker(const StopTrackingParams & params = {});

  void set_parameters(const StopTrackingParams & params);

  /** @brief Updates stop histories using PredictedObjects.header.stamp as the observation time. */
  void update(const autoware_perception_msgs::msg::PredictedObjects & objects);

  /** @brief Returns the continuous stop duration, or nullopt if the object is not tracked. */
  [[nodiscard]] std::optional<rclcpp::Duration> get_stopped_duration(
    const unique_identifier_msgs::msg::UUID & object_id) const;

private:
  using ObjectId = std::array<uint8_t, 16>;

  static constexpr std::size_t kExpectedMaxObjectCount = 1024;

  StopTrackingParams params_;
  std::unordered_map<ObjectId, detail::StopTimeRecord, UuidHash> stop_times_;
  std::optional<rclcpp::Time> last_update_time_;
};

/**
 * @brief Tracks how long the ego vehicle has continuously remained stopped.
 *
 * The tracker stores a single stop record. Odometry.header.stamp is used as the observation time,
 * and a gap longer than the configured history timeout starts a new stop history.
 */
class EgoStopTracker
{
public:
  explicit EgoStopTracker(const StopTrackingParams & params = {});

  void set_parameters(const StopTrackingParams & params);

  /** @brief Updates the ego stop history using Odometry.header.stamp as the observation time. */
  void update(const nav_msgs::msg::Odometry & odometry);

  /** @brief Returns the continuous ego stop duration, or nullopt if the ego is moving. */
  [[nodiscard]] std::optional<rclcpp::Duration> get_stopped_duration() const;

private:
  StopTrackingParams params_;
  std::optional<detail::StopTimeRecord> stop_time_;
  std::optional<rclcpp::Time> last_update_time_;
};

/**
 * @brief Stop-history trackers used during collision assessment.
 */
class StopTrackers
{
public:
  EgoStopTracker ego_;
  ObjectStopTracker object_;
  [[nodiscard]] std::optional<rclcpp::Duration> get_stopped_duration(
    const unique_identifier_msgs::msg::UUID & object_id) const;
};
}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // FILTERS__SAFETY__COLLISION_CHECK_FILTER__STOP_TRACKER_HPP_
