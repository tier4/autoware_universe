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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_CONTEXT_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_CONTEXT_HPP_

#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit_clear_command.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_modifier
{

/// @brief The one external limit currently in force, shared by both velocity stages.
class ExternalVelocityLimitState
{
public:
  using Limit = autoware_internal_planning_msgs::msg::VelocityLimit;
  using ClearCommand = autoware_internal_planning_msgs::msg::VelocityLimitClearCommand;

  void update(
    const Limit::ConstSharedPtr & limit, const ClearCommand::ConstSharedPtr & clear,
    const std::optional<rclcpp::Time> & limit_time = std::nullopt,
    const std::optional<rclcpp::Time> & clear_time = std::nullopt)
  {
    // When both topics have pending samples, use DDS source time. A tie or unknown order gives
    // the clear command precedence, so a queued limit cannot immediately undo a clear.
    if (limit && clear && limit_time && clear_time && *limit_time > *clear_time) {
      apply_clear(*clear, clear_time);
      apply_limit(limit, limit_time);
    } else {
      apply_limit(limit, limit_time);
      if (clear) {
        apply_clear(*clear, clear_time);
      }
    }
  }

  [[nodiscard]] const Limit::ConstSharedPtr & active_limit() const { return active_limit_; }
  [[nodiscard]] std::size_t clear_revision() const { return clear_revision_; }

private:
  void apply_limit(
    const Limit::ConstSharedPtr & limit, const std::optional<rclcpp::Time> & source_time)
  {
    if (!limit || !std::isfinite(limit->max_velocity) || limit->max_velocity < 0.0F) {
      return;
    }
    if (source_time) {
      const auto last_clear = last_clear_times_.find(limit->sender);
      if (last_clear != last_clear_times_.end() && *source_time <= last_clear->second) {
        return;
      }
    }
    active_limit_ = limit;
    active_time_ = source_time;
  }

  void apply_clear(const ClearCommand & clear, const std::optional<rclcpp::Time> & source_time)
  {
    if (!clear.command) {
      return;
    }
    if (
      active_limit_ && active_limit_->sender == clear.sender && active_time_ && source_time &&
      *source_time < *active_time_) {
      return;
    }
    if (source_time) {
      const auto [last_clear, inserted] = last_clear_times_.try_emplace(clear.sender, *source_time);
      if (!inserted && last_clear->second < *source_time) {
        last_clear->second = *source_time;
      }
    }
    if (active_limit_ && active_limit_->sender == clear.sender) {
      active_limit_.reset();
      active_time_.reset();
      ++clear_revision_;
    }
  }

  Limit::ConstSharedPtr active_limit_{nullptr};
  std::optional<rclcpp::Time> active_time_;
  std::unordered_map<std::string, rclcpp::Time> last_clear_times_;
  std::size_t clear_revision_{0U};
};

/// @brief Stable vehicle and transform services shared by trajectory modifier plugins.
struct TrajectoryModifierContext
{
  /// @brief Construct shared services from the hosting ROS node.
  explicit TrajectoryModifierContext(rclcpp::Node * node)
  : vehicle_info(autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo()),
    tf_buffer{node->get_clock()},
    tf_listener{tf_buffer},
    external_limit_sub{node, "~/input/external_velocity_limit_mps", rclcpp::QoS{1}},
    external_limit_clear_sub{
      node, "~/input/velocity_limit_clear_command", rclcpp::QoS{10}.transient_local()}
  {
  }

  void refresh_external_velocity_limit()
  {
    const auto limit = external_limit_sub.take_data();
    auto limit_time = external_limit_sub.last_taken_data_timestamp();
    if (limit_time && limit_time->nanoseconds() <= 0) {
      limit_time.reset();
    }

    struct TimedClear
    {
      ExternalVelocityLimitState::ClearCommand::ConstSharedPtr command;
      std::optional<rclcpp::Time> source_time;
    };
    std::vector<TimedClear> clears;
    rclcpp::MessageInfo message_info;
    for (;;) {
      auto command = std::make_shared<ExternalVelocityLimitState::ClearCommand>();
      if (!external_limit_clear_sub.subscriber()->take(*command, message_info)) {
        break;
      }
      const rclcpp::Time source_time{
        message_info.get_rmw_message_info().source_timestamp, RCL_ROS_TIME};
      clears.push_back(
        {command,
         source_time.nanoseconds() > 0 ? std::optional<rclcpp::Time>{source_time} : std::nullopt});
    }
    // Preserve clear commands from multiple senders, and order them around the latest limit.
    std::stable_sort(clears.begin(), clears.end(), [](const auto & lhs, const auto & rhs) {
      if (!lhs.source_time) {
        return false;
      }
      if (!rhs.source_time) {
        return true;
      }
      return *lhs.source_time < *rhs.source_time;
    });
    std::size_t first_after_limit = 0U;
    if (limit) {
      while (first_after_limit < clears.size() && limit_time &&
             clears[first_after_limit].source_time &&
             *clears[first_after_limit].source_time < *limit_time) {
        external_velocity_limit.update(
          nullptr, clears[first_after_limit].command, std::nullopt,
          clears[first_after_limit].source_time);
        ++first_after_limit;
      }
      external_velocity_limit.update(limit, nullptr, limit_time);
    }
    for (std::size_t i = first_after_limit; i < clears.size(); ++i) {
      external_velocity_limit.update(
        nullptr, clears[i].command, std::nullopt, clears[i].source_time);
    }
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  ExternalVelocityLimitState external_velocity_limit;
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    ExternalVelocityLimitState::Limit, autoware_utils_rclcpp::polling_policy::Newest>
    external_limit_sub;
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    ExternalVelocityLimitState::ClearCommand, autoware_utils_rclcpp::polling_policy::All>
    external_limit_clear_sub;
};

}  // namespace autoware::trajectory_modifier

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_CONTEXT_HPP_
