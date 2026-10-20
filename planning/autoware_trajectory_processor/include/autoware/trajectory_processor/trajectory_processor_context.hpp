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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_CONTEXT_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_CONTEXT_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware/agnocast_wrapper/tf2.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit_clear_command.hpp>

#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>

namespace autoware::trajectory_processor
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

/// @brief Stable vehicle and transform services shared by trajectory processor plugins.
struct TrajectoryProcessorContext
{
  using LimitSubscriber = autoware::agnocast_wrapper::polling::PollingSubscriber<
    ExternalVelocityLimitState::Limit, autoware_utils_rclcpp::polling_policy::Newest>;
  using ClearSubscriber = autoware::agnocast_wrapper::polling::PollingSubscriber<
    ExternalVelocityLimitState::ClearCommand, autoware_utils_rclcpp::polling_policy::All>;

  /// @brief Construct shared services from the hosting ROS node.
  template <typename NodeT>
  explicit TrajectoryProcessorContext(NodeT * node)
  : vehicle_info(autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo()),
    tf_buffer{node->get_clock()},
    tf_listener{tf_buffer},
    external_limit_sub{make_polling_subscriber<
      ExternalVelocityLimitState::Limit, autoware_utils_rclcpp::polling_policy::Newest>(
      node, "~/input/external_velocity_limit_mps", rclcpp::QoS{1}.transient_local())},
    external_limit_clear_sub{make_polling_subscriber<
      ExternalVelocityLimitState::ClearCommand, autoware_utils_rclcpp::polling_policy::All>(
      node, "~/input/velocity_limit_clear_command", rclcpp::QoS{10}.transient_local())}
  {
  }

  /// @brief Drain the external velocity limit and clear command subscriptions into the shared
  /// state. The agnocast-compatible polling subscribers carry no DDS source timestamp, so when a
  /// limit and a clear command are pending together the clear command takes precedence.
  void refresh_external_velocity_limit()
  {
    if (const auto limit = external_limit_sub->take_data()) {
      external_velocity_limit.update(limit, nullptr);
    }
    // Preserve clear commands from multiple senders.
    for (const auto & clear : external_limit_clear_sub->take_data()) {
      external_velocity_limit.update(nullptr, clear);
    }
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
  autoware::agnocast_wrapper::Buffer tf_buffer;
  autoware::agnocast_wrapper::TransformListener tf_listener;
  ExternalVelocityLimitState external_velocity_limit;
  LimitSubscriber::SharedPtr external_limit_sub;
  ClearSubscriber::SharedPtr external_limit_clear_sub;

private:
  /// @brief Create a polling subscriber on either an agnocast_wrapper::Node (or a node derived
  /// from it) or a plain rclcpp::Node.
  template <typename MessageT, template <typename> class PollingPolicy, typename NodeT>
  static typename autoware::agnocast_wrapper::polling::PollingSubscriber<
    MessageT, PollingPolicy>::SharedPtr
  make_polling_subscriber(NodeT * node, const std::string & topic_name, const rclcpp::QoS & qos)
  {
    if constexpr (std::is_base_of_v<autoware::agnocast_wrapper::Node, NodeT>) {
      return autoware::agnocast_wrapper::polling::create_polling_subscriber<
        MessageT, PollingPolicy>(
        static_cast<autoware::agnocast_wrapper::Node *>(node), topic_name, qos);
    } else {
      return std::make_shared<
        autoware::agnocast_wrapper::polling::ROS2PollingSubscriber<MessageT, PollingPolicy>>(
        static_cast<rclcpp::Node *>(node), topic_name, qos);
    }
  }
};

}  // namespace autoware::trajectory_processor

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_CONTEXT_HPP_
