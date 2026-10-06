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

#ifndef AUTOWARE__MPPI_OPTIMIZER__TRAJECTORY_MPPI_OPTIMIZER_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__TRAJECTORY_MPPI_OPTIMIZER_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"

#include <autoware/avoidance_target_detector/boundary.hpp>
#include <autoware/avoidance_target_detector/object_filtering.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware/trajectory_processor/trajectory_processor_plugin_base.hpp>
#include <autoware_mppi_optimizer/trajectory_mppi_optimizer_parameters.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::mppi_optimizer::plugin
{

using TrajectoryPoints = autoware::trajectory_processor::plugin::TrajectoryPoints;

using autoware::trajectory_processor::plugin::PublisherHandle;

/// @brief Debug publisher usable from either supported node type.
class DebugPublisherAdapter
{
public:
  DebugPublisherAdapter(rclcpp::Node * node, const char * ns)
  : rclcpp_impl_(
      std::make_unique<autoware_utils_debug::BasicDebugPublisher<rclcpp::Node>>(node, ns))
  {
  }

  DebugPublisherAdapter(autoware::agnocast_wrapper::Node * node, const char * ns)
  : agnocast_impl_(
      std::make_unique<
        autoware_utils_debug::BasicDebugPublisher<autoware::agnocast_wrapper::Node>>(node, ns))
  {
  }

  template <typename T, typename... Args>
  void publish(Args &&... args)
  {
    if (rclcpp_impl_) {
      rclcpp_impl_->template publish<T>(std::forward<Args>(args)...);
    } else {
      agnocast_impl_->template publish<T>(std::forward<Args>(args)...);
    }
  }

private:
  std::unique_ptr<autoware_utils_debug::BasicDebugPublisher<rclcpp::Node>> rclcpp_impl_;
  std::unique_ptr<autoware_utils_debug::BasicDebugPublisher<autoware::agnocast_wrapper::Node>>
    agnocast_impl_;
};

/// @brief Diagnostics interface usable from either supported node type.
class DiagnosticsAdapter
{
public:
  DiagnosticsAdapter(rclcpp::Node * node, const std::string & name)
  : rclcpp_impl_(
      std::make_unique<autoware_utils_diagnostics::BasicDiagnosticsInterface<rclcpp::Node>>(
        node, name))
  {
  }

  DiagnosticsAdapter(autoware::agnocast_wrapper::Node * node, const std::string & name)
  : agnocast_impl_(
      std::make_unique<autoware_utils_diagnostics::BasicDiagnosticsInterface<
        autoware::agnocast_wrapper::Node>>(node, name))
  {
  }

  void clear()
  {
    if (rclcpp_impl_) {
      rclcpp_impl_->clear();
    } else {
      agnocast_impl_->clear();
    }
  }

  template <typename... Args>
  void add_key_value(Args &&... args)
  {
    if (rclcpp_impl_) {
      rclcpp_impl_->add_key_value(std::forward<Args>(args)...);
    } else {
      agnocast_impl_->add_key_value(std::forward<Args>(args)...);
    }
  }

  void update_level_and_message(const int8_t level, const std::string & message)
  {
    if (rclcpp_impl_) {
      rclcpp_impl_->update_level_and_message(level, message);
    } else {
      agnocast_impl_->update_level_and_message(level, message);
    }
  }

  void publish(const rclcpp::Time & publish_time_stamp)
  {
    if (rclcpp_impl_) {
      rclcpp_impl_->publish(publish_time_stamp);
    } else {
      agnocast_impl_->publish(publish_time_stamp);
    }
  }

private:
  std::unique_ptr<autoware_utils_diagnostics::BasicDiagnosticsInterface<rclcpp::Node>>
    rclcpp_impl_;
  std::unique_ptr<
    autoware_utils_diagnostics::BasicDiagnosticsInterface<autoware::agnocast_wrapper::Node>>
    agnocast_impl_;
};

/** @brief Applies first-order Dubins MPPI to the primary candidate trajectory. */
class TrajectoryMppiOptimizer final
: public autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase
{
public:
  /** @brief Optimizes candidate zero and preserves all other candidates. */
  autoware::trajectory_processor::plugin::ProcessingResult process(
    TrajectoryPoints & trajectory_points,
    autoware::trajectory_processor::TrajectoryProcessorData & data) override;

  /** @brief Accepts the common processor update hook. */
  void update_params(
    const autoware::trajectory_processor::TrajectoryProcessorParams & params) override;

  /** @brief Publishes debug trajectories from the most recent MPPI pass. */
  void publish_debug_data(const std::string & ns) const override;

protected:
  /** @brief Creates plugin parameters, publishers, and diagnostics. */
  void on_initialize(
    const autoware::trajectory_processor::TrajectoryProcessorParams & params) override;

private:
  using MppiParams = trajectory_mppi_optimizer::Params;
  using Trajectory = autoware_planning_msgs::msg::Trajectory;
  using VelocityLimit = autoware_internal_planning_msgs::msg::VelocityLimit;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using DiagnosticsInterface = autoware_utils_diagnostics::DiagnosticsInterface;

  /** @brief Recreates state after a parameter, route, or map change. */
  void reset_optimizer();

  /** @brief Updates route-dependent boundary indexes when their inputs change. */
  void update_route_context(const autoware::trajectory_processor::TrajectoryProcessorData & data);

  /** @brief Creates and configures the GPU optimizer on first use. */
  void ensure_optimizer();

  /** @brief Publishes whether MPPI replaced the primary candidate. */
  void publish_enabled(bool enabled) const;

  /** @brief Publishes the MPPI cost breakdown and result status. */
  void publish_cost_diagnostics(
    const FirstOrderDubinsMppiDebug & debug, bool was_applied, const rclcpp::Time & stamp);

  /** @brief Publishes a diagnostic for a skipped or failed MPPI pass. */
  void publish_status_diagnostic(
    std::uint8_t level, const std::string & message, const rclcpp::Time & stamp);

  /** @brief Publishes MPPI wall-time subdivisions under ~/debug/processing_time_ms/. */
  void publish_processing_time(const FirstOrderDubinsMppiTiming & timing);

  /** @brief Publishes planar distance from ego to the first DP reference point [m]. */
  void publish_ego_to_dp_first_point_distance(
    const nav_msgs::msg::Odometry & odometry, const Trajectory & reference) const;

  /** @brief Signed cross-track from ego to the raw DP polyline (+ = left); matches MPPI cost. */
  void publish_ego_signed_lateral_error_on_dp(
    const nav_msgs::msg::Odometry & odometry, const Trajectory & reference) const;

  /** @brief Publishes open-loop plant replay error vs measured ego since the previous cycle. */
  void publish_prediction_accuracy(const FirstOrderDubinsMppiPredictionAccuracy & accuracy) const;

  /** @brief Deletes stale MPPI markers. */
  void clear_markers(const std_msgs::msg::Header & header) const;

  std::unique_ptr<trajectory_mppi_optimizer::ParamListener> param_listener_;
  MppiParams params_;
  std::unique_ptr<FirstOrderDubinsMppiInterface> optimizer_;
  std::shared_ptr<autoware::avoidance_target_detector::ExtendedRouteHandler>
    extended_route_handler_;
  autoware::avoidance_target_detector::TrackedObjectSelector object_selector_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_map_msgs::msg::LaneletMapBin) current_map_;
  std::optional<unique_identifier_msgs::msg::UUID> current_route_uuid_;
  double object_filter_margin_m_{0.0};
  double object_filter_prediction_extension_s_{0.0};
  autoware::avoidance_target_detector::ExtendedRouteHandler::VelocityLimitOverrides
    map_velocity_limit_overrides_;

  typename autoware::agnocast_wrapper::polling::PollingSubscriber<VelocityLimit>::SharedPtr
    velocity_limit_sub_;

  PublisherHandle<Trajectory> reference_trajectory_pub_;
  PublisherHandle<Trajectory> nominal_control_trajectory_pub_;
  PublisherHandle<Trajectory> optimized_trajectory_pub_;
  PublisherHandle<Trajectory> nominal_trajectory_pub_;
  PublisherHandle<Trajectory> velocity_limit_trajectory_pub_;
  PublisherHandle<MarkerArray> markers_pub_;
  PublisherHandle<std_msgs::msg::Bool> enabled_pub_;
  std::unique_ptr<DebugPublisherAdapter> debug_publisher_;
  std::unique_ptr<DiagnosticsAdapter> cost_diagnostics_;

  std::optional<FirstOrderDubinsMppiDebug> pending_debug_;
  MarkerArray pending_markers_;
  std_msgs::msg::Header pending_debug_header_;
  mutable bool debug_pending_{false};
};

}  // namespace autoware::mppi_optimizer::plugin

#endif  // AUTOWARE__MPPI_OPTIMIZER__TRAJECTORY_MPPI_OPTIMIZER_HPP_
