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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_EVENT_CLASSIFIER_NODE_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_EVENT_CLASSIFIER_NODE_HPP_

#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_lane_event_classifier/debug.hpp>
#include <autoware_lane_event_classifier/lane_change/classifier.hpp>
#include <autoware_lane_event_classifier/lane_crossing/classifier.hpp>
#include <autoware_lane_event_classifier/lane_event_classifier_base.hpp>
#include <autoware_lane_event_classifier/lane_event_classifier_parameters.hpp>
#include <autoware_lane_event_classifier/lane_following/checker.hpp>
#include <autoware_lane_event_classifier/lane_tracker.hpp>
#include <autoware_lane_event_classifier/msg/driving_factor.hpp>
#include <autoware_lane_event_classifier/types.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::lane_event_classifier
{

/** @brief ROS node that classifies lane events from trajectory, odometry, route, and map inputs. */
class LaneEventClassifierNode : public rclcpp::Node
{
public:
  explicit LaneEventClassifierNode(const rclcpp::NodeOptions & node_options);

private:
  void on_trajectory(
    const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory_msg);
  void map_callback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg);
  tl::expected<void, std::string> take_data(
    const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory_msg);
  void build_classifiers();

  /**
   * @brief Returns true when the tracking state can no longer be trusted and must be reset.
   *
   * Triggers on a reposition jump (localization discontinuity), or the ego straying far from a
   * held reference lane (e.g. a manual takeover that drives away from the route). Advances the
   * ego-pose bookkeeping each cycle and records the cause in tracking_reset_reason_ for
   * reset_tracking_state() to log.
   */
  bool is_tracking_state_stale();

  /**
   * @brief Resets the tracking state: restarts the classifiers from LANE_FOLLOWING and releases the
   * reference-lane hold so it re-anchors, logging the cause recorded by is_tracking_state_stale().
   */
  void reset_tracking_state();

  // Publishers
  rclcpp::Publisher<autoware_lane_event_classifier::msg::DrivingFactor>::SharedPtr
    pub_driving_factor_;

  // Parameters

  ::lane_event_classifier::ParamListener param_listener_;
  ::lane_event_classifier::Params params_;

  // Map: one-shot callback — fires once (or on rare map reload)
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;

  // Route: polling — lane IDs refreshed only on UUID change
  autoware_utils::InterProcessPollingSubscriber<
    autoware_planning_msgs::msg::LaneletRoute, autoware_utils::polling_policy::Latest>
    sub_route_{this, "/planning/mission_planning/route", rclcpp::QoS{1}.transient_local()};

  // Trajectory: callback-driven — one classification per received trajectory
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;

  // Dynamic inputs (polled on trajectory arrival)
  autoware_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> sub_odometry_{
    this, "/localization/kinematic_state", rclcpp::QoS{1}};

  autoware_utils::InterProcessPollingSubscriber<autoware_perception_msgs::msg::PredictedObjects>
    sub_objects_{this, "/perception/object_recognition/objects", rclcpp::QoS{1}};

  autoware_utils::InterProcessPollingSubscriber<autoware_vehicle_msgs::msg::TurnIndicatorsReport>
    sub_turn_indicators_{this, "/vehicle/status/turn_indicators_status"};

  // Internal state passed to classifiers each cycle
  LaneEventInput input_;
  LaneTracker lane_tracker_;

  // Lane-following gate — evaluated here (outside the tracker) and written into the context.
  LaneFollowingChecker lane_following_checker_;

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // Observability (markers, processing-time messages, logging) — owns the debug publishers.
  LaneEventClassifierDebug debug_;

  // Classifiers — instantiated in constructor based on enable_* params
  std::vector<std::unique_ptr<LaneEventClassifierBase>> classifiers_;

  // Previous ego pose and its stamp — a step between cycles that exceeds the motion the reported
  // speed can explain (speed * dt) is treated as a reposition jump and resets the tracking state.
  std::optional<lanelet::BasicPoint2d> previous_ego_position_;
  std::optional<rclcpp::Time> previous_ego_stamp_;

  // Cause of the pending reset, set by is_tracking_state_stale() and logged by
  // reset_tracking_state().
  std::string tracking_reset_reason_;
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_EVENT_CLASSIFIER_NODE_HPP_
