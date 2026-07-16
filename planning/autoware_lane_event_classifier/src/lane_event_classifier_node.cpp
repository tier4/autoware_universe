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

#include "autoware_lane_event_classifier/lane_event_classifier_node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <autoware_lane_event_classifier/detail/geometry_utils.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::lane_event_classifier
{

LaneEventClassifierNode::LaneEventClassifierNode(const rclcpp::NodeOptions & node_options)
: Node("lane_event_classifier", node_options),
  param_listener_{this->get_node_parameters_interface()},
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()},
  debug_{*this}
{
  params_ = param_listener_.get_params();
  build_classifiers();

  sub_map_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    [this](const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg) {
      map_callback(msg);
    });

  pub_driving_factor_ = create_publisher<autoware_lane_event_classifier::msg::DrivingFactor>(
    "/planning/driving_factor", rclcpp::QoS{1});

  sub_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "/planning/trajectory", rclcpp::QoS{1},
    [this](const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & msg) {
      on_trajectory(msg);
    });
}

void LaneEventClassifierNode::build_classifiers()
{
  lane_following_checker_ = LaneFollowingChecker(params_.lane_following);

  // Classifiers are constructed here (no plugin/pluginlib): the node owns a vector of concrete
  // LaneEventClassifierBase implementations and iterates it in on_trajectory(). Each classifier
  // derives its own per-cycle geometry from the shared LaneTracker's generic queries. The
  // intentional-crossing classifier is still a no-op stub until its own follow-up PR.
  classifiers_.clear();
  classifiers_.emplace_back(
    std::make_unique<LaneChangeClassifier>(
      params_.lane_change.enable_classifier, params_.lane_change, lane_tracker_));
  classifiers_.emplace_back(std::make_unique<IntentionalCrossingClassifier>(true));
}

void LaneEventClassifierNode::map_callback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg)
{
  const auto map_const = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg);
  auto map = autoware::experimental::lanelet2_utils::remove_const(map_const);

  const auto result = lane_tracker_.set_lanelet_map(map);
  if (!result) {
    throw std::runtime_error(result.error());
  }
}

tl::expected<void, std::string> LaneEventClassifierNode::take_data(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory_msg)
{
  input_.trajectory_ptr = trajectory_msg;

  const auto odometry_msg = sub_odometry_.take_data();
  if (!odometry_msg) {
    return tl::make_unexpected("odometry_msg not available");
  }
  input_.odometry_ptr = odometry_msg;

  auto objects_msg = sub_objects_.take_data();
  if (!objects_msg) {
    return tl::make_unexpected("perceived objects_msg not available");
  }
  input_.objects_ptr = objects_msg;

  if (!lane_tracker_.has_lanelet_map()) {
    return tl::make_unexpected("lanelet map not yet available");
  }

  auto route_msg = sub_route_.take_data();
  if (!route_msg) {
    return tl::make_unexpected("route_msg not available");
  }

  if (route_msg->segments.empty()) {
    return tl::make_unexpected("route_msg does not contain any segments");
  }

  if (!input_.route_ptr || route_msg->uuid != input_.route_ptr->uuid) {
    input_.route_ptr = route_msg;
  }

  if (!input_.vehicle_info_ptr) {
    input_.vehicle_info_ptr =
      std::make_unique<autoware::vehicle_info_utils::VehicleInfo>(vehicle_info_);
  }

  // Turn indicator is optional: keep the previous / default value when it is unavailable rather
  // than failing the cycle (the blinker confidence signal just stays inactive).
  if (const auto turn_indicators_msg = sub_turn_indicators_.take_data()) {
    input_.turn_indicator = turn_indicators_msg->report;
  }

  input_.footprint = compute_footprint(input_);

  return {};
}

tl::expected<void, std::string> LaneEventClassifierNode::check_tracking_state()
{
  const auto & odometry = *input_.odometry_ptr;
  const auto & ego_position = odometry.pose.pose.position;
  const lanelet::BasicPoint2d ego_point{ego_position.x, ego_position.y};
  const rclcpp::Time ego_stamp{odometry.header.stamp};

  // Trigger 1 — a reposition jump (localization discontinuity). A fixed distance threshold cannot
  // tell a jump from normal driving: a small backward nudge at a standstill is a reposition, while
  // a large forward step at speed is not. So compare the measured step against the motion the
  // reported speed can explain over the elapsed cycle (speed * dt); anything beyond that plus a
  // localization-noise margin is treated as a reposition jump, independent of the vehicle's speed.
  bool ego_jumped = false;
  if (previous_ego_position_ && previous_ego_stamp_) {
    const double elapsed_s = (ego_stamp - *previous_ego_stamp_).seconds();
    if (elapsed_s > 0.0) {
      const double measured_step_m = (ego_point - *previous_ego_position_).norm();
      const auto & velocity = odometry.twist.twist.linear;
      const double ego_speed_mps = std::hypot(velocity.x, velocity.y);
      const double explainable_step_m =
        ego_speed_mps * elapsed_s + params_.reposition_jump_margin_m;
      ego_jumped = measured_step_m > explainable_step_m;
    }
  }
  previous_ego_position_ = ego_point;
  previous_ego_stamp_ = ego_stamp;
  if (ego_jumped) {
    return tl::make_unexpected("reposition jump (step exceeds reported motion)");
  }

  // Trigger 2 — while the reference lane is held (an event is active) the tracker never
  // re-anchors, so a manual takeover that drives far from the held lane would keep the reference
  // lane held forever. Reset once the ego strays past the departure threshold.
  if (lane_tracker_.is_reference_lane_held()) {
    const auto distance_to_reference =
      lane_tracker_.distance_to_lane(lane_tracker_.reference_lane().reference_lane_id, ego_point);
    if (distance_to_reference && *distance_to_reference > params_.lane_departure_reset_distance_m) {
      return tl::make_unexpected("ego departed far from the held reference lane");
    }
  }

  return {};
}

void LaneEventClassifierNode::reset_tracking_state(const std::string & tracking_reset_reason)
{
  build_classifiers();                     // restart classifiers from LANE_FOLLOWING
  lane_tracker_.release_reference_lane();  // re-anchor the reference lane to the new lane
  debug_.log_reset(tracking_reset_reason);
}

void LaneEventClassifierNode::on_trajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory_msg)
{
  if (param_listener_.is_old(params_)) {
    params_ = param_listener_.get_params();
    build_classifiers();
  }

  autoware::universe_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  autoware_lane_event_classifier::msg::DrivingFactor out;
  out.header.stamp = trajectory_msg->header.stamp;
  out.header.frame_id = "map";

  const auto lane_event_inputs_updated = take_data(trajectory_msg);
  if (!lane_event_inputs_updated) {
    debug_.log_warn(lane_event_inputs_updated.error());
    out.driving_state.state = DrivingState::UNKNOWN;
    pub_driving_factor_->publish(out);
    return;
  }

  if (const auto is_tracking_ok = check_tracking_state(); !is_tracking_ok) {
    reset_tracking_state(is_tracking_ok.error());
  }

  stop_watch.tic("lane_tracker");
  lane_tracker_.update(input_);
  const double lane_tracker_time_ms = stop_watch.toc("lane_tracker");

  const auto & ego_position = input_.odometry_ptr->pose.pose.position;
  stop_watch.tic("lane_following");
  const auto lane_following_result = lane_following_checker_.evaluate(
    lane_tracker_.lanelet_map_ptr(), lane_tracker_.routing_graph_ptr(),
    lane_tracker_.reference_lane().reference_lane_id, {ego_position.x, ego_position.y});
  const double lane_following_time_ms = stop_watch.toc("lane_following");

  uint8_t current_state_val = DrivingState::LANE_FOLLOWING;
  bool is_any_event_active = false;
  std::vector<std::pair<std::string, double>> classifier_processing_times_ms;
  classifier_processing_times_ms.reserve(classifiers_.size());
  for (auto & classifier : classifiers_) {
    if (!classifier->is_enabled()) {
      continue;
    }
    stop_watch.tic(classifier->name());
    classifier->update(input_);
    classifier_processing_times_ms.emplace_back(
      classifier->name(), stop_watch.toc(classifier->name()));
    const uint8_t candidate_state = classifier->get_state();
    // Only a confirmed event counts; LANE_FOLLOWING or UNKNOWN from a classifier is no event.
    if (
      candidate_state == DrivingState::LANE_FOLLOWING || candidate_state == DrivingState::UNKNOWN) {
      continue;
    }
    if (!is_any_event_active) {
      current_state_val = candidate_state;  // first confirmed classifier wins (priority order)
    }
    is_any_event_active = true;
  }
  // No confirmed event: fall back to the lane-following check. A departure with no classified event
  // is UNKNOWN.
  if (!is_any_event_active && !lane_following_result.is_following) {
    current_state_val = DrivingState::UNKNOWN;
  }

  debug_.log_state(current_state_val, input_, lane_following_result, lane_tracker_, classifiers_);

  out.driving_state.state = current_state_val;
  pub_driving_factor_->publish(out);

  const double total_time_ms = stop_watch.toc();

  std::vector<std::pair<std::string, double>> section_times_ms;
  section_times_ms.reserve(classifier_processing_times_ms.size() + 2);
  section_times_ms.emplace_back("lane_tracker", lane_tracker_time_ms);
  section_times_ms.emplace_back("lane_following", lane_following_time_ms);
  section_times_ms.insert(
    section_times_ms.end(), classifier_processing_times_ms.begin(),
    classifier_processing_times_ms.end());
  debug_.publish_processing_time(trajectory_msg->header.stamp, total_time_ms, section_times_ms);
}
}  // namespace autoware::lane_event_classifier

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lane_event_classifier::LaneEventClassifierNode)
