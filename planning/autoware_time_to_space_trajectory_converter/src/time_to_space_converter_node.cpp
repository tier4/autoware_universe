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

#include "time_to_space_converter_node.hpp"

#include "class_helper.hpp"
#include "spatial_preprocessor.hpp"

#include <tl_expected/expected.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{
TimeToSpaceConverterNode::TimeToSpaceConverterNode(const rclcpp::NodeOptions & options)
: Node("autoware_time_to_space_trajectory_converter", options)
{
  rclcpp::QoS qos{1};
  qos.transient_local();
  pub_traj_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/time_to_space_converter/output/trajectory", qos);

  params_ = init_node_param(*this);
  set_param_res_ = this->add_on_set_parameters_callback(
    [&](const auto & rclcpp_param) { return update_node_param(rclcpp_param, params_); });

  auto period_s = std::chrono::duration<double>(1.0 / params_.update_rate_hz);
  timer_ = rclcpp::create_timer(this, this->get_clock(), period_s, [this]() { on_timer(); });
}

void TimeToSpaceConverterNode::take_data()
{
  if (const auto sub_traj_msg = polling_sub_traj_.take_data()) {
    sub_traj_ptr_ = sub_traj_msg;
  }

  if (const auto sub_odom_msg = polling_sub_odom_.take_data()) {
    sub_odom_ptr_ = sub_odom_msg;
  }
}

std::optional<std::string> TimeToSpaceConverterNode::has_invalid_data() const
{
  const auto current_time = this->now();

  constexpr double timeout_traj_msg = 0.5;
  if (auto err = helper::check_trajectory_msg(sub_traj_ptr_, current_time, timeout_traj_msg)) {
    return err;
  }

  constexpr double timeout_odom_msg = 0.5;
  if (auto err = helper::check_odometry_msg(sub_odom_ptr_, current_time, timeout_odom_msg)) {
    return err;
  }

  return std::nullopt;
}

std::vector<PlannerPoint> TimeToSpaceConverterNode::get_current_trajectory_points() const
{
  return helper::convert_msg_to_planner_points(*sub_traj_ptr_);
}

void TimeToSpaceConverterNode::on_timer()
{
  take_data();

  if (const auto err = has_invalid_data(); err) {
    warn_throttle("Invalid data: %s", err->c_str());
    return;
  }

  const auto combined_points = get_current_trajectory_points();

  // 2. Combine History + Future Points

  // 3. Preprocess the UNIFIED path
  SplineData spatial_processed;
  if (
    auto err = SpatialPreprocessor::process(combined_points, spatial_processed, params_.spatial)) {
    warn_throttle("Invalid preprocessing: %s", err->c_str());
    return;
  }

  // 4. Resample (Generates smooth path & velocity profile)
  // This spline currently starts at t=0 at the beginning of history (Oldest point)
  auto resampled_spatial = helper::resample(
    spatial_processed, params_.resampling_resolution_m, params_.recompute_acceleration);

  // 5. Time Shift (Crucial for Option B)
  // We need to shift timestamps so that t=0 corresponds to the CURRENT Ego Pose.
  // Points behind ego will become negative (History), points ahead positive (Future).
  double min_dist_sq = std::numeric_limits<double>::max();
  double ego_time_offset = 0.0;
  const auto & ego_pos = sub_odom_ptr_->pose.pose.position;

  for (size_t i = 0; i < resampled_spatial.s.size(); ++i) {
    double dx = resampled_spatial.x[i] - ego_pos.x;
    double dy = resampled_spatial.y[i] - ego_pos.y;
    double d2 = dx * dx + dy * dy;

    if (d2 < min_dist_sq) {
      min_dist_sq = d2;
      ego_time_offset = resampled_spatial.t[i];
    }
  }

  // Apply shift
  for (double & t : resampled_spatial.t) {
    t -= ego_time_offset;
  }

  // 6. Update State Machine (using the final corrected spline)
  stop_state_machine_.update(resampled_spatial, *sub_odom_ptr_, this->now());

  // Inspect the state
  const auto stop_state = stop_state_machine_.get_state();
  if (stop_state.knot_index != -1) {
    double total_required_wait = resampled_spatial.wait_times[stop_state.knot_index];
    double remaining_time = std::max(0.0, total_required_wait - stop_state.accumulated_wait);

    if (!stop_state.is_completed) {
      warn_throttle(
        "Stop Point %d: Waiting... %.1fs remaining (Accumulated: %.1fs)", stop_state.knot_index,
        remaining_time, stop_state.accumulated_wait);
    } else {
      warn_throttle("Stop Point %d: Wait Complete! Ready to depart.", stop_state.knot_index);
    }
  }

  // 7. Publish
  auto traj_msg =
    helper::convert_spline_data_to_trajectory_msg(resampled_spatial, sub_traj_ptr_->header);
  if (!traj_msg.points.empty()) {
    // Overwrite the first point's orientation with the actual Odometry orientation
    traj_msg.points[0].pose.orientation = sub_odom_ptr_->pose.pose.orientation;

    // Optional: Smoothly blend the next few points if the jump is large
    // (But usually fixing point 0 is enough to get it moving)
  }
  pub_traj_->publish(traj_msg);
}
}  // namespace autoware::time_to_space_trajectory_converter

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::time_to_space_trajectory_converter::TimeToSpaceConverterNode)
