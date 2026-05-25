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

#include "in_lane_mrm_planner_node.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>

namespace autoware::in_lane_mrm_planner
{

InLaneMrmPlannerNode::InLaneMrmPlannerNode(const rclcpp::NodeOptions & options)
: Node("in_lane_mrm_planner", options),
  param_listener_(std::make_shared<::in_lane_mrm_planner::ParamListener>(
    get_node_parameters_interface())),
  params_(param_listener_->get_params()),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()),
  time_keeper_(std::make_shared<autoware_utils_debug::TimeKeeper>()),
  velocity_planner_(params_),
  route_subscriber_(this, "~/input/route", rclcpp::QoS{1}.transient_local()),
  vector_map_subscriber_(this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()),
  kinematic_state_subscriber_(this, "~/input/kinematic_state"),
  acceleration_subscriber_(this, "~/input/acceleration"),
  objects_subscriber_(this, "~/input/objects"),
  trigger_subscriber_(this, "~/input/trigger")
{
  path_planner_ = std::make_unique<PathPlanner>(
    get_logger(), get_clock(), time_keeper_, params_, vehicle_info_);
  trajectory_planner_ =
    std::make_unique<InLaneMrmTrajectoryPlanner>(*path_planner_, params_, vehicle_info_);

  trajectory_smoother_.initialize(this, params_);
  trajectory_modifier_.initialize(this, vehicle_info_, params_);

  pub_trajectory_ = create_publisher<Trajectory>("~/output/trajectory", 1);

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params_.planning_frequency_hz).period(),
    std::bind(&InLaneMrmPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "InLaneMrmPlannerNode started.");
}

void InLaneMrmPlannerNode::on_timer()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto input_data = take_data();
  path_planner_->set_planner_data(input_data.lanelet_map_bin_ptr, input_data.route_ptr);

  if (param_listener_->is_old(params_)) {
    update_params();
  }

  if (!is_data_ready(input_data)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for map, route, odometry, or acceleration.");
    return;
  }

  const auto & odom = *input_data.odometry_ptr;
  const auto & accel = *input_data.acceleration_ptr;
  PredictedObjects empty_objects;
  const auto & live_objects =
    input_data.objects_ptr ? *input_data.objects_ptr : empty_objects;

  const bool trigger_active = input_data.trigger_ptr && input_data.trigger_ptr->data;
  const auto trigger_edges = trigger_edge_detector_.update(trigger_active);
  if (trigger_edges.rising) {
    objects_latcher_.latch(live_objects, params_.latch.use_latched_objects);
    trajectory_latcher_.latch();
  }
  if (trigger_edges.falling) {
    objects_latcher_.unlatch();
    trajectory_latcher_.unlatch();
  }

  const auto planned_traj = trajectory_planner_->plan(odom);
  if (planned_traj) {
    auto traj = *planned_traj;
    trajectory_smoother_.smooth(traj.points, odom.pose.pose);
    trajectory_modifier_.set_objects(objects_latcher_.objects_for_planning(live_objects));
    trajectory_modifier_.apply(traj.points, odom, accel);
    velocity_planner_.apply(traj.points, odom, accel);
    trajectory_modifier_.publish_planning_factor();

    traj.header.stamp = now();
    traj.header.frame_id = odom.header.frame_id;
    trajectory_latcher_.update_candidate(trajectory_selector_.select(traj));
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Trajectory planning failed.");
  }

  if (const auto output = trajectory_latcher_.output()) {
    auto published = *output;
    published.header.stamp = now();
    if (published.header.frame_id.empty()) {
      published.header.frame_id = odom.header.frame_id;
    }
    pub_trajectory_->publish(published);
  }
}

InLaneMrmPlannerNode::InputData InLaneMrmPlannerNode::take_data()
{
  InputData input_data;
  input_data.route_ptr = route_ptr_;
  input_data.lanelet_map_bin_ptr = lanelet_map_bin_ptr_;
  input_data.odometry_ptr = odometry_ptr_;
  input_data.acceleration_ptr = acceleration_ptr_;
  input_data.objects_ptr = objects_ptr_;
  input_data.trigger_ptr = trigger_ptr_;

  if (const auto msg = route_subscriber_.take_data()) {
    if (!msg->segments.empty()) {
      route_ptr_ = msg;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Received empty route; ignoring.");
    }
  }
  input_data.route_ptr = route_ptr_;

  if (const auto msg = vector_map_subscriber_.take_data()) {
    lanelet_map_bin_ptr_ = msg;
  }
  input_data.lanelet_map_bin_ptr = lanelet_map_bin_ptr_;

  if (const auto msg = kinematic_state_subscriber_.take_data()) {
    odometry_ptr_ = msg;
  }
  input_data.odometry_ptr = odometry_ptr_;

  if (const auto msg = acceleration_subscriber_.take_data()) {
    acceleration_ptr_ = msg;
  }
  input_data.acceleration_ptr = acceleration_ptr_;

  if (const auto msg = objects_subscriber_.take_data()) {
    objects_ptr_ = msg;
  }
  input_data.objects_ptr = objects_ptr_;

  if (const auto msg = trigger_subscriber_.take_data()) {
    trigger_ptr_ = msg;
  }
  input_data.trigger_ptr = trigger_ptr_;

  return input_data;
}

bool InLaneMrmPlannerNode::is_data_ready(const InputData & input_data) const
{
  return input_data.lanelet_map_bin_ptr && input_data.route_ptr && input_data.odometry_ptr &&
         input_data.acceleration_ptr;
}

void InLaneMrmPlannerNode::update_params()
{
  params_ = param_listener_->get_params();
  path_planner_->update_params(params_);
  velocity_planner_.update_params(params_);
}

}  // namespace autoware::in_lane_mrm_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::in_lane_mrm_planner::InLaneMrmPlannerNode)
