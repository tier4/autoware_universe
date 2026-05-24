// Copyright 2025 Autoware Foundation
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

#include "autoware/control_safety_monitor_host/plugin/trajectory_obstacle_collision_plugin.hpp"

#include "autoware/obstacle_collision_checker/debug.hpp"

#include <autoware_utils/ros/update_param.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <functional>
#include <string>

namespace autoware::control_safety_monitor_host::plugin
{

void TrajectoryObstacleCollisionPlugin::on_initialize()
{
  auto * node = get_node_ptr();
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo();

  self_pose_listener_ = std::make_shared<autoware_utils::SelfPoseListener>(node);
  transform_listener_ = std::make_shared<autoware_utils::TransformListener>(node);

  using std::placeholders::_1;
  sub_obstacle_pointcloud_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/obstacle_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&TrajectoryObstacleCollisionPlugin::on_obstacle_pointcloud, this, _1));
  sub_reference_trajectory_ = node->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/reference_trajectory", 1,
    std::bind(&TrajectoryObstacleCollisionPlugin::on_reference_trajectory, this, _1));
  sub_predicted_trajectory_ = node->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/predicted_trajectory", 1,
    std::bind(&TrajectoryObstacleCollisionPlugin::on_predicted_trajectory, this, _1));
  sub_odom_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "input/odometry", 1, std::bind(&TrajectoryObstacleCollisionPlugin::on_odom, this, _1));

  debug_publisher_ = std::make_shared<autoware_utils::DebugPublisher>(node, "debug/obstacle_collision");
  time_publisher_ = std::make_shared<autoware_utils::ProcessingTimePublisher>(node);

  updater_ = std::make_unique<diagnostic_updater::Updater>(node);
  updater_->setHardwareID("obstacle_collision_checker");
  updater_->add(
    "obstacle_collision_checker: obstacle_collision_checker", this,
    &TrajectoryObstacleCollisionPlugin::check_collision);

  self_pose_listener_->wait_for_first_pose();
}

void TrajectoryObstacleCollisionPlugin::set_up_params()
{
  auto * node = get_node_ptr();
  update_rate_ = node->declare_parameter<double>(param_name("update_rate"));
  input_.param.delay_time = node->declare_parameter<double>(param_name("delay_time"));
  input_.param.footprint_margin = node->declare_parameter<double>(param_name("footprint_margin"));
  input_.param.max_deceleration = node->declare_parameter<double>(param_name("max_deceleration"));
  input_.param.resample_interval = node->declare_parameter<double>(param_name("resample_interval"));
  input_.param.search_radius = node->declare_parameter<double>(param_name("search_radius"));
}

void TrajectoryObstacleCollisionPlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  try {
    using autoware_utils::update_param;
    update_param(parameters, param_name("update_rate"), update_rate_);
    update_param(parameters, param_name("delay_time"), input_.param.delay_time);
    update_param(parameters, param_name("footprint_margin"), input_.param.footprint_margin);
    update_param(parameters, param_name("max_deceleration"), input_.param.max_deceleration);
    update_param(parameters, param_name("resample_interval"), input_.param.resample_interval);
    update_param(parameters, param_name("search_radius"), input_.param.search_radius);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    RCLCPP_WARN(get_node_ptr()->get_logger(), "Parameter update failed: %s", e.what());
  }
}

std::string TrajectoryObstacleCollisionPlugin::param_name(const std::string & name) const
{
  return param_prefix_ + name;
}

void TrajectoryObstacleCollisionPlugin::on_obstacle_pointcloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  obstacle_pointcloud_ = msg;
}

void TrajectoryObstacleCollisionPlugin::on_reference_trajectory(
  const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  reference_trajectory_ = msg;
}

void TrajectoryObstacleCollisionPlugin::on_predicted_trajectory(
  const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  predicted_trajectory_ = msg;
}

void TrajectoryObstacleCollisionPlugin::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_twist_ = std::make_shared<geometry_msgs::msg::Twist>(msg->twist.twist);
}

bool TrajectoryObstacleCollisionPlugin::is_data_ready()
{
  auto * node = get_node_ptr();
  if (!current_pose_) {
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000, "waiting for current_pose...");
    return false;
  }
  if (!obstacle_pointcloud_) {
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000, "waiting for obstacle_pointcloud msg...");
    return false;
  }
  if (!obstacle_transform_) {
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000, "waiting for obstacle_transform...");
    return false;
  }
  if (!reference_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000, "waiting for reference_trajectory msg...");
    return false;
  }
  if (!predicted_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000, "waiting for predicted_trajectory msg...");
    return false;
  }
  if (!current_twist_) {
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000, "waiting for current_twist msg...");
    return false;
  }
  return true;
}

bool TrajectoryObstacleCollisionPlugin::is_data_timeout()
{
  auto * node = get_node_ptr();
  const auto now = node->now();
  constexpr double th_pose_timeout = 1.0;
  const auto pose_time_diff =
    rclcpp::Time(current_pose_->header.stamp).seconds() - now.seconds();
  if (pose_time_diff > th_pose_timeout) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000, "pose is timeout...");
    return true;
  }
  return false;
}

void TrajectoryObstacleCollisionPlugin::on_timer()
{
  current_pose_ = self_pose_listener_->get_current_pose();
  if (obstacle_pointcloud_) {
    const auto & header = obstacle_pointcloud_->header;
    try {
      obstacle_transform_ = transform_listener_->get_transform(
        "map", header.frame_id, header.stamp, rclcpp::Duration::from_seconds(0.01));
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(
        get_node_ptr()->get_logger(), "Could not transform map to %s: %s", header.frame_id.c_str(),
        ex.what());
      return;
    }
  }

  if (!is_data_ready() || is_data_timeout()) {
    return;
  }

  input_.current_pose = current_pose_;
  input_.obstacle_pointcloud = obstacle_pointcloud_;
  input_.obstacle_transform = obstacle_transform_;
  input_.reference_trajectory = reference_trajectory_;
  input_.predicted_trajectory = predicted_trajectory_;
  input_.current_twist = current_twist_;
  input_.vehicle_info = vehicle_info_;

  output_ = autoware::obstacle_collision_checker::check_for_collisions(input_);

  debug_publisher_->publish(
    "marker_array", autoware::obstacle_collision_checker::create_marker_array(
                      output_, current_pose_->pose.position.z, get_node_ptr()->now()));

  time_publisher_->publish(output_.processing_time_map);
}

void TrajectoryObstacleCollisionPlugin::check_collision(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (output_.will_collide) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "vehicle will collide with obstacles";
  }

  stat.summary(level, msg);
}

void TrajectoryObstacleCollisionPlugin::update()
{
  if (!is_enabled()) {
    return;
  }
  on_timer();
  if (updater_) {
    updater_->force_update();
  }
}

}  // namespace autoware::control_safety_monitor_host::plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::control_safety_monitor_host::plugin::TrajectoryObstacleCollisionPlugin,
  autoware::control_safety_monitor_host::plugin::SafetyMonitorPluginBase)
