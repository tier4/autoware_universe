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

#ifndef PLUGINS__OBSTACLE_STOP_MODIFIER_HPP_
#define PLUGINS__OBSTACLE_STOP_MODIFIER_HPP_

#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner::plugin
{

using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::TrajectoryModifierParams;
using autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using Trajectory = autoware::experimental::trajectory::Trajectory<TrajectoryPoint>;

struct StopObstacle
{
  StopObstacle(
    const unique_identifier_msgs::msg::UUID & arg_uuid, const rclcpp::Time & arg_stamp,
    const geometry_msgs::msg::Pose & arg_pose, const double arg_velocity,
    const autoware_perception_msgs::msg::Shape & arg_shape,
    const geometry_msgs::msg::Point & arg_collision_point, const double arg_dist_to_collide_on_traj)
  : uuid(arg_uuid),
    stamp(arg_stamp),
    pose(arg_pose),
    velocity(arg_velocity),
    shape(arg_shape),
    collision_point(arg_collision_point),
    dist_to_collide_on_traj(arg_dist_to_collide_on_traj)
  {
  }

  unique_identifier_msgs::msg::UUID uuid;
  rclcpp::Time stamp;
  geometry_msgs::msg::Pose pose;
  double velocity;
  autoware_perception_msgs::msg::Shape shape;
  geometry_msgs::msg::Point collision_point;
  double dist_to_collide_on_traj;
};

class ObstacleStopModifier : public TrajectoryModifierPluginBase
{
public:
  ObstacleStopModifier(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper,
    const TrajectoryModifierParams & params);

  void modify_trajectory(
    TrajectoryPoints & traj_points, const TrajectoryModifierParams & params,
    const TrajectoryModifierData & data) override;

  void set_up_params() override;

  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

  bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, const TrajectoryModifierParams & params,
    const TrajectoryModifierData & data) const override;

  void set_predicted_objects(const PredictedObjects::ConstSharedPtr & objects);

private:
  // --- ported from obstacle_stop_module ---
  std::vector<StopObstacle> filter_stop_obstacle_for_predicted_object(
    const TrajectoryModifierData & data, const Trajectory & traj,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<double> & decimated_s_values, const double x_offset_to_bumper);

  void plan_stop(
    Trajectory & traj, const std::vector<StopObstacle> & stop_obstacles,
    const TrajectoryModifierData & data);

  static std::vector<StopObstacle> get_closest_stop_obstacles(
    const std::vector<StopObstacle> & stop_obstacles);

  struct Parameters
  {
    double stop_margin_m{5.0};
    double detection_range_m{50.0};
    double lateral_margin_m{0.0};
    double decimate_step_length_m{2.0};
  };

  Parameters params_;
  PredictedObjects::ConstSharedPtr predicted_objects_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  std::vector<StopObstacle> prev_stop_obstacles_;
};

}  // namespace autoware::minimum_rule_based_planner::plugin

#endif  // PLUGINS__OBSTACLE_STOP_MODIFIER_HPP_
