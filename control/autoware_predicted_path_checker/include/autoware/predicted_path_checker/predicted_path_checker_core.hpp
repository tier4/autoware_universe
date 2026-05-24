// Copyright 2023 LeoDrive A.Ş. All rights reserved.
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

#ifndef AUTOWARE__PREDICTED_PATH_CHECKER__PREDICTED_PATH_CHECKER_CORE_HPP_
#define AUTOWARE__PREDICTED_PATH_CHECKER__PREDICTED_PATH_CHECKER_CORE_HPP_

#include <autoware/component_interface_specs_universe/control.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/predicted_path_checker/collision_checker.hpp>
#include <autoware/predicted_path_checker/utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/self_pose_listener.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::predicted_path_checker
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_utils::Point2d;
using autoware_utils::Polygon2d;
using geometry_msgs::msg::Pose;

struct NodeParam
{
  double update_rate;
  double delay_time;
  double max_deceleration;
  double resample_interval;
  double ego_nearest_dist_threshold;
  double ego_nearest_yaw_threshold;
  double stop_margin;
  double min_trajectory_check_length;
  double trajectory_check_time;
  double distinct_point_distance_threshold;
  double distinct_point_yaw_threshold;
  double filtering_distance_threshold;
  bool use_object_prediction;
};

enum class State {
  DRIVE = 0,
  EMERGENCY = 1,
  STOP = 2,
};

/// Core logic shared by composable node and safety monitor host plugin.
class PredictedPathCheckerCore
{
public:
  PredictedPathCheckerCore(
    rclcpp::Node * node, const std::string & param_prefix = "",
    const std::string & objects_topic = "~/input/objects",
    const std::string & reference_trajectory_topic = "~/input/reference_trajectory",
    const std::string & predicted_trajectory_topic = "~/input/predicted_trajectory",
    const std::string & odom_topic = "~/input/odometry",
    const std::string & accel_topic = "~/input/current_accel");

  void declare_parameters();
  void on_timer_cycle();
  void check_vehicle_state(diagnostic_updater::DiagnosticStatusWrapper & stat);

  double get_update_rate() const { return node_param_.update_rate; }

private:
  std::string param_name(const std::string & name) const;

  void onDynamicObjects(PredictedObjects::ConstSharedPtr msg);
  void onReferenceTrajectory(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
  void onPredictedTrajectory(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onAccel(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);
  void onIsStopped(
    const autoware::component_interface_specs_universe::control::IsStopped::Message::ConstSharedPtr
      msg);

  bool isDataReady();
  bool isDataTimeout();
  bool isThereStopPointOnReferenceTrajectory(
    const geometry_msgs::msg::Pose & pose, const TrajectoryPoints & reference_trajectory_array);
  void onTimer();
  void checkVehicleState(diagnostic_updater::DiagnosticStatusWrapper & stat);

  TrajectoryPoints trimTrajectoryFromSelfPose(
    const TrajectoryPoints & input, const Pose & self_pose) const;

  void sendRequest(bool make_stop_vehicle);

  bool isItDiscretePoint(
    const TrajectoryPoints & reference_trajectory, const TrajectoryPoint & collision_point) const;

  static Trajectory cutTrajectory(const Trajectory & trajectory, const double length);

  size_t insertStopPoint(
    TrajectoryPoints & trajectory, const geometry_msgs::msg::Point collision_point);

  void extendTrajectoryPointsArray(TrajectoryPoints & trajectory);

  static std::pair<double, double> calculateProjectedVelAndAcc(
    const PredictedObject & object, const TrajectoryPoint & trajectory_point);

  void filterObstacles(
    const Pose & ego_pose, const TrajectoryPoints & traj, const double dist_threshold,
    const bool use_prediction, PredictedObjects & filtered_objects);

  rclcpp::Node * node_;
  std::string param_prefix_;

  rclcpp::CallbackGroup::SharedPtr group_cli_;

  std::shared_ptr<autoware_utils::SelfPoseListener> self_pose_listener_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_dynamic_objects_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_reference_trajectory_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_predicted_trajectory_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr sub_accel_;
  autoware::component_interface_utils::Subscription<
    autoware::component_interface_specs_universe::control::IsStopped>::SharedPtr sub_stop_state_;

  autoware::component_interface_utils::Client<
    autoware::component_interface_specs_universe::control::SetStop>::SharedPtr cli_set_stop_;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;
  geometry_msgs::msg::Twist::ConstSharedPtr current_twist_;
  geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr current_accel_;
  PredictedObjects::ConstSharedPtr object_ptr_{nullptr};
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr reference_trajectory_;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr predicted_trajectory_;
  autoware::component_interface_specs_universe::control::IsStopped::Message::ConstSharedPtr
    is_stopped_ptr_{nullptr};

  std::unique_ptr<CollisionChecker> collision_checker_;
  std::shared_ptr<PredictedPathCheckerDebugNode> debug_ptr_;

  State current_state_{State::DRIVE};
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  bool is_calling_set_stop_{false};
  bool is_stopped_by_node_{false};

  CollisionCheckerParam collision_checker_param_;
  NodeParam node_param_;
};

}  // namespace autoware::predicted_path_checker

#endif  // AUTOWARE__PREDICTED_PATH_CHECKER__PREDICTED_PATH_CHECKER_CORE_HPP_
