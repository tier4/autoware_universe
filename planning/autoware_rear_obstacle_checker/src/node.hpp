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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_rear_obstacle_checker/rear_obstacle_checker_node_parameters.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_planning_msgs/msg/planning_factor_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::rear_obstacle_checker
{

using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::LaneletRoute;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;
using tier4_planning_msgs::msg::PlanningFactor;
using tier4_planning_msgs::msg::PlanningFactorArray;
using visualization_msgs::msg::MarkerArray;

struct DebugData
{
  lanelet::BasicPolygon3d predicted_stop_pose_footprint;

  lanelet::ConstLanelets detection_lanes_for_objects;

  lanelet::BasicPolygons3d detection_areas_for_pointcloud;

  behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap collision_check;

  std::vector<geometry_msgs::msg::Point> obstacle_pointcloud;

  std::string text{"NONE"};
};

class RearObstacleCheckerNode : public rclcpp::Node
{
public:
  explicit RearObstacleCheckerNode(const rclcpp::NodeOptions & node_options);

private:
  void on_timer();

  void take_data();

  bool is_ready() const;

  bool is_safe(const PredictedObjects & objects, DebugData & debug) const;

  bool is_safe(const pcl::PointCloud<pcl::PointXYZ> & pointcloud, DebugData & debug) const;

  bool is_safe(DebugData & debug);

  void update(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void publish_marker(const DebugData & debug) const;

  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;

  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_marker_;

  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    sub_route_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};

  autoware_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils::polling_policy::Newest>
    sub_lanelet_map_bin_{this, "~/input/lanelet_map_bin", rclcpp::QoS{1}.transient_local()};

  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped> sub_accleration_{
    this, "~/input/acceleration"};

  autoware_utils::InterProcessPollingSubscriber<Odometry> sub_odometry_{this, "~/input/odometry"};

  autoware_utils::InterProcessPollingSubscriber<PointCloud2> sub_pointcloud_{
    this, "~/input/pointcloud", autoware_utils::single_depth_sensor_qos()};

  autoware_utils::InterProcessPollingSubscriber<PredictedObjects> sub_dynamic_objects_{
    this, "~/input/objects"};

  autoware_utils::InterProcessPollingSubscriber<PathWithLaneId> sub_path_{
    this, "~/input/path_with_lane_id"};

  std::unordered_map<
    std::string, autoware_utils::InterProcessPollingSubscriber<PlanningFactorArray>>
    sub_planning_factor_map_;

  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler_;

  std::unique_ptr<rear_obstacle_checker_node::ParamListener> param_listener_;

  std::unique_ptr<diagnostic_updater::Updater> diag_updater_;

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  Odometry::ConstSharedPtr odometry_ptr_;

  AccelWithCovarianceStamped::ConstSharedPtr acceleration_ptr_;

  PredictedObjects::ConstSharedPtr object_ptr_;

  PathWithLaneId::ConstSharedPtr path_ptr_;

  PlanningFactorArray::ConstSharedPtr factors_ptr_;

  pcl::PointCloud<pcl::PointXYZ> pointcloud_;

  rclcpp::Time last_safe_time_{this->now()};

  rclcpp::Time last_unsafe_time_{this->now()};

  auto get_predicted_path_params() const
    -> behavior_path_planner::utils::path_safety_checker::EgoPredictedPathParams
  {
    const auto p = param_listener_->get_params();

    behavior_path_planner::utils::path_safety_checker::EgoPredictedPathParams params{};
    params.min_velocity = p.common.predicted_path.min_velocity;
    params.max_velocity = p.common.predicted_path.max_velocity;
    params.acceleration = p.common.predicted_path.acceleration;
    params.time_horizon_for_front_object = p.common.predicted_path.time_horizon;
    params.time_horizon_for_rear_object = p.common.predicted_path.time_horizon;
    params.time_resolution = p.common.predicted_path.time_resolution;
    params.delay_until_departure = p.common.predicted_path.delay_until_departure;

    return params;
  }

  // rss parameters
  auto get_rss_params() const -> behavior_path_planner::utils::path_safety_checker::RSSparams
  {
    const auto p = param_listener_->get_params();

    behavior_path_planner::utils::path_safety_checker::RSSparams params{};

    params.extended_polygon_policy = p.common.rss.extended_polygon_policy;
    params.longitudinal_distance_min_threshold = p.common.rss.longitudinal_distance_min_threshold;
    params.longitudinal_velocity_delta_time = p.common.rss.longitudinal_velocity_delta_time;
    params.front_vehicle_deceleration = p.common.rss.vehicle_deceleration;
    params.rear_vehicle_deceleration = p.common.rss.vehicle_deceleration;
    params.rear_vehicle_reaction_time = p.common.rss.vehicle_reaction_time;
    params.rear_vehicle_safety_time_margin = p.common.rss.vehicle_safety_time_margin;
    params.lateral_distance_max_threshold = p.common.rss.lateral_distance_max_threshold;

    return params;
  }

  auto get_integral_params() const
    -> behavior_path_planner::utils::path_safety_checker::IntegralPredictedPolygonParams
  {
    const auto p = param_listener_->get_params();

    behavior_path_planner::utils::path_safety_checker::IntegralPredictedPolygonParams params{};

    params.forward_margin = 0.0;
    params.backward_margin = 0.0;
    params.lat_margin = p.common.rss.lateral_distance_max_threshold;
    params.time_horizon = p.common.predicted_path.time_horizon;

    return params;
  }

  auto get_vehicle_params() const -> BehaviorPathPlannerParameters
  {
    BehaviorPathPlannerParameters params{};

    params.vehicle_info = vehicle_info_;
    params.vehicle_width = vehicle_info_.vehicle_width_m;
    params.vehicle_length = vehicle_info_.vehicle_length_m;
    params.wheel_tread = vehicle_info_.wheel_tread_m;
    params.wheel_base = vehicle_info_.wheel_base_m;
    params.front_overhang = vehicle_info_.front_overhang_m;
    params.rear_overhang = vehicle_info_.rear_overhang_m;
    params.left_over_hang = vehicle_info_.left_overhang_m;
    params.right_over_hang = vehicle_info_.right_overhang_m;
    params.base_link2front = vehicle_info_.max_longitudinal_offset_m;
    params.base_link2rear = params.rear_overhang;

    return params;
  }
};
}  // namespace autoware::rear_obstacle_checker

#endif  // NODE_HPP_
