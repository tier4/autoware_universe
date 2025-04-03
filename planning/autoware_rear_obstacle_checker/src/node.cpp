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

#include "node.hpp"

#include "utils.hpp"

#include <autoware_lanelet2_extension/visualization/visualization.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Polygon.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace autoware::rear_obstacle_checker
{

using std::chrono_literals::operator""ms;

RearObstacleCheckerNode::RearObstacleCheckerNode(const rclcpp::NodeOptions & node_options)
: Node("rear_obstacle_checker_node", node_options),
  timer_{rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&RearObstacleCheckerNode::on_timer, this))},
  tf_buffer_{this->get_clock()},
  tf_listener_{tf_buffer_},
  pub_debug_marker_{this->create_publisher<MarkerArray>("~/debug_marker", 20)},
  pub_debug_processing_time_detail_{this->create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1)},
  route_handler_{std::make_shared<autoware::route_handler::RouteHandler>()},
  param_listener_{std::make_unique<rear_obstacle_checker_node::ParamListener>(
    this->get_node_parameters_interface())},
  diag_updater_{std::make_unique<diagnostic_updater::Updater>(this)},
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()},
  time_keeper_{std::make_shared<autoware_utils::TimeKeeper>(pub_debug_processing_time_detail_)}
{
  for (const auto & [key, param] : param_listener_->get_params().scene_map) {
    sub_planning_factor_map_.emplace(
      key, autoware_utils::InterProcessPollingSubscriber<PlanningFactorArray>{this, param.topic});
  }

  diag_updater_->setHardwareID("rear_obstacle_checker");
  diag_updater_->add("collision_risk", this, &RearObstacleCheckerNode::update);
}

void RearObstacleCheckerNode::take_data()
{
  // route
  {
    const auto msg = sub_route_.take_data();
    if (msg) {
      if (msg->segments.empty()) {
        RCLCPP_ERROR(get_logger(), "input route is empty. ignored");
      } else {
        route_handler_->setRoute(*msg);
      }
    }
  }

  // map
  {
    const auto msg = sub_lanelet_map_bin_.take_data();
    if (msg) {
      route_handler_->setMap(*msg);
    }
  }

  // odometry
  {
    odometry_ptr_ = sub_odometry_.take_data();
  }

  // acceleration
  {
    acceleration_ptr_ = sub_accleration_.take_data();
  }

  // objects
  {
    object_ptr_ = sub_dynamic_objects_.take_data();
  }

  // pointcloud
  {
    const auto msg = sub_pointcloud_.take_data();
    if (msg && !msg->data.empty()) {
      geometry_msgs::msg::TransformStamped transform_stamped;
      try {
        transform_stamped = tf_buffer_.lookupTransform(
          "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
      } catch (tf2::TransformException & e) {
        RCLCPP_WARN(get_logger(), "no transform found for no_ground_pointcloud: %s", e.what());
      }

      Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
      pcl::fromROSMsg(*msg, pointcloud_);
      autoware_utils::transform_pointcloud(pointcloud_, pointcloud_, isometry);
    } else {
      pointcloud_.clear();
    }
  }

  // trajectory
  {
    path_ptr_ = sub_path_.take_data();
  }

  {
    PlanningFactorArray planning_factors;
    for (auto & [name, subscriber] : sub_planning_factor_map_) {
      const auto msg = subscriber.take_data();
      if (!msg) {
        continue;
      }

      planning_factors.factors.insert(
        planning_factors.factors.end(), msg->factors.begin(), msg->factors.end());
    }

    factors_ptr_ = std::make_shared<PlanningFactorArray>(planning_factors);
  }
}

bool RearObstacleCheckerNode::is_ready() const
{
  if (!route_handler_->isHandlerReady()) {
    return false;
  }

  if (!odometry_ptr_) {
    return false;
  }

  if (!acceleration_ptr_) {
    return false;
  }

  if (!object_ptr_) {
    return false;
  }

  if (!path_ptr_) {
    return false;
  }

  return true;
}

void RearObstacleCheckerNode::update(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  take_data();

  if (!is_ready()) {
    return;
  }

  DebugData debug_data;

  if (!is_safe(debug_data)) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "obstacles exist beside ego");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "validated.");
  }

  publish_marker(debug_data);
}

void RearObstacleCheckerNode::on_timer()
{
  diag_updater_->force_update();
}

bool RearObstacleCheckerNode::is_safe(DebugData & debug)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params();
  const auto forward_range =
    std::max(p.common.range.object.forward, p.common.range.pointcloud.forward);
  const auto backward_range =
    std::max(p.common.range.object.backward, p.common.range.pointcloud.backward);

  const auto resampled_path = behavior_path_planner::utils::resamplePathWithSpline(*path_ptr_, 0.3);
  const auto predicted_stop_pose = utils::calc_predicted_stop_pose(
    resampled_path.points, odometry_ptr_->pose.pose, odometry_ptr_->twist.twist.linear.x,
    acceleration_ptr_->accel.accel.linear.x, p);

  if (!predicted_stop_pose.has_value()) {
    return true;
  }

  {
    const auto transform = autoware_utils::pose2transform(predicted_stop_pose.value());
    debug.predicted_stop_pose_footprint = utils::to_basic_polygon3d(
      autoware_utils::transform_vector(vehicle_info_.createFootprint(), transform),
      odometry_ptr_->pose.pose.position.z);
  }

  const auto current_lanes = utils::get_current_lanes(
    predicted_stop_pose.value(), route_handler_, vehicle_info_, forward_range, backward_range);
  if (current_lanes.empty()) {
    debug.text = {
      "CAUSION!!!\nEGO CAN'T STOP WITHIN CURRENT LANE.",
      autoware_utils::create_marker_color(1.0, 0.67, 0.0, 0.999)};
  }

  PredictedObjects objects_on_target_lane;
  PredictedObjects objects_filtered_by_class;
  pcl::PointCloud<pcl::PointXYZ> obstacle_pointcloud;

  for (const auto & factor : factors_ptr_->factors) {
    if (p.scene_map.count(factor.module) == 0) {
      continue;
    }

    if (!utils::should_activate(factor, p)) {
      continue;
    }

    if (utils::should_check_objects(factor, p)) {
      time_keeper_->start_track("prepare_detection_area_for_objects");
      const auto detection_lanes_for_objects = utils::generate_detection_area_for_object(
        factor, current_lanes, odometry_ptr_->pose.pose, route_handler_, vehicle_info_, p);
      debug.detection_lanes_for_objects.insert(
        debug.detection_lanes_for_objects.end(), detection_lanes_for_objects.begin(),
        detection_lanes_for_objects.end());
      time_keeper_->end_track("prepare_detection_area_for_objects");

      time_keeper_->start_track("filter_objects");
      const auto [targets, others] =
        behavior_path_planner::utils::path_safety_checker::separateObjectsByLanelets(
          *object_ptr_, detection_lanes_for_objects,
          [&factor, &p](const auto & obj, const auto & lane, const auto yaw_threshold = M_PI_2) {
            return behavior_path_planner::utils::path_safety_checker::isPolygonOverlapLanelet(
              obj, lane, yaw_threshold);
          });

      objects_on_target_lane.objects.insert(
        objects_on_target_lane.objects.end(), targets.objects.begin(), targets.objects.end());

      for (const auto & object : targets.objects) {
        if (utils::is_target(object, factor, p)) {
          objects_filtered_by_class.objects.push_back(object);
        }
      }
      time_keeper_->end_track("filter_objects");
    }

    if (utils::should_check_pointcloud(factor, p)) {
      time_keeper_->start_track("prepare_detection_area_for_pointcloud");
      const auto detection_areas_for_pointcloud = utils::generate_detection_area_for_pointcloud(
        factor, current_lanes, odometry_ptr_->pose.pose, odometry_ptr_->twist.twist.linear.x,
        acceleration_ptr_->accel.accel.linear.x, route_handler_, vehicle_info_, p);
      debug.detection_areas_for_pointcloud.insert(
        debug.detection_areas_for_pointcloud.end(), detection_areas_for_pointcloud.begin(),
        detection_areas_for_pointcloud.end());
      time_keeper_->end_track("prepare_detection_area_for_pointcloud");

      time_keeper_->start_track("filter_pointcloud");
      obstacle_pointcloud += utils::filter_lost_object_pointcloud(
        objects_on_target_lane,
        utils::get_obstacle_points(detection_areas_for_pointcloud, pointcloud_),
        p.common.pointcloud.object_buffer);
      time_keeper_->end_track("filter_pointcloud");
    }
  }

  {
    const auto now = this->now();
    if (is_safe(objects_filtered_by_class, debug) && is_safe(obstacle_pointcloud, debug)) {
      last_safe_time_ = now;
      if ((now - last_unsafe_time_).seconds() > p.common.off_time_buffer) {
        return true;
      }
    } else {
      if ((now - last_safe_time_).seconds() < p.common.on_time_buffer) {
        return true;
      }
    }

    {
      debug.text = {
        "RISK OF COLLISION!!!", autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.999)};
    }
  }

  return false;
}

bool RearObstacleCheckerNode::is_safe(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud, DebugData & debug) const
{
  for (const auto & p : pointcloud) {
    debug.obstacle_pointcloud.push_back(autoware_utils::create_point(p.x, p.y, p.z));
  }
  return pointcloud.points.empty();
}

bool RearObstacleCheckerNode::is_safe(const PredictedObjects & objects, DebugData & debug) const
{
  const auto p = param_listener_->get_params();

  const auto ego_predicted_path_params =
    std::make_shared<behavior_path_planner::utils::path_safety_checker::EgoPredictedPathParams>(
      get_predicted_path_params());

  std::vector<behavior_path_planner::utils::path_safety_checker::ExtendedPredictedObject>
    target_objects;

  std::for_each(objects.objects.begin(), objects.objects.end(), [&](const auto & object) {
    target_objects.push_back(behavior_path_planner::utils::path_safety_checker::transform(
      object, p.common.predicted_path.time_horizon, p.common.predicted_path.time_resolution));
  });

  const bool limit_to_max_velocity = false;
  const size_t ego_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path_ptr_->points, odometry_ptr_->pose.pose, 1.0, M_PI_2);
  const auto ego_predicted_path =
    behavior_path_planner::utils::path_safety_checker::createPredictedPath(
      ego_predicted_path_params, path_ptr_->points, odometry_ptr_->pose.pose,
      odometry_ptr_->twist.twist.linear.x, ego_seg_idx, true, limit_to_max_velocity);

  for (const auto & object : target_objects) {
    auto current_debug_data =
      behavior_path_planner::utils::path_safety_checker::createObjectDebug(object);

    const auto obj_polygon = autoware_utils::to_polygon2d(object.initial_pose, object.shape);

    const auto is_object_front =
      behavior_path_planner::utils::path_safety_checker::isTargetObjectFront(
        odometry_ptr_->pose.pose, obj_polygon, vehicle_info_.max_longitudinal_offset_m);
    if (is_object_front) {
      continue;
    }

    const auto obj_predicted_paths =
      behavior_path_planner::utils::path_safety_checker::getPredictedPathFromObj(object, false);

    for (const auto & obj_path : obj_predicted_paths) {
      if (!behavior_path_planner::utils::path_safety_checker::checkCollision(
            *path_ptr_, ego_predicted_path, object, obj_path, get_vehicle_params(),
            get_rss_params(), 1.0, M_PI_2, current_debug_data.second)) {
        behavior_path_planner::utils::path_safety_checker::updateCollisionCheckDebugMap(
          debug.collision_check, current_debug_data, false);

        return false;
      }
    }
    behavior_path_planner::utils::path_safety_checker::updateCollisionCheckDebugMap(
      debug.collision_check, current_debug_data, true);
  }

  return true;
}

void RearObstacleCheckerNode::publish_marker(const DebugData & debug) const
{
  MarkerArray msg;

  const auto add = [&msg](const MarkerArray & added) {
    autoware_utils::append_marker_array(added, &msg);
  };

  {
    add(utils::create_polygon_marker_array(
      {debug.predicted_stop_pose_footprint}, "predicted_stop_pose_footprint",
      autoware_utils::create_marker_color(0.16, 1.0, 0.69, 0.999)));
  }

  {
    add(lanelet::visualization::laneletsAsTriangleMarkerArray(
      "detection_lanes_for_objects", debug.detection_lanes_for_objects,
      autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.2)));
  }

  {
    add(utils::createPointsMarkerArray(debug.obstacle_pointcloud, "obstacle_pointcloud"));
    add(utils::create_polygon_marker_array(
      debug.detection_areas_for_pointcloud, "detection_areas_for_pointcloud",
      autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.999)));
  }

  {
    add(utils::showSafetyCheckInfo(debug.collision_check, "object_debug_info"));
    add(utils::showPredictedPath(debug.collision_check, "ego_predicted_path"));
    add(utils::showPolygon(debug.collision_check, "ego_and_target_polygon_relation"));
  }

  {
    auto marker = autoware_utils::create_default_marker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "text", 0L,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      autoware_utils::create_marker_scale(0.5, 0.5, 0.5), debug.text.second);
    marker.text = debug.text.first.c_str();
    marker.pose = odometry_ptr_->pose.pose;
    marker.pose.position.z += 5.0;
    msg.markers.push_back(marker);
  }

  std::for_each(msg.markers.begin(), msg.markers.end(), [](auto & marker) {
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  });

  pub_debug_marker_->publish(msg);
}
}  // namespace autoware::rear_obstacle_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::rear_obstacle_checker::RearObstacleCheckerNode)
