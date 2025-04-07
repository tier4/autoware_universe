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

#include "autoware/signal_processing/lowpass_filter_1d.hpp"
#include "utils.hpp"

#include <autoware_lanelet2_extension/visualization/visualization.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

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
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

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
    if (msg) {
      header_ptr_ = std::make_shared<const std_msgs::msg::Header>(msg->header);
      pointcloud_ = std::make_shared<PointCloud>();

      if (!msg->data.empty()) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
          transform_stamped = tf_buffer_.lookupTransform(
            "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
        } catch (tf2::TransformException & e) {
          RCLCPP_WARN(get_logger(), "no transform found for no_ground_pointcloud: %s", e.what());
        }

        Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
        pcl::fromROSMsg(*msg, *pointcloud_);
        autoware_utils::transform_pointcloud(*pointcloud_, *pointcloud_, isometry);
      }
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

  if (!pointcloud_) {
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

  const auto [is_left_shift, is_right_shift] =
    utils::is_shift_path(current_lanes, resampled_path.points, vehicle_info_);

  const auto filtered = filter_pointcloud(pointcloud_);

  PredictedObjects objects_on_target_lane;
  PredictedObjects objects_filtered_by_class;
  PointCloudObjects pointcloud_objects;

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
      time_keeper_->start_track("get_pointcloud_objects");
      const auto objects = get_pointcloud_objects(factor, current_lanes, debug);
      pointcloud_objects.insert(pointcloud_objects.end(), objects.begin(), objects.end());
      time_keeper_->end_track("get_pointcloud_objects");

      // time_keeper_->start_track("prepare_detection_area_for_pointcloud");
      // const auto detection_areas_for_pointcloud = utils::generate_detection_area_for_pointcloud(
      //   factor, current_lanes, odometry_ptr_->pose.pose, odometry_ptr_->twist.twist.linear.x,
      //   acceleration_ptr_->accel.accel.linear.x, route_handler_, vehicle_info_, p);
      // debug.detection_areas_for_pointcloud.insert(
      //   debug.detection_areas_for_pointcloud.end(), detection_areas_for_pointcloud.begin(),
      //   detection_areas_for_pointcloud.end());
      // time_keeper_->end_track("prepare_detection_area_for_pointcloud");

      // time_keeper_->start_track("filter_pointcloud");
      // obstacle_pointcloud += utils::filter_lost_object_pointcloud(
      //   objects_on_target_lane,
      //   utils::get_obstacle_points(detection_areas_for_pointcloud, *pointcloud_),
      //   p.common.pointcloud.object_buffer);
      // time_keeper_->end_track("filter_pointcloud");
    }
  }

  {
    const auto now = this->now();
    if (is_safe(objects_filtered_by_class, debug) && is_safe(pointcloud_objects, debug)) {
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

bool RearObstacleCheckerNode::is_safe(const PointCloudObjects & objects, DebugData & debug) const
{
  const auto p = param_listener_->get_params();

  const auto delay_ego = p.common.prediction.object.reaction_time;
  const auto max_deceleration_ego = p.common.prediction.ego.max_deceleration;
  const auto max_deceleration_object = p.common.prediction.object.max_deceleration;
  const auto current_velocity = odometry_ptr_->twist.twist.linear.x;
  const auto current_acceleration = acceleration_ptr_->accel.accel.linear.x;

  for (const auto & object : objects) {
    const auto stop_distance_object =
      0.5 * std::pow(object.velocity, 2.0) / std::abs(max_deceleration_object);
    const auto stop_distance_ego =
      current_velocity * delay_ego + 0.5 * current_acceleration * std::pow(delay_ego, 2.0) +
      0.5 * std::pow(current_velocity + current_acceleration * delay_ego, 2.0) /
        std::abs(max_deceleration_ego);

    const auto rss_distance = stop_distance_object - stop_distance_ego;
    if (rss_distance > object.relative_distance) {
      return false;
    }
  }

  {
    debug.pointcloud_objects = objects;
  }

  return true;
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

auto RearObstacleCheckerNode::filter_pointcloud(const PointCloud::Ptr in) const -> PointCloud::Ptr
{
  auto out = in;
  const auto z_coordinate = odometry_ptr_->pose.pose.position.z;
  // apply z-axis filter for removing False Positive points
  constexpr double detection_range_min_height_ = 0.0;
  constexpr double detection_range_max_height_margin_ = 0.0;
  pcl::PassThrough<pcl::PointXYZ> height_filter;
  height_filter.setInputCloud(out);
  height_filter.setFilterFieldName("z");
  height_filter.setFilterLimits(
    detection_range_min_height_ + z_coordinate,
    vehicle_info_.vehicle_height_m + detection_range_max_height_margin_ + z_coordinate);
  height_filter.filter(*out);
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(out);
  constexpr double voxel_grid_x_ = 0.1;
  constexpr double voxel_grid_y_ = 0.1;
  constexpr double voxel_grid_z_ = 0.5;
  filter.setLeafSize(voxel_grid_x_, voxel_grid_y_, voxel_grid_z_);
  filter.filter(*out);

  return get_clustered_pointcloud(out);
}

auto RearObstacleCheckerNode::get_clustered_pointcloud(const PointCloud::Ptr in) const
  -> PointCloud::Ptr
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto points_belonging_to_cluster_hulls = std::make_shared<PointCloud>();
  // eliminate noisy points by only considering points belonging to clusters of at least a certain
  // size
  if (in->empty()) return std::make_shared<PointCloud>();
  const std::vector<pcl::PointIndices> cluster_indices = std::invoke([&]() {
    std::vector<pcl::PointIndices> cluster_idx;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(in);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    constexpr double cluster_tolerance_ = 0.15;
    constexpr size_t minimum_cluster_size_ = 10;
    constexpr size_t maximum_cluster_size_ = 10000;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(minimum_cluster_size_);
    ec.setMaxClusterSize(maximum_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in);
    ec.extract(cluster_idx);
    return cluster_idx;
  });
  std::vector<autoware_utils::Polygon3d> hull_polygons;
  for (const auto & indices : cluster_indices) {
    PointCloud::Ptr cluster(new PointCloud);
    bool cluster_surpasses_threshold_height{false};
    constexpr double cluster_minimum_height_ = 0.1;
    for (const auto & index : indices.indices) {
      const auto & p = (*in)[index];
      cluster_surpasses_threshold_height = (cluster_surpasses_threshold_height)
                                             ? cluster_surpasses_threshold_height
                                             : (p.z > cluster_minimum_height_);
      cluster->push_back(p);
    }
    if (!cluster_surpasses_threshold_height) continue;
    // Make a 2d convex hull for the objects
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setDimension(2);
    hull.setInputCloud(cluster);
    std::vector<pcl::Vertices> polygons;
    PointCloud::Ptr surface_hull(new PointCloud);
    hull.reconstruct(*surface_hull, polygons);
    autoware_utils::Polygon3d hull_polygon;
    for (const auto & p : *surface_hull) {
      points_belonging_to_cluster_hulls->push_back(p);
      // if (publish_debug_markers_) {
      //   const auto geom_point = autoware_utils::create_point(p.x, p.y, p.z);
      //   appendPointToPolygon(hull_polygon, geom_point);
      // }
    }
    hull_polygons.push_back(hull_polygon);
  }
  return points_belonging_to_cluster_hulls;
  // if (publish_debug_markers_ && !hull_polygons.empty()) {
  //   constexpr colorTuple debug_color = {255.0 / 256.0, 51.0 / 256.0, 255.0 / 256.0, 0.999};
  //   addClusterHullMarkers(now(), hull_polygons, debug_color, "hulls", debug_markers);
  // }
}

auto RearObstacleCheckerNode::get_pointcloud_objects(
  const PlanningFactor & factor, const lanelet::ConstLanelets & current_lanes, DebugData & debug)
  -> PointCloudObjects
{
  const auto p = param_listener_->get_params();

  const auto config = p.scene_map.at(factor.module);

  PointCloudObjects objects;

  constexpr double v_object = 10.0;

  const auto delay_ego = p.common.prediction.object.reaction_time;
  const auto max_deceleration_ego = p.common.prediction.ego.max_deceleration;
  const auto max_deceleration_object = p.common.prediction.object.max_deceleration;
  const auto current_velocity = odometry_ptr_->twist.twist.linear.x;
  const auto current_acceleration = acceleration_ptr_->accel.accel.linear.x;

  const auto stop_distance_object =
    0.5 * std::pow(v_object, 2.0) / std::abs(max_deceleration_object);
  const auto stop_distance_ego =
    current_velocity * delay_ego + 0.5 * current_acceleration * std::pow(delay_ego, 2.0) +
    0.5 * std::pow(current_velocity + current_acceleration * delay_ego, 2.0) /
      std::abs(max_deceleration_ego);

  const auto forward_distance =
    p.common.range.pointcloud.forward + vehicle_info_.max_longitudinal_offset_m;
  const auto backward_distance = p.common.range.pointcloud.backward -
                                 vehicle_info_.min_longitudinal_offset_m +
                                 std::max(0.0, stop_distance_object - stop_distance_ego);

  if (
    factor.behavior == PlanningFactor::SHIFT_LEFT || factor.behavior == PlanningFactor::TURN_LEFT) {
    if (config.adjacent_lane) {
      const auto pointcloud_objects =
        get_pointcloud_objects(current_lanes, false, forward_distance, backward_distance, debug);
      objects.insert(objects.end(), pointcloud_objects.begin(), pointcloud_objects.end());
    }

    // if (config.current_lane) {
    //   const auto half_lanes = [&current_lanes, &vehicle_info]() {
    //     lanelet::ConstLanelets ret{};
    //     for (const auto & lane : current_lanes) {
    //       ret.push_back(
    //         utils::generate_half_lanelet(lane, false, 0.5 * vehicle_info.vehicle_width_m));
    //     }
    //     return ret;
    //   }();
    //   detection_polygons.push_back(utils::generate_detection_polygon(
    //     half_lanes, ego_pose, forward_distance, backward_distance));
    // }
  }

  if (
    factor.behavior == PlanningFactor::SHIFT_RIGHT ||
    factor.behavior == PlanningFactor::TURN_RIGHT) {
    if (config.adjacent_lane) {
      const auto pointcloud_objects =
        get_pointcloud_objects(current_lanes, true, forward_distance, backward_distance, debug);
      objects.insert(objects.end(), pointcloud_objects.begin(), pointcloud_objects.end());
    }

    // if (config.current_lane) {
    //   const auto half_lanes = [&current_lanes, &vehicle_info]() {
    //     lanelet::ConstLanelets ret{};
    //     for (const auto & lane : current_lanes) {
    //       ret.push_back(
    //         utils::generate_half_lanelet(lane, true, 0.5 * vehicle_info.vehicle_width_m));
    //     }
    //     return ret;
    //   }();
    //   detection_polygons.push_back(utils::generate_detection_polygon(
    //     half_lanes, ego_pose, forward_distance, backward_distance));
    // }
  }

  return objects;
}

auto RearObstacleCheckerNode::get_pointcloud_objects(
  const lanelet::ConstLanelets & current_lanes, const bool is_right, const double forward_distance,
  const double backward_distance, DebugData & debug) -> PointCloudObjects
{
  const auto ego_coordinate_on_arc =
    lanelet::utils::getArcCoordinates(current_lanes, odometry_ptr_->pose.pose);

  PointCloudObjects objects{};

  lanelet::ConstLanelets connected_adjacent_lanes{};

  double length = 0.0;
  for (const auto & lane : current_lanes) {
    const auto current_lane_length = lanelet::utils::getLaneletLength2d(lane);

    length += current_lane_length;

    const auto ego_to_furthest_point = length - ego_coordinate_on_arc.length;
    const auto residual_distance = ego_to_furthest_point - forward_distance;

    const auto opt_adjacent_lane = [&lane, &is_right, this]() {
      return is_right ? route_handler_->getRightLanelet(lane, true, true)
                      : route_handler_->getLeftLanelet(lane, true, true);
    }();

    if (opt_adjacent_lane.has_value()) {
      connected_adjacent_lanes.push_back(opt_adjacent_lane.value());
    }

    if (!connected_adjacent_lanes.empty() && residual_distance > 0.0) {
      const auto detection_areas = utils::get_previous_polygons_with_lane_recursively(
        connected_adjacent_lanes, residual_distance,
        residual_distance + forward_distance + backward_distance, route_handler_);

      {
        debug.detection_areas.insert(
          debug.detection_areas.end(), detection_areas.begin(), detection_areas.end());
      }

      auto opt_pointcloud_object = utils::get_pointcloud_object(
        header_ptr_->stamp, pointcloud_, detection_areas, route_handler_);

      if (!opt_pointcloud_object.has_value()) {
        return objects;
      }

      opt_pointcloud_object.value().relative_distance =
        opt_pointcloud_object.value().absolute_distance - ego_to_furthest_point -
        std::abs(vehicle_info_.min_longitudinal_offset_m);

      if (history_.count(connected_adjacent_lanes.front().id()) == 0) {
        history_.emplace(connected_adjacent_lanes.front().id(), opt_pointcloud_object.value());
      } else {
        const auto previous_data = history_.at(connected_adjacent_lanes.front().id());
        const auto dx =
          previous_data.relative_distance - opt_pointcloud_object.value().relative_distance;
        const auto dt =
          (opt_pointcloud_object.value().last_update_time - previous_data.last_update_time)
            .seconds();

        if (dt > 1e-6) {
          opt_pointcloud_object.value().velocity =
            odometry_ptr_->twist.twist.linear.x +
            autoware::signal_processing::lowpassFilter(dx / dt, previous_data.velocity, 0.5);
        } else {
          opt_pointcloud_object.value().velocity = previous_data.velocity;
        }

        history_.at(connected_adjacent_lanes.front().id()) = opt_pointcloud_object.value();
      }

      objects.push_back(opt_pointcloud_object.value());

      return objects;
    }

    if (!connected_adjacent_lanes.empty() && !opt_adjacent_lane.has_value()) {
      const auto detection_areas = utils::get_previous_polygons_with_lane_recursively(
        connected_adjacent_lanes, 0.0,
        ego_to_furthest_point - current_lane_length + backward_distance, route_handler_);

      {
        debug.detection_areas.insert(
          debug.detection_areas.end(), detection_areas.begin(), detection_areas.end());
      }

      connected_adjacent_lanes.clear();

      auto opt_pointcloud_object = utils::get_pointcloud_object(
        header_ptr_->stamp, pointcloud_, detection_areas, route_handler_);

      if (!opt_pointcloud_object.has_value()) {
        continue;
      }

      opt_pointcloud_object.value().relative_distance =
        opt_pointcloud_object.value().absolute_distance - ego_to_furthest_point -
        std::abs(vehicle_info_.min_longitudinal_offset_m);

      if (history_.count(connected_adjacent_lanes.front().id()) == 0) {
        history_.emplace(connected_adjacent_lanes.front().id(), opt_pointcloud_object.value());
      } else {
        const auto previous_data = history_.at(connected_adjacent_lanes.front().id());
        const auto dx =
          previous_data.relative_distance - opt_pointcloud_object.value().relative_distance;
        const auto dt =
          (opt_pointcloud_object.value().last_update_time - previous_data.last_update_time)
            .seconds();

        if (dt > 1e-6) {
          opt_pointcloud_object.value().velocity =
            odometry_ptr_->twist.twist.linear.x +
            autoware::signal_processing::lowpassFilter(dx / dt, previous_data.velocity, 0.5);
        } else {
          opt_pointcloud_object.value().velocity = previous_data.velocity;
        }

        history_.at(connected_adjacent_lanes.front().id()) = opt_pointcloud_object.value();
      }

      objects.push_back(opt_pointcloud_object.value());
    }
  }

  return objects;
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
      "detection_lanes_for_objects", debug.get_detection_lanes(),
      autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.2)));
  }

  {
    add(
      utils::create_pointcloud_object_marker_array(debug.pointcloud_objects, "pointcloud_objects"));
    add(utils::createPointsMarkerArray(debug.obstacle_pointcloud, "obstacle_pointcloud"));
    add(utils::create_polygon_marker_array(
      debug.get_detection_polygons(), "detection_areas_for_pointcloud",
      autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.999)));
  }

  {
    // add(utils::showSafetyCheckInfo(debug.collision_check, "object_debug_info"));
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
