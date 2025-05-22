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
#include <limits>
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
  pub_obstacle_pointcloud_{
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/obstacle_pointcloud", 1)},
  pub_debug_marker_{this->create_publisher<MarkerArray>("~/debug/marker", 20)},
  pub_debug_processing_time_detail_{this->create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1)},
  pub_string_{this->create_publisher<StringStamped>("~/debug/state", 1)},
  route_handler_{std::make_shared<autoware::route_handler::RouteHandler>()},
  param_listener_{std::make_unique<rear_obstacle_checker_node::ParamListener>(
    this->get_node_parameters_interface())},
  diag_updater_{std::make_unique<diagnostic_updater::Updater>(this)},
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()},
  time_keeper_{std::make_shared<autoware_utils::TimeKeeper>(pub_debug_processing_time_detail_)}
{
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
    pointcloud_ptr_ = sub_pointcloud_.take_data();
  }

  // path
  {
    path_ptr_ = sub_path_.take_data();
  }

  // trajectory
  {
    trajectory_ptr_ = sub_trajectory_.take_data();
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

  if (!trajectory_ptr_) {
    return false;
  }

  if (!pointcloud_ptr_) {
    return false;
  }

  return true;
}

void RearObstacleCheckerNode::update(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto start_time = this->now();

  take_data();

  if (!is_ready()) {
    return;
  }

  DebugData debug_data;

  if (!is_safe(debug_data)) {
    RCLCPP_ERROR(get_logger(), "[ROC] Continuous collision risk detected.");
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "obstacles exist beside ego");
    debug_data.is_safe = false;
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "validated.");
    debug_data.is_safe = true;
  }

  post_process();

  debug_data.processing_time_detail_ms = (this->now() - start_time).seconds() * 1e3;

  publish_marker(debug_data);
}

void RearObstacleCheckerNode::on_timer()
{
  diag_updater_->force_update();
}

void RearObstacleCheckerNode::post_process()
{
  auto itr = history_.begin();
  while (itr != history_.end()) {
    if ((this->now() - itr->second.last_update_time).seconds() > 1.0) {
      itr = history_.erase(itr);
    } else {
      itr++;
    }
  }
}

bool RearObstacleCheckerNode::is_safe(DebugData & debug)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params();
  const auto forward_range =
    std::max(p.common.object.range.forward, p.common.pointcloud.range.forward);
  const auto backward_range =
    std::max(p.common.object.range.backward, p.common.pointcloud.range.backward);

  {
    const auto obstacle_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    obstacle_pointcloud->header.stamp = pointcloud_ptr_->header.stamp;
    obstacle_pointcloud->header.frame_id = "map";
    debug.obstacle_pointcloud = obstacle_pointcloud;
  }

  const auto resampled_path =
    behavior_path_planner::utils::resamplePathWithSpline(*path_ptr_, 1.0, true);
  const auto predicted_stop_pose = utils::calc_predicted_stop_pose(
    resampled_path.points, odometry_ptr_->pose.pose, odometry_ptr_->twist.twist.linear.x,
    acceleration_ptr_->accel.accel.linear.x, p);

  if (!predicted_stop_pose.has_value()) {
    return true;
  }

  const auto ego_footprint = utils::createFootprint(vehicle_info_);

  {
    const auto transform = autoware_utils::pose2transform(predicted_stop_pose.value());
    debug.predicted_stop_pose_footprint = utils::to_basic_polygon3d(
      autoware_utils::transform_vector(ego_footprint, transform),
      odometry_ptr_->pose.pose.position.z);
  }

  const auto current_lanes = utils::get_current_lanes(
    predicted_stop_pose.value(), route_handler_, vehicle_info_, ego_footprint, forward_range,
    backward_range);
  {
    debug.current_lanes = current_lanes;
  }

  if (current_lanes.empty()) {
    debug.text = "EGO CAN'T STOP WITHIN CURRENT LANE.";
  }

  const auto turn_behavior = utils::check_turn_behavior(
    current_lanes, trajectory_ptr_->points, odometry_ptr_->pose.pose, route_handler_, vehicle_info_,
    ego_footprint, p);

  const auto shift_behavior = utils::check_shift_behavior(
    current_lanes, trajectory_ptr_->points, odometry_ptr_->pose.pose, ego_footprint);

  {
    debug.turn_behavior = turn_behavior;
    debug.shift_behavior = shift_behavior;
  }

  if (turn_behavior == Behavior::NONE && shift_behavior == Behavior::NONE) {
    return true;
  }

  {
    debug.is_active = true;
  }

  PredictedObjects objects_on_target_lane;
  PointCloudObjects pointcloud_objects{};

  if (utils::should_check_objects(p)) {
    time_keeper_->start_track("prepare_detection_area_for_objects");
    const auto detection_lanes_for_objects =
      generate_detection_area_for_object(current_lanes, shift_behavior, turn_behavior);
    debug.detection_lanes_for_objects.insert(
      debug.detection_lanes_for_objects.end(), detection_lanes_for_objects.begin(),
      detection_lanes_for_objects.end());
    time_keeper_->end_track("prepare_detection_area_for_objects");

    time_keeper_->start_track("filter_objects");
    const auto [targets, others] =
      behavior_path_planner::utils::path_safety_checker::separateObjectsByLanelets(
        *object_ptr_, detection_lanes_for_objects,
        [&p](const auto & object, const auto & lane, const auto yaw_threshold = M_PI_2) {
          if (!utils::is_target(object, p)) {
            return false;
          }
          return behavior_path_planner::utils::path_safety_checker::isPolygonOverlapLanelet(
            object, lane, yaw_threshold);
        });

    objects_on_target_lane.objects.insert(
      objects_on_target_lane.objects.end(), targets.objects.begin(), targets.objects.end());

    time_keeper_->end_track("filter_objects");
  }

  if (utils::should_check_pointcloud(p)) {
    time_keeper_->start_track("pointcloud_base");
    pointcloud_objects =
      get_pointcloud_objects(current_lanes, shift_behavior, turn_behavior, debug);
    fill_rss_distance(pointcloud_objects);
    time_keeper_->end_track("pointcloud_base");
  }

  {
    const auto now = this->now();
    if (is_safe(objects_on_target_lane, debug) && is_safe(pointcloud_objects, debug)) {
      last_safe_time_ = now;
      if ((now - last_unsafe_time_).seconds() > p.common.off_time_buffer) {
        return true;
      }
    } else {
      RCLCPP_WARN(get_logger(), "[ROC] Momentary collision risk detected.");
      last_unsafe_time_ = now;
      if ((now - last_safe_time_).seconds() < p.common.on_time_buffer) {
        return true;
      }
    }

    {
      debug.text = "RISK OF COLLISION!!!";
    }
  }

  return false;
}

void RearObstacleCheckerNode::fill_rss_distance(PointCloudObjects & objects) const
{
  const auto p = param_listener_->get_params();

  const auto delay_object = p.common.object.reaction_time;
  const auto max_deceleration_ego = p.common.ego.max_deceleration;
  const auto max_deceleration_object = p.common.object.max_deceleration;
  const auto current_velocity = odometry_ptr_->twist.twist.linear.x;

  for (auto & object : objects) {
    const auto stop_distance_object =
      delay_object * object.velocity +
      0.5 * std::pow(object.velocity, 2.0) / std::abs(max_deceleration_object);
    const auto stop_distance_ego =
      0.5 * std::pow(current_velocity, 2.0) / std::abs(max_deceleration_ego);

    object.rss_distance = stop_distance_object - stop_distance_ego;
    object.safe = object.rss_distance < object.relative_distance_with_delay_compensation;
    object.ignore = object.velocity < p.common.filter.min_velocity;
  }
}

bool RearObstacleCheckerNode::is_safe(const PointCloudObjects & objects, DebugData & debug) const
{
  autoware_utils::ScopedTimeTrack st("is_safe_pointcloud", *time_keeper_);

  const auto p = param_listener_->get_params();

  {
    debug.pointcloud_objects = objects;
  }

  for (const auto & object : objects) {
    if (object.tracking_duration < p.common.pointcloud.velocity_estimation.observation_time) {
      continue;
    }

    if (object.ignore) {
      continue;
    }

    if (object.safe) {
      continue;
    }

    return false;
  }

  return true;
}

bool RearObstacleCheckerNode::is_safe(const PredictedObjects & objects, DebugData & debug) const
{
  autoware_utils::ScopedTimeTrack st("is_safe_objects", *time_keeper_);

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

auto RearObstacleCheckerNode::filter_pointcloud([[maybe_unused]] DebugData & debug) const
  -> PointCloud::Ptr
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params().common.pointcloud;

  auto output = std::make_shared<PointCloud>();

  if (pointcloud_ptr_->data.empty()) {
    return output;
  }

  {
    pcl::fromROSMsg(*pointcloud_ptr_, *output);
    debug.pointcloud_nums.push_back(output->size());
  }

  {
    autoware_utils::ScopedTimeTrack st("crop_x", *time_keeper_);

    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(output);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(p.crop_box_filter.x.min, p.crop_box_filter.x.max);
    filter.filter(*output);
    debug.pointcloud_nums.push_back(output->size());
  }

  if (output->empty()) {
    return output;
  }

  {
    autoware_utils::ScopedTimeTrack st("crop_z", *time_keeper_);

    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(output);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(
      p.crop_box_filter.z.min, vehicle_info_.vehicle_height_m + p.crop_box_filter.z.max);
    filter.filter(*output);
    debug.pointcloud_nums.push_back(output->size());
  }

  if (output->empty()) {
    return output;
  }

  {
    autoware_utils::ScopedTimeTrack st("transform", *time_keeper_);

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        "map", pointcloud_ptr_->header.frame_id, pointcloud_ptr_->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "no transform found for no_ground_pointcloud: %s", e.what());
    }

    Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
    autoware_utils::transform_pointcloud(*output, *output, isometry);
    debug.pointcloud_nums.push_back(output->size());
  }

  {
    autoware_utils::ScopedTimeTrack st("voxel_grid_filter", *time_keeper_);

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(output);
    filter.setLeafSize(p.voxel_grid_filter.x, p.voxel_grid_filter.y, p.voxel_grid_filter.z);
    filter.filter(*output);
    debug.pointcloud_nums.push_back(output->size());
  }

  return output;
}

auto RearObstacleCheckerNode::get_clustered_pointcloud(
  const PointCloud::Ptr in, DebugData & debug) const -> PointCloud::Ptr
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params().common.pointcloud;

  const auto points_belonging_to_cluster_hulls = std::make_shared<PointCloud>();
  // eliminate noisy points by only considering points belonging to clusters of at least a certain
  // size
  if (in->empty()) return std::make_shared<PointCloud>();
  const std::vector<pcl::PointIndices> cluster_indices = std::invoke([&]() {
    std::vector<pcl::PointIndices> cluster_idx;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(in);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(p.clustering.cluster_tolerance);
    ec.setMinClusterSize(p.clustering.min_cluster_size);
    ec.setMaxClusterSize(p.clustering.max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in);
    ec.extract(cluster_idx);
    return cluster_idx;
  });
  for (const auto & indices : cluster_indices) {
    PointCloud::Ptr cluster(new PointCloud);
    bool cluster_surpasses_threshold_height{false};
    for (const auto & index : indices.indices) {
      const auto & point = (*in)[index];
      cluster_surpasses_threshold_height = (cluster_surpasses_threshold_height)
                                             ? cluster_surpasses_threshold_height
                                             : (point.z > p.clustering.min_cluster_height);
      cluster->push_back(point);
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
      const auto point = autoware_utils::Point3d{p.x, p.y, p.z};
      boost::geometry::append(hull_polygon.outer(), point);
    }
    debug.hull_polygons.push_back(hull_polygon);
  }

  {
    const auto obstacle_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*points_belonging_to_cluster_hulls, *obstacle_pointcloud);
    obstacle_pointcloud->header.stamp = pointcloud_ptr_->header.stamp;
    obstacle_pointcloud->header.frame_id = "map";
    debug.obstacle_pointcloud = obstacle_pointcloud;
  }

  return points_belonging_to_cluster_hulls;
}

auto RearObstacleCheckerNode::generate_detection_area_for_object(
  const lanelet::ConstLanelets & current_lanes, const Behavior & shift_behavior,
  const Behavior & turn_behavior) const -> lanelet::ConstLanelets
{
  const auto p = param_listener_->get_params();

  lanelet::ConstLanelets detection_lanes_for_objects{};

  if (shift_behavior == Behavior::SHIFT_LEFT) {
    const auto adjacent_lanes = utils::get_adjacent_lanes(
      current_lanes, odometry_ptr_->pose.pose, route_handler_, false,
      p.common.object.range.backward);
    detection_lanes_for_objects.insert(
      detection_lanes_for_objects.end(), adjacent_lanes.begin(), adjacent_lanes.end());
  }

  if (shift_behavior == Behavior::SHIFT_RIGHT) {
    const auto adjacent_lanes = utils::get_adjacent_lanes(
      current_lanes, odometry_ptr_->pose.pose, route_handler_, true,
      p.common.object.range.backward);
    detection_lanes_for_objects.insert(
      detection_lanes_for_objects.end(), adjacent_lanes.begin(), adjacent_lanes.end());
  }

  if (turn_behavior == Behavior::TURN_LEFT) {
    const auto half_lanes = [&current_lanes, &p, this]() {
      lanelet::ConstLanelets ret{};
      for (const auto & lane : current_lanes) {
        ret.push_back(utils::generate_half_lanelet(
          lane, false, 0.5 * vehicle_info_.vehicle_width_m + p.common.blind_spot.offset.inner,
          p.common.blind_spot.offset.outer));
      }
      return ret;
    }();
    detection_lanes_for_objects.insert(
      detection_lanes_for_objects.end(), half_lanes.begin(), half_lanes.end());
  }

  if (turn_behavior == Behavior::TURN_RIGHT) {
    const auto half_lanes = [&current_lanes, &p, this]() {
      lanelet::ConstLanelets ret{};
      for (const auto & lane : current_lanes) {
        ret.push_back(utils::generate_half_lanelet(
          lane, true, 0.5 * vehicle_info_.vehicle_width_m + p.common.blind_spot.offset.inner,
          p.common.blind_spot.offset.outer));
      }
      return ret;
    }();
    detection_lanes_for_objects.insert(
      detection_lanes_for_objects.end(), half_lanes.begin(), half_lanes.end());
  }

  return detection_lanes_for_objects;
}

auto RearObstacleCheckerNode::get_pointcloud_objects_on_adjacent_lane(
  const lanelet::ConstLanelets & current_lanes, const Behavior & shift_behavior,
  const double forward_distance, const double backward_distance,
  const PointCloud::Ptr & obstacle_pointcloud, DebugData & debug) -> PointCloudObjects
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params();

  PointCloudObjects objects{};

  if (shift_behavior == Behavior::NONE) {
    return objects;
  }

  const auto ego_coordinate_on_arc =
    lanelet::utils::getArcCoordinates(current_lanes, odometry_ptr_->pose.pose);

  lanelet::ConstLanelets connected_adjacent_lanes{};

  double length = 0.0;
  for (const auto & lane : current_lanes) {
    const auto current_lane_length = lanelet::utils::getLaneletLength2d(lane);

    length += current_lane_length;

    const auto ego_to_furthest_point = length - ego_coordinate_on_arc.length;
    const auto residual_distance = ego_to_furthest_point - forward_distance;

    const auto opt_adjacent_lane = [&lane, &shift_behavior, this]() {
      const auto is_right = shift_behavior == Behavior::SHIFT_RIGHT;
      return is_right ? route_handler_->getRightLanelet(lane, true, true)
                      : route_handler_->getLeftLanelet(lane, true, true);
    }();

    if (opt_adjacent_lane.has_value()) {
      connected_adjacent_lanes.push_back(opt_adjacent_lane.value());
    }

    if (!connected_adjacent_lanes.empty() && residual_distance > 0.0) {
      const auto detection_areas = utils::get_previous_polygons_with_lane_recursively(
        connected_adjacent_lanes, residual_distance,
        residual_distance + forward_distance + backward_distance, route_handler_,
        p.common.adjacent_lane.offset.left, p.common.adjacent_lane.offset.right);

      {
        debug.detection_areas.insert(
          debug.detection_areas.end(), detection_areas.begin(), detection_areas.end());
      }

      time_keeper_->start_track("get_pointcloud_object");
      auto opt_pointcloud_object = get_pointcloud_object(
        pointcloud_ptr_->header.stamp, obstacle_pointcloud, detection_areas, debug);
      time_keeper_->end_track("get_pointcloud_object");

      if (!opt_pointcloud_object.has_value()) {
        return objects;
      }

      opt_pointcloud_object.value().relative_distance =
        opt_pointcloud_object.value().absolute_distance - ego_to_furthest_point -
        std::abs(vehicle_info_.min_longitudinal_offset_m);

      if (
        opt_pointcloud_object.value().relative_distance <
        p.common.pointcloud.range.dead_zone - vehicle_info_.max_longitudinal_offset_m) {
        return objects;
      }

      fill_velocity(opt_pointcloud_object.value());

      objects.push_back(opt_pointcloud_object.value());

      return objects;
    }

    if (!connected_adjacent_lanes.empty() && !opt_adjacent_lane.has_value()) {
      const auto detection_areas = utils::get_previous_polygons_with_lane_recursively(
        connected_adjacent_lanes, 0.0,
        ego_to_furthest_point - current_lane_length + backward_distance, route_handler_,
        p.common.adjacent_lane.offset.left, p.common.adjacent_lane.offset.right);

      {
        debug.detection_areas.insert(
          debug.detection_areas.end(), detection_areas.begin(), detection_areas.end());
      }

      connected_adjacent_lanes.clear();

      time_keeper_->start_track("get_pointcloud_object");
      auto opt_pointcloud_object = get_pointcloud_object(
        pointcloud_ptr_->header.stamp, obstacle_pointcloud, detection_areas, debug);
      time_keeper_->end_track("get_pointcloud_object");

      if (!opt_pointcloud_object.has_value()) {
        continue;
      }

      opt_pointcloud_object.value().relative_distance =
        opt_pointcloud_object.value().absolute_distance - ego_to_furthest_point -
        std::abs(vehicle_info_.min_longitudinal_offset_m);

      if (
        opt_pointcloud_object.value().relative_distance <
        p.common.pointcloud.range.dead_zone - vehicle_info_.max_longitudinal_offset_m) {
        return objects;
      }

      fill_velocity(opt_pointcloud_object.value());

      objects.push_back(opt_pointcloud_object.value());
    }
  }

  return objects;
}

void RearObstacleCheckerNode::fill_velocity(PointCloudObject & pointcloud_object)
{
  const auto p = param_listener_->get_params();

  const auto update_history = [this](const auto & pointcloud_object) {
    if (history_.count(pointcloud_object.furthest_lane.id()) == 0) {
      history_.emplace(pointcloud_object.furthest_lane.id(), pointcloud_object);
    } else {
      history_.at(pointcloud_object.furthest_lane.id()) = pointcloud_object;
    }
  };

  const auto fill_velocity = [&p, this](auto & pointcloud_object, const auto & previous_data) {
    const auto dx = previous_data.relative_distance - pointcloud_object.relative_distance;
    const auto dt = (pointcloud_object.last_update_time - previous_data.last_update_time).seconds();

    if (dt < 1e-6) {
      pointcloud_object.velocity = previous_data.velocity;
      pointcloud_object.tracking_duration = previous_data.tracking_duration + dt;
      pointcloud_object.relative_distance_with_delay_compensation =
        pointcloud_object.relative_distance -
        pointcloud_object.velocity * p.common.pointcloud.latency;
      return;
    }

    constexpr double assumed_acceleration = 30.0;
    const auto raw_velocity = dx / dt + odometry_ptr_->twist.twist.linear.x;
    const auto is_reliable =
      previous_data.tracking_duration > p.common.pointcloud.velocity_estimation.observation_time;

    if (
      is_reliable && std::abs(raw_velocity - previous_data.velocity) / dt > assumed_acceleration) {
      // closest point may jumped. reset tracking history.
      pointcloud_object.velocity = 0.0;
      pointcloud_object.tracking_duration = 0.0;
    } else {
      // keep tracking.
      pointcloud_object.velocity =
        autoware::signal_processing::lowpassFilter(raw_velocity, previous_data.velocity, 0.5);
      pointcloud_object.tracking_duration = previous_data.tracking_duration + dt;
    }

    pointcloud_object.relative_distance_with_delay_compensation =
      pointcloud_object.relative_distance -
      pointcloud_object.velocity * p.common.pointcloud.latency;
  };

  if (history_.count(pointcloud_object.furthest_lane.id()) == 0) {
    const auto previous_lanes =
      route_handler_->getPreviousLanelets(pointcloud_object.furthest_lane);
    for (const auto & previous_lane : previous_lanes) {
      if (history_.count(previous_lane.id()) != 0) {
        fill_velocity(pointcloud_object, history_.at(previous_lane.id()));
        update_history(pointcloud_object);
        return;
      }
    }

    update_history(pointcloud_object);
    return;
  }

  fill_velocity(pointcloud_object, history_.at(pointcloud_object.furthest_lane.id()));
  update_history(pointcloud_object);
}

auto RearObstacleCheckerNode::get_pointcloud_objects_at_blind_spot(
  const lanelet::ConstLanelets & current_lanes, const Behavior & turn_behavior,
  const double forward_distance, const double backward_distance,
  const PointCloud::Ptr & obstacle_pointcloud, DebugData & debug) -> PointCloudObjects
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params();

  PointCloudObjects objects{};

  if (turn_behavior == Behavior::NONE) {
    return objects;
  }

  const auto half_lanes = [&current_lanes, &turn_behavior, &p, this]() {
    const auto is_right = turn_behavior == Behavior::TURN_RIGHT;
    lanelet::ConstLanelets ret{};
    for (const auto & lane : current_lanes) {
      ret.push_back(utils::generate_half_lanelet(
        lane, is_right, 0.5 * vehicle_info_.vehicle_width_m + p.common.blind_spot.offset.inner,
        p.common.blind_spot.offset.outer));
    }
    return ret;
  }();
  const auto detection_polygon = utils::generate_detection_polygon(
    half_lanes, odometry_ptr_->pose.pose, forward_distance, backward_distance);

  DetectionAreas detection_areas{};
  detection_areas.emplace_back(detection_polygon, half_lanes);

  {
    debug.detection_areas.insert(
      debug.detection_areas.end(), detection_areas.begin(), detection_areas.end());
  }

  time_keeper_->start_track("get_pointcloud_object");
  auto opt_pointcloud_object = get_pointcloud_object(
    pointcloud_ptr_->header.stamp, obstacle_pointcloud, detection_areas, debug);
  time_keeper_->end_track("get_pointcloud_object");

  if (!opt_pointcloud_object.has_value()) {
    return objects;
  }

  const auto ego_coordinate_on_arc =
    lanelet::utils::getArcCoordinates(current_lanes, odometry_ptr_->pose.pose);

  const auto ego_to_furthest_point =
    lanelet::utils::getLaneletLength2d(half_lanes) - ego_coordinate_on_arc.length;

  opt_pointcloud_object.value().relative_distance =
    opt_pointcloud_object.value().absolute_distance - ego_to_furthest_point -
    std::abs(vehicle_info_.min_longitudinal_offset_m);

  if (
    opt_pointcloud_object.value().relative_distance <
    p.common.pointcloud.range.dead_zone - vehicle_info_.max_longitudinal_offset_m) {
    return objects;
  }

  fill_velocity(opt_pointcloud_object.value());

  objects.push_back(opt_pointcloud_object.value());

  return objects;
}

auto RearObstacleCheckerNode::get_pointcloud_object(
  const rclcpp::Time & now, const PointCloud::Ptr & pointcloud_ptr,
  const DetectionAreas & detection_areas, DebugData & debug) -> std::optional<PointCloudObject>
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  std::optional<PointCloudObject> opt_object = std::nullopt;
  for (const auto & [polygon, lanes] : detection_areas) {
    const auto pointcloud =
      *get_clustered_pointcloud(utils::get_obstacle_points({polygon}, *pointcloud_ptr), debug);

    const auto path =
      route_handler_->getCenterLinePath(lanes, 0.0, std::numeric_limits<double>::max());
    const auto resampled_path = behavior_path_planner::utils::resamplePathWithSpline(path, 2.0);

    for (const auto & point : pointcloud) {
      const auto p_geom = autoware_utils::create_point(point.x, point.y, point.z);
      const size_t src_seg_idx =
        autoware::motion_utils::findNearestSegmentIndex(resampled_path.points, p_geom);
      const double signed_length_src_offset =
        autoware::motion_utils::calcLongitudinalOffsetToSegment(
          resampled_path.points, src_seg_idx, p_geom);

      const double obj_arc_length =
        autoware::motion_utils::calcSignedArcLength(
          resampled_path.points, src_seg_idx, resampled_path.points.size() - 1) -
        signed_length_src_offset;
      const auto pose_on_center_line = autoware::motion_utils::calcLongitudinalOffsetPose(
        resampled_path.points, src_seg_idx, signed_length_src_offset);

      if (!pose_on_center_line.has_value()) {
        continue;
      }

      if (!opt_object.has_value()) {
        PointCloudObject object;
        object.last_update_time = now;
        object.pose = pose_on_center_line.value();
        object.furthest_lane = lanes.back();
        object.tracking_duration = 0.0;
        object.absolute_distance = obj_arc_length;
        object.velocity = 0.0;
        opt_object = object;
      } else if (opt_object.value().absolute_distance > obj_arc_length) {
        opt_object.value().last_update_time = now;
        opt_object.value().pose = pose_on_center_line.value();
        opt_object.value().furthest_lane = lanes.back();
        opt_object.value().tracking_duration = 0.0;
        opt_object.value().absolute_distance = obj_arc_length;
        opt_object.value().velocity = 0.0;
      }
    }
  }

  return opt_object;
}

auto RearObstacleCheckerNode::get_pointcloud_objects(
  const lanelet::ConstLanelets & current_lanes, const Behavior & shift_behavior,
  const Behavior & turn_behavior, DebugData & debug) -> PointCloudObjects
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params();

  PointCloudObjects objects{};

  const auto obstacle_pointcloud = filter_pointcloud(debug);

  const auto max_deceleration_ego = p.common.ego.max_deceleration;
  const auto current_velocity = odometry_ptr_->twist.twist.linear.x;

  {
    const auto delay_object = p.common.vru.reaction_time;
    const auto max_deceleration_object = p.common.vru.max_deceleration;

    const auto stop_distance_object =
      delay_object * p.common.vru.max_velocity +
      0.5 * std::pow(p.common.vru.max_velocity, 2.0) / std::abs(max_deceleration_object);
    const auto stop_distance_ego =
      0.5 * std::pow(current_velocity, 2.0) / std::abs(max_deceleration_ego);

    const auto forward_distance =
      p.common.pointcloud.range.forward + vehicle_info_.max_longitudinal_offset_m;
    const auto backward_distance = p.common.pointcloud.range.backward -
                                   vehicle_info_.min_longitudinal_offset_m +
                                   std::max(0.0, stop_distance_object - stop_distance_ego);

    const auto objects_at_blind_spot = get_pointcloud_objects_at_blind_spot(
      current_lanes, turn_behavior, forward_distance, backward_distance, obstacle_pointcloud,
      debug);
    objects.insert(objects.end(), objects_at_blind_spot.begin(), objects_at_blind_spot.end());
  }

  {
    const auto delay_object = p.common.vehicle.reaction_time;
    const auto max_deceleration_object = p.common.vehicle.max_deceleration;

    const auto stop_distance_object =
      delay_object * p.common.vehicle.max_velocity +
      0.5 * std::pow(p.common.vehicle.max_velocity, 2.0) / std::abs(max_deceleration_object);
    const auto stop_distance_ego =
      0.5 * std::pow(current_velocity, 2.0) / std::abs(max_deceleration_ego);

    const auto forward_distance =
      p.common.pointcloud.range.forward + vehicle_info_.max_longitudinal_offset_m;
    const auto backward_distance = p.common.pointcloud.range.backward -
                                   vehicle_info_.min_longitudinal_offset_m +
                                   std::max(0.0, stop_distance_object - stop_distance_ego);

    const auto objects_on_adjacent_lane = get_pointcloud_objects_on_adjacent_lane(
      current_lanes, shift_behavior, forward_distance, backward_distance, obstacle_pointcloud,
      debug);
    objects.insert(objects.end(), objects_on_adjacent_lane.begin(), objects_on_adjacent_lane.end());
  }

  return objects;
}

void RearObstacleCheckerNode::publish_marker(const DebugData & debug) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

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
    add(lanelet::visualization::laneletsAsTriangleMarkerArray(
      "current_lanes", debug.current_lanes,
      autoware_utils::create_marker_color(0.16, 1.0, 0.69, 0.2)));
  }

  {
    add(
      utils::create_pointcloud_object_marker_array(debug.pointcloud_objects, "pointcloud_objects"));
    // add(utils::createPointsMarkerArray(debug.obstacle_pointcloud, "obstacle_pointcloud"));
    add(utils::create_polygon_marker_array(
      debug.get_detection_polygons(), "detection_areas_for_pointcloud",
      autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.999)));
    add(utils::create_polygon_marker_array(
      debug.hull_polygons, "hull_polygons",
      autoware_utils::create_marker_color(0.0, 0.0, 1.0, 0.999)));
  }

  {
    // add(utils::showSafetyCheckInfo(debug.collision_check, "object_debug_info"));
    add(utils::showPredictedPath(debug.collision_check, "ego_predicted_path"));
    add(utils::showPolygon(debug.collision_check, "ego_and_target_polygon_relation"));
  }

  std::for_each(msg.markers.begin(), msg.markers.end(), [](auto & marker) {
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  });

  pub_debug_marker_->publish(msg);

  if (debug.obstacle_pointcloud) {
    pub_obstacle_pointcloud_->publish(*debug.obstacle_pointcloud);
  }

  {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << std::boolalpha;
    ss << "ACTIVE:" << debug.is_active << "\n";
    ss << "SAFE:" << debug.is_safe << "\n";
    ss << "TURN:" << magic_enum::enum_name(debug.turn_behavior) << "\n";
    ss << "SHIFT:" << magic_enum::enum_name(debug.shift_behavior) << "\n";
    ss << "INFO:" << debug.text.c_str() << "\n";
    ss << "TRACKING OBJECTS:" << debug.pointcloud_objects.size() << "\n";
    ss << "PC NUM:";
    for (const auto num : debug.pointcloud_nums) {
      ss << num << "->";
    }
    ss << "\n";
    ss << "PROCESSING TIME:" << debug.processing_time_detail_ms << "[ms]\n";

    StringStamped string_stamp;
    string_stamp.stamp = this->now();
    string_stamp.data = ss.str();
    pub_string_->publish(string_stamp);
  }
}
}  // namespace autoware::rear_obstacle_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::rear_obstacle_checker::RearObstacleCheckerNode)
