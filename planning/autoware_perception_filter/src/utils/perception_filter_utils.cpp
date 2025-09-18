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

#include "autoware/perception_filter/perception_filter_utils.hpp"

#include "autoware/perception_filter/perception_filter_core.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/transform_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::perception_filter
{

autoware_internal_planning_msgs::msg::PlanningFactorArray createPlanningFactors(
  const ObjectClassification & classification,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & would_be_filtered_point_cloud,
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & planning_trajectory)
{
  autoware_internal_planning_msgs::msg::PlanningFactorArray planning_factors;

  // Set header with current timestamp
  planning_factors.header.stamp = rclcpp::Clock().now();
  planning_factors.header.frame_id = "map";

  // Get UUIDs of objects that pass through now but would be filtered if RTC approved
  std::vector<unique_identifier_msgs::msg::UUID> objects_to_be_filtered;
  for (const auto & object : classification.would_be_removed) {
    objects_to_be_filtered.push_back(object.object_id);
  }

  // Create PlanningFactor when there are objects that would be filtered when RTC is approved
  // OR when there are points that would be filtered when RTC is approved
  const bool has_pointcloud =
    would_be_filtered_point_cloud && !would_be_filtered_point_cloud->data.empty();
  if (!objects_to_be_filtered.empty() || has_pointcloud) {
    autoware_internal_planning_msgs::msg::PlanningFactor factor;
    factor.module = "supervised_perception_filter";
    factor.behavior = autoware_internal_planning_msgs::msg::PlanningFactor::STOP;

    // Set detail message based on what is being filtered
    const bool has_objects = !objects_to_be_filtered.empty();

    if (has_objects && has_pointcloud) {
      factor.detail = "Objects and pointcloud that would be filtered when RTC is approved";
    } else if (has_objects) {
      factor.detail = "Objects that would be filtered when RTC is approved";
    } else if (has_pointcloud) {
      factor.detail = "Pointcloud that would be filtered when RTC is approved";
    }

    // Add control point (nearest point on path)
    if (planning_trajectory && !planning_trajectory->points.empty()) {
      autoware_internal_planning_msgs::msg::ControlPoint control_point;
      control_point.pose = planning_trajectory->points.front().pose;
      control_point.distance = 0.0;
      factor.control_points.push_back(control_point);
    }

    // Add safety factors
    factor.safety_factors.is_safe = false;

    // Add object safety factors
    for (const auto & uuid : objects_to_be_filtered) {
      autoware_internal_planning_msgs::msg::SafetyFactor safety_factor;
      safety_factor.type = autoware_internal_planning_msgs::msg::SafetyFactor::OBJECT;
      safety_factor.is_safe = false;
      safety_factor.object_id = uuid;

      // Add object position
      for (const auto & object : classification.would_be_removed) {
        if (object.object_id.uuid == uuid.uuid) {
          geometry_msgs::msg::Point point;
          point.x = object.kinematics.initial_pose_with_covariance.pose.position.x;
          point.y = object.kinematics.initial_pose_with_covariance.pose.position.y;
          point.z = object.kinematics.initial_pose_with_covariance.pose.position.z;
          safety_factor.points.push_back(point);
          break;
        }
      }

      factor.safety_factors.factors.push_back(safety_factor);
    }

    // Add pointcloud safety factors (only points that would be filtered when RTC is approved)
    if (has_pointcloud) {
      autoware_internal_planning_msgs::msg::SafetyFactor pointcloud_safety_factor;
      pointcloud_safety_factor.type =
        autoware_internal_planning_msgs::msg::SafetyFactor::POINTCLOUD;
      pointcloud_safety_factor.is_safe = false;

      // Convert PointCloud2 to PCL and add point positions
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*would_be_filtered_point_cloud, *pcl_cloud);

      // Add would-be-filtered points positions
      for (const auto & point : pcl_cloud->points) {
        geometry_msgs::msg::Point ros_point;
        ros_point.x = point.x;
        ros_point.y = point.y;
        ros_point.z = point.z;
        pointcloud_safety_factor.points.push_back(ros_point);
      }

      factor.safety_factors.factors.push_back(pointcloud_safety_factor);
    }

    planning_factors.factors.push_back(factor);
  }

  return planning_factors;
}

double getMinDistanceToPath(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const autoware_planning_msgs::msg::Trajectory & path)
{
  if (path.points.empty()) {
    return std::numeric_limits<double>::max();
  }

  // Convert object to polygon using autoware_universe_utils
  const auto object_polygon = autoware::universe_utils::toPolygon2d(object);

  // Convert trajectory to linestring for distance calculation
  autoware::universe_utils::LineString2d trajectory_line;
  for (const auto & point : path.points) {
    autoware::universe_utils::Point2d bg_point;
    bg_point.x() = point.pose.position.x;
    bg_point.y() = point.pose.position.y;
    trajectory_line.push_back(bg_point);
  }

  // Calculate minimum distance between object polygon and trajectory line using boost::geometry
  double min_distance = std::numeric_limits<double>::max();
  for (const auto & vertex : object_polygon.outer()) {
    const double distance = boost::geometry::distance(vertex, trajectory_line);
    min_distance = std::min(min_distance, distance);
  }

  return min_distance;
}

bool shouldIgnoreObject(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const std::vector<std::string> & ignore_object_classes)
{
  if (ignore_object_classes.empty()) {
    return false;  // No classes to ignore
  }

  const uint8_t object_label = getMostProbableLabel(object);
  const std::string label_string = labelToString(object_label);

  // Check if the object's label is in the ignore list
  return std::find(ignore_object_classes.begin(), ignore_object_classes.end(), label_string) !=
         ignore_object_classes.end();
}

uint8_t getMostProbableLabel(const autoware_perception_msgs::msg::PredictedObject & object)
{
  if (object.classification.empty()) {
    return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  }

  // Find the classification with the highest probability
  double highest_probability = 0.0;
  uint8_t most_probable_label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;

  for (const auto & classification : object.classification) {
    if (classification.probability > highest_probability) {
      highest_probability = classification.probability;
      most_probable_label = classification.label;
    }
  }

  return most_probable_label;
}

std::string labelToString(uint8_t label)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  switch (label) {
    case ObjectClassification::UNKNOWN:
      return "UNKNOWN";
    case ObjectClassification::CAR:
      return "CAR";
    case ObjectClassification::TRUCK:
      return "TRUCK";
    case ObjectClassification::BUS:
      return "BUS";
    case ObjectClassification::TRAILER:
      return "TRAILER";
    case ObjectClassification::MOTORCYCLE:
      return "MOTORCYCLE";
    case ObjectClassification::BICYCLE:
      return "BICYCLE";
    case ObjectClassification::PEDESTRIAN:
      return "PEDESTRIAN";
    default:
      return "UNKNOWN";
  }
}

uint8_t stringToLabel(const std::string & label_string)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  if (label_string == "UNKNOWN") return ObjectClassification::UNKNOWN;
  if (label_string == "CAR") return ObjectClassification::CAR;
  if (label_string == "TRUCK") return ObjectClassification::TRUCK;
  if (label_string == "BUS") return ObjectClassification::BUS;
  if (label_string == "TRAILER") return ObjectClassification::TRAILER;
  if (label_string == "MOTORCYCLE") return ObjectClassification::MOTORCYCLE;
  if (label_string == "BICYCLE") return ObjectClassification::BICYCLE;
  if (label_string == "PEDESTRIAN") return ObjectClassification::PEDESTRIAN;
  return ObjectClassification::UNKNOWN;
}

std::optional<geometry_msgs::msg::Pose> getEgoPose(const tf2_ros::Buffer & tf_buffer)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer.lookupTransform(
      "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    return std::optional<geometry_msgs::msg::Pose>{};
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.transform.translation.x;
  pose.position.y = transform.transform.translation.y;
  pose.position.z = transform.transform.translation.z;
  pose.orientation = transform.transform.rotation;

  return std::optional<geometry_msgs::msg::Pose>{pose};
}

std::vector<autoware::universe_utils::Polygon2d> createTrajectoryPolygons(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & traj_points,
  const double lat_margin)
{
  std::vector<autoware::universe_utils::Polygon2d> output_polygons;

  if (traj_points.empty()) {
    return output_polygons;
  }

  autoware::universe_utils::Polygon2d accumulated_polygon;

  for (size_t i = 0; i < traj_points.size(); ++i) {
    const auto & pose = traj_points.at(i).pose;

    // Create left and right offset points from trajectory point
    const auto left_point =
      autoware::universe_utils::calcOffsetPose(pose, 0.0, lat_margin, 0.0).position;
    const auto right_point =
      autoware::universe_utils::calcOffsetPose(pose, 0.0, -lat_margin, 0.0).position;

    // Create current step polygon (rectangle-like shape)
    autoware::universe_utils::Polygon2d current_polygon;
    boost::geometry::append(
      current_polygon, autoware::universe_utils::Point2d(left_point.x, left_point.y));
    boost::geometry::append(
      current_polygon, autoware::universe_utils::Point2d(right_point.x, right_point.y));
    boost::geometry::append(
      current_polygon,
      autoware::universe_utils::Point2d(left_point.x, left_point.y));  // Close polygon

    // Add current polygon points to accumulated polygon
    for (const auto & point : current_polygon.outer()) {
      boost::geometry::append(accumulated_polygon, point);
    }

    // Calculate convex hull of accumulated polygon
    autoware::universe_utils::Polygon2d hull_polygon;
    boost::geometry::convex_hull(accumulated_polygon, hull_polygon);
    boost::geometry::correct(hull_polygon);

    output_polygons.push_back(hull_polygon);

    // Update accumulated polygon for next iteration (keep only current polygon for next step)
    accumulated_polygon = std::move(current_polygon);
  }

  return output_polygons;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filterByTrajectoryPolygonsCropBox(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const std::vector<autoware::universe_utils::Polygon2d> & traj_polygons,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
  const std::vector<autoware::universe_utils::Polygon2d> & crop_box_polygons,
  const double height_margin, const bool keep_inside)
{
  if (traj_polygons.empty() || trajectory_points.empty() || crop_box_polygons.empty()) {
    return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }

  // Use pre-calculated crop box polygon for XY bounds
  const auto & crop_box = crop_box_polygons.front();
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  for (const auto & point : crop_box.outer()) {
    x_min = std::min(x_min, point.x());
    x_max = std::max(x_max, point.x());
    y_min = std::min(y_min, point.y());
    y_max = std::max(y_max, point.y());
  }

  // Calculate Z bounds from trajectory points
  double lowest_traj_height = std::numeric_limits<double>::max();
  double highest_traj_height = std::numeric_limits<double>::lowest();
  for (const auto & trajectory_point : trajectory_points) {
    lowest_traj_height = std::min(lowest_traj_height, trajectory_point.pose.position.z);
    highest_traj_height = std::max(highest_traj_height, trajectory_point.pose.position.z);
  }

  // Apply filter with keep_inside control
  auto filtered_pointcloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto & point : input_pointcloud_ptr->points) {
    // Check if point is inside crop box bounds
    bool is_inside_crop_box =
      (point.x >= x_min && point.x <= x_max && point.y >= y_min && point.y <= y_max &&
       point.z >= (lowest_traj_height - height_margin) &&
       point.z <= (highest_traj_height + height_margin));

    // Keep points based on keep_inside parameter
    if ((keep_inside && is_inside_crop_box) || (!keep_inside && !is_inside_crop_box)) {
      filtered_pointcloud_ptr->points.push_back(point);
    }
  }

  filtered_pointcloud_ptr->width = filtered_pointcloud_ptr->points.size();
  filtered_pointcloud_ptr->height = 1;
  filtered_pointcloud_ptr->is_dense = true;

  return filtered_pointcloud_ptr;
}

std::optional<Eigen::Matrix4d> getEigenTransform(
  const std::shared_ptr<autoware::universe_utils::TransformListener> & transform_listener,
  const std::string & target_frame, const std::string & source_frame)
{
  // Get transform between frames
  const auto transform_opt = transform_listener->getTransform(
    target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  if (!transform_opt) {
    std::cerr << "Failed to get transform from " << source_frame << " to " << target_frame
              << std::endl;
    return std::nullopt;
  }

  const auto eigen_transform = tf2::transformToEigen(transform_opt->transform);
  return std::make_optional(eigen_transform.matrix());
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> transformTrajectoryToBaseLink(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & map_trajectory_points,
  const std::shared_ptr<autoware::universe_utils::TransformListener> & transform_listener)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> base_link_trajectory_points;

  if (map_trajectory_points.empty()) {
    return base_link_trajectory_points;
  }

  // Get transform from map to base_link using common helper
  const auto transform_matrix_opt = getEigenTransform(transform_listener, "base_link", "map");
  if (!transform_matrix_opt) {
    return base_link_trajectory_points;
  }

  const Eigen::Matrix4d eigen_transform = transform_matrix_opt.value();

  // Transform trajectory points to base_link frame
  for (const auto & map_point : map_trajectory_points) {
    autoware_planning_msgs::msg::TrajectoryPoint base_link_point = map_point;

    // Transform position
    Eigen::Vector3d map_pos_3d(
      map_point.pose.position.x, map_point.pose.position.y, map_point.pose.position.z);
    Eigen::Vector3d base_link_pos_3d =
      eigen_transform.block<3, 3>(0, 0) * map_pos_3d + eigen_transform.block<3, 1>(0, 3);

    base_link_point.pose.position.x = base_link_pos_3d.x();
    base_link_point.pose.position.y = base_link_pos_3d.y();
    base_link_point.pose.position.z = base_link_pos_3d.z();

    // Transform orientation (quaternion)
    Eigen::Quaterniond map_quat(
      map_point.pose.orientation.w, map_point.pose.orientation.x, map_point.pose.orientation.y,
      map_point.pose.orientation.z);

    // Extract rotation part from transform
    Eigen::Matrix3d rotation_matrix = eigen_transform.block<3, 3>(0, 0);
    Eigen::Quaterniond base_link_quat(rotation_matrix * map_quat.toRotationMatrix());

    base_link_point.pose.orientation.w = base_link_quat.w();
    base_link_point.pose.orientation.x = base_link_quat.x();
    base_link_point.pose.orientation.y = base_link_quat.y();
    base_link_point.pose.orientation.z = base_link_quat.z();

    base_link_trajectory_points.push_back(base_link_point);
  }

  return base_link_trajectory_points;
}

std::vector<autoware::universe_utils::Polygon2d> generateTrajectoryPolygons(
  const autoware_planning_msgs::msg::Trajectory & planning_trajectory, double max_filter_distance,
  const std::shared_ptr<autoware::universe_utils::TransformListener> & transform_listener)
{
  std::vector<autoware::universe_utils::Polygon2d> traj_polygons;

  if (planning_trajectory.points.empty()) {
    return traj_polygons;
  }

  // Transform trajectory points to base_link frame
  const auto base_link_trajectory_points =
    transformTrajectoryToBaseLink(planning_trajectory.points, transform_listener);

  if (base_link_trajectory_points.empty()) {
    return traj_polygons;
  }

  // Create polygons in base_link frame
  traj_polygons = createTrajectoryPolygons(base_link_trajectory_points, max_filter_distance);

  return traj_polygons;
}

std::vector<autoware::universe_utils::Polygon2d> generateCropBoxPolygons(
  const std::vector<autoware::universe_utils::Polygon2d> & traj_polygons)
{
  std::vector<autoware::universe_utils::Polygon2d> crop_box_polygons;

  if (traj_polygons.empty()) {
    return crop_box_polygons;
  }

  // Calculate XY bounding box from trajectory polygons
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  for (const auto & poly : traj_polygons) {
    for (const auto & point : poly.outer()) {
      x_min = std::min(x_min, point.x());
      x_max = std::max(x_max, point.x());
      y_min = std::min(y_min, point.y());
      y_max = std::max(y_max, point.y());
    }
  }

  // Create bounding box polygon
  autoware::universe_utils::Polygon2d bounding_polygon;
  boost::geometry::append(bounding_polygon, autoware::universe_utils::Point2d(x_min, y_min));
  boost::geometry::append(bounding_polygon, autoware::universe_utils::Point2d(x_max, y_min));
  boost::geometry::append(bounding_polygon, autoware::universe_utils::Point2d(x_max, y_max));
  boost::geometry::append(bounding_polygon, autoware::universe_utils::Point2d(x_min, y_max));
  boost::geometry::append(
    bounding_polygon, autoware::universe_utils::Point2d(x_min, y_min));  // Close polygon
  boost::geometry::correct(bounding_polygon);

  crop_box_polygons.push_back(bounding_polygon);

  return crop_box_polygons;
}

std::vector<autoware::universe_utils::Polygon2d> generateSegmentedCropBoxPolygons(
  const std::vector<autoware::universe_utils::Polygon2d> & traj_polygons)
{
  std::vector<autoware::universe_utils::Polygon2d> segmented_crop_box_polygons;

  if (traj_polygons.empty()) {
    return segmented_crop_box_polygons;
  }

  // Create a crop box for each trajectory polygon
  for (const auto & poly : traj_polygons) {
    if (poly.outer().empty()) {
      continue;
    }

    // Calculate bounding box for this polygon
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();

    for (const auto & point : poly.outer()) {
      x_min = std::min(x_min, point.x());
      x_max = std::max(x_max, point.x());
      y_min = std::min(y_min, point.y());
      y_max = std::max(y_max, point.y());
    }

    // Create crop box for this polygon
    autoware::universe_utils::Polygon2d crop_box;
    boost::geometry::append(crop_box, autoware::universe_utils::Point2d(x_min, y_min));
    boost::geometry::append(crop_box, autoware::universe_utils::Point2d(x_max, y_min));
    boost::geometry::append(crop_box, autoware::universe_utils::Point2d(x_max, y_max));
    boost::geometry::append(crop_box, autoware::universe_utils::Point2d(x_min, y_max));
    boost::geometry::append(
      crop_box, autoware::universe_utils::Point2d(x_min, y_min));  // Close polygon
    boost::geometry::correct(crop_box);

    segmented_crop_box_polygons.push_back(crop_box);
  }

  return segmented_crop_box_polygons;
}

autoware::universe_utils::Polygon2d combineTrajectoryPolygons(
  const std::vector<autoware::universe_utils::Polygon2d> & polygons)
{
  autoware::universe_utils::Polygon2d combined_polygon;

  if (polygons.empty()) {
    return combined_polygon;
  }

  if (polygons.size() == 1) {
    return polygons[0];
  }

  // Use boost::geometry::union_ to combine all polygons
  autoware::universe_utils::MultiPolygon2d result_polygons;

  // Start with the first polygon
  autoware::universe_utils::MultiPolygon2d temp_polygons;
  temp_polygons.push_back(polygons[0]);

  // Union with each subsequent polygon
  for (size_t i = 1; i < polygons.size(); ++i) {
    autoware::universe_utils::MultiPolygon2d current_result;
    boost::geometry::union_(temp_polygons, polygons[i], current_result);
    temp_polygons = current_result;
  }

  // If we have a result, take the first (largest) polygon
  if (!temp_polygons.empty()) {
    combined_polygon = temp_polygons[0];
    boost::geometry::correct(combined_polygon);
  }

  return combined_polygon;
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointIndices::Ptr>
filterByMultiTrajectoryPolygon(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const std::vector<autoware::universe_utils::Polygon2d> & traj_polygons,
  autoware::universe_utils::TimeKeeper * time_keeper)
{
  auto ret_pointcloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto indices_to_remove = std::make_shared<pcl::PointIndices>();

  ret_pointcloud_ptr->header = input_pointcloud_ptr->header;

  // Define types for Boost.Geometry
  namespace bg = boost::geometry;
  namespace bgi = boost::geometry::index;
  using BoostPoint2D = bg::model::point<double, 2, bg::cs::cartesian>;
  using BoostValue = std::pair<BoostPoint2D, size_t>;  // point + index

  // Build R-tree from input points
  std::vector<BoostValue> rtree_data;
  rtree_data.reserve(input_pointcloud_ptr->points.size());

  {
    std::unique_ptr<autoware::universe_utils::ScopedTimeTrack> st_rtree_build;
    if (time_keeper) {
      st_rtree_build =
        std::make_unique<autoware::universe_utils::ScopedTimeTrack>("rtree_build", *time_keeper);
    }
    std::transform(
      input_pointcloud_ptr->points.begin(), input_pointcloud_ptr->points.end(),
      std::back_inserter(rtree_data), [i = 0](const pcl::PointXYZ & pt) mutable {
        return std::make_pair(BoostPoint2D(pt.x, pt.y), i++);
      });
  }

  bgi::rtree<BoostValue, bgi::quadratic<16>> rtree(rtree_data.begin(), rtree_data.end());

  std::unordered_set<size_t> selected_indices;

  {
    std::unique_ptr<autoware::universe_utils::ScopedTimeTrack> st_polygon_query;
    if (time_keeper) {
      st_polygon_query =
        std::make_unique<autoware::universe_utils::ScopedTimeTrack>("polygon_query", *time_keeper);
    }
    std::for_each(
      traj_polygons.begin(), traj_polygons.end(),
      [&](const autoware::universe_utils::Polygon2d & one_step_polygon) {
        bg::model::box<BoostPoint2D> bbox;
        bg::envelope(one_step_polygon, bbox);

        std::vector<BoostValue> result_s;
        rtree.query(bgi::intersects(bbox), std::back_inserter(result_s));

        for (const auto & val : result_s) {
          const BoostPoint2D & pt = val.first;
          if (bg::within(pt, one_step_polygon)) {
            selected_indices.insert(val.second);
          }
        }
      });
  }

  {
    std::unique_ptr<autoware::universe_utils::ScopedTimeTrack> st_result_build;
    if (time_keeper) {
      st_result_build =
        std::make_unique<autoware::universe_utils::ScopedTimeTrack>("result_build", *time_keeper);
    }

    ret_pointcloud_ptr->points.reserve(selected_indices.size());
    std::transform(
      selected_indices.begin(), selected_indices.end(),
      std::back_inserter(ret_pointcloud_ptr->points),
      [&](const size_t idx) { return input_pointcloud_ptr->points[idx]; });

    ret_pointcloud_ptr->width = ret_pointcloud_ptr->points.size();
    ret_pointcloud_ptr->height = 1;
    ret_pointcloud_ptr->is_dense = true;

    indices_to_remove->indices.reserve(selected_indices.size());
    std::copy(
      selected_indices.begin(), selected_indices.end(),
      std::back_inserter(indices_to_remove->indices));
  }

  return std::make_pair(ret_pointcloud_ptr, indices_to_remove);
}

std::vector<autoware::universe_utils::Polygon2d> createDifferencePolygons(
  const std::vector<autoware::universe_utils::Polygon2d> & max_polygons,
  const std::vector<autoware::universe_utils::Polygon2d> & min_polygons)
{
  std::vector<autoware::universe_utils::Polygon2d> difference_polygons;

  if (max_polygons.empty()) {
    return difference_polygons;
  }

  // If min_polygons is empty, return max_polygons as is
  if (min_polygons.empty()) {
    return max_polygons;
  }

  // Combine all max_polygons into a single polygon using union
  autoware::universe_utils::Polygon2d combined_max_polygon;
  if (max_polygons.size() == 1) {
    combined_max_polygon = max_polygons[0];
  } else {
    combined_max_polygon = combineTrajectoryPolygons(max_polygons);
  }

  // Combine all min_polygons into a single polygon using union
  autoware::universe_utils::Polygon2d combined_min_polygon;
  if (min_polygons.size() == 1) {
    combined_min_polygon = min_polygons[0];
  } else {
    combined_min_polygon = combineTrajectoryPolygons(min_polygons);
  }

  // Perform difference operation using boost::geometry::difference
  autoware::universe_utils::MultiPolygon2d result_polygons;
  boost::geometry::difference(combined_max_polygon, combined_min_polygon, result_polygons);

  // Convert MultiPolygon2d to vector of Polygon2d
  for (const auto & polygon : result_polygons) {
    autoware::universe_utils::Polygon2d corrected_polygon = polygon;
    boost::geometry::correct(corrected_polygon);
    difference_polygons.push_back(corrected_polygon);
  }

  return difference_polygons;
}

autoware_planning_msgs::msg::Trajectory cutTrajectoryByFilteringDistance(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & ego_pose, const double filtering_start_distance,
  const double filtering_end_distance)
{
  // Initialize cut trajectory with input trajectory
  auto cut_trajectory = trajectory;

  // Find nearest segment index to ego pose
  const auto nearest_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(trajectory.points, ego_pose.position);

  if (!nearest_segment_idx) {
    // If nearest segment is not found, return original trajectory
    return cut_trajectory;
  }

  // Calculate distance from trajectory start to ego pose
  const double ego_distance_from_start =
    autoware::motion_utils::calcSignedArcLength(trajectory.points, 0, nearest_segment_idx);

  // Calculate start and end distances for cutting
  const double start_distance = ego_distance_from_start + filtering_start_distance;
  const double end_distance = ego_distance_from_start + filtering_end_distance;

  // Find indices for cutting by converting distance to pose first
  const auto start_pose =
    autoware::motion_utils::calcInterpolatedPose(trajectory.points, start_distance);
  const auto end_pose =
    autoware::motion_utils::calcInterpolatedPose(trajectory.points, end_distance);

  const auto start_idx =
    autoware::motion_utils::findNearestIndex(trajectory.points, start_pose.position);
  const auto end_idx =
    autoware::motion_utils::findNearestIndex(trajectory.points, end_pose.position);

  if (start_idx < end_idx) {
    // Create cut trajectory
    cut_trajectory.points.clear();
    cut_trajectory.points.insert(
      cut_trajectory.points.end(), trajectory.points.begin() + start_idx,
      trajectory.points.begin() + end_idx + 1);
  }

  return cut_trajectory;
}

autoware_planning_msgs::msg::Trajectory cutTrajectoryByPoses(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & end_pose)
{
  // Initialize cut trajectory with input trajectory
  auto cut_trajectory = trajectory;

  // Find nearest indices to start and end poses
  const size_t start_idx =
    autoware::motion_utils::findNearestIndex(trajectory.points, start_pose.position);
  const size_t end_idx =
    autoware::motion_utils::findNearestIndex(trajectory.points, end_pose.position);

  // Ensure start_idx is before end_idx
  const size_t actual_start_idx = std::min(start_idx, end_idx);
  const size_t actual_end_idx = std::max(start_idx, end_idx);

  if (actual_start_idx >= actual_end_idx) {
    RCLCPP_WARN(
      rclcpp::get_logger("perception_filter_utils"),
      "Invalid trajectory range: start_idx >= end_idx");
    return cut_trajectory;
  }

  // Create cut trajectory
  cut_trajectory.points.clear();
  cut_trajectory.points.insert(
    cut_trajectory.points.end(), trajectory.points.begin() + actual_start_idx,
    trajectory.points.begin() + actual_end_idx + 1);

  return cut_trajectory;
}

std::vector<autoware::universe_utils::Polygon2d> transformPolygonsToMap(
  const std::vector<autoware::universe_utils::Polygon2d> & base_link_polygons,
  const std::shared_ptr<autoware::universe_utils::TransformListener> & transform_listener)
{
  std::vector<autoware::universe_utils::Polygon2d> map_polygons;

  if (base_link_polygons.empty()) {
    return map_polygons;
  }

  // Get transform from base_link to map using common helper
  const auto transform_matrix_opt = getEigenTransform(transform_listener, "map", "base_link");
  if (!transform_matrix_opt) {
    return map_polygons;
  }

  const Eigen::Matrix4d eigen_transform = transform_matrix_opt.value();

  // Transform each polygon from base_link to map frame
  for (const auto & base_link_polygon : base_link_polygons) {
    autoware::universe_utils::Polygon2d map_polygon;

    // Transform each point in the polygon
    for (const auto & base_link_point : base_link_polygon.outer()) {
      // Convert boost::geometry point to Eigen vector
      Eigen::Vector3d base_link_pos_3d(base_link_point.x(), base_link_point.y(), 0.0);

      // Transform to map frame
      Eigen::Vector3d map_pos_3d =
        eigen_transform.block<3, 3>(0, 0) * base_link_pos_3d + eigen_transform.block<3, 1>(0, 3);

      // Create new point in map frame
      autoware::universe_utils::Point2d map_point;
      map_point.x() = map_pos_3d.x();
      map_point.y() = map_pos_3d.y();

      // Add point to map polygon
      boost::geometry::append(map_polygon, map_point);
    }

    // Correct the polygon geometry
    boost::geometry::correct(map_polygon);
    map_polygons.push_back(map_polygon);
  }

  return map_polygons;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extractPointsOutsideCropBox(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const autoware::universe_utils::Polygon2d & crop_box_polygon)
{
  auto filtered_pointcloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  if (input_pointcloud_ptr->points.empty() || crop_box_polygon.outer().empty()) {
    return filtered_pointcloud_ptr;
  }

  // Calculate bounding box from crop box polygon
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  for (const auto & point : crop_box_polygon.outer()) {
    x_min = std::min(x_min, point.x());
    x_max = std::max(x_max, point.x());
    y_min = std::min(y_min, point.y());
    y_max = std::max(y_max, point.y());
  }

  // Create PCL CropBox filter
  pcl::CropBox<pcl::PointXYZ> crop_box_filter;

  // Set crop box parameters (center and size)
  const double center_x = (x_min + x_max) / 2.0;
  const double center_y = (y_min + y_max) / 2.0;
  const double center_z = 0.0;  // Z center at ground level

  const double size_x = x_max - x_min;
  const double size_y = y_max - y_min;
  const double size_z = 100.0;  // Large Z size to include all points vertically

  // Set the crop box center and size
  crop_box_filter.setTranslation(Eigen::Vector3f(center_x, center_y, center_z));
  crop_box_filter.setInputCloud(input_pointcloud_ptr);

  // Set crop box size (half-widths)
  crop_box_filter.setMin(Eigen::Vector4f(-size_x / 2.0, -size_y / 2.0, -size_z / 2.0, 1.0));
  crop_box_filter.setMax(Eigen::Vector4f(size_x / 2.0, size_y / 2.0, size_z / 2.0, 1.0));

  // First, get points inside the crop box
  auto points_inside_cropbox = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  crop_box_filter.filter(*points_inside_cropbox);

  // Get indices of points inside the crop box
  pcl::PointIndices::Ptr indices_inside = std::make_shared<pcl::PointIndices>();
  crop_box_filter.getRemovedIndices(*indices_inside);

  // Use ExtractIndices to get points outside the crop box
  pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
  extract_indices.setInputCloud(input_pointcloud_ptr);
  extract_indices.setIndices(indices_inside);
  extract_indices.setNegative(true);  // Extract points NOT in the indices (outside crop box)
  extract_indices.filter(*filtered_pointcloud_ptr);

  // Set pointcloud properties
  filtered_pointcloud_ptr->width = filtered_pointcloud_ptr->points.size();
  filtered_pointcloud_ptr->height = 1;
  filtered_pointcloud_ptr->is_dense = true;
  filtered_pointcloud_ptr->header = input_pointcloud_ptr->header;

  return filtered_pointcloud_ptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extractPointsInsideCropBox(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const autoware::universe_utils::Polygon2d & crop_box_polygon)
{
  auto filtered_pointcloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  if (input_pointcloud_ptr->points.empty() || crop_box_polygon.outer().empty()) {
    return filtered_pointcloud_ptr;
  }

  // Calculate bounding box from crop box polygon
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  for (const auto & point : crop_box_polygon.outer()) {
    x_min = std::min(x_min, point.x());
    x_max = std::max(x_max, point.x());
    y_min = std::min(y_min, point.y());
    y_max = std::max(y_max, point.y());
  }

  // Create PCL CropBox filter
  pcl::CropBox<pcl::PointXYZ> crop_box_filter;

  // Set crop box parameters (center and size)
  const double center_x = (x_min + x_max) / 2.0;
  const double center_y = (y_min + y_max) / 2.0;
  const double center_z = 0.0;  // Z center at ground level

  const double size_x = x_max - x_min;
  const double size_y = y_max - y_min;
  const double size_z = 100.0;  // Large Z size to include all points vertically

  // Set the crop box center and size
  crop_box_filter.setTranslation(Eigen::Vector3f(center_x, center_y, center_z));
  crop_box_filter.setInputCloud(input_pointcloud_ptr);

  // Set crop box size (half-widths)
  crop_box_filter.setMin(Eigen::Vector4f(-size_x/2.0, -size_y/2.0, -size_z/2.0, 1.0));
  crop_box_filter.setMax(Eigen::Vector4f(size_x/2.0, size_y/2.0, size_z/2.0, 1.0));

  // Get points inside the crop box
  crop_box_filter.filter(*filtered_pointcloud_ptr);

  // Set pointcloud properties
  filtered_pointcloud_ptr->width = filtered_pointcloud_ptr->points.size();
  filtered_pointcloud_ptr->height = 1;
  filtered_pointcloud_ptr->is_dense = true;
  filtered_pointcloud_ptr->header = input_pointcloud_ptr->header;

  return filtered_pointcloud_ptr;
}

pcl::PointIndices::Ptr extractIndicesInsideCropBox(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const autoware::universe_utils::Polygon2d & crop_box_polygon)
{
  auto indices_inside = std::make_shared<pcl::PointIndices>();

  if (input_pointcloud_ptr->points.empty() || crop_box_polygon.outer().empty()) {
    return indices_inside;
  }

  // Calculate bounding box
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  for (const auto & point : crop_box_polygon.outer()) {
    x_min = std::min(x_min, point.x());
    x_max = std::max(x_max, point.x());
    y_min = std::min(y_min, point.y());
    y_max = std::max(y_max, point.y());
  }

  pcl::CropBox<pcl::PointXYZ> crop_box_filter;
  crop_box_filter.setInputCloud(input_pointcloud_ptr);

  // Set Min/Max in absolute coordinates
  crop_box_filter.setMin(Eigen::Vector4f(x_min, y_min, -100.0, 1.0));
  crop_box_filter.setMax(Eigen::Vector4f(x_max, y_max, 100.0, 1.0));

  // Set to keep points inside the box
  crop_box_filter.setNegative(false);

  // Filter and get indices
  crop_box_filter.filter(indices_inside->indices);

  return indices_inside;
}

}  // namespace autoware::perception_filter
