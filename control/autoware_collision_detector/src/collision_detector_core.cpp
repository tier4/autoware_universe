// Copyright 2024-2025 TIER IV, Inc.
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

#include "autoware/collision_detector/collision_detector_core.hpp"

#include "autoware/collision_detector/debug.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace autoware::collision_detector
{
namespace bg = boost::geometry;
using autoware_perception_msgs::msg::Shape;
using autoware_utils::create_point;
using autoware_utils::pose2transform;

namespace
{

geometry_msgs::msg::Point32 createPoint32(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

autoware_utils_geometry::Polygon2d createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Polygon & footprint)
{
  geometry_msgs::msg::Polygon transformed_polygon{};
  geometry_msgs::msg::TransformStamped geometry_tf{};
  geometry_tf.transform = pose2transform(pose);
  tf2::doTransform(footprint, transformed_polygon, geometry_tf);

  autoware_utils_geometry::Polygon2d object_polygon;
  for (const auto & p : transformed_polygon.points) {
    object_polygon.outer().push_back(autoware_utils_geometry::Point2d(p.x, p.y));
  }

  bg::correct(object_polygon);

  return object_polygon;
}

autoware_utils_geometry::Polygon2d createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & size)
{
  const double length_m = size.x / 2.0;
  const double width_m = size.y / 2.0;

  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(length_m, -width_m, 0.0));
  polygon.points.push_back(createPoint32(length_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-length_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-length_m, -width_m, 0.0));

  return createObjPolygon(pose, polygon);
}

autoware_utils_geometry::Polygon2d createObjPolygonForCylinder(
  const geometry_msgs::msg::Pose & pose, const double diameter)
{
  geometry_msgs::msg::Polygon polygon{};

  const double radius = diameter * 0.5;
  // add hexagon points
  for (int i = 0; i < 6; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / 6.0;
    const double x = radius * std::cos(angle);
    const double y = radius * std::sin(angle);
    polygon.points.push_back(createPoint32(x, y, 0.0));
  }

  return createObjPolygon(pose, polygon);
}

autoware_utils_geometry::Polygon2d createSelfPolygon(
  const VehicleInfo & vehicle_info, const double extra_offset, const bool ignore_behind_rear_axle)
{
  const double & front_m = vehicle_info.max_longitudinal_offset_m + extra_offset;
  const double & width_left_m = vehicle_info.max_lateral_offset_m + extra_offset;
  const double & width_right_m = vehicle_info.min_lateral_offset_m - extra_offset;
  const double & rear_m =
    ignore_behind_rear_axle ? 0.0 : vehicle_info.min_longitudinal_offset_m - extra_offset;

  autoware_utils_geometry::Polygon2d ego_polygon;

  ego_polygon.outer().push_back(autoware_utils_geometry::Point2d(front_m, width_left_m));
  ego_polygon.outer().push_back(autoware_utils_geometry::Point2d(front_m, width_right_m));
  ego_polygon.outer().push_back(autoware_utils_geometry::Point2d(rear_m, width_right_m));
  ego_polygon.outer().push_back(autoware_utils_geometry::Point2d(rear_m, width_left_m));

  bg::correct(ego_polygon);

  return ego_polygon;
}
}  // namespace

CollisionDetectorCore::CollisionDetectorCore(
  rclcpp::Node * node, const std::string & param_prefix, const std::string & odom_topic,
  const std::string & pointcloud_topic, const std::string & objects_topic)
: node_(node),
  param_prefix_(param_prefix),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_),
  sub_odometry_(node, odom_topic),
  sub_pointcloud_(node, pointcloud_topic, autoware_utils::single_depth_sensor_qos()),
  sub_dynamic_objects_(node, objects_topic),
  sub_operation_mode_(node, "/api/operation_mode/state", rclcpp::QoS{1}.transient_local())
{
  pub_debug_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug_markers", 1);
  declare_parameters();
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*node_).getVehicleInfo();
  vehicle_stop_checker_ = std::make_unique<autoware::motion_utils::VehicleStopChecker>(node_);
}

std::string CollisionDetectorCore::param_name(const std::string & name) const
{
  return param_prefix_ + name;
}

void CollisionDetectorCore::declare_parameters()
{
  auto & p = node_param_;
  p.use_pointcloud = node_->declare_parameter<bool>(param_name("use_pointcloud"));
  p.use_dynamic_object = node_->declare_parameter<bool>(param_name("use_dynamic_object"));
  p.collision_distance = node_->declare_parameter<double>(param_name("collision_distance"));
  p.nearby_filter_radius = node_->declare_parameter<double>(param_name("nearby_filter_radius"));
  p.keep_ignoring_time = node_->declare_parameter<double>(param_name("keep_ignoring_time"));
  p.nearby_object_type_filters.filter_car =
    node_->declare_parameter<bool>(param_name("nearby_object_type_filters.filter_car"));
  p.nearby_object_type_filters.filter_truck =
    node_->declare_parameter<bool>(param_name("nearby_object_type_filters.filter_truck"));
  p.nearby_object_type_filters.filter_bus =
    node_->declare_parameter<bool>(param_name("nearby_object_type_filters.filter_bus"));
  p.nearby_object_type_filters.filter_trailer =
    node_->declare_parameter<bool>(param_name("nearby_object_type_filters.filter_trailer"));
  p.nearby_object_type_filters.filter_unknown =
    node_->declare_parameter<bool>(param_name("nearby_object_type_filters.filter_unknown"));
  p.nearby_object_type_filters.filter_bicycle =
    node_->declare_parameter<bool>(param_name("nearby_object_type_filters.filter_bicycle"));
  p.nearby_object_type_filters.filter_motorcycle =
    node_->declare_parameter<bool>(param_name("nearby_object_type_filters.filter_motorcycle"));
  p.nearby_object_type_filters.filter_pedestrian =
    node_->declare_parameter<bool>(param_name("nearby_object_type_filters.filter_pedestrian"));
  p.ignore_behind_rear_axle = node_->declare_parameter<bool>(param_name("ignore_behind_rear_axle"));
  p.time_buffer.on = node_->declare_parameter<double>(param_name("time_buffer.on_duration"));
  p.time_buffer.off = node_->declare_parameter<double>(param_name("time_buffer.off_duration"));
  p.time_buffer.off_distance_hysteresis =
    node_->declare_parameter<double>(param_name("time_buffer.off_distance_hysteresis"));
}

void CollisionDetectorCore::setup_diagnostics() {}

PredictedObjects CollisionDetectorCore::filterObjects(const PredictedObjects & input_objects)
{
  PredictedObjects filtered_objects;
  filtered_objects.header = input_objects.header;
  filtered_objects.header.stamp = node_->now();

  const rclcpp::Time current_object_time = input_objects.header.stamp;
  const rclcpp::Duration observed_objects_keep_time =
    rclcpp::Duration::from_seconds(0.5);  //  0.5 sec
  const rclcpp::Duration ignored_objects_keep_time =
    rclcpp::Duration::from_seconds(10.0);  // 10 seconds

  // Remove old objects from observed_objects_ and ignored_objects_
  removeOldObjects(observed_objects_, current_object_time, observed_objects_keep_time);
  removeOldObjects(ignored_objects_, current_object_time, ignored_objects_keep_time);

  // Get transform from object frame to base_link
  const auto transform_stamped =
    getTransform("base_link", input_objects.header.frame_id, input_objects.header.stamp, 0.5);

  if (!transform_stamped) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get transform from object frame to base_link");
    return filtered_objects;
  }

  Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped->transform).cast<float>();

  for (const auto & object : input_objects.objects) {
    // Transform object position to base_link frame
    Eigen::Vector3f object_position(
      object.kinematics.initial_pose_with_covariance.pose.position.x,
      object.kinematics.initial_pose_with_covariance.pose.position.y,
      object.kinematics.initial_pose_with_covariance.pose.position.z);
    Eigen::Vector3f transformed_position = isometry * object_position;

    // Calculate object distance from base_link
    const double object_distance = transformed_position.head<2>().norm();
    const bool is_within_range = (object_distance <= node_param_.nearby_filter_radius);

    // Determine if the object should be excluded based on its classification
    bool should_be_excluded = shouldBeExcluded(object.classification.front().label);

    const bool is_within_range_and_filtering_class = is_within_range && should_be_excluded;

    // If the object is not within range or not a class to be filtered, add it directly
    if (!is_within_range_and_filtering_class) {
      filtered_objects.objects.push_back(object);

      // Update observed_objects_
      auto observed_it = std::find_if(
        observed_objects_.begin(), observed_objects_.end(),
        [&object](const auto & observed_object) {
          return observed_object.object_id == object.object_id;
        });
      if (observed_it != observed_objects_.end()) {
        observed_it->timestamp = current_object_time;
      } else {
        observed_objects_.push_back({object.object_id, current_object_time});
      }

      continue;
    }

    // Check if the object exists in ignored_objects_
    auto ignored_it = std::find_if(
      ignored_objects_.begin(), ignored_objects_.end(), [&object](const auto & ignored_object) {
        return ignored_object.object_id == object.object_id;
      });
    const bool was_ignored = (ignored_it != ignored_objects_.end());

    // If the object was ignored and is still within the ignore period, continue filtering
    if (
      was_ignored && (current_object_time - ignored_it->timestamp) <
                       rclcpp::Duration::from_seconds(node_param_.keep_ignoring_time)) {
      // Check if the object exists in observed_objects_
      auto observed_it = std::find_if(
        observed_objects_.begin(), observed_objects_.end(),
        [&object](const auto & observed_object) {
          return observed_object.object_id == object.object_id;
        });
      const bool was_observed = (observed_it != observed_objects_.end());
      if (was_observed) {
        observed_it->timestamp = current_object_time;
      } else {
        // Add as a newly observed object and to the ignore list
        observed_objects_.push_back({object.object_id, current_object_time});
      }
      continue;
    }

    // Check if the object exists in observed_objects_
    auto observed_it = std::find_if(
      observed_objects_.begin(), observed_objects_.end(), [&object](const auto & observed_object) {
        return observed_object.object_id == object.object_id;
      });
    const bool was_observed = (observed_it != observed_objects_.end());

    if (was_observed) {
      observed_it->timestamp = current_object_time;
      // Add without exclusion check
      filtered_objects.objects.push_back(object);
    } else {
      // Add as a newly observed object and to the ignore list
      observed_objects_.push_back({object.object_id, current_object_time});
      ignored_objects_.push_back({object.object_id, current_object_time});
      // Continue filtering
      continue;
    }
  }

  return filtered_objects;
}

void CollisionDetectorCore::removeOldObjects(
  std::vector<TimestampedObject> & container, const rclcpp::Time & current_time,
  const rclcpp::Duration & duration_sec)
{
  container.erase(
    std::remove_if(
      container.begin(), container.end(),
      [&](const TimestampedObject & obj) { return (current_time - obj.timestamp) > duration_sec; }),
    container.end());
}

bool CollisionDetectorCore::shouldBeExcluded(
  const autoware_perception_msgs::msg::ObjectClassification::_label_type & classification) const
{
  switch (classification) {
    case autoware_perception_msgs::msg::ObjectClassification::CAR:
      return node_param_.nearby_object_type_filters.filter_car;
    case autoware_perception_msgs::msg::ObjectClassification::TRUCK:
      return node_param_.nearby_object_type_filters.filter_truck;
    case autoware_perception_msgs::msg::ObjectClassification::BUS:
      return node_param_.nearby_object_type_filters.filter_bus;
    case autoware_perception_msgs::msg::ObjectClassification::TRAILER:
      return node_param_.nearby_object_type_filters.filter_trailer;
    case autoware_perception_msgs::msg::ObjectClassification::UNKNOWN:
      return node_param_.nearby_object_type_filters.filter_unknown;
    case autoware_perception_msgs::msg::ObjectClassification::BICYCLE:
      return node_param_.nearby_object_type_filters.filter_bicycle;
    case autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE:
      return node_param_.nearby_object_type_filters.filter_motorcycle;
    case autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN:
      return node_param_.nearby_object_type_filters.filter_pedestrian;
    default:
      return false;
  }
}

void CollisionDetectorCore::update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  odometry_ptr_ = sub_odometry_.take_data();

  if (!odometry_ptr_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000 /* ms */, "waiting for current odometry...");
    return;
  }

  if (vehicle_stop_checker_->isVehicleStopped()) {
    is_error_diag_ = false;
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "vehicle is stopping");
    return;
  }

  pointcloud_ptr_ = sub_pointcloud_.take_data();
  object_ptr_ = sub_dynamic_objects_.take_data();
  operation_mode_ptr_ = sub_operation_mode_.take_data();

  if (node_param_.use_pointcloud && !pointcloud_ptr_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000 /* ms */, "waiting for pointcloud info...");
    return;
  }

  if (node_param_.use_dynamic_object && !object_ptr_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000 /* ms */, "waiting for dynamic object info...");
    return;
  }

  if (!operation_mode_ptr_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000 /* ms */, "waiting for operation mode info...");
    return;
  }
  filtered_object_ptr_ = std::make_shared<PredictedObjects>(filterObjects(*object_ptr_));

  const auto hysteresis = is_error_diag_ ? node_param_.time_buffer.off_distance_hysteresis : 0.0;
  const auto ego_polygon =
    createSelfPolygon(vehicle_info_, hysteresis, node_param_.ignore_behind_rear_axle);
  const auto nearest_obstacle = getNearestObstacle(ego_polygon);

  const auto is_collision_found =
    !nearest_obstacle ? false : nearest_obstacle->first < node_param_.collision_distance;

  // When a collision is detected, update timestamps to track collision duration
  // - start_of_consecutive_collision_stamp_: marks when a continuous collision began
  // - most_recent_collision_stamp_: records the latest collision detection time
  if (is_collision_found) {
    if (!start_of_consecutive_collision_stamp_.has_value()) {
      start_of_consecutive_collision_stamp_ = node_->now();
    }
    most_recent_collision_stamp_ = node_->now();
  } else {
    start_of_consecutive_collision_stamp_.reset();
  }

  // Define condition to determine error state based on diagnostic mode
  // 1. When already in error state (is_error_diag_ == true):
  //    - Stay in error if time since last collision is less than off_buffer time
  //    - This creates hysteresis to prevent rapid switching between states
  // 2. When in normal state (is_error_diag_ == false):
  //    - Enter error if collision has been continuous for longer than on_buffer time
  //    - This prevents triggering on brief/momentary collisions
  const auto condition_to_trigger_error = [&]() {
    if (is_error_diag_) {
      return (node_->now() - *most_recent_collision_stamp_).seconds() < node_param_.time_buffer.off;
    }
    return start_of_consecutive_collision_stamp_.has_value() &&
           (node_->now() - *start_of_consecutive_collision_stamp_).seconds() >=
             node_param_.time_buffer.on;
  };

  diagnostic_msgs::msg::DiagnosticStatus status;
  if (operation_mode_ptr_->mode == OperationModeState::AUTONOMOUS && condition_to_trigger_error()) {
    is_error_diag_ = true;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "collision detected";
    if (nearest_obstacle) {
      stat.addf("Distance to nearest neighbor object", "%lf", nearest_obstacle->first);
    } else {
      stat.addf(
        "Time since last detection", "%lf",
        (node_->now() - *most_recent_collision_stamp_).seconds() < node_param_.time_buffer.off);
    }
  } else {
    is_error_diag_ = false;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  stat.summary(status.level, status.message);

  pub_debug_->publish(generate_debug_markers(ego_polygon, nearest_obstacle, is_error_diag_));
}

std::optional<Obstacle> CollisionDetectorCore::getNearestObstacle(
  const autoware_utils_geometry::Polygon2d & ego_polygon) const
{
  std::optional<Obstacle> nearest_pointcloud;
  std::optional<Obstacle> nearest_object;

  if (node_param_.use_pointcloud) {
    nearest_pointcloud = getNearestObstacleByPointCloud(ego_polygon);
  }

  if (node_param_.use_dynamic_object) {
    nearest_object = getNearestObstacleByDynamicObject(ego_polygon);
  }

  if (!nearest_pointcloud && !nearest_object) {
    return {};
  }

  if (!nearest_pointcloud) {
    return nearest_object;
  }

  if (!nearest_object) {
    return nearest_pointcloud;
  }

  return nearest_pointcloud->first < nearest_object->first ? nearest_pointcloud : nearest_object;
}

std::optional<Obstacle> CollisionDetectorCore::getNearestObstacleByPointCloud(
  const autoware_utils_geometry::Polygon2d & ego_polygon) const
{
  const auto transform_stamped =
    getTransform("base_link", pointcloud_ptr_->header.frame_id, pointcloud_ptr_->header.stamp, 0.5);

  geometry_msgs::msg::Point nearest_point;
  auto minimum_distance = std::numeric_limits<double>::max();

  if (!transform_stamped) {
    return {};
  }

  Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped->transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(*pointcloud_ptr_, transformed_pointcloud);
  pcl::transformPointCloud(transformed_pointcloud, transformed_pointcloud, isometry);

  for (const auto & p : transformed_pointcloud) {
    autoware_utils_geometry::Point2d boost_point(p.x, p.y);

    const auto distance_to_object = bg::distance(ego_polygon, boost_point);

    if (distance_to_object < minimum_distance) {
      nearest_point = create_point(p.x, p.y, p.z);
      minimum_distance = distance_to_object;
    }
  }

  return std::make_pair(minimum_distance, nearest_point);
}

std::optional<Obstacle> CollisionDetectorCore::getNearestObstacleByDynamicObject(
  const autoware_utils_geometry::Polygon2d & ego_polygon) const
{
  const auto transform_stamped = getTransform(
    filtered_object_ptr_->header.frame_id, "base_link", filtered_object_ptr_->header.stamp, 0.5);

  geometry_msgs::msg::Point nearest_point;
  auto minimum_distance = std::numeric_limits<double>::max();

  if (!transform_stamped) {
    return {};
  }

  tf2::Transform tf_src2target;
  tf2::fromMsg(transform_stamped->transform, tf_src2target);

  for (const auto & object : filtered_object_ptr_->objects) {
    const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;

    tf2::Transform tf_src2object;
    tf2::fromMsg(object_pose, tf_src2object);

    geometry_msgs::msg::Pose transformed_object_pose;
    tf2::toMsg(tf_src2target.inverse() * tf_src2object, transformed_object_pose);

    const auto object_polygon = [&]() {
      switch (object.shape.type) {
        case Shape::POLYGON:
          return createObjPolygon(transformed_object_pose, object.shape.footprint);
        case Shape::CYLINDER:
          return createObjPolygonForCylinder(transformed_object_pose, object.shape.dimensions.x);
        case Shape::BOUNDING_BOX:
          return createObjPolygon(transformed_object_pose, object.shape.dimensions);
        default:
          // node return warning
          RCLCPP_WARN(node_->get_logger(), "Unsupported shape type: %d", object.shape.type);
          return createObjPolygon(transformed_object_pose, object.shape.dimensions);
      }
    }();

    const auto distance_to_object = bg::distance(ego_polygon, object_polygon);

    if (distance_to_object < minimum_distance) {
      nearest_point = object_pose.position;
      minimum_distance = distance_to_object;
    }
  }

  return std::make_pair(minimum_distance, nearest_point);
}

std::optional<geometry_msgs::msg::TransformStamped> CollisionDetectorCore::getTransform(
  const std::string & source, const std::string & target, const rclcpp::Time & stamp,
  double duration_sec) const
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped =
      tf_buffer_.lookupTransform(source, target, stamp, tf2::durationFromSec(duration_sec));
  } catch (const tf2::TransformException & ex) {
    return {};
  }

  return transform_stamped;
}

}  // namespace autoware::collision_detector
