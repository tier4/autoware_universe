// Copyright 2020 Tier IV, Inc.
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

#include "collision_checker/node.hpp"

#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace collision_checker
{
namespace bg = boost::geometry;
using Point2d = bg::model::d2::point_xy<double>;
using Polygon2d = bg::model::polygon<Point2d>;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::pose2transform;

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

Polygon2d createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Polygon & footprint)
{
  geometry_msgs::msg::Polygon transformed_polygon{};
  geometry_msgs::msg::TransformStamped geometry_tf{};
  geometry_tf.transform = pose2transform(pose);
  tf2::doTransform(footprint, transformed_polygon, geometry_tf);

  Polygon2d object_polygon;
  for (const auto & p : transformed_polygon.points) {
    object_polygon.outer().push_back(Point2d(p.x, p.y));
  }

  bg::correct(object_polygon);

  return object_polygon;
}

Polygon2d createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & size)
{
  const double & length_m = size.x / 2.0;
  const double & width_m = size.y / 2.0;

  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(length_m, -width_m, 0.0));
  polygon.points.push_back(createPoint32(length_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-length_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-length_m, -width_m, 0.0));

  return createObjPolygon(pose, polygon);
}

Polygon2d createSelfPolygon(const VehicleInfo & vehicle_info)
{
  const double & front_m = vehicle_info.max_longitudinal_offset_m;
  const double & width_left_m = vehicle_info.max_lateral_offset_m;
  const double & width_right_m = vehicle_info.min_lateral_offset_m;
  const double & rear_m = vehicle_info.min_longitudinal_offset_m;

  Polygon2d ego_polygon;

  ego_polygon.outer().push_back(Point2d(front_m, width_left_m));
  ego_polygon.outer().push_back(Point2d(front_m, width_right_m));
  ego_polygon.outer().push_back(Point2d(rear_m, width_right_m));
  ego_polygon.outer().push_back(Point2d(rear_m, width_left_m));

  bg::correct(ego_polygon);

  return ego_polygon;
}
}  // namespace

CollisionCheckerNode::CollisionCheckerNode(const rclcpp::NodeOptions & node_options)
: Node("collision_checker_node", node_options), updater_(this)
{
  // Parameters
  {
    auto & p = node_param_;
    p.use_pointcloud = this->declare_parameter("use_pointcloud", true);
    p.use_dynamic_object = this->declare_parameter("use_dynamic_object", true);
    p.collision_distance = this->declare_parameter("collision_distance", 0.15);
  }

  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  ;

  // Subscribers
  sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&CollisionCheckerNode::onPointCloud, this, std::placeholders::_1));
  sub_dynamic_objects_ = this->create_subscription<PredictedObjects>(
    "~/input/objects", 1,
    std::bind(&CollisionCheckerNode::onDynamicObjects, this, std::placeholders::_1));

  sub_operation_mode_ = this->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&CollisionCheckerNode::onOperationMode, this, std::placeholders::_1));

  // Diagnostics Updater
  updater_.setHardwareID("collision_checker");
  updater_.add("collision_check", this, &CollisionCheckerNode::checkCollision);
  updater_.setPeriod(0.1);
}

void CollisionCheckerNode::checkCollision(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (node_param_.use_pointcloud && !pointcloud_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for pointcloud info...");
    return;
  }

  if (node_param_.use_dynamic_object && !object_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for dynamic object info...");
    return;
  }

  if (!operation_mode_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for operation mode info...");
    return;
  }

  const auto nearest_obstacle = getNearestObstacle();

  const auto is_obstacle_found =
    !nearest_obstacle ? false : nearest_obstacle.get().first < node_param_.collision_distance;

  diagnostic_msgs::msg::DiagnosticStatus status;
  if (operation_mode_ptr_->mode == OperationModeState::AUTONOMOUS && is_obstacle_found) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "collision detected";
    stat.addf("Distance to nearest neighbor object", "%lf", nearest_obstacle.get().first);
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  stat.summary(status.level, status.message);
}

void CollisionCheckerNode::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  pointcloud_ptr_ = msg;
}

void CollisionCheckerNode::onDynamicObjects(const PredictedObjects::ConstSharedPtr msg)
{
  object_ptr_ = msg;
}

void CollisionCheckerNode::onOperationMode(const OperationModeState::ConstSharedPtr msg)
{
  operation_mode_ptr_ = msg;
}

boost::optional<Obstacle> CollisionCheckerNode::getNearestObstacle() const
{
  boost::optional<Obstacle> nearest_pointcloud{boost::none};
  boost::optional<Obstacle> nearest_object{boost::none};

  if (node_param_.use_pointcloud) {
    nearest_pointcloud = getNearestObstacleByPointCloud();
  }

  if (node_param_.use_dynamic_object) {
    nearest_object = getNearestObstacleByDynamicObject();
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

  return nearest_pointcloud.get().first < nearest_object.get().first ? nearest_pointcloud
                                                                     : nearest_object;
}

boost::optional<Obstacle> CollisionCheckerNode::getNearestObstacleByPointCloud() const
{
  const auto transform_stamped =
    getTransform("base_link", pointcloud_ptr_->header.frame_id, pointcloud_ptr_->header.stamp, 0.5);

  geometry_msgs::msg::Point nearest_point;
  auto minimum_distance = std::numeric_limits<double>::max();

  if (!transform_stamped) {
    return {};
  }

  Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.get().transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(*pointcloud_ptr_, transformed_pointcloud);
  pcl::transformPointCloud(transformed_pointcloud, transformed_pointcloud, isometry);

  const auto ego_polygon = createSelfPolygon(vehicle_info_);

  for (const auto & p : transformed_pointcloud) {
    Point2d boost_point(p.x, p.y);

    const auto distance_to_object = bg::distance(ego_polygon, boost_point);

    if (distance_to_object < minimum_distance) {
      nearest_point = createPoint(p.x, p.y, p.z);
      minimum_distance = distance_to_object;
    }
  }

  return std::make_pair(minimum_distance, nearest_point);
}

boost::optional<Obstacle> CollisionCheckerNode::getNearestObstacleByDynamicObject() const
{
  const auto transform_stamped =
    getTransform(object_ptr_->header.frame_id, "base_link", object_ptr_->header.stamp, 0.5);

  geometry_msgs::msg::Point nearest_point;
  auto minimum_distance = std::numeric_limits<double>::max();

  if (!transform_stamped) {
    return {};
  }

  tf2::Transform tf_src2target;
  tf2::fromMsg(transform_stamped.get().transform, tf_src2target);

  const auto ego_polygon = createSelfPolygon(vehicle_info_);

  for (const auto & object : object_ptr_->objects) {
    const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;

    tf2::Transform tf_src2object;
    tf2::fromMsg(object_pose, tf_src2object);

    geometry_msgs::msg::Pose transformed_object_pose;
    tf2::toMsg(tf_src2target.inverse() * tf_src2object, transformed_object_pose);

    const auto object_polygon =
      object.shape.type == Shape::POLYGON
        ? createObjPolygon(transformed_object_pose, object.shape.footprint)
        : createObjPolygon(transformed_object_pose, object.shape.dimensions);

    const auto distance_to_object = bg::distance(ego_polygon, object_polygon);

    if (distance_to_object < minimum_distance) {
      nearest_point = object_pose.position;
      minimum_distance = distance_to_object;
    }
  }

  return std::make_pair(minimum_distance, nearest_point);
}

boost::optional<geometry_msgs::msg::TransformStamped> CollisionCheckerNode::getTransform(
  const std::string & source, const std::string & target, const rclcpp::Time & stamp,
  double duration_sec) const
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped =
      tf_buffer_.lookupTransform(source, target, stamp, tf2::durationFromSec(duration_sec));
  } catch (tf2::TransformException & ex) {
    return {};
  }

  return transform_stamped;
}

}  // namespace collision_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(collision_checker::CollisionCheckerNode)
