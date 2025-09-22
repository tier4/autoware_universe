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

#ifndef AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__NODE_HPP_
#define AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_simulation_msgs/msg/dummy_object.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#include <optional>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
// #include "autoware/dummy_perception_publisher/predicted_object_movement_base_plugin_class.hpp"
// If the file exists under a different name or path, update the include accordingly, e.g.:
#include "autoware/dummy_perception_publisher/dummy_object_movement_base_plugin.hpp"
#include "autoware/dummy_perception_publisher/object_info.hpp"
#include "autoware/dummy_perception_publisher/predicted_object_movement_plugin.hpp"
#include "autoware/dummy_perception_publisher/straight_line_object_movement_plugin.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::dummy_perception_publisher
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::TwistWithCovariance;
using tier4_simulation_msgs::msg::DummyObject;

class PointCloudCreator
{
public:
  virtual ~PointCloudCreator() = default;

  virtual std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> create_pointclouds(
    const std::vector<ObjectInfo> & obj_infos, const tf2::Transform & tf_base_link2map,
    std::mt19937 & random_generator,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & merged_pointcloud) const = 0;
};

class ObjectCentricPointCloudCreator : public PointCloudCreator
{
public:
  explicit ObjectCentricPointCloudCreator(bool enable_ray_tracing)
  : enable_ray_tracing_(enable_ray_tracing)
  {
  }

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> create_pointclouds(
    const std::vector<ObjectInfo> & obj_infos, const tf2::Transform & tf_base_link2map,
    std::mt19937 & random_generator,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & merged_pointcloud) const override;

private:
  void create_object_pointcloud(
    const ObjectInfo & obj_info, const tf2::Transform & tf_base_link2map,
    std::mt19937 & random_generator, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) const;

  bool enable_ray_tracing_;
};

class EgoCentricPointCloudCreator : public PointCloudCreator
{
public:
  explicit EgoCentricPointCloudCreator(double visible_range) : visible_range_(visible_range) {}
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> create_pointclouds(
    const std::vector<ObjectInfo> & obj_infos, const tf2::Transform & tf_base_link2map,
    std::mt19937 & random_generator,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & merged_pointcloud) const override;

private:
  double visible_range_;
};

class DummyPerceptionPublisherNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    detected_object_with_feature_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr
    ground_truth_objects_pub_;
  rclcpp::Subscription<DummyObject>::SharedPtr object_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double visible_range_;
  double detection_successful_rate_;
  bool enable_ray_tracing_;
  bool use_object_recognition_;
  bool use_base_link_z_;
  bool publish_ground_truth_objects_;
  std::unique_ptr<PointCloudCreator> pointcloud_creator_;
  // dummy object movement plugins
  pluginlib::PredictedObjectMovementPlugin dummy_predicted_movement_plugin_;
  pluginlib::StraightLineObjectMovementPlugin dummy_straight_line_movement_plugin_;
  double angle_increment_;
  std::mt19937 random_generator_;

  void timerCallback();
  void objectCallback(const DummyObject::ConstSharedPtr msg);

public:
  DummyPerceptionPublisherNode();
};

}  // namespace autoware::dummy_perception_publisher

#endif  // AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__NODE_HPP_
