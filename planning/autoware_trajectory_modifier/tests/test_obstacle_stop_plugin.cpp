// Copyright 2026 TIER IV, Inc.
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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/obstacle_stop.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp"

#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <memory>

using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::plugin::ObstacleStop;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::TrajectoryPoint;

class ObstacleStopIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_obstacle_stop_node");
    node_->declare_parameter<double>("wheel_radius", 0.383);
    node_->declare_parameter<double>("wheel_width", 0.235);
    node_->declare_parameter<double>("wheel_base", 2.79);
    node_->declare_parameter<double>("wheel_tread", 1.64);
    node_->declare_parameter<double>("front_overhang", 1.0);
    node_->declare_parameter<double>("rear_overhang", 1.1);
    node_->declare_parameter<double>("left_overhang", 0.5);
    node_->declare_parameter<double>("right_overhang", 0.5);
    node_->declare_parameter<double>("vehicle_height", 2.5);
    node_->declare_parameter<double>("max_steer_angle", 0.7);

    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();
    data_ = std::make_shared<TrajectoryModifierData>(node_.get());

    setup_params();

    plugin_ = std::make_unique<ObstacleStop>();
    plugin_->initialize("test_obstacle_stop", node_.get(), time_keeper_, data_, params_);

    setup_identity_transform();
  }

  void TearDown() override
  {
    plugin_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void setup_params()
  {
    params_.use_obstacle_stop = true;
    params_.trajectory_time_step = 0.1;

    auto & obstacle_stop_params = params_.obstacle_stop;
    obstacle_stop_params.use_objects = false;
    obstacle_stop_params.use_pointcloud = false;
    obstacle_stop_params.enable_stop = false;
    obstacle_stop_params.stop_margin_m = 2.0;
    obstacle_stop_params.nom_stopping_decel = 1.0;
    obstacle_stop_params.max_stopping_decel = 4.0;
    obstacle_stop_params.stopping_jerk = 3.0;
    obstacle_stop_params.lateral_margin_m = 0.5;
    obstacle_stop_params.duplicate_check_threshold_m = 1.0;
    obstacle_stop_params.arrived_distance_threshold_m = 0.5;

    obstacle_stop_params.objects.object_types = {"car",        "truck",   "bus",        "trailer",
                                                 "motorcycle", "bicycle", "pedestrian", "unknown"};
    obstacle_stop_params.objects.max_velocity_th = 1.0;

    obstacle_stop_params.pointcloud.height_buffer = 0.5;
    obstacle_stop_params.pointcloud.min_height = 0.2;
    obstacle_stop_params.pointcloud.detection_range = 100.0;
    obstacle_stop_params.pointcloud.voxel_grid_filter.x = 0.5;
    obstacle_stop_params.pointcloud.voxel_grid_filter.y = 0.5;
    obstacle_stop_params.pointcloud.voxel_grid_filter.z = 0.5;
    obstacle_stop_params.pointcloud.voxel_grid_filter.min_size = 1;
    obstacle_stop_params.pointcloud.clustering.tolerance = 0.5;
    obstacle_stop_params.pointcloud.clustering.min_height = 0.1;
    obstacle_stop_params.pointcloud.clustering.min_size = 2;
    obstacle_stop_params.pointcloud.clustering.max_size = 10000;
  }

  void setup_identity_transform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.header.stamp = node_->get_clock()->now();
    transform.transform.rotation.w = 1.0;
    data_->tf_buffer.setTransform(transform, "test", true);
  }

  TrajectoryPoint create_trajectory_point(double x, double y, double velocity)
  {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    point.pose.position.z = 0.0;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = velocity;
    return point;
  }

  TrajectoryPoints create_straight_trajectory(
    double start_x, double end_x, double step, double velocity)
  {
    TrajectoryPoints trajectory;
    for (double x = start_x; x <= end_x + 1e-6; x += step) {
      trajectory.push_back(create_trajectory_point(x, 0.0, velocity));
    }
    return trajectory;
  }

  void set_odometry(double x, double y, double velocity)
  {
    auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
    odometry->pose.pose.position.x = x;
    odometry->pose.pose.position.y = y;
    odometry->pose.pose.position.z = 0.0;
    odometry->pose.pose.orientation.w = 1.0;
    odometry->twist.twist.linear.x = velocity;
    data_->current_odometry = odometry;
  }

  void set_acceleration(double acceleration)
  {
    auto accel = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
    accel->accel.accel.linear.x = acceleration;
    data_->current_acceleration = accel;
  }

  void set_predicted_objects(const PredictedObjects & objects)
  {
    data_->predicted_objects = std::make_shared<PredictedObjects>(objects);
  }

  void set_pointcloud(const pcl::PointCloud<pcl::PointXYZ> & cloud)
  {
    auto ros_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud, *ros_cloud);
    ros_cloud->header.frame_id = "base_link";
    ros_cloud->header.stamp = node_->get_clock()->now();
    data_->obstacle_pointcloud = ros_cloud;
  }

  PredictedObject create_object_on_trajectory(
    double x, double y, double size_x, double size_y, double velocity)
  {
    PredictedObject object;
    object.kinematics.initial_pose_with_covariance.pose.position.x = x;
    object.kinematics.initial_pose_with_covariance.pose.position.y = y;
    object.kinematics.initial_pose_with_covariance.pose.position.z = 0.0;
    object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
    object.kinematics.initial_twist_with_covariance.twist.linear.x = velocity;
    object.shape.type = Shape::BOUNDING_BOX;
    object.shape.dimensions.x = size_x;
    object.shape.dimensions.y = size_y;
    object.shape.dimensions.z = 1.0;
    ObjectClassification classification;
    classification.label = ObjectClassification::CAR;
    classification.probability = 1.0;
    object.classification.push_back(classification);
    return object;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::unique_ptr<ObstacleStop> plugin_;
  trajectory_modifier_params::Params params_;
  std::shared_ptr<TrajectoryModifierData> data_;
};

TEST_F(ObstacleStopIntegrationTest, ModifyTrajectory_WhenDisabled_ReturnsFalse)
{
  // Arrange
  params_.use_obstacle_stop = false;
  plugin_->update_params(params_);
  set_odometry(0.0, 0.0, 0.0);
  set_acceleration(0.0);
  auto trajectory = create_straight_trajectory(0.0, 50.0, 1.0, 5.0);

  // Act
  const bool result = plugin_->modify_trajectory(trajectory);

  // Assert
  EXPECT_FALSE(result);
}

TEST_F(ObstacleStopIntegrationTest, ModifyTrajectory_NoCollision_ReturnsFalse)
{
  // Arrange
  set_odometry(0.0, 0.0, 0.0);
  set_acceleration(0.0);
  // No objects or pointcloud set -> no collision
  auto trajectory = create_straight_trajectory(0.0, 50.0, 1.0, 5.0);

  // Act
  const bool result = plugin_->modify_trajectory(trajectory);

  // Assert
  EXPECT_FALSE(result);
}

TEST_F(ObstacleStopIntegrationTest, OnlyObjectCollision_DetectsAndStops)
{
  // Arrange
  params_.obstacle_stop.use_objects = true;
  params_.obstacle_stop.use_pointcloud = false;
  params_.obstacle_stop.enable_stop = true;
  plugin_->update_params(params_);

  set_odometry(0.0, 0.0, 0.0);
  set_acceleration(0.0);

  PredictedObjects objects;
  objects.objects.push_back(create_object_on_trajectory(20.0, 0.0, 2.0, 2.0, 0.0));
  set_predicted_objects(objects);

  auto trajectory = create_straight_trajectory(0.0, 50.0, 1.0, 5.0);

  // Act
  const bool result = plugin_->modify_trajectory(trajectory);

  // Assert
  EXPECT_TRUE(result);
  // apply_stopping uses spline interpolation; final velocity may be small but non-zero
  EXPECT_LT(trajectory.back().longitudinal_velocity_mps, 0.1);
}

TEST_F(ObstacleStopIntegrationTest, OnlyPcdCollision_DetectsAndStops)
{
  // Arrange
  params_.obstacle_stop.use_objects = false;
  params_.obstacle_stop.use_pointcloud = true;
  params_.obstacle_stop.enable_stop = true;
  params_.obstacle_stop.pointcloud.voxel_grid_filter.x = 0.1;
  params_.obstacle_stop.pointcloud.voxel_grid_filter.y = 0.1;
  params_.obstacle_stop.pointcloud.voxel_grid_filter.z = 0.1;
  params_.obstacle_stop.pointcloud.clustering.min_size = 3;
  params_.obstacle_stop.pointcloud.clustering.min_height = 0.0;
  plugin_->update_params(params_);

  set_odometry(0.0, 0.0, 0.0);
  set_acceleration(0.0);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  // Create a 2D cluster spread wide enough to survive voxel grid downsampling
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      cloud.push_back(pcl::PointXYZ(
        15.0f + static_cast<float>(i) * 0.15f, -0.3f + static_cast<float>(j) * 0.15f, 0.5f));
    }
  }
  set_pointcloud(cloud);

  auto trajectory = create_straight_trajectory(0.0, 50.0, 1.0, 5.0);

  // Act
  const bool result = plugin_->modify_trajectory(trajectory);

  // Assert
  EXPECT_TRUE(result);
  EXPECT_LT(trajectory.back().longitudinal_velocity_mps, 0.1);
}

TEST_F(ObstacleStopIntegrationTest, CloseCollision_CreatesMinimalTrajectory)
{
  // Arrange: object very close so target_stop_arc_length < arrived_distance_threshold
  params_.obstacle_stop.use_objects = true;
  params_.obstacle_stop.use_pointcloud = false;
  params_.obstacle_stop.enable_stop = true;
  params_.obstacle_stop.arrived_distance_threshold_m = 10.0;
  params_.obstacle_stop.stop_margin_m = 0.0;
  plugin_->update_params(params_);

  set_odometry(0.0, 0.0, 0.0);
  set_acceleration(0.0);

  PredictedObjects objects;
  // Object right in front, collision arc_length will be small
  objects.objects.push_back(create_object_on_trajectory(5.0, 0.0, 4.0, 4.0, 0.0));
  set_predicted_objects(objects);

  auto trajectory = create_straight_trajectory(0.0, 50.0, 1.0, 5.0);

  // Act
  const bool result = plugin_->modify_trajectory(trajectory);

  // Assert: should create minimal 2-point trajectory at ego position
  EXPECT_TRUE(result);
  EXPECT_EQ(trajectory.size(), 2u);
  EXPECT_NEAR(trajectory[0].longitudinal_velocity_mps, 0.0, 1e-3);
  EXPECT_NEAR(trajectory[1].longitudinal_velocity_mps, 0.0, 1e-3);
}

TEST_F(ObstacleStopIntegrationTest, CheckPredictedObjects_Disabled_ReturnsNoCollision)
{
  // Arrange: use_objects = false
  params_.obstacle_stop.use_objects = false;
  params_.obstacle_stop.use_pointcloud = false;
  plugin_->update_params(params_);

  set_odometry(0.0, 0.0, 0.0);
  set_acceleration(0.0);

  PredictedObjects objects;
  objects.objects.push_back(create_object_on_trajectory(10.0, 0.0, 2.0, 2.0, 0.0));
  set_predicted_objects(objects);

  auto trajectory = create_straight_trajectory(0.0, 50.0, 1.0, 5.0);

  // Act
  const bool result = plugin_->modify_trajectory(trajectory);

  // Assert
  EXPECT_FALSE(result);
}

TEST_F(ObstacleStopIntegrationTest, PublishDebugData_NoCollision_DoesNotCrash)
{
  // Arrange
  set_odometry(0.0, 0.0, 0.0);
  set_acceleration(0.0);
  auto trajectory = create_straight_trajectory(0.0, 50.0, 1.0, 5.0);
  plugin_->modify_trajectory(trajectory);

  // Act & Assert: should not crash
  EXPECT_NO_THROW(plugin_->publish_debug_data("test_ns"));
}
