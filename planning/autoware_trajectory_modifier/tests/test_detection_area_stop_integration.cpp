// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/detection_area_stop.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <utility>

namespace
{
using autoware::trajectory_modifier::TrajectoryModifierContext;
using autoware::trajectory_modifier::plugin::DetectionAreaStop;
using autoware::trajectory_modifier::plugin::InputData;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

TrajectoryPoints make_trajectory(const double y = 0.0)
{
  TrajectoryPoints trajectory;
  for (double x = 0.0; x <= 30.0; x += 1.0) {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 5.0F;
    trajectory.push_back(point);
  }
  return trajectory;
}

Odometry::ConstSharedPtr make_odometry(const double y = 0.0)
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 5.0;
  return std::make_shared<const Odometry>(odometry);
}

PredictedObjects::ConstSharedPtr make_car_in_area()
{
  PredictedObjects objects;
  objects.header.frame_id = "map";
  PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 6.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 2.0;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 1.5;
  ObjectClassification classification;
  classification.label = ObjectClassification::CAR;
  classification.probability = 1.0F;
  object.classification.push_back(classification);
  objects.objects.push_back(object);
  return std::make_shared<const PredictedObjects>(objects);
}

sensor_msgs::msg::PointCloud2::ConstSharedPtr make_pointcloud_in_area()
{
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header.frame_id = "map";
  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(*cloud, "z");
  *x = 6.0F;
  *y = 0.0F;
  *z = 0.0F;
  return cloud;
}

std::shared_ptr<lanelet::LaneletMap> make_map()
{
  const auto point = [](const double x, const double y) {
    return lanelet::Point3d(lanelet::utils::getId(), x, y, 0.0);
  };
  lanelet::LineString3d stop_line(lanelet::utils::getId(), {point(10.0, -3.0), point(10.0, 3.0)});
  lanelet::Polygon3d area;
  area.push_back(point(4.0, -2.0));
  area.push_back(point(4.0, 2.0));
  area.push_back(point(8.0, 2.0));
  area.push_back(point(8.0, -2.0));
  const auto detection_area = lanelet::autoware::DetectionArea::make(
    lanelet::utils::getId(), {}, lanelet::Polygons3d{area}, stop_line);

  lanelet::LineString3d left(
    lanelet::utils::getId(), {point(0.0, -5.0), point(30.0, -5.0)});
  lanelet::LineString3d right(
    lanelet::utils::getId(), {point(0.0, 5.0), point(30.0, 5.0)});
  lanelet::Lanelet lane(lanelet::utils::getId(), left, right);
  lane.addRegulatoryElement(detection_area);
  return lanelet::utils::createMap({lane});
}

LaneletRoute::ConstSharedPtr make_route(const lanelet::Id lane_id)
{
  auto route = std::make_shared<LaneletRoute>();
  LaneletSegment segment;
  segment.preferred_primitive.id = lane_id;
  route->segments.push_back(segment);
  return route;
}

InputData make_input(
  std::shared_ptr<lanelet::LaneletMap> map = nullptr, LaneletRoute::ConstSharedPtr route = nullptr,
  PredictedObjects::ConstSharedPtr objects = nullptr,
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud = nullptr, const double y = 0.0)
{
  InputData input;
  input.current_odometry = make_odometry(y);
  input.lanelet_map = std::move(map);
  input.route = std::move(route);
  input.predicted_objects = std::move(objects);
  input.obstacle_pointcloud = std::move(pointcloud);
  return input;
}
}  // namespace

class DetectionAreaStopIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    const auto test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    autoware::test_utils::updateNodeOptions(
      options, {test_utils_dir + "/config/test_vehicle_info.param.yaml"});
    node_ = std::make_shared<rclcpp::Node>("test_detection_area_stop_node", options);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();
    params_.use_detection_area_stop = true;
    params_.detection_area.target_filtering.pointcloud = false;
    params_.detection_area.target_filtering.car = true;
    params_.detection_area.stop_margin = 0.0;
    context_ = std::make_shared<TrajectoryModifierContext>(node_.get());
    plugin_ = std::make_unique<DetectionAreaStop>();
    plugin_->initialize("test_detection_area_stop", node_.get(), time_keeper_, context_, params_);
  }

  void TearDown() override
  {
    plugin_.reset();
    context_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::shared_ptr<TrajectoryModifierContext> context_;
  std::unique_ptr<DetectionAreaStop> plugin_;
  trajectory_modifier_params::Params params_;
};

TEST_F(DetectionAreaStopIntegrationTest, DisabledPluginDoesNotModifyTrajectory)
{
  params_.use_detection_area_stop = false;
  plugin_->update_params(params_);
  auto trajectory = make_trajectory();
  const auto original = trajectory;
  const auto map = make_map();
  auto input = make_input(map, make_route(map->laneletLayer.begin()->id()), make_car_in_area());
  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  EXPECT_EQ(trajectory, original);
}

TEST_F(DetectionAreaStopIntegrationTest, MissingMapOrRouteIsFailOpen)
{
  auto trajectory = make_trajectory();
  const auto original = trajectory;
  auto input = make_input();
  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  EXPECT_EQ(trajectory, original);
}

TEST_F(DetectionAreaStopIntegrationTest, DetectedObjectStopsAtDetectionAreaStopLine)
{
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  auto trajectory = make_trajectory();
  plugin_->begin_cycle(input);
  EXPECT_TRUE(plugin_->modify_trajectory(trajectory, input));
  EXPECT_GT(trajectory.at(4).longitudinal_velocity_mps, 0.0F);
  EXPECT_EQ(trajectory.at(10).longitudinal_velocity_mps, 0.0F);
}

TEST_F(DetectionAreaStopIntegrationTest, CandidateWithoutStopLineIntersectionIsUnchanged)
{
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area(), nullptr, 10.0);
  auto trajectory = make_trajectory(10.0);
  const auto original = trajectory;
  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  EXPECT_EQ(trajectory, original);
}

TEST_F(DetectionAreaStopIntegrationTest, PointCloudCanTriggerDetectionWithoutObjects)
{
  params_.detection_area.target_filtering.pointcloud = true;
  params_.detection_area.target_filtering.car = false;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, nullptr, make_pointcloud_in_area());
  auto trajectory = make_trajectory();
  plugin_->begin_cycle(input);
  EXPECT_TRUE(plugin_->modify_trajectory(trajectory, input));
  EXPECT_EQ(trajectory.at(10).longitudinal_velocity_mps, 0.0F);
}

TEST_F(DetectionAreaStopIntegrationTest, CandidatesUseIndependentStopLineIntersections)
{
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  auto stopping_trajectory = make_trajectory();
  auto non_intersecting_trajectory = make_trajectory(10.0);
  const auto original = non_intersecting_trajectory;
  plugin_->begin_cycle(input);
  EXPECT_TRUE(plugin_->modify_trajectory(stopping_trajectory, input));
  EXPECT_FALSE(plugin_->modify_trajectory(non_intersecting_trajectory, input));
  EXPECT_EQ(non_intersecting_trajectory, original);
}

TEST_F(DetectionAreaStopIntegrationTest, DebugPublishersAreAvailable)
{
  const auto marker_topic =
    node_->get_node_topics_interface()->resolve_topic_name("~/detection_area_stop/debug/marker");
  const auto text_topic =
    node_->get_node_topics_interface()->resolve_topic_name("~/detection_area_stop/debug/text");
  EXPECT_EQ(node_->count_publishers(marker_topic), 1U);
  EXPECT_EQ(node_->count_publishers(text_topic), 1U);

  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  auto trajectory = make_trajectory();
  plugin_->begin_cycle(input);
  plugin_->modify_trajectory(trajectory, input);
  EXPECT_NO_THROW(plugin_->publish_debug_data("trajectory_0"));
}
