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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/detection_area_stop.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

namespace
{
using autoware::trajectory_modifier::TrajectoryModifierContext;
using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::plugin::DetectionAreaStop;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

constexpr double stop_line_x = 10.0;

struct StoppedVelocityCase
{
  double velocity_x;
  double velocity_y;
  double velocity_z;
  double threshold;
  bool is_stopped;
};

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

TrajectoryPoints make_short_trajectory(const double end_x = 10.0, const double y = 0.0)
{
  TrajectoryPoints trajectory;
  for (double x = 0.0; x <= end_x; x += 1.0) {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 5.0F;
    trajectory.push_back(point);
  }
  return trajectory;
}

Odometry::ConstSharedPtr make_stopped_odometry_at(const double x, const double y = 0.0)
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 0.0;
  return std::make_shared<const Odometry>(odometry);
}

Odometry::ConstSharedPtr make_odometry_at(
  const double x, const double y = 0.0, const double velocity = 5.0)
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = velocity;
  return std::make_shared<const Odometry>(odometry);
}

Odometry::ConstSharedPtr make_odometry(const double y = 0.0)
{
  return make_odometry_at(0.0, y, 5.0);
}

Odometry::ConstSharedPtr make_stopped_odometry(const double y = 0.0)
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 0.0;
  return std::make_shared<const Odometry>(odometry);
}

AccelWithCovarianceStamped::ConstSharedPtr make_acceleration()
{
  AccelWithCovarianceStamped acceleration;
  return std::make_shared<const AccelWithCovarianceStamped>(acceleration);
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
  lanelet::LineString3d stop_line(
    lanelet::utils::getId(), {point(stop_line_x, -3.0), point(stop_line_x, 3.0)});
  lanelet::Polygon3d area;
  area.push_back(point(4.0, -2.0));
  area.push_back(point(4.0, 2.0));
  area.push_back(point(8.0, 2.0));
  area.push_back(point(8.0, -2.0));
  const auto detection_area = lanelet::autoware::DetectionArea::make(
    lanelet::utils::getId(), {}, lanelet::Polygons3d{area}, stop_line);

  lanelet::LineString3d left(lanelet::utils::getId(), {point(0.0, -5.0), point(30.0, -5.0)});
  lanelet::LineString3d right(lanelet::utils::getId(), {point(0.0, 5.0), point(30.0, 5.0)});
  lanelet::Lanelet lane(lanelet::utils::getId(), left, right);
  lane.addRegulatoryElement(detection_area);
  return lanelet::utils::createMap({lane});
}

LaneletRoute::ConstSharedPtr make_route(
  const lanelet::Id preferred_id, const std::vector<lanelet::Id> & primitive_ids = {})
{
  auto route = std::make_shared<LaneletRoute>();
  LaneletSegment segment;
  segment.preferred_primitive.id = preferred_id;
  autoware_planning_msgs::msg::LaneletPrimitive preferred;
  preferred.id = preferred_id;
  segment.primitives.push_back(preferred);
  for (const auto id : primitive_ids) {
    autoware_planning_msgs::msg::LaneletPrimitive primitive;
    primitive.id = id;
    segment.primitives.push_back(primitive);
  }
  route->segments.push_back(segment);
  return route;
}

TrajectoryPoints with_zero_velocity(TrajectoryPoints trajectory)
{
  for (auto & point : trajectory) {
    point.longitudinal_velocity_mps = 0.0F;
  }
  return trajectory;
}

TrajectoryModifierData make_input(
  std::shared_ptr<lanelet::LaneletMap> map = nullptr, LaneletRoute::ConstSharedPtr route = nullptr,
  PredictedObjects::ConstSharedPtr objects = nullptr,
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud = nullptr, const double y = 0.0)
{
  TrajectoryModifierData input;
  input.current_odometry = make_odometry(y);
  input.current_acceleration = make_acceleration();
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
    const auto test_utils_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
    autoware::test_utils::updateNodeOptions(
      options, {test_utils_dir + "/config/test_vehicle_info.param.yaml"});
    node_ = std::make_shared<rclcpp::Node>("test_detection_area_stop_node", options);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();
    params_.use_detection_area_stop = true;
    params_.trajectory_time_step = 0.1;
    params_.stopping_constraints.nominal_deceleration = 1.0;
    // Keep the nominal 5 m/s fixture stoppable with the 0.5 s response delay.
    // Unstoppable-policy tests override the shared deceleration below.
    params_.stopping_constraints.maximum_deceleration = 5.0;
    params_.stopping_constraints.delay_response_time = 0.5;
    params_.stopping_constraints.jerk_limit = 3.0;
    params_.stopping_constraints.arrived_distance_threshold = 0.5;
    params_.detection_area_stop.target_filtering.pointcloud = false;
    params_.detection_area_stop.target_filtering.car = true;
    params_.detection_area_stop.stop_margin = 0.0;
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

  bool process_candidate(TrajectoryPoints & trajectory, TrajectoryModifierData & snapshot)
  {
    // Match the framework: each candidate gets a fresh copy with its position in this callback.
    auto input = snapshot;
    const auto result = plugin_->process(trajectory, input);
    ++snapshot.candidate_index;
    return result == autoware::trajectory_modifier::plugin::ProcessingResult::Modified;
  }

  void expect_stop_before_stop_line(const TrajectoryPoints & trajectory) const
  {
    ASSERT_FALSE(trajectory.empty());
    EXPECT_GT(trajectory.front().longitudinal_velocity_mps, 0.0F);
    EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
    EXPECT_LT(trajectory.back().pose.position.x, stop_line_x);

    const auto expected_stop_margin =
      params_.detection_area_stop.stop_margin + context_->vehicle_info.max_longitudinal_offset_m;
    EXPECT_NEAR(stop_line_x - trajectory.back().pose.position.x, expected_stop_margin, 0.5);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::shared_ptr<TrajectoryModifierContext> context_;
  std::unique_ptr<DetectionAreaStop> plugin_;
  trajectory_modifier_params::Params params_;
};

class DetectionAreaStopStoppedVelocityTest
: public DetectionAreaStopIntegrationTest,
  public ::testing::WithParamInterface<StoppedVelocityCase>
{
protected:
  void SetUp() override
  {
    DetectionAreaStopIntegrationTest::SetUp();
    params_.stopping_constraints.ego_stopped_vel_th = GetParam().threshold;
    params_.detection_area_stop.suppress_pass_judge_when_stopping = true;
    plugin_->update_params(params_);
  }

  Odometry::ConstSharedPtr make_noisy_odometry_at_stop() const
  {
    // Slightly overshoot the nominal stop position to exercise holding at ego and the
    // stopped exception to the braking-distance check, within the stop-line tolerance.
    const double ego_x = stop_line_x - params_.detection_area_stop.stop_margin -
                         context_->vehicle_info.max_longitudinal_offset_m + 0.2;
    auto odometry = std::make_shared<Odometry>(*make_odometry_at(ego_x, 0.0, GetParam().velocity_x));
    odometry->twist.twist.linear.y = GetParam().velocity_y;
    odometry->twist.twist.linear.z = GetParam().velocity_z;
    return odometry;
  }
};

TEST_F(DetectionAreaStopIntegrationTest, DisabledPluginDoesNotModifyTrajectory)
{
  params_.use_detection_area_stop = false;
  plugin_->update_params(params_);
  auto trajectory = make_trajectory();
  const auto original = trajectory;
  const auto map = make_map();
  auto input = make_input(map, make_route(map->laneletLayer.begin()->id()), make_car_in_area());
  input.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(trajectory, input));
  EXPECT_EQ(trajectory, original);
}

TEST_F(DetectionAreaStopIntegrationTest, MissingMapOrRouteIsFailOpen)
{
  auto trajectory = make_trajectory();
  const auto original = trajectory;
  auto input = make_input();
  input.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(trajectory, input));
  EXPECT_EQ(trajectory, original);
}

TEST_F(DetectionAreaStopIntegrationTest, DetectedObjectStopsAtDetectionAreaStopLine)
{
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  auto trajectory = make_trajectory();
  input.candidate_index = 0U;
  EXPECT_TRUE(process_candidate(trajectory, input));
  expect_stop_before_stop_line(trajectory);
}

TEST_F(DetectionAreaStopIntegrationTest, ClearedObstaclePreservesUpstreamZeroVelocity)
{
  params_.detection_area_stop.state_clear_time = 0.05;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input_with_obstacle = make_input(map, route, make_car_in_area());
  auto stopping_trajectory = make_trajectory();
  input_with_obstacle.candidate_index = 0U;
  EXPECT_TRUE(process_candidate(stopping_trajectory, input_with_obstacle));
  expect_stop_before_stop_line(stopping_trajectory);

  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  auto input_without_obstacle = make_input(map, route);
  input_without_obstacle.current_odometry = make_stopped_odometry();
  auto released_trajectory = with_zero_velocity(make_trajectory());
  const auto original_back_x = released_trajectory.back().pose.position.x;
  const auto original_size = released_trajectory.size();
  input_without_obstacle.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(released_trajectory, input_without_obstacle));
  ASSERT_EQ(released_trajectory.size(), original_size);
  EXPECT_DOUBLE_EQ(released_trajectory.back().pose.position.x, original_back_x);
  EXPECT_FLOAT_EQ(released_trajectory.front().longitudinal_velocity_mps, 0.0F);
  EXPECT_FLOAT_EQ(released_trajectory.back().longitudinal_velocity_mps, 0.0F);
}

TEST_F(DetectionAreaStopIntegrationTest, HoldDoesNotModifyUnrelatedCandidate)
{
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input_with_obstacle = make_input(map, route, make_car_in_area());
  auto stopping_trajectory = make_trajectory();
  input_with_obstacle.candidate_index = 0U;
  EXPECT_TRUE(process_candidate(stopping_trajectory, input_with_obstacle));

  auto stopped_input = make_input(map, route, make_car_in_area(), nullptr, 10.0);
  stopped_input.current_odometry = make_stopped_odometry(10.0);
  auto offset_trajectory = make_trajectory(10.0);
  const auto original = offset_trajectory;
  stopped_input.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(offset_trajectory, stopped_input));
  EXPECT_EQ(offset_trajectory, original);
}

TEST_F(DetectionAreaStopIntegrationTest, CollapsedStopTrajectoryIsLeftToUpstream)
{
  params_.detection_area_stop.state_clear_time = 0.05;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input_with_obstacle = make_input(map, route, make_car_in_area());
  auto stopping_trajectory = make_trajectory();
  input_with_obstacle.candidate_index = 0U;
  EXPECT_TRUE(process_candidate(stopping_trajectory, input_with_obstacle));

  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  auto input_without_obstacle = make_input(map, route);
  input_without_obstacle.current_odometry = make_stopped_odometry();
  TrajectoryPoints collapsed;
  for (int i = 0; i < 3; ++i) {
    TrajectoryPoint point;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 0.0F;
    collapsed.push_back(point);
  }
  const auto original = collapsed;
  input_without_obstacle.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(collapsed, input_without_obstacle));
  EXPECT_EQ(collapsed, original);
  EXPECT_EQ(collapsed.size(), 3U);
}

TEST_F(DetectionAreaStopIntegrationTest, ClearedObstacleDoesNotFabricatePathPastTrajectoryEnd)
{
  params_.detection_area_stop.state_clear_time = 0.05;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input_with_obstacle = make_input(map, route, make_car_in_area());
  auto stopping_trajectory = make_short_trajectory();
  input_with_obstacle.candidate_index = 0U;
  EXPECT_TRUE(process_candidate(stopping_trajectory, input_with_obstacle));
  expect_stop_before_stop_line(stopping_trajectory);

  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  constexpr double ego_x = 12.0;
  auto input_without_obstacle = make_input(map, route);
  input_without_obstacle.current_odometry = make_stopped_odometry_at(ego_x);
  auto released_trajectory = with_zero_velocity(make_short_trajectory());
  const auto original_back_x = released_trajectory.back().pose.position.x;
  const auto original_size = released_trajectory.size();
  input_without_obstacle.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(released_trajectory, input_without_obstacle));
  ASSERT_EQ(released_trajectory.size(), original_size);
  EXPECT_DOUBLE_EQ(released_trajectory.back().pose.position.x, original_back_x);
  EXPECT_LT(released_trajectory.back().pose.position.x, ego_x + 1.0);
  EXPECT_FLOAT_EQ(released_trajectory.front().longitudinal_velocity_mps, 0.0F);
}

TEST_F(DetectionAreaStopIntegrationTest, CandidateWithoutStopLineIntersectionIsUnchanged)
{
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area(), nullptr, 10.0);
  auto trajectory = make_trajectory(10.0);
  const auto original = trajectory;
  input.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(trajectory, input));
  EXPECT_EQ(trajectory, original);
}

TEST_F(DetectionAreaStopIntegrationTest, PointCloudCanTriggerDetectionWithoutObjects)
{
  params_.detection_area_stop.target_filtering.pointcloud = true;
  params_.detection_area_stop.target_filtering.car = false;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, nullptr, make_pointcloud_in_area());
  auto trajectory = make_trajectory();
  input.candidate_index = 0U;
  EXPECT_TRUE(process_candidate(trajectory, input));
  expect_stop_before_stop_line(trajectory);
}

TEST_F(DetectionAreaStopIntegrationTest, CandidatesUseIndependentStopLineIntersections)
{
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  auto stopping_trajectory = make_trajectory();
  auto non_intersecting_trajectory = make_trajectory(10.0);
  const auto original = non_intersecting_trajectory;
  input.candidate_index = 0U;
  EXPECT_TRUE(process_candidate(stopping_trajectory, input));
  EXPECT_FALSE(process_candidate(non_intersecting_trajectory, input));
  EXPECT_EQ(non_intersecting_trajectory, original);
  expect_stop_before_stop_line(stopping_trajectory);
}

TEST_F(DetectionAreaStopIntegrationTest, DebugPublishersAreAvailable)
{
  const auto marker_topic =
    node_->get_node_topics_interface()->resolve_topic_name("~/detection_area_stop/debug/marker");
  const auto text_topic =
    node_->get_node_topics_interface()->resolve_topic_name("~/detection_area_stop/debug/text");
  rclcpp::spin_some(node_);
  EXPECT_FALSE(node_->get_publishers_info_by_topic(marker_topic).empty());
  EXPECT_FALSE(node_->get_publishers_info_by_topic(text_topic).empty());

  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  auto trajectory = make_trajectory();
  input.candidate_index = 0U;
  process_candidate(trajectory, input);
  EXPECT_NO_THROW(plugin_->publish_debug_data("trajectory_0"));
}

TEST_F(DetectionAreaStopIntegrationTest, SharedResponseDelayUpdateChangesStoppingFeasibility)
{
  params_.detection_area_stop.unstoppable_policy = "go";
  params_.stopping_constraints.delay_response_time = 2.0;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  auto trajectory = make_trajectory();
  const auto original = trajectory;
  input.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(trajectory, input));
  EXPECT_EQ(trajectory, original);

  params_.stopping_constraints.delay_response_time = 0.0;
  plugin_->update_params(params_);
  input.candidate_index = 0U;
  trajectory = make_trajectory();
  ASSERT_TRUE(process_candidate(trajectory, input));
  expect_stop_before_stop_line(trajectory);
}

TEST_F(DetectionAreaStopIntegrationTest, UnstoppableGoPolicyDoesNotInsertStop)
{
  params_.detection_area_stop.unstoppable_policy = "go";
  params_.stopping_constraints.maximum_deceleration = 0.1;
  params_.stopping_constraints.delay_response_time = 0.5;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  auto trajectory = make_trajectory();
  const auto original = trajectory;
  input.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(trajectory, input));
  EXPECT_EQ(trajectory, original);
}

TEST_F(DetectionAreaStopIntegrationTest, UnstoppableForceStopStillInsertsStop)
{
  params_.detection_area_stop.unstoppable_policy = "force_stop";
  params_.stopping_constraints.maximum_deceleration = 0.1;
  params_.stopping_constraints.delay_response_time = 0.5;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  auto trajectory = make_trajectory();
  input.candidate_index = 0U;
  EXPECT_TRUE(process_candidate(trajectory, input));
  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
}

TEST_F(DetectionAreaStopIntegrationTest, UnstoppableStopAfterStoplineMovesStopForward)
{
  params_.detection_area_stop.unstoppable_policy = "stop_after_stopline";
  params_.stopping_constraints.maximum_deceleration = 0.1;
  params_.stopping_constraints.delay_response_time = 0.5;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  auto trajectory = make_trajectory();
  input.candidate_index = 0U;
  EXPECT_TRUE(process_candidate(trajectory, input));
  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
  const auto expected_nominal_stop = stop_line_x - params_.detection_area_stop.stop_margin -
                                     context_->vehicle_info.max_longitudinal_offset_m;
  EXPECT_GT(trajectory.back().pose.position.x, expected_nominal_stop + 0.5);
}

TEST_F(DetectionAreaStopIntegrationTest, DeadLineIgnoresDetectionAreaAfterPassing)
{
  params_.detection_area_stop.use_dead_line = true;
  params_.detection_area_stop.dead_line_margin = 5.0;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  input.current_odometry = make_odometry_at(20.0);
  auto trajectory = make_trajectory();
  const auto original = trajectory;
  input.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(trajectory, input));
  EXPECT_EQ(trajectory, original);
}

TEST_F(DetectionAreaStopIntegrationTest, PrecedingPartialStopIsNotReleased)
{
  params_.detection_area_stop.state_clear_time = 0.05;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input_with_obstacle = make_input(map, route, make_car_in_area());
  auto stopping_trajectory = make_trajectory();
  input_with_obstacle.candidate_index = 0U;
  EXPECT_TRUE(process_candidate(stopping_trajectory, input_with_obstacle));

  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  auto input_without_obstacle = make_input(map, route);
  input_without_obstacle.current_odometry = make_stopped_odometry();
  auto os_stopped = make_trajectory();
  for (size_t i = 0; i < os_stopped.size(); ++i) {
    os_stopped[i].longitudinal_velocity_mps = i < 10 ? 5.0F : 0.0F;
  }
  const auto original = os_stopped;
  input_without_obstacle.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(os_stopped, input_without_obstacle));
  EXPECT_EQ(os_stopped, original);
}

TEST_F(DetectionAreaStopIntegrationTest, LaterCandidateCanEnterStopState)
{
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  auto offset_trajectory = make_trajectory(10.0);
  auto stopping_trajectory = make_trajectory();
  const auto original_offset = offset_trajectory;
  input.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(offset_trajectory, input));
  EXPECT_EQ(offset_trajectory, original_offset);
  EXPECT_TRUE(process_candidate(stopping_trajectory, input));
  expect_stop_before_stop_line(stopping_trajectory);
}

TEST_F(DetectionAreaStopIntegrationTest, RegistersDetectionAreaOnNonPreferredPrimitive)
{
  auto map = make_map();
  const auto da_lane_id = map->laneletLayer.begin()->id();
  const auto point = [](const double x, const double y) {
    return lanelet::Point3d(lanelet::utils::getId(), x, y, 0.0);
  };
  lanelet::LineString3d left(lanelet::utils::getId(), {point(0.0, -15.0), point(30.0, -15.0)});
  lanelet::LineString3d right(lanelet::utils::getId(), {point(0.0, -10.0), point(30.0, -10.0)});
  lanelet::Lanelet extra(lanelet::utils::getId(), left, right);
  map->add(extra);

  const auto route = make_route(extra.id(), {da_lane_id});
  auto input = make_input(map, route, make_car_in_area());
  auto trajectory = make_trajectory();
  input.candidate_index = 0U;
  EXPECT_TRUE(process_candidate(trajectory, input));
  expect_stop_before_stop_line(trajectory);
}

TEST_P(DetectionAreaStopStoppedVelocityTest, StoppedEgoHoldsInsteadOfTakingUnstoppableGoPolicy)
{
  params_.detection_area_stop.unstoppable_policy = "go";
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  input.current_odometry = make_noisy_odometry_at_stop();
  input.candidate_index = 0U;
  auto trajectory = make_trajectory();
  const auto original = trajectory;

  ASSERT_EQ(process_candidate(trajectory, input), GetParam().is_stopped);
  if (GetParam().is_stopped) {
    EXPECT_FLOAT_EQ(trajectory.front().longitudinal_velocity_mps, 0.0F);
    EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
    EXPECT_NEAR(trajectory.back().pose.position.x, input.current_odometry->pose.pose.position.x, 0.1);
  } else {
    EXPECT_EQ(trajectory, original);
  }
}

TEST_P(DetectionAreaStopStoppedVelocityTest, SuppressPassJudgeKeepsObservedStopWithOdometryNoise)
{
  params_.detection_area_stop.unstoppable_policy = "force_stop";
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input_with_obstacle = make_input(map, route, make_car_in_area());
  input_with_obstacle.current_odometry = make_noisy_odometry_at_stop();
  input_with_obstacle.candidate_index = 0U;
  auto observed_stop = with_zero_velocity(make_trajectory());
  EXPECT_FALSE(process_candidate(observed_stop, input_with_obstacle));

  // Clear the obstacle immediately; an observed STOP must stay latched without relying
  // on the obstacle grace period or waiting for wall-clock time to advance.
  params_.detection_area_stop.state_clear_time = 0.0;
  plugin_->update_params(params_);
  auto input_without_obstacle = make_input(map, route);
  input_without_obstacle.current_odometry = make_noisy_odometry_at_stop();
  input_without_obstacle.candidate_index = 0U;
  auto trajectory = make_trajectory();
  const auto original = trajectory;

  ASSERT_EQ(process_candidate(trajectory, input_without_obstacle), GetParam().is_stopped);
  if (GetParam().is_stopped) {
    EXPECT_FLOAT_EQ(trajectory.front().longitudinal_velocity_mps, 0.0F);
    EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
    EXPECT_NEAR(
      trajectory.back().pose.position.x, input_without_obstacle.current_odometry->pose.pose.position.x,
      0.1);
  } else {
    EXPECT_EQ(trajectory, original);
  }
}

INSTANTIATE_TEST_SUITE_P(
  StoppedSpeedThreshold, DetectionAreaStopStoppedVelocityTest,
  ::testing::Values(
    StoppedVelocityCase{0.0, 0.0, 0.0, 0.1, true},
    StoppedVelocityCase{0.02, 0.0, 0.0, 0.1, true},
    StoppedVelocityCase{-0.02, 0.0, 0.0, 0.1, true},
    StoppedVelocityCase{0.1, 0.0, 0.0, 0.1, true},
    StoppedVelocityCase{-0.1, 0.0, 0.0, 0.1, true},
    StoppedVelocityCase{0.11, 0.0, 0.0, 0.1, false},
    StoppedVelocityCase{-0.11, 0.0, 0.0, 0.1, false},
    StoppedVelocityCase{0.05, 0.05, 0.05, 0.1, true},
    StoppedVelocityCase{0.06, 0.06, 0.06, 0.1, false},
    StoppedVelocityCase{0.05, 0.0, 0.0, 0.04, false},
    StoppedVelocityCase{0.15, 0.0, 0.0, 0.2, true}));

TEST_F(DetectionAreaStopIntegrationTest, SuppressPassJudgeKeepsStopAndDoesNotRelease)
{
  params_.detection_area_stop.unstoppable_policy = "force_stop";
  params_.detection_area_stop.suppress_pass_judge_when_stopping = true;
  params_.detection_area_stop.state_clear_time = 0.05;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input_with_obstacle = make_input(map, route, make_car_in_area());
  auto stopping_trajectory = make_trajectory();
  input_with_obstacle.candidate_index = 0U;
  EXPECT_TRUE(process_candidate(stopping_trajectory, input_with_obstacle));
  expect_stop_before_stop_line(stopping_trajectory);

  // A speculative candidate cannot latch STOP. Observe the actual stopped ego first.
  input_with_obstacle.current_odometry =
    make_stopped_odometry_at(stopping_trajectory.back().pose.position.x);
  input_with_obstacle.candidate_index = 0U;
  auto observed_stop = with_zero_velocity(make_trajectory());
  EXPECT_FALSE(process_candidate(observed_stop, input_with_obstacle));
  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  auto input_without_obstacle = make_input(map, route);
  input_without_obstacle.current_odometry =
    make_stopped_odometry_at(stopping_trajectory.back().pose.position.x);
  auto zero_trajectory = with_zero_velocity(make_trajectory());
  const auto original = zero_trajectory;
  input_without_obstacle.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(zero_trajectory, input_without_obstacle));
  EXPECT_EQ(zero_trajectory, original);

  auto moving_candidate = make_trajectory();
  EXPECT_TRUE(process_candidate(moving_candidate, input_without_obstacle));
  EXPECT_FLOAT_EQ(moving_candidate.back().longitudinal_velocity_mps, 0.0F);
  EXPECT_NEAR(
    moving_candidate.back().pose.position.x,
    input_without_obstacle.current_odometry->pose.pose.position.x, 0.1);

  auto input_past_line = make_input(map, route, make_car_in_area());
  input_past_line.current_odometry = make_odometry_at(12.0);
  auto trajectory_past_line = make_trajectory();
  input_past_line.candidate_index = 0U;
  const auto original_past_line = trajectory_past_line;
  EXPECT_FALSE(process_candidate(trajectory_past_line, input_past_line));
  EXPECT_EQ(trajectory_past_line, original_past_line);
}

TEST_F(DetectionAreaStopIntegrationTest, WithoutSuppressPastLineObstacleIsIgnoredAfterGo)
{
  params_.detection_area_stop.unstoppable_policy = "force_stop";
  params_.detection_area_stop.suppress_pass_judge_when_stopping = false;
  params_.detection_area_stop.state_clear_time = 0.05;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input_with_obstacle = make_input(map, route, make_car_in_area());
  auto stopping_trajectory = make_trajectory();
  input_with_obstacle.candidate_index = 0U;
  EXPECT_TRUE(process_candidate(stopping_trajectory, input_with_obstacle));

  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  auto input_without_obstacle = make_input(map, route);
  input_without_obstacle.current_odometry =
    make_stopped_odometry_at(stopping_trajectory.back().pose.position.x);
  input_without_obstacle.candidate_index = 0U;
  auto released_trajectory = with_zero_velocity(make_trajectory());
  EXPECT_FALSE(process_candidate(released_trajectory, input_without_obstacle));

  auto input_past_line = make_input(map, route, make_car_in_area());
  input_past_line.current_odometry = make_odometry_at(12.0);
  auto trajectory_past_line = make_trajectory();
  const auto original = trajectory_past_line;
  input_past_line.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(trajectory_past_line, input_past_line));
  EXPECT_EQ(trajectory_past_line, original);
}

TEST_F(DetectionAreaStopIntegrationTest, ReleaseMustNotCancelUnrelatedZeroTrajectory)
{
  params_.detection_area_stop.state_clear_time = 0.01;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  auto trajectory = make_trajectory();
  input.candidate_index = 0U;
  ASSERT_TRUE(process_candidate(trajectory, input));
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  input = make_input(map, route);
  input.current_odometry = make_stopped_odometry();
  auto unrelated = with_zero_velocity(make_trajectory(10.0));
  const auto original = unrelated;
  input.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(unrelated, input));
  EXPECT_EQ(unrelated, original);
}

TEST_F(DetectionAreaStopIntegrationTest, ForwardOffsetMustClearAfterObstacleEpisode)
{
  params_.detection_area_stop.unstoppable_policy = "stop_after_stopline";
  params_.stopping_constraints.maximum_deceleration = 1.0;
  params_.detection_area_stop.state_clear_time = 0.01;
  plugin_->update_params(params_);
  const auto map = make_map();
  const auto route = make_route(map->laneletLayer.begin()->id());
  auto input = make_input(map, route, make_car_in_area());
  auto trajectory = make_trajectory();
  input.candidate_index = 0U;
  ASSERT_TRUE(process_candidate(trajectory, input));
  ASSERT_GT(trajectory.back().pose.position.x, 10.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  input = make_input(map, route);
  auto cleared_candidate = make_trajectory();
  EXPECT_FALSE(process_candidate(cleared_candidate, input));
  input = make_input(map, route, make_car_in_area());
  input.current_odometry = make_odometry_at(0.0, 0.0, 1.0);
  trajectory = make_trajectory();
  input.candidate_index = 0U;
  ASSERT_TRUE(process_candidate(trajectory, input));
  EXPECT_NEAR(
    trajectory.back().pose.position.x, 10.0 - context_->vehicle_info.max_longitudinal_offset_m,
    0.1);
}

TEST_F(DetectionAreaStopIntegrationTest, CandidatesAreIndependentForEveryPolicy)
{
  const auto map = make_map();
  auto input = make_input(map, make_route(map->laneletLayer.begin()->id()), make_car_in_area());
  for (const auto & policy : {"go", "force_stop", "stop_after_stopline"}) {
    params_.detection_area_stop.unstoppable_policy = policy;
    params_.stopping_constraints.maximum_deceleration = 2.0;
    plugin_->update_params(params_);
    const auto straight = make_trajectory();
    auto curved = straight;
    for (auto & p : curved) {
      const auto x = p.pose.position.x;
      if (x < 10.0) p.pose.position.y = 5.0 * std::sin(3.141592653589793 * x / 10.0);
    }
    const std::vector<TrajectoryPoints> candidates{straight, curved, make_short_trajectory(8.0)};
    input.candidate_count = candidates.size();
    std::vector<TrajectoryPoints> expected;
    std::vector<bool> changed;
    input.candidate_index = 0U;
    for (const auto & original : candidates) {
      auto candidate = original;
      changed.push_back(process_candidate(candidate, input));
      expected.push_back(candidate);
    }
    input.candidate_index = 0U;
    for (size_t i = candidates.size(); i-- > 0;) {
      auto candidate = candidates[i];
      EXPECT_EQ(process_candidate(candidate, input), changed[i]) << policy;
      EXPECT_EQ(candidate, expected[i]) << policy;
    }
    if (std::string(policy) == "go") {
      EXPECT_FALSE(changed[0]);
      EXPECT_TRUE(changed[1]);
    }
  }
}

TEST_F(DetectionAreaStopIntegrationTest, ObservationTimeIsFrozenWithinCycle)
{
  params_.detection_area_stop.state_clear_time = 0.05;
  plugin_->update_params(params_);
  const auto map = make_map();
  auto input = make_input(map, make_route(map->laneletLayer.begin()->id()), make_car_in_area());
  auto detected_candidate = make_trajectory();
  ASSERT_TRUE(process_candidate(detected_candidate, input));

  // Start a callback without an obstacle while the previous observation is still active.
  // Even an unrelated first candidate must prepare the snapshot for later candidates.
  input.predicted_objects.reset();
  input.candidate_index = 0U;
  input.candidate_count = 2U;
  auto unrelated = make_trajectory(10.0);
  EXPECT_FALSE(process_candidate(unrelated, input));
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  auto candidate = make_trajectory();
  EXPECT_TRUE(process_candidate(candidate, input));
  expect_stop_before_stop_line(candidate);

  // The next callback refreshes the observation time and clears the expired obstacle.
  input.candidate_index = 0U;
  input.candidate_count = 1U;
  candidate = make_trajectory();
  const auto original = candidate;
  EXPECT_FALSE(process_candidate(candidate, input));
  EXPECT_EQ(candidate, original);
}

TEST_F(DetectionAreaStopIntegrationTest, ShortHorizonStopsBeforeVehicleFrontCrossesLine)
{
  const auto map = make_map();
  auto input = make_input(map, make_route(map->laneletLayer.begin()->id()), make_car_in_area());
  input.current_odometry = make_odometry_at(0.0, 0.0, 1.0);
  auto candidate = make_short_trajectory(8.0);
  input.candidate_index = 0U;
  EXPECT_TRUE(process_candidate(candidate, input));
  expect_stop_before_stop_line(candidate);
  auto safe = make_short_trajectory(5.0);
  const auto original = safe;
  EXPECT_FALSE(process_candidate(safe, input));
  EXPECT_EQ(safe, original);
}

TEST_F(DetectionAreaStopIntegrationTest, ClearedObstacleAllowsNewMovingCandidate)
{
  params_.detection_area_stop.state_clear_time = 0.01;
  plugin_->update_params(params_);
  const auto map = make_map();
  auto input = make_input(map, make_route(map->laneletLayer.begin()->id()), make_car_in_area());
  auto candidate = make_trajectory();
  input.candidate_index = 0U;
  ASSERT_TRUE(process_candidate(candidate, input));
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  input.predicted_objects.reset();
  input.candidate_index = 0U;
  candidate = make_trajectory();
  const auto original = candidate;
  EXPECT_FALSE(process_candidate(candidate, input));
  EXPECT_EQ(candidate, original);
}

TEST_F(DetectionAreaStopIntegrationTest, StopAfterLineContinuesWhenLineIsBehindCandidate)
{
  params_.detection_area_stop.unstoppable_policy = "stop_after_stopline";
  params_.stopping_constraints.maximum_deceleration = 1.0;
  plugin_->update_params(params_);
  const auto map = make_map();
  auto input = make_input(map, make_route(map->laneletLayer.begin()->id()), make_car_in_area());
  input.current_odometry = make_odometry_at(12.0, 0.0, 2.0);
  auto candidate = make_trajectory();
  for (auto & p : candidate) p.pose.position.x += 12.0;
  input.candidate_index = 0U;
  ASSERT_TRUE(process_candidate(candidate, input));
  EXPECT_NEAR(candidate.back().pose.position.x, 14.0, 0.1);
  EXPECT_FLOAT_EQ(candidate.back().longitudinal_velocity_mps, 0.0F);

  params_.detection_area_stop.use_dead_line = true;
  plugin_->update_params(params_);
  candidate = make_trajectory();
  for (auto & p : candidate) p.pose.position.x += 12.0;
  const auto original = candidate;
  input.candidate_index = 0U;
  EXPECT_FALSE(process_candidate(candidate, input));
  EXPECT_EQ(candidate, original);
}
