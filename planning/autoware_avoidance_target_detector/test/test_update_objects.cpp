// Copyright 2026 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/avoidance_target_detector/object_filtering.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/utility/Utilities.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::avoidance_target_detector
{
namespace
{

using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletSegment;

constexpr std::size_t k_lanelet_count = 3;
constexpr double k_lanelet_length_m = 10.0;
constexpr double k_main_left_y_m = 2.0;
constexpr double k_main_right_y_m = -2.0;
constexpr double k_disconnected_left_y_m = 4.5;
constexpr double k_disconnected_right_y_m = 2.5;
constexpr lanelet::Id k_main_lanelet_id_base = 1000000000;
constexpr lanelet::Id k_disconnected_lanelet_id_base = 2000000000;

rclcpp::Time make_time(const int32_t seconds, const uint32_t nanoseconds = 0)
{
  return {seconds, nanoseconds, RCL_ROS_TIME};
}

geometry_msgs::msg::Pose & mutable_pose(PredictedObject & object)
{
  return object.kinematics.initial_pose_with_covariance.pose;
}

geometry_msgs::msg::Pose & mutable_pose(TrackedObject & object)
{
  return object.kinematics.pose_with_covariance.pose;
}

geometry_msgs::msg::Twist & mutable_twist(PredictedObject & object)
{
  return object.kinematics.initial_twist_with_covariance.twist;
}

geometry_msgs::msg::Twist & mutable_twist(TrackedObject & object)
{
  return object.kinematics.twist_with_covariance.twist;
}

template <typename ObjectT>
ObjectT make_object(
  const uint8_t id, const double x, const double y, const double velocity_mps = 0.0,
  const uint8_t label = ObjectClassification::CAR, const float probability = 1.0F)
{
  ObjectT object;
  object.object_id.uuid.fill(0U);
  object.object_id.uuid.at(0) = id;

  ObjectClassification classification;
  classification.label = label;
  classification.probability = probability;
  object.classification.push_back(classification);

  auto & pose = mutable_pose(object);
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  mutable_twist(object).linear.x = velocity_mps;

  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 1.8;
  object.shape.dimensions.z = 1.5;
  return object;
}

template <typename ObjectT>
typename ObjectContainerTraits<ObjectT>::Objects make_objects(
  const std::vector<ObjectT> & input_objects)
{
  typename ObjectContainerTraits<ObjectT>::Objects objects;
  objects.header.frame_id = "map";
  objects.header.stamp.sec = 42;
  objects.objects = input_objects;
  return objects;
}

Trajectory make_trajectory()
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  for (std::size_t i = 0; i < 30; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = 0.5 + static_cast<double>(i);
    point.pose.orientation.w = 1.0;
    trajectory.points.push_back(point);
  }
  return trajectory;
}

Trajectory make_single_point_trajectory(const double x)
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.orientation.w = 1.0;
  trajectory.points.push_back(point);
  return trajectory;
}

std::vector<lanelet::Lanelet> make_lanelet_chain(
  const double left_y, const double right_y, const lanelet::Id lanelet_id_base)
{
  std::vector<lanelet::Point3d> left_points;
  std::vector<lanelet::Point3d> right_points;
  left_points.reserve(k_lanelet_count + 1);
  right_points.reserve(k_lanelet_count + 1);
  for (std::size_t i = 0; i <= k_lanelet_count; ++i) {
    const double x = static_cast<double>(i) * k_lanelet_length_m;
    left_points.emplace_back(lanelet::utils::getId(), x, left_y, 0.0);
    right_points.emplace_back(lanelet::utils::getId(), x, right_y, 0.0);
  }

  std::vector<lanelet::Lanelet> lanelets;
  lanelets.reserve(k_lanelet_count);
  for (std::size_t i = 0; i < k_lanelet_count; ++i) {
    lanelet::LineString3d left_bound{
      lanelet::utils::getId(), {left_points.at(i), left_points.at(i + 1)}};
    lanelet::LineString3d right_bound{
      lanelet::utils::getId(), {right_points.at(i), right_points.at(i + 1)}};
    lanelet::Lanelet lanelet{
      lanelet_id_base + static_cast<lanelet::Id>(i), left_bound, right_bound};
    lanelet.setAttribute(lanelet::AttributeName::Type, lanelet::AttributeValueString::Lanelet);
    lanelet.setAttribute(lanelet::AttributeName::Subtype, lanelet::AttributeValueString::Road);
    lanelet.setAttribute(lanelet::AttributeName::Location, lanelet::AttributeValueString::Urban);
    lanelet.setAttribute(lanelet::AttributeName::OneWay, "yes");
    lanelets.push_back(lanelet);
  }
  return lanelets;
}

std::shared_ptr<ExtendedRouteHandler> make_route_handler()
{
  const auto main_lanelets =
    make_lanelet_chain(k_main_left_y_m, k_main_right_y_m, k_main_lanelet_id_base);
  const auto disconnected_lanelets = make_lanelet_chain(
    k_disconnected_left_y_m, k_disconnected_right_y_m, k_disconnected_lanelet_id_base);

  auto map = std::make_shared<lanelet::LaneletMap>();
  for (const auto & lanelet : main_lanelets) {
    map->add(lanelet);
  }
  for (const auto & lanelet : disconnected_lanelets) {
    map->add(lanelet);
  }

  LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose.position.x = 0.5;
  route.start_pose.orientation.w = 1.0;
  route.goal_pose.position.x = 29.5;
  route.goal_pose.orientation.w = 1.0;
  for (std::size_t i = 0; i < k_lanelet_count; ++i) {
    LaneletPrimitive main_primitive;
    main_primitive.id = main_lanelets.at(i).id();
    main_primitive.primitive_type = "lane";
    LaneletPrimitive disconnected_primitive;
    disconnected_primitive.id = disconnected_lanelets.at(i).id();
    disconnected_primitive.primitive_type = "lane";

    LaneletSegment segment;
    segment.preferred_primitive = main_primitive;
    segment.primitives = {main_primitive, disconnected_primitive};
    route.segments.push_back(segment);
  }

  LaneletMapBin map_msg;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  lanelet::utils::conversion::toBinMsg(map, &map_msg);
#pragma GCC diagnostic pop
  map_msg.header.frame_id = "map";

  auto handler = std::make_shared<ExtendedRouteHandler>(map_msg, route);
  handler->create_map();
  return handler;
}

const ExtendedRouteHandler & route_handler()
{
  static const auto handler = make_route_handler();
  return *handler;
}

template <typename ObjectsT>
std::vector<uint8_t> object_ids(const ObjectsT & objects)
{
  std::vector<uint8_t> ids;
  ids.reserve(objects.objects.size());
  for (const auto & object : objects.objects) {
    ids.push_back(object.object_id.uuid.at(0));
  }
  return ids;
}

template <typename ObjectT>
class UpdateObjectsTest : public testing::Test
{
protected:
  using Objects = typename ObjectContainerTraits<ObjectT>::Objects;
  using Selector = ObjectSelectorBase<ObjectT>;

  const Trajectory trajectory_{make_trajectory()};
  const RouteBounds empty_route_bounds_{};
};

using SupportedObjectTypes = testing::Types<PredictedObject, TrackedObject>;
TYPED_TEST_SUITE(UpdateObjectsTest, SupportedObjectTypes);

TYPED_TEST(UpdateObjectsTest, EmptyInputProducesEmptyOutputs)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>({});

  selector.update_objects(make_time(0), objects, Trajectory{}, route_handler());

  EXPECT_TRUE(selector.get_avoidance_targets(objects, Trajectory{}, this->empty_route_bounds_)
                .objects.empty());
  EXPECT_TRUE(selector.get_driving_along_vehicles(objects).objects.empty());
}

TYPED_TEST(UpdateObjectsTest, StationaryDeviatedObjectIsImmediatelyAvoidanceTarget)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 2.0)});

  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  const auto avoidance_targets =
    selector.get_avoidance_targets(objects, this->trajectory_, this->empty_route_bounds_);

  EXPECT_EQ(object_ids(avoidance_targets), std::vector<uint8_t>({1}));
  EXPECT_TRUE(selector.get_driving_along_vehicles(objects).objects.empty());
}

TYPED_TEST(UpdateObjectsTest, StationaryAlignedObjectIsNotAvoidanceTarget)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 0.0)});

  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());

  EXPECT_TRUE(selector.get_avoidance_targets(objects, this->trajectory_, this->empty_route_bounds_)
                .objects.empty());
  EXPECT_TRUE(selector.get_driving_along_vehicles(objects).objects.empty());
}

TYPED_TEST(UpdateObjectsTest, MovingConnectedObjectIsNotDrivingAlongVehicle)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 0.0, 2.0)});

  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());

  EXPECT_TRUE(selector.get_driving_along_vehicles(objects).objects.empty());
}

TYPED_TEST(UpdateObjectsTest, MovingDisconnectedObjectIsDrivingAlongVehicle)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 2.6, 2.0)});

  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());

  EXPECT_EQ(object_ids(selector.get_driving_along_vehicles(objects)), std::vector<uint8_t>({1}));
}

TYPED_TEST(UpdateObjectsTest, ClassificationProbabilityThresholdIsStrict)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>(
    {make_object<TypeParam>(1, 15.0, 2.0, 0.0, ObjectClassification::CAR, 0.1F),
     make_object<TypeParam>(2, 15.0, 2.0, 0.0, ObjectClassification::CAR, 0.1001F),
     make_object<TypeParam>(3, 15.0, 2.0, 0.0, ObjectClassification::PEDESTRIAN, 1.0F)});

  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  const auto avoidance_targets =
    selector.get_avoidance_targets(objects, this->trajectory_, this->empty_route_bounds_);

  EXPECT_EQ(object_ids(avoidance_targets), std::vector<uint8_t>({2}));
}

TYPED_TEST(UpdateObjectsTest, MixedObjectsPreserveHeaderAndSurvivingOrder)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>(
    {make_object<TypeParam>(4, 12.0, 2.0), make_object<TypeParam>(2, 15.0, 0.0),
     make_object<TypeParam>(7, 18.0, 2.0), make_object<TypeParam>(9, 15.0, 2.6, 2.0)});

  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  const auto avoidance_targets =
    selector.get_avoidance_targets(objects, this->trajectory_, this->empty_route_bounds_);
  const auto driving_along = selector.get_driving_along_vehicles(objects);

  EXPECT_EQ(object_ids(avoidance_targets), std::vector<uint8_t>({4, 7}));
  EXPECT_EQ(object_ids(driving_along), std::vector<uint8_t>({9}));
  EXPECT_EQ(avoidance_targets.header, objects.header);
  EXPECT_EQ(driving_along.header, objects.header);
  EXPECT_EQ(objects.objects.size(), 4U);
}

TYPED_TEST(UpdateObjectsTest, EmptyAndSinglePointTrajectoriesUseCurrentFallbackBehavior)
{
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 0.0)});

  typename TestFixture::Selector empty_trajectory_selector;
  empty_trajectory_selector.update_objects(make_time(0), objects, Trajectory{}, route_handler());
  EXPECT_EQ(
    object_ids(empty_trajectory_selector.get_avoidance_targets(
      objects, Trajectory{}, this->empty_route_bounds_)),
    std::vector<uint8_t>({1}));

  const auto single_point_trajectory = make_single_point_trajectory(15.0);
  typename TestFixture::Selector single_point_selector;
  single_point_selector.update_objects(
    make_time(0), objects, single_point_trajectory, route_handler());
  EXPECT_EQ(
    object_ids(single_point_selector.get_avoidance_targets(
      objects, single_point_trajectory, this->empty_route_bounds_)),
    std::vector<uint8_t>({1}));
}

TYPED_TEST(UpdateObjectsTest, SinglePointTrajectorySupportsIdenticalPolygonEndpoints)
{
  typename TestFixture::Selector selector;
  const auto trajectory = make_single_point_trajectory(15.0);
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 2.6, 2.0)});

  selector.update_objects(make_time(0), objects, trajectory, route_handler());

  EXPECT_EQ(object_ids(selector.get_driving_along_vehicles(objects)), std::vector<uint8_t>({1}));
}

TYPED_TEST(UpdateObjectsTest, StaleThresholdComparisonIsStrict)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 2.0)});
  const typename TestFixture::Objects empty_objects;

  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  ASSERT_EQ(
    selector.get_avoidance_targets(objects, this->trajectory_, this->empty_route_bounds_)
      .objects.size(),
    1U);

  selector.update_objects(make_time(1), empty_objects, this->trajectory_, route_handler());
  EXPECT_EQ(
    selector.get_avoidance_targets(objects, this->trajectory_, this->empty_route_bounds_)
      .objects.size(),
    1U);

  selector.update_objects(make_time(1, 1), empty_objects, this->trajectory_, route_handler());
  EXPECT_TRUE(selector.get_avoidance_targets(objects, this->trajectory_, this->empty_route_bounds_)
                .objects.empty());
}

TYPED_TEST(UpdateObjectsTest, AvoidanceHysteresisCountAccumulatesDuringTimeLockout)
{
  typename TestFixture::Selector selector;
  const auto deviated_objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 2.0)});
  const auto aligned_objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 0.0)});

  selector.update_objects(make_time(0), deviated_objects, this->trajectory_, route_handler());
  ASSERT_EQ(
    selector.get_avoidance_targets(deviated_objects, this->trajectory_, this->empty_route_bounds_)
      .objects.size(),
    1U);

  selector.update_objects(
    make_time(0, 100000000), aligned_objects, this->trajectory_, route_handler());
  // Repeated getter calls intentionally advance the state-change counter without advancing time.
  for (std::size_t i = 0; i < FilterManagerParams::count_threshold; ++i) {
    EXPECT_EQ(
      selector.get_avoidance_targets(aligned_objects, this->trajectory_, this->empty_route_bounds_)
        .objects.size(),
      1U);
  }

  selector.update_objects(
    make_time(0, 500000000), aligned_objects, this->trajectory_, route_handler());
  EXPECT_TRUE(
    selector.get_avoidance_targets(aligned_objects, this->trajectory_, this->empty_route_bounds_)
      .objects.empty());
}

TYPED_TEST(UpdateObjectsTest, DrivingAlongHysteresisCountAccumulatesDuringTimeLockout)
{
  typename TestFixture::Selector selector;
  const auto disconnected_objects =
    make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 2.6, 2.0)});
  const auto connected_objects =
    make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 0.0, 2.0)});

  selector.update_objects(make_time(0), disconnected_objects, this->trajectory_, route_handler());
  ASSERT_EQ(selector.get_driving_along_vehicles(disconnected_objects).objects.size(), 1U);

  selector.update_objects(
    make_time(0, 100000000), connected_objects, this->trajectory_, route_handler());
  // Repeated getter calls intentionally advance the state-change counter without advancing time.
  for (std::size_t i = 0; i < FilterManagerParams::count_threshold; ++i) {
    EXPECT_EQ(selector.get_driving_along_vehicles(connected_objects).objects.size(), 1U);
  }

  selector.update_objects(make_time(5), connected_objects, this->trajectory_, route_handler());
  EXPECT_TRUE(selector.get_driving_along_vehicles(connected_objects).objects.empty());
}

TYPED_TEST(UpdateObjectsTest, FirstObservationBypassesAllTransitionMatrices)
{
  const auto object = make_object<TypeParam>(1, 15.0, 0.0);
  const auto current_time = make_time(0);

  TargetFilter<TypeParam> target_filter(object, current_time);
  StationaryFilter<TypeParam> stationary_filter(object, current_time);
  DeviationFilter<TypeParam> deviation_filter(object, current_time);

  target_filter.observe_and_update(current_time, object, this->trajectory_);
  stationary_filter.observe_and_update(current_time, object, this->trajectory_);
  deviation_filter.observe_and_update(current_time, object, this->trajectory_);

  EXPECT_DOUBLE_EQ(target_filter.get_posterior(), 0.95);
  EXPECT_DOUBLE_EQ(stationary_filter.get_posterior(), 0.99);
  EXPECT_DOUBLE_EQ(deviation_filter.get_posterior(), 0.05);
}

TYPED_TEST(UpdateObjectsTest, LongitudinalFilterPrunesStateQualifiedTarget)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, -10.0, 2.0)});

  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());

  EXPECT_TRUE(selector.get_avoidance_targets(objects, this->trajectory_, this->empty_route_bounds_)
                .objects.empty());
}

}  // namespace
}  // namespace autoware::avoidance_target_detector
