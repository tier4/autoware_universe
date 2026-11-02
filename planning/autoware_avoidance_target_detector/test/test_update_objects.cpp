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
#include <geometry_msgs/msg/point32.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/utility/Utilities.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
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

/// Outside the centre of both the main lane (y=0) and adjacent lane (y=3.5).
constexpr double k_deviated_object_y_m = 5.0;

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
  const uint8_t label = ObjectClassification::CAR, const float probability = 1.0F,
  const double yaw = 0.0)
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
  pose.orientation.z = std::sin(yaw / 2.0);
  pose.orientation.w = std::cos(yaw / 2.0);
  mutable_twist(object).linear.x = velocity_mps;

  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 1.8;
  object.shape.dimensions.z = 1.5;
  return object;
}

/// Cylinder carrying only a diameter in dimensions.x, as perception publishes them.
template <typename ObjectT>
ObjectT make_cylinder_object(
  const uint8_t id, const double x, const double y, const double diameter)
{
  auto object = make_object<ObjectT>(id, x, y);
  object.shape.type = Shape::CYLINDER;
  object.shape.dimensions.x = diameter;
  object.shape.dimensions.y = 0.0;
  object.shape.dimensions.z = 1.5;
  return object;
}

/// Polygon whose footprint is a square of half_extent, deliberately much smaller than the
/// 4.0 x 1.8 bounding box make_object() reports, so the two rear-edge sources are distinguishable.
template <typename ObjectT>
ObjectT make_polygon_object(
  const uint8_t id, const double x, const double y, const double half_extent)
{
  auto object = make_object<ObjectT>(id, x, y);
  object.shape.type = Shape::POLYGON;

  const std::vector<std::pair<double, double>> corners{
    {half_extent, half_extent},
    {-half_extent, half_extent},
    {-half_extent, -half_extent},
    {half_extent, -half_extent}};
  object.shape.footprint.points.clear();
  object.shape.footprint.points.reserve(corners.size());
  for (const auto & [corner_x, corner_y] : corners) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(corner_x);
    point.y = static_cast<float>(corner_y);
    point.z = 0.0F;
    object.shape.footprint.points.push_back(point);
  }
  return object;
}

template <typename ObjectT>
typename ObjectContainerTraits<ObjectT>::Objects make_objects(
  const std::vector<ObjectT> & input_objects)
{
  typename ObjectContainerTraits<ObjectT>::Objects objects;
  objects.header.frame_id = "map";
  objects.header.stamp.sec = 0;
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

Trajectory make_timed_filter_trajectory()
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  TrajectoryPoint start;
  start.pose.orientation.w = 1.0;
  trajectory.points.push_back(start);
  TrajectoryPoint end;
  end.pose.position.x = 10.0;
  end.pose.orientation.w = 1.0;
  end.time_from_start.sec = 5;
  trajectory.points.push_back(end);
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

TYPED_TEST(UpdateObjectsTest, ParkedObjectIsImmediatelyAvoidanceTarget)
{
  typename TestFixture::Selector selector;
  const auto objects =
    make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, k_deviated_object_y_m)});

  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  const auto avoidance_targets =
    selector.get_avoidance_targets(objects, this->trajectory_, this->empty_route_bounds_);

  EXPECT_EQ(object_ids(avoidance_targets), std::vector<uint8_t>({1}));
  EXPECT_TRUE(selector.get_driving_along_vehicles(objects).objects.empty());
}

TYPED_TEST(UpdateObjectsTest, StationaryTrafficLaneObjectIsNotAvoidanceTarget)
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
    {make_object<TypeParam>(1, 15.0, k_deviated_object_y_m, 0.0, ObjectClassification::CAR, 0.1F),
     make_object<TypeParam>(
       2, 15.0, k_deviated_object_y_m, 0.0, ObjectClassification::CAR, 0.1001F),
     make_object<TypeParam>(
       3, 15.0, k_deviated_object_y_m, 0.0, ObjectClassification::PEDESTRIAN, 1.0F)});

  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  const auto avoidance_targets =
    selector.get_avoidance_targets(objects, this->trajectory_, this->empty_route_bounds_);

  EXPECT_EQ(object_ids(avoidance_targets), std::vector<uint8_t>({2}));
}

TYPED_TEST(UpdateObjectsTest, MixedObjectsPreserveHeaderAndSurvivingOrder)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>(
    {make_object<TypeParam>(4, 12.0, k_deviated_object_y_m), make_object<TypeParam>(2, 15.0, 0.0),
     make_object<TypeParam>(7, 18.0, k_deviated_object_y_m),
     make_object<TypeParam>(9, 15.0, 2.6, 2.0)});

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

TYPED_TEST(UpdateObjectsTest, InvalidReferencesHaveExplicitStatusAndNoTargets)
{
  for (const auto & trajectory : {Trajectory{}, make_single_point_trajectory(15.0)}) {
    typename TestFixture::Selector selector;
    const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 5.0)});
    selector.update_objects(make_time(0), objects, trajectory, route_handler());
    const auto result = selector.evaluate(objects, {});
    // An empty frame is invalid as well as an empty reference.
    EXPECT_NE(result.status, SelectionStatus::ready);
    EXPECT_TRUE(result.avoidance_targets.objects.empty());
    EXPECT_TRUE(result.driving_along_vehicles.objects.empty());
    ASSERT_EQ(result.decisions.size(), 1U);
    EXPECT_EQ(result.decisions.front().disposition, ObjectDisposition::unknown);
  }
}

TYPED_TEST(UpdateObjectsTest, CachedMeasurementExpiresWithoutRenewingItsAge)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 5.0)});
  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  const auto initial = selector.evaluate(objects, {});
  ASSERT_EQ(initial.avoidance_targets.objects.size(), 1U);
  selector.update_objects(make_time(1), objects, this->trajectory_, route_handler());
  const auto at_threshold = selector.evaluate(objects, {});
  EXPECT_EQ(at_threshold.status, SelectionStatus::ready);
  ASSERT_EQ(at_threshold.decisions.size(), 1U);
  EXPECT_DOUBLE_EQ(
    at_threshold.decisions.front().stationary_probability,
    initial.decisions.front().stationary_probability);
  selector.update_objects(make_time(1, 1), objects, this->trajectory_, route_handler());
  EXPECT_EQ(selector.status(), SelectionStatus::stale_objects);
  EXPECT_TRUE(selector.evaluate(objects, {}).avoidance_targets.objects.empty());
}

TYPED_TEST(UpdateObjectsTest, GettersAreIdempotentAndDoNotAdvanceConfirmation)
{
  typename TestFixture::Selector selector;
  auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 0.0)});
  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  mutable_pose(objects.objects.front()).position.y = 5.0;
  objects.header.stamp = make_time(0, 100000000);
  selector.update_objects(make_time(0, 100000000), objects, this->trajectory_, route_handler());
  for (int i = 0; i < 20; ++i) {
    EXPECT_TRUE(selector.get_avoidance_targets(objects, this->trajectory_, {}).objects.empty());
  }
  // Only distinct observations, continuously qualifying for 0.5 seconds, confirm the change.
  for (int tick = 2; tick <= 6; ++tick) {
    objects.header.stamp = make_time(0, tick * 100000000);
    selector.update_objects(
      make_time(0, tick * 100000000), objects, this->trajectory_, route_handler());
  }
  EXPECT_EQ(selector.evaluate(objects, {}).avoidance_targets.objects.size(), 1U);
}

TYPED_TEST(UpdateObjectsTest, StoppedVehicleLeavesDrivingAlongSet)
{
  typename TestFixture::Selector selector;
  auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 2.6, 2.0)});
  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  ASSERT_EQ(selector.get_driving_along_vehicles(objects).objects.size(), 1U);
  mutable_pose(objects.objects.front()).position.y = 0.0;
  mutable_twist(objects.objects.front()).linear.x = 0.0;
  for (int tick = 1; tick <= 200; ++tick) {
    const auto time = make_time(tick / 10, (tick % 10) * 100000000);
    objects.header.stamp = time;
    selector.update_objects(time, objects, this->trajectory_, route_handler());
  }
  const auto result = selector.evaluate(objects, {});
  EXPECT_TRUE(result.driving_along_vehicles.objects.empty());
  EXPECT_TRUE(result.avoidance_targets.objects.empty());
  EXPECT_EQ(result.yield_targets.objects.size(), 1U);
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

TYPED_TEST(UpdateObjectsTest, RangeFilterUsesCircularObjectFootprints)
{
  const auto objects = make_objects<TypeParam>(
    {make_object<TypeParam>(1, 5.0, 4.0), make_object<TypeParam>(2, 5.0, 5.0)});

  const auto filtered = filter_objects_in_range(objects, make_timed_filter_trajectory(), 2.0);

  EXPECT_EQ(object_ids(filtered), std::vector<uint8_t>({1}));
  EXPECT_EQ(filtered.header, objects.header);
}

TYPED_TEST(UpdateObjectsTest, RangeFilterRetainsObjectsProjectedIntoRange)
{
  const auto objects = make_objects<TypeParam>(
    {make_object<TypeParam>(1, -10.0, 0.0, 3.0), make_object<TypeParam>(2, -10.0, 10.0, 3.0)});

  const auto filtered = filter_objects_in_range(objects, make_timed_filter_trajectory(), 1.0);

  EXPECT_EQ(object_ids(filtered), std::vector<uint8_t>({1}));
}

TYPED_TEST(UpdateObjectsTest, RangeFilterIncludesAdditionalPredictionHorizon)
{
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, -8.3, 0.0, 1.0)});

  const auto without_additional_horizon =
    filter_objects_in_range(objects, make_timed_filter_trajectory(), 1.0);
  const auto with_additional_horizon =
    filter_objects_in_range(objects, make_timed_filter_trajectory(), 1.0, 0.2);

  EXPECT_TRUE(without_additional_horizon.objects.empty());
  EXPECT_EQ(object_ids(with_additional_horizon), std::vector<uint8_t>({1}));
}

TYPED_TEST(UpdateObjectsTest, RangeFilterRetainsMovingObjectsWithoutProjectionHorizon)
{
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, -100.0, 100.0, 3.0)});

  const auto filtered = filter_objects_in_range(objects, this->trajectory_, 1.0);

  EXPECT_EQ(object_ids(filtered), std::vector<uint8_t>({1}));
}

TYPED_TEST(UpdateObjectsTest, RangeFilterRejectsInvalidMargins)
{
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 5.0, 0.0)});

  EXPECT_THROW(
    {
      const auto filtered = filter_objects_in_range(objects, make_timed_filter_trajectory(), -1.0);
      (void)filtered;
    },
    std::invalid_argument);
}

TYPED_TEST(UpdateObjectsTest, RangeFilterRejectsInvalidAdditionalPredictionHorizon)
{
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 5.0, 0.0)});

  EXPECT_THROW(
    {
      const auto filtered =
        filter_objects_in_range(objects, make_timed_filter_trajectory(), 1.0, -0.1);
      (void)filtered;
    },
    std::invalid_argument);
}

TYPED_TEST(UpdateObjectsTest, LongitudinalFilterPrunesStateQualifiedTarget)
{
  typename TestFixture::Selector selector;
  const auto objects =
    make_objects<TypeParam>({make_object<TypeParam>(1, -5.0, k_deviated_object_y_m)});

  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());

  EXPECT_TRUE(selector.get_avoidance_targets(objects, this->trajectory_, this->empty_route_bounds_)
                .objects.empty());
}

TYPED_TEST(UpdateObjectsTest, BoundingBoxRearEdgeUsesCornersInsteadOfCenter)
{
  // The center sits 2.0 m from the trajectory, beyond magnitude_threshold_m (1.5 m), but the rear
  // edge of the 1.8 m wide bounding box reaches to 1.1 m, so the object counts as on-trajectory.
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 2.0)});

  EXPECT_TRUE(should_filter_out_on_trajectory_object(this->trajectory_, objects.objects.front()));
}

TYPED_TEST(UpdateObjectsTest, CylinderRearEdgeUsesDiameterSquare)
{
  // The 4.0 m diameter cylinder is bounded by a 4.0 x 4.0 square, so its rear edge spans
  // y = 0.5 to 4.5 and reaches within magnitude_threshold_m of the trajectory. Neither the object
  // center (2.5 m out) nor a zero-width box would come close enough.
  const auto objects =
    make_objects<TypeParam>({make_cylinder_object<TypeParam>(1, 15.0, 2.5, 4.0)});

  EXPECT_TRUE(should_filter_out_on_trajectory_object(this->trajectory_, objects.objects.front()));
}

TYPED_TEST(UpdateObjectsTest, PolygonWithValidDimensionsPrefersBoundingBox)
{
  // The small footprint alone puts the rear edge 1.8 m out; the populated dimensions define
  // a 1.8 m wide box whose rear edge reaches 1.1 m and satisfies the alignment diagnostic.
  const auto objects = make_objects<TypeParam>({make_polygon_object<TypeParam>(1, 15.0, 2.0, 0.2)});

  EXPECT_TRUE(should_filter_out_on_trajectory_object(this->trajectory_, objects.objects.front()));
}

TYPED_TEST(UpdateObjectsTest, BoundingBoxRearEdgeFollowsObjectOrientation)
{
  // Yawed 90 degrees, the 4.0 m long box points across the trajectory: its rear edge swings from
  // y = 2.0 down onto the centerline, which the center-only fallback would never detect.
  const auto objects = make_objects<TypeParam>(
    {make_object<TypeParam>(1, 15.0, 2.0, 0.0, ObjectClassification::CAR, 1.0F, M_PI_2)});

  EXPECT_TRUE(should_filter_out_on_trajectory_object(this->trajectory_, objects.objects.front()));
}

TYPED_TEST(UpdateObjectsTest, ParkedEligibilityDoesNotDependOnReferenceAlignment)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, -2.5)});
  auto reference = this->trajectory_;
  for (auto & point : reference.points) point.pose.position.y = -2.5;
  selector.update_objects(make_time(0), objects, reference, route_handler());
  const auto result = selector.evaluate(objects, {}, SelectionGeometryParams{6.9});
  ASSERT_EQ(result.avoidance_targets.objects.size(), 1U);
  EXPECT_EQ(result.decisions.front().reason, "parked_off_lane_center");
  EXPECT_DOUBLE_EQ(result.decisions.front().deviation_probability, 0.05);
}

TYPED_TEST(UpdateObjectsTest, VehiclesAtTrafficLaneCentersAreYieldTargets)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>(
    {make_object<TypeParam>(1, 15.0, 0.0), make_object<TypeParam>(2, 15.0, 3.5)});
  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  const auto result = selector.evaluate(objects, {});
  EXPECT_TRUE(result.avoidance_targets.objects.empty());
  EXPECT_EQ(result.yield_targets.objects.size(), 2U);
}

TYPED_TEST(UpdateObjectsTest, EndpointEnvelopeRetainsPotentialFrontFootprintCollision)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 33.0, -2.5)});
  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  EXPECT_TRUE(selector.evaluate(objects, {}).avoidance_targets.objects.empty());
  const auto result = selector.evaluate(objects, {}, SelectionGeometryParams{6.9});
  EXPECT_EQ(result.avoidance_targets.objects.size(), 1U);
}

TYPED_TEST(UpdateObjectsTest, BeyondEndpointUsesUnclampedCoordinate)
{
  const auto beyond = make_object<TypeParam>(1, 50.0, 0.0);
  const auto within = make_object<TypeParam>(2, 15.0, 0.0);
  EXPECT_TRUE(is_object_beyond_trajectory_end(this->trajectory_, beyond));
  EXPECT_FALSE(is_object_beyond_trajectory_end(this->trajectory_, within));
}

RouteBounds make_review_bounds()
{
  auto bound = [](const double y) {
    lanelet::LineString2d line(lanelet::utils::getId());
    for (double x : {0.0, 30.0}) {
      line.push_back(lanelet::Point2d(lanelet::utils::getId(), {x, y, 0.0}));
    }
    return line;
  };
  return {bound(2.0), bound(-2.0)};
}

TYPED_TEST(UpdateObjectsTest, FootprintSpanningCorridorIsRetained)
{
  auto object = make_object<TypeParam>(1, 15.0, 0.0, 0.0, ObjectClassification::CAR, 1.0F, M_PI_2);
  object.shape.dimensions.x = 6.0;
  EXPECT_FALSE(
    should_filter_out_by_lateral_distance(make_review_bounds(), this->trajectory_, object));
  mutable_pose(object).position.y = 10.0;
  EXPECT_TRUE(
    should_filter_out_by_lateral_distance(make_review_bounds(), this->trajectory_, object));
}

TYPED_TEST(UpdateObjectsTest, HairpinInteriorIsNotRejectedByStartTangent)
{
  auto trajectory = this->trajectory_;
  trajectory.points.clear();
  auto add = [&](double x, double y, double yaw) {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    point.pose.orientation.z = std::sin(yaw / 2.0);
    point.pose.orientation.w = std::cos(yaw / 2.0);
    trajectory.points.push_back(point);
  };
  for (int x = 0; x < 10; ++x) add(x, 0, 0);
  for (int y = 0; y < 10; ++y) add(10, y, M_PI_2);
  for (int x = 10; x >= -10; --x) add(x, 10, M_PI);
  const auto object = make_object<TypeParam>(1, -5.0, 10.0);
  EXPECT_FALSE(should_filter_out_by_longitudinal_distance(trajectory, object));
}

TYPED_TEST(UpdateObjectsTest, GeometryIsIndependentOfAbsoluteAltitude)
{
  auto reference = this->trajectory_;
  auto object = make_object<TypeParam>(1, 23.0, 3.5);
  for (auto & point : reference.points) point.pose.position.z = 0.1 * point.pose.position.x;
  mutable_pose(object).position.z = 2.3;
  auto bounds = make_review_bounds();
  bounds.first.clear();
  for (const auto & [x, y] :
       std::vector<std::pair<double, double>>{{0.5, 1.0}, {10.0, 1.0}, {20.0, 5.0}, {29.5, 5.0}}) {
    bounds.first.push_back(lanelet::Point2d(lanelet::utils::getId(), {x, y, 0.0}));
  }
  const bool lateral = should_filter_out_by_lateral_distance(bounds, reference, object);
  EXPECT_FALSE(lateral);
  const bool longitudinal = should_filter_out_by_longitudinal_distance(reference, object);
  const bool aligned = should_filter_out_on_trajectory_object(reference, object);
  for (auto & point : reference.points) point.pose.position.z += 100.0;
  mutable_pose(object).position.z += 100.0;
  EXPECT_EQ(lateral, should_filter_out_by_lateral_distance(bounds, reference, object));
  EXPECT_EQ(longitudinal, should_filter_out_by_longitudinal_distance(reference, object));
  EXPECT_EQ(aligned, should_filter_out_on_trajectory_object(reference, object));
}

TYPED_TEST(UpdateObjectsTest, RearEdgeOrderingDoesNotDependOnHeadingSign)
{
  const auto forward = make_object<TypeParam>(1, 15.0, 2.0);
  const auto reverse =
    make_object<TypeParam>(1, 15.0, 2.0, 0.0, ObjectClassification::CAR, 1.0F, M_PI);
  EXPECT_EQ(
    should_filter_out_on_trajectory_object(this->trajectory_, forward),
    should_filter_out_on_trajectory_object(this->trajectory_, reverse));
}

TYPED_TEST(UpdateObjectsTest, AlignmentUsesSpatialRatherThanInputSampleSpacing)
{
  auto arc = [](const int count) {
    Trajectory trajectory;
    trajectory.header.frame_id = "map";
    for (int i = 0; i < count; ++i) {
      const double theta = -0.2 + 0.4 * i / (count - 1);
      TrajectoryPoint point;
      point.pose.position.x = 5.0 * std::sin(theta);
      point.pose.position.y = 5.0 * (1.0 - std::cos(theta));
      point.pose.orientation.z = std::sin(theta / 2.0);
      point.pose.orientation.w = std::cos(theta / 2.0);
      trajectory.points.push_back(point);
    }
    return trajectory;
  };
  const auto object = make_object<TypeParam>(1, 6.0, 1.0);
  EXPECT_EQ(
    should_filter_out_on_trajectory_object(arc(4), object),
    should_filter_out_on_trajectory_object(arc(41), object));
}

TYPED_TEST(UpdateObjectsTest, NegativeClassificationEvidenceUpdatesExistingTrack)
{
  typename TestFixture::Selector selector;
  auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 5.0)});
  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  ASSERT_EQ(selector.evaluate(objects, {}).avoidance_targets.objects.size(), 1U);
  objects.objects.front().classification.front().label = ObjectClassification::PEDESTRIAN;
  for (int tick = 1; tick <= 9; ++tick) {
    objects.header.stamp = make_time(0, tick * 100000000);
    selector.update_objects(
      make_time(0, tick * 100000000), objects, this->trajectory_, route_handler());
  }
  const auto result = selector.evaluate(objects, {});
  EXPECT_TRUE(result.avoidance_targets.objects.empty());
  ASSERT_EQ(result.decisions.size(), 1U);
  EXPECT_LT(result.decisions.front().interest_probability, 0.5);
}

TYPED_TEST(UpdateObjectsTest, OutOfOrderAndFutureMeasurementsAreExplicitlyRejected)
{
  typename TestFixture::Selector selector;
  auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 5.0)});
  objects.header.stamp = make_time(1);
  selector.update_objects(make_time(1), objects, this->trajectory_, route_handler());
  objects.header.stamp = make_time(0, 900000000);
  selector.update_objects(make_time(1, 100000000), objects, this->trajectory_, route_handler());
  EXPECT_EQ(selector.status(), SelectionStatus::out_of_order_objects);
  objects.header.stamp = make_time(3);
  selector.update_objects(make_time(1, 200000000), objects, this->trajectory_, route_handler());
  EXPECT_EQ(selector.status(), SelectionStatus::future_objects);
}

TYPED_TEST(UpdateObjectsTest, ClockResetDiscardsPreviousHistory)
{
  typename TestFixture::Selector selector;
  auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 0.0, 2.0)});
  objects.header.stamp = make_time(10);
  selector.update_objects(make_time(10), objects, this->trajectory_, route_handler());
  objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, -2.5)});
  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  EXPECT_EQ(selector.evaluate(objects, {}).avoidance_targets.objects.size(), 1U);
}

TYPED_TEST(UpdateObjectsTest, InvalidObjectsAndFramesAreNotReportedAsEmptySafeScenes)
{
  typename TestFixture::Selector selector;
  auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 5.0)});
  objects.objects.front().shape.dimensions.x = std::numeric_limits<double>::quiet_NaN();
  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  EXPECT_EQ(selector.evaluate(objects, {}).status, SelectionStatus::invalid_objects);
  objects.header.frame_id = "base_link";
  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  EXPECT_EQ(selector.status(), SelectionStatus::frame_mismatch);
}

TYPED_TEST(UpdateObjectsTest, DrivingAlongExitIsBoundedWhileObjectKeepsMoving)
{
  typename TestFixture::Selector selector;
  auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, 2.6, 2.0)});
  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  ASSERT_EQ(selector.get_driving_along_vehicles(objects).objects.size(), 1U);
  mutable_pose(objects.objects.front()).position.y = 10.0;
  for (int tick = 1; tick <= 51; ++tick) {
    const auto time = make_time(tick / 10, (tick % 10) * 100000000);
    objects.header.stamp = time;
    selector.update_objects(time, objects, this->trajectory_, route_handler());
    if (tick == 50) {
      EXPECT_EQ(selector.get_driving_along_vehicles(objects).objects.size(), 1U);
    }
  }
  EXPECT_TRUE(selector.get_driving_along_vehicles(objects).objects.empty());
}

TYPED_TEST(UpdateObjectsTest, RecentlyMovingVehicleMustWaitBeforeBecomingParked)
{
  typename TestFixture::Selector selector;
  auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, -2.5, 2.0)});
  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  mutable_twist(objects.objects.front()).linear.x = 0.0;
  for (int tick = 1; tick <= 55; ++tick) {
    const auto time = make_time(tick / 10, (tick % 10) * 100000000);
    objects.header.stamp = time;
    selector.update_objects(time, objects, this->trajectory_, route_handler());
    if (tick < 55) {
      EXPECT_TRUE(selector.evaluate(objects, {}).avoidance_targets.objects.empty());
    }
  }
  EXPECT_EQ(selector.evaluate(objects, {}).avoidance_targets.objects.size(), 1U);
}

TYPED_TEST(UpdateObjectsTest, ReappearingUuidDoesNotReviveExpiredMotionHistory)
{
  typename TestFixture::Selector selector;
  auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, -2.5, 2.0)});
  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  mutable_twist(objects.objects.front()).linear.x = 0.0;
  objects.header.stamp = make_time(2);
  selector.update_objects(make_time(2), objects, this->trajectory_, route_handler());
  EXPECT_EQ(selector.evaluate(objects, {}).avoidance_targets.objects.size(), 1U);
}

TYPED_TEST(UpdateObjectsTest, UnrelatedCrossingLaneDoesNotHideParkedVehicle)
{
  auto handler = make_route_handler();
  auto bound = [](const double x) {
    return lanelet::LineString3d{
      lanelet::utils::getId(),
      {lanelet::Point3d{lanelet::utils::getId(), x, 2.0, 0.0},
       lanelet::Point3d{lanelet::utils::getId(), x, 8.0, 0.0}}};
  };
  lanelet::Lanelet crossing{lanelet::utils::getId(), bound(14.0), bound(16.0)};
  crossing.setAttribute(lanelet::AttributeName::Subtype, lanelet::AttributeValueString::Road);
  handler->getRouteMap()->add(crossing);
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, -2.5)});
  selector.update_objects(make_time(0), objects, this->trajectory_, *handler);
  const auto result = selector.evaluate(objects, {});
  ASSERT_EQ(result.avoidance_targets.objects.size(), 1U);
  ASSERT_TRUE(result.decisions.front().lane_center_offset_m);
  EXPECT_DOUBLE_EQ(*result.decisions.front().lane_center_offset_m, 2.5);
}

TYPED_TEST(UpdateObjectsTest, EnvelopePreservesObjectsOutsideRouteAndFutureCreepingConflicts)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>(
    {make_object<TypeParam>(1, 5.0, -8.0, 0.2, ObjectClassification::CAR, 1.0F, M_PI_2)});
  const auto trajectory = make_timed_filter_trajectory();
  selector.update_objects(make_time(0), objects, trajectory, route_handler());
  EXPECT_TRUE(selector.evaluate(objects, make_review_bounds()).avoidance_targets.objects.empty());
  EXPECT_TRUE(selector.evaluate(objects, make_review_bounds(), SelectionGeometryParams{2.0})
                .avoidance_targets.objects.empty());
  EXPECT_EQ(
    selector.evaluate(objects, make_review_bounds(), SelectionGeometryParams{2.0, 20.0})
      .avoidance_targets.objects.size(),
    1U);
}

TYPED_TEST(UpdateObjectsTest, SuppliedPathAdmissionSurvivesEnvelopeWithoutChangingOwnership)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 5.0, -8.0)});
  const auto trajectory = make_timed_filter_trajectory();
  selector.update_objects(make_time(0), objects, trajectory, route_handler());

  SelectionGeometryParams geometry{2.0};
  EXPECT_TRUE(selector.evaluate(objects, {}, geometry).avoidance_targets.objects.empty());
  geometry.predicted_path_target_ids.push_back(objects.objects.front().object_id.uuid);
  const auto selected = selector.evaluate(objects, {}, geometry);
  ASSERT_EQ(selected.avoidance_targets.objects.size(), 1U);
  EXPECT_EQ(selected.decisions.front().disposition, ObjectDisposition::avoidance);

  // A prediction cannot promote a lane-centred vehicle into the parked-vehicle class.
  auto lane_objects = make_objects<TypeParam>({make_object<TypeParam>(2, 5.0, 0.0)});
  lane_objects.header.stamp = make_time(1);
  selector.update_objects(make_time(1), lane_objects, trajectory, route_handler());
  geometry.predicted_path_target_ids = {lane_objects.objects.front().object_id.uuid};
  const auto lane_result = selector.evaluate(lane_objects, {}, geometry);
  EXPECT_TRUE(lane_result.avoidance_targets.objects.empty());
  ASSERT_EQ(lane_result.decisions.size(), 1U);
  EXPECT_EQ(lane_result.decisions.front().disposition, ObjectDisposition::yield);
}

TYPED_TEST(UpdateObjectsTest, InvalidNonemptyRouteBoundsHaveExplicitStatus)
{
  typename TestFixture::Selector selector;
  const auto objects = make_objects<TypeParam>({make_object<TypeParam>(1, 15.0, -2.5)});
  selector.update_objects(make_time(0), objects, this->trajectory_, route_handler());
  auto bounds = make_review_bounds();
  bounds.second.clear();
  EXPECT_EQ(selector.evaluate(objects, bounds).status, SelectionStatus::invalid_route_bounds);
  // MPPI's envelope deliberately replaces route-bound pruning.
  EXPECT_EQ(
    selector.evaluate(objects, bounds, SelectionGeometryParams{6.9}).status,
    SelectionStatus::ready);
}

}  // namespace
}  // namespace autoware::avoidance_target_detector
