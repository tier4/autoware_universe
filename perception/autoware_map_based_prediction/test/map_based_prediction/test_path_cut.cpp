// Copyright 2026 TIER IV, inc.
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

#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"
#include "autoware/map_based_prediction/predictor_vru/guard_rail.hpp"
#include "autoware/map_based_prediction/predictor_vru/road_border.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace autoware::map_based_prediction::path_cut
{
namespace
{
using autoware_perception_msgs::msg::ObjectClassification;

PredictedPath make_path_of_length(const size_t num_poses)
{
  PredictedPath path;
  path.confidence = 0.7F;
  path.time_step.sec = 1;
  path.time_step.nanosec = 500000000;
  for (size_t i = 0; i < num_poses; ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = static_cast<double>(i);
    path.path.push_back(pose);
  }
  return path;
}

TEST(PathCutUtils, StoppingDistanceIsZeroWhenNotMoving)
{
  EXPECT_DOUBLE_EQ(distance_to_stop_with_max_deceleration(0.0, 5.0), 0.0);
  EXPECT_DOUBLE_EQ(distance_to_stop_with_max_deceleration(-1.0, 5.0), 0.0);
}

TEST(PathCutUtils, StoppingDistanceUsesConstantDeceleration)
{
  // v^2 / (2 a) = 100 / (2 * 5) = 10
  EXPECT_DOUBLE_EQ(distance_to_stop_with_max_deceleration(10.0, 5.0), 10.0);
}

TEST(PathCutUtils, StoppingDistanceIsInfiniteForNonPositiveDeceleration)
{
  EXPECT_TRUE(std::isinf(distance_to_stop_with_max_deceleration(10.0, 0.0)));
  EXPECT_TRUE(std::isinf(distance_to_stop_with_max_deceleration(10.0, -1.0)));
}

TEST(PathCutUtils, CanStopBeforeTheLineIsInclusiveAtTheBoundary)
{
  // distance_to_stop_with_max_deceleration(10, 5) == 10, so distance == 10 counts as stoppable.
  EXPECT_TRUE(can_stop_before_the_line(10.0, 10.0, 5.0));
  EXPECT_TRUE(can_stop_before_the_line(10.0001, 10.0, 5.0));
  EXPECT_FALSE(can_stop_before_the_line(9.9999, 10.0, 5.0));
}

TEST(PathCutUtils, CanStopBeforeTheLineIsFalseForNonPositiveDeceleration)
{
  EXPECT_FALSE(can_stop_before_the_line(1000.0, 1.0, 0.0));
  EXPECT_FALSE(can_stop_before_the_line(1000.0, 0.0, -1.0));
}

TEST(PathCutUtils, MaxDecelerationForLabelMapsEachClass)
{
  MaxDecelerationParams params;
  params.vehicle = 5.0;
  params.pedestrian = 1.5;
  params.bicycle = 2.0;
  params.motorcycle = 3.0;

  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::CAR), 5.0);
  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::BUS), 5.0);
  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::TRUCK), 5.0);
  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::TRAILER), 5.0);
  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::MOTORCYCLE), 3.0);
  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::PEDESTRIAN), 1.5);
  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::BICYCLE), 2.0);
  // Unknown falls back to the vehicle value.
  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::UNKNOWN), 5.0);
}

TEST(PathCutUtils, ForceCutKeepsInclusiveRangeAndMetadata)
{
  const auto path = make_path_of_length(5);
  const auto cut = force_cut_at_index(path, 2);

  ASSERT_EQ(cut.path.size(), 3U);  // keep indices [0, 1, 2]
  EXPECT_DOUBLE_EQ(cut.path.back().position.x, 2.0);
  EXPECT_FLOAT_EQ(cut.confidence, 0.7F);
  EXPECT_EQ(cut.time_step.sec, 1);
  EXPECT_EQ(cut.time_step.nanosec, 500000000U);
}

TEST(PathCutUtils, ForceCutClampsToPathSize)
{
  const auto path = make_path_of_length(3);
  const auto cut = force_cut_at_index(path, 100);
  EXPECT_EQ(cut.path.size(), 3U);
}

TEST(PathCutUtils, ForceCutKeepsSinglePoseAtIndexZero)
{
  const auto path = make_path_of_length(4);
  const auto cut = force_cut_at_index(path, 0);
  ASSERT_EQ(cut.path.size(), 1U);
  EXPECT_DOUBLE_EQ(cut.path.front().position.x, 0.0);
}

}  // namespace
}  // namespace autoware::map_based_prediction::path_cut

namespace autoware::map_based_prediction
{
namespace
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::TrackedObject;

// road border is a vertical line crossing the straight path (along +x) at x = kBorderX.
constexpr double kBorderX = 5.5;

lanelet::LineString3d make_road_border(const lanelet::Id id)
{
  lanelet::LineString3d border(
    id, {lanelet::Point3d(id + 1, kBorderX, -10.0, 0.0),
         lanelet::Point3d(id + 2, kBorderX, 10.0, 0.0)});
  border.attributes()[lanelet::AttributeNamesString::Type] = std::string("road_border");
  return border;
}

// crosswalk rectangle covering x in [x_min, x_max], y in [-2, 2].
lanelet::Lanelet make_crosswalk(const lanelet::Id id, const double x_min, const double x_max)
{
  const lanelet::LineString3d left(
    id + 1,
    {lanelet::Point3d(id + 2, x_min, -2.0, 0.0), lanelet::Point3d(id + 3, x_max, -2.0, 0.0)});
  const lanelet::LineString3d right(
    id + 4, {lanelet::Point3d(id + 5, x_min, 2.0, 0.0), lanelet::Point3d(id + 6, x_max, 2.0, 0.0)});
  lanelet::Lanelet crosswalk(id, left, right);
  crosswalk.attributes()[lanelet::AttributeNamesString::Subtype] = std::string("crosswalk");
  return crosswalk;
}

std::shared_ptr<lanelet::LaneletMap> make_map()
{
  return lanelet::utils::createMap(lanelet::LineStrings3d{make_road_border(100)});
}

std::shared_ptr<lanelet::LaneletMap> make_map_with_crosswalk(const double x_min, const double x_max)
{
  std::shared_ptr<lanelet::LaneletMap> map =
    lanelet::utils::createMap(lanelet::Lanelets{make_crosswalk(200, x_min, x_max)});
  map->add(make_road_border(100));
  return map;
}

PredictedPath make_straight_path(const size_t num_poses)
{
  PredictedPath path;
  path.confidence = 1.0F;
  path.time_step.sec = 0;
  path.time_step.nanosec = 500000000;
  for (size_t i = 0; i < num_poses; ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = static_cast<double>(i);
    pose.position.y = 0.0;
    pose.orientation.w = 1.0;
    path.path.push_back(pose);
  }
  return path;
}

TrackedObject make_object(const uint8_t label, const double speed)
{
  TrackedObject object;
  ObjectClassification classification;
  classification.label = label;
  classification.probability = 1.0;
  object.classification.push_back(classification);
  object.kinematics.twist_with_covariance.twist.linear.x = speed;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 1.2;
  object.shape.dimensions.y = 1.0;
  object.shape.dimensions.z = 1.0;
  return object;
}

RoadBorderModule make_module(std::shared_ptr<lanelet::LaneletMap> map)
{
  RoadBorderModule module;
  module.build_from_map(std::move(map), {"road_border"});
  return module;
}

TEST(RoadBorderModule, CutsWhenPedestrianCanStopBeforeBorder)
{
  const auto module = make_module(make_map());
  const auto path = make_straight_path(11);
  const auto object = make_object(ObjectClassification::PEDESTRIAN, 1.0);
  path_cut::MaxDecelerationParams params;
  params.pedestrian = 2.0;  // stopping distance 0.25 << border distance 5.5

  const auto cut = module.cut_path_at_road_border(path, object, params);

  EXPECT_LT(cut.path.size(), path.path.size());
  ASSERT_FALSE(cut.path.empty());
  EXPECT_LT(cut.path.back().position.x, kBorderX);
}

TEST(RoadBorderModule, KeepsPathWhenPedestrianCannotStop)
{
  const auto module = make_module(make_map());
  const auto path = make_straight_path(11);
  const auto object = make_object(ObjectClassification::PEDESTRIAN, 5.0);
  path_cut::MaxDecelerationParams params;
  params.pedestrian = 0.5;  // stopping distance 25 > border distance 5.5

  const auto cut = module.cut_path_at_road_border(path, object, params);

  EXPECT_EQ(cut.path.size(), path.path.size());
}

TEST(RoadBorderModule, DoesNotCutInsideCrosswalk)
{
  const auto module = make_module(make_map_with_crosswalk(4.0, 7.0));
  const auto path = make_straight_path(11);
  const auto object = make_object(ObjectClassification::PEDESTRIAN, 1.0);
  path_cut::MaxDecelerationParams params;
  params.pedestrian = 2.0;  // would cut, but crossing is inside the crosswalk

  const auto cut = module.cut_path_at_road_border(path, object, params);

  EXPECT_EQ(cut.path.size(), path.path.size());
}

TEST(RoadBorderModule, DoesNotCutJustOutsideCrosswalkWithinMargin)
{
  const auto module = make_module(make_map_with_crosswalk(5.5, 9.0));
  const auto path = make_straight_path(11);
  const auto object = make_object(ObjectClassification::PEDESTRIAN, 1.0);
  path_cut::MaxDecelerationParams params;
  params.pedestrian = 2.0;

  const auto cut = module.cut_path_at_road_border(path, object, params);

  EXPECT_EQ(cut.path.size(), path.path.size());
}

TEST(RoadBorderModule, CutsWhenCrosswalkIsBeyondMargin)
{
  const auto module = make_module(make_map_with_crosswalk(8.0, 11.0));
  const auto path = make_straight_path(11);
  const auto object = make_object(ObjectClassification::PEDESTRIAN, 1.0);
  path_cut::MaxDecelerationParams params;
  params.pedestrian = 2.0;

  const auto cut = module.cut_path_at_road_border(path, object, params);

  EXPECT_LT(cut.path.size(), path.path.size());
}

TEST(RoadBorderModule, KeepsPathWhenNoBorderCrossing)
{
  const auto module = make_module(make_map());
  const auto path = make_straight_path(4);  // reaches only x = 3, border at 5.5
  const auto object = make_object(ObjectClassification::PEDESTRIAN, 1.0);
  path_cut::MaxDecelerationParams params;
  params.pedestrian = 2.0;

  const auto cut = module.cut_path_at_road_border(path, object, params);

  EXPECT_EQ(cut.path.size(), path.path.size());
}

TEST(RoadBorderModule, UsesClassSpecificDeceleration)
{
  const auto module = make_module(make_map());
  const auto path = make_straight_path(11);
  path_cut::MaxDecelerationParams params;
  params.pedestrian = 0.1;  // stopping distance 20 > 5.5 -> keep
  params.bicycle = 10.0;    // stopping distance 0.2 < 5.5 -> cut

  const auto pedestrian = make_object(ObjectClassification::PEDESTRIAN, 2.0);
  const auto bicycle = make_object(ObjectClassification::BICYCLE, 2.0);

  EXPECT_EQ(module.cut_path_at_road_border(path, pedestrian, params).path.size(), path.path.size());
  EXPECT_LT(module.cut_path_at_road_border(path, bicycle, params).path.size(), path.path.size());
}

}  // namespace
}  // namespace autoware::map_based_prediction

namespace autoware::map_based_prediction
{
namespace
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::Shape;

constexpr double kGuardRailX = 5.5;

lanelet::LineString3d make_guard_rail(const lanelet::Id id)
{
  lanelet::LineString3d guard_rail(
    id, {lanelet::Point3d(id + 1, kGuardRailX, -10.0, 0.0),
         lanelet::Point3d(id + 2, kGuardRailX, 10.0, 0.0)});
  guard_rail.attributes()[lanelet::AttributeNamesString::Type] = std::string("guard_rail");
  return guard_rail;
}

std::shared_ptr<lanelet::LaneletMap> make_guard_rail_map()
{
  return lanelet::utils::createMap(lanelet::LineStrings3d{make_guard_rail(300)});
}

PredictedObject make_box_object(const size_t num_poses, const double dim_x, const double dim_y)
{
  PredictedObject object;
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = dim_x;
  object.shape.dimensions.y = dim_y;
  object.shape.dimensions.z = 1.0;

  PredictedPath path;
  path.confidence = 1.0F;
  path.time_step.sec = 0;
  path.time_step.nanosec = 500000000;
  for (size_t i = 0; i < num_poses; ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = static_cast<double>(i);
    pose.position.y = 0.0;
    pose.orientation.w = 1.0;
    path.path.push_back(pose);
  }
  object.kinematics.predicted_paths.push_back(path);
  return object;
}

GuardRailModule make_guard_rail_module(std::shared_ptr<lanelet::LaneletMap> map)
{
  GuardRailModule module;
  module.build_from_map(std::move(map));
  return module;
}

TEST(GuardRailModule, ForceCutsWhenFootprintCrossesGuardRail)
{
  const auto module = make_guard_rail_module(make_guard_rail_map());
  const auto object = make_box_object(11, 1.0, 1.0);

  const auto cut = module.cut_paths_crossing_guard_rail(object);

  ASSERT_EQ(cut.size(), 1U);
  EXPECT_LT(cut.front().path.size(), object.kinematics.predicted_paths.front().path.size());
  ASSERT_FALSE(cut.front().path.empty());
  EXPECT_LT(cut.front().path.back().position.x, kGuardRailX);
}

TEST(GuardRailModule, KeepsPathWhenNoCrossing)
{
  const auto module = make_guard_rail_module(make_guard_rail_map());
  const auto object = make_box_object(4, 1.0, 1.0);

  const auto cut = module.cut_paths_crossing_guard_rail(object);

  ASSERT_EQ(cut.size(), 1U);
  EXPECT_EQ(cut.front().path.size(), object.kinematics.predicted_paths.front().path.size());
}

TEST(GuardRailModule, ConsidersFootprintWidthBeyondCenterline)
{
  const auto module = make_guard_rail_module(make_guard_rail_map());
  const auto object = make_box_object(6, 2.0, 1.0);

  const auto cut = module.cut_paths_crossing_guard_rail(object);

  ASSERT_EQ(cut.size(), 1U);
  EXPECT_LT(cut.front().path.size(), object.kinematics.predicted_paths.front().path.size());
}

}  // namespace
}  // namespace autoware::map_based_prediction
