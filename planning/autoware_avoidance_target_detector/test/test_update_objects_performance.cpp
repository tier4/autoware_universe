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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::avoidance_target_detector
{
namespace
{

using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletSegment;

constexpr double k_lanelet_length_m = 10.0;
constexpr double k_half_lane_width_m = 2.0;
constexpr auto k_max_update_duration = std::chrono::seconds{30};

struct PerformanceCase
{
  const char * name;
  std::size_t object_count;
  std::size_t trajectory_point_count;
  std::size_t lanelet_count;
};

lanelet::LaneletMapPtr make_lanelet_map(
  const std::size_t lanelet_count, std::vector<lanelet::Id> & lanelet_ids)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  std::vector<lanelet::Point3d> left_points;
  std::vector<lanelet::Point3d> right_points;
  left_points.reserve(lanelet_count + 1);
  right_points.reserve(lanelet_count + 1);

  for (std::size_t i = 0; i <= lanelet_count; ++i) {
    const double x = static_cast<double>(i) * k_lanelet_length_m;
    left_points.emplace_back(lanelet::utils::getId(), x, k_half_lane_width_m, 0.0);
    right_points.emplace_back(lanelet::utils::getId(), x, -k_half_lane_width_m, 0.0);
  }

  lanelet_ids.reserve(lanelet_count);
  for (std::size_t i = 0; i < lanelet_count; ++i) {
    lanelet::LineString3d left_bound{
      lanelet::utils::getId(), {left_points.at(i), left_points.at(i + 1)}};
    lanelet::LineString3d right_bound{
      lanelet::utils::getId(), {right_points.at(i), right_points.at(i + 1)}};
    lanelet::Lanelet lanelet{lanelet::utils::getId(), left_bound, right_bound};
    lanelet.setAttribute(lanelet::AttributeName::Type, lanelet::AttributeValueString::Lanelet);
    lanelet.setAttribute(lanelet::AttributeName::Subtype, lanelet::AttributeValueString::Road);
    lanelet.setAttribute(lanelet::AttributeName::Location, lanelet::AttributeValueString::Urban);
    lanelet.setAttribute(lanelet::AttributeName::OneWay, "yes");
    lanelet_ids.push_back(lanelet.id());
    map->add(lanelet);
  }

  return map;
}

LaneletRoute make_route(const std::vector<lanelet::Id> & lanelet_ids)
{
  LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose.position.x = 0.5;
  route.start_pose.orientation.w = 1.0;
  route.goal_pose.position.x = static_cast<double>(lanelet_ids.size()) * k_lanelet_length_m - 0.5;
  route.goal_pose.orientation.w = 1.0;
  route.segments.reserve(lanelet_ids.size());

  for (const auto lanelet_id : lanelet_ids) {
    LaneletPrimitive primitive;
    primitive.id = lanelet_id;
    primitive.primitive_type = "lane";

    LaneletSegment segment;
    segment.preferred_primitive = primitive;
    segment.primitives.push_back(primitive);
    route.segments.push_back(segment);
  }

  return route;
}

std::shared_ptr<ExtendedRouteHandler> make_extended_route_handler(const std::size_t lanelet_count)
{
  std::vector<lanelet::Id> lanelet_ids;
  auto map = make_lanelet_map(lanelet_count, lanelet_ids);

  LaneletMapBin map_msg;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  lanelet::utils::conversion::toBinMsg(map, &map_msg);
#pragma GCC diagnostic pop
  map_msg.header.frame_id = "map";

  auto route_handler = std::make_shared<ExtendedRouteHandler>(map_msg, make_route(lanelet_ids));
  route_handler->create_map();
  return route_handler;
}

const ExtendedRouteHandler & get_extended_route_handler(const std::size_t lanelet_count)
{
  static std::map<std::size_t, std::shared_ptr<ExtendedRouteHandler>> handlers;
  const auto [it, inserted] = handlers.try_emplace(lanelet_count);
  if (inserted) {
    it->second = make_extended_route_handler(lanelet_count);
  }
  return *it->second;
}

PredictedObjects make_objects(const std::size_t object_count, const std::size_t lanelet_count)
{
  PredictedObjects objects;
  objects.header.frame_id = "map";
  objects.objects.reserve(object_count);
  const double road_length_m = static_cast<double>(lanelet_count) * k_lanelet_length_m;
  const double usable_length_m = std::max(1.0, road_length_m - 2.0);

  for (std::size_t i = 0; i < object_count; ++i) {
    PredictedObject object;
    for (std::size_t byte = 0; byte < object.object_id.uuid.size(); ++byte) {
      const auto shift = static_cast<unsigned int>((byte % sizeof(i)) * 8U);
      object.object_id.uuid.at(byte) = static_cast<uint8_t>((i >> shift) & 0xffU);
    }

    ObjectClassification classification;
    classification.label = ObjectClassification::CAR;
    classification.probability = 1.0F;
    object.classification.push_back(classification);

    auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    pose.position.x = 1.0 + std::fmod(static_cast<double>(i) * 0.73, usable_length_m);
    pose.orientation.w = 1.0;
    object.kinematics.initial_twist_with_covariance.twist.linear.x = 2.0;
    object.shape.type = Shape::BOUNDING_BOX;
    object.shape.dimensions.x = 4.0;
    object.shape.dimensions.y = 1.8;
    object.shape.dimensions.z = 1.5;
    objects.objects.push_back(object);
  }

  return objects;
}

Trajectory make_trajectory(
  const std::size_t trajectory_point_count, const std::size_t lanelet_count)
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.points.reserve(trajectory_point_count);
  const double road_length_m = static_cast<double>(lanelet_count) * k_lanelet_length_m;

  for (std::size_t i = 0; i < trajectory_point_count; ++i) {
    TrajectoryPoint point;
    const double ratio =
      trajectory_point_count == 1
        ? 0.5
        : static_cast<double>(i) / static_cast<double>(trajectory_point_count - 1);
    point.pose.position.x = 0.5 + ratio * (road_length_m - 1.0);
    point.pose.orientation.w = 1.0;
    trajectory.points.push_back(point);
  }

  return trajectory;
}

class UpdateObjectsPerformanceTest : public testing::TestWithParam<PerformanceCase>
{
};

TEST_P(UpdateObjectsPerformanceTest, CompletesWithinPerformanceBudget)
{
  const auto & test_case = GetParam();
  const auto objects = make_objects(test_case.object_count, test_case.lanelet_count);
  const auto trajectory =
    make_trajectory(test_case.trajectory_point_count, test_case.lanelet_count);
  const auto & route_handler = get_extended_route_handler(test_case.lanelet_count);
  ASSERT_EQ(route_handler.getRouteMap()->laneletLayer.size(), test_case.lanelet_count);

  PredictedObjectSelector selector;
  const auto start = std::chrono::steady_clock::now();
  selector.update_objects(rclcpp::Time{1, 0}, objects, trajectory, route_handler);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  const auto elapsed_ms = std::chrono::duration<double, std::milli>{elapsed}.count();

  RecordProperty("object_count", std::to_string(test_case.object_count));
  RecordProperty("trajectory_point_count", std::to_string(test_case.trajectory_point_count));
  RecordProperty("lanelet_count", std::to_string(test_case.lanelet_count));
  RecordProperty("elapsed_ms", std::to_string(elapsed_ms));
  std::cout << test_case.name << ": update_objects() took " << elapsed_ms << " ms" << std::endl;

  EXPECT_LT(elapsed, k_max_update_duration);
}

INSTANTIATE_TEST_SUITE_P(
  InputSizes, UpdateObjectsPerformanceTest,
  testing::Values(
    PerformanceCase{"Objects0", 0, 100, 10}, PerformanceCase{"Objects1", 1, 100, 10},
    PerformanceCase{"Objects10", 10, 100, 10}, PerformanceCase{"Objects100", 100, 100, 10},
    PerformanceCase{"Objects1000", 1000, 100, 10}, PerformanceCase{"Trajectory0", 100, 0, 10},
    PerformanceCase{"Trajectory1", 100, 1, 10}, PerformanceCase{"Trajectory10", 100, 10, 10},
    PerformanceCase{"Trajectory1000", 100, 1000, 10}, PerformanceCase{"Lanelets1", 100, 100, 1},
    PerformanceCase{"Lanelets100", 100, 100, 100}, PerformanceCase{"FullScale", 1000, 1000, 100}),
  [](const testing::TestParamInfo<PerformanceCase> & info) { return info.param.name; });

}  // namespace
}  // namespace autoware::avoidance_target_detector
