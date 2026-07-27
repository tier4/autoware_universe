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

#include "autoware/map_based_prediction/predictor_vru/vegetation.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <memory>
#include <vector>

namespace autoware::map_based_prediction
{
namespace
{
lanelet::Point3d make_point(const lanelet::Id id, const double x, const double y)
{
  return lanelet::Point3d(id, x, y, 0.0);
}

lanelet::Polygon3d make_vegetation_polygon(
  lanelet::Id & next_id, const std::vector<std::pair<double, double>> & vertices)
{
  std::vector<lanelet::Point3d> points;
  points.reserve(vertices.size());
  for (const auto & [x, y] : vertices) {
    points.push_back(make_point(next_id++, x, y));
  }

  lanelet::Polygon3d polygon(next_id++, points);
  polygon.setAttribute(lanelet::AttributeName::Type, "area");
  polygon.setAttribute(lanelet::AttributeName::Subtype, "vegetation");
  return polygon;
}

PredictedPath make_path(const std::vector<std::pair<double, double>> & xy_points)
{
  PredictedPath path;
  for (const auto & [x, y] : xy_points) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
    path.path.push_back(pose);
  }
  return path;
}

autoware_perception_msgs::msg::PredictedObject make_predicted_object(
  const PredictedPath & path, const double length, const double width)
{
  autoware_perception_msgs::msg::PredictedObject object;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = length;
  object.shape.dimensions.y = width;
  object.kinematics.predicted_paths.push_back(path);
  return object;
}

std::shared_ptr<lanelet::LaneletMap> make_vegetation_map(
  const std::vector<std::pair<double, double>> & vertices)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Id next_id = 1000;
  map->add(make_vegetation_polygon(next_id, vertices));
  return map;
}

TEST(VegetationModule, CutsPathWhenLongSegmentCrossesVegetation)
{
  VegetationModule module;
  module.build_from_map(make_vegetation_map({{4.8, -1.0}, {5.2, -1.0}, {5.2, 1.0}, {4.8, 1.0}}));

  const auto path = make_path({{0.0, 0.0}, {10.0, 0.0}});
  const auto object = make_predicted_object(path, 1.0, 1.0);

  const auto cut_paths = module.cut_paths_crossing_vegetation(object);

  ASSERT_EQ(cut_paths.size(), 1U);
  ASSERT_EQ(cut_paths.front().path.size(), 1U);
  EXPECT_DOUBLE_EQ(cut_paths.front().path.front().position.x, 0.0);
}

TEST(VegetationModule, CutsPathWhenSweptFootprintTouchesVegetationOutsideCenterLineBBox)
{
  VegetationModule module;
  module.build_from_map(make_vegetation_map({{4.8, 0.7}, {5.2, 0.7}, {5.2, 0.9}, {4.8, 0.9}}));

  // The center line stays at y=1.1 and does not intersect the vegetation polygon.
  // The object's width makes the swept footprint reach down to y=0.6, so the segment should be
  // cut once the convex hull of the endpoint footprints is considered.
  const auto path = make_path({{0.0, 1.1}, {10.0, 1.1}});
  const auto object = make_predicted_object(path, 1.0, 1.0);

  const auto cut_paths = module.cut_paths_crossing_vegetation(object);

  ASSERT_EQ(cut_paths.size(), 1U);
  ASSERT_EQ(cut_paths.front().path.size(), 1U);
  EXPECT_DOUBLE_EQ(cut_paths.front().path.front().position.y, 1.1);
}

}  // namespace
}  // namespace autoware::map_based_prediction
