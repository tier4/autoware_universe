// Copyright 2024 TIER IV, Inc.
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

#include "../src/utils.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <vector>

using autoware::behavior_velocity_planner::virtual_traffic_light::calcCenter;
using autoware::behavior_velocity_planner::virtual_traffic_light::calcHeadPose;
using autoware::behavior_velocity_planner::virtual_traffic_light::convertToGeomPoint;
using autoware::behavior_velocity_planner::virtual_traffic_light::createKeyValue;
using autoware::behavior_velocity_planner::virtual_traffic_light::findLastCollisionBeforeEndLine;
using autoware::behavior_velocity_planner::virtual_traffic_light::toAutowarePoints;

using Trajectory = autoware::experimental::trajectory::Trajectory<
  autoware_internal_planning_msgs::msg::PathPointWithLaneId>;

Trajectory generateStraightPath()
{
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> path_points;
  for (size_t i = 0; i < 10; ++i) {
    autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
    point.point.pose.position.x = static_cast<double>(i);
    point.point.pose.position.y = 0;
    point.point.pose.position.z = 0;
    point.point.longitudinal_velocity_mps = 10.0;
    path_points.push_back(point);
  }
  return *Trajectory::Builder{}.build(path_points);
}

lanelet::ConstLineString3d generateLineString(const std::vector<lanelet::BasicPoint3d> & points)
{
  lanelet::Points3d lanelet_points(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    lanelet_points[i] = lanelet::Point3d{lanelet::InvalId, points[i]};
  }
  return lanelet::ConstLineString3d{lanelet::InvalId, lanelet_points};
}

TEST(VirtualTrafficLightTest, CreateKeyValue)
{
  auto key_value = createKeyValue("test_key", "test_value");
  EXPECT_EQ(key_value.key, "test_key");
  EXPECT_EQ(key_value.value, "test_value");
}

TEST(VirtualTrafficLightTest, ToAutowarePoints)
{
  // lanelet::ConstLineString3d line_string
  {
    lanelet::LineString3d line_string;
    line_string.push_back(lanelet::Point3d(1, 1.0, 2.0, 3.0));
    line_string.push_back(lanelet::Point3d(2, 4.0, 5.0, 6.0));

    const auto result = toAutowarePoints(line_string);
    ASSERT_EQ(result.size(), 2);
    EXPECT_DOUBLE_EQ(result[0].x(), 1.0);
    EXPECT_DOUBLE_EQ(result[0].y(), 2.0);
    EXPECT_DOUBLE_EQ(result[0].z(), 3.0);
    EXPECT_DOUBLE_EQ(result[1].x(), 4.0);
    EXPECT_DOUBLE_EQ(result[1].y(), 5.0);
    EXPECT_DOUBLE_EQ(result[1].z(), 6.0);
  }

  // lanelet::Optional<lanelet::ConstLineString3d> line_string
  {
    lanelet::LineString3d line_string;
    line_string = lanelet::LineString3d();
    line_string.push_back(lanelet::Point3d(1, 1.0, 2.0, 3.0));
    line_string.push_back(lanelet::Point3d(2, 4.0, 5.0, 6.0));
    lanelet::Optional<lanelet::ConstLineString3d> line_string_opt = line_string;

    const auto result = toAutowarePoints(line_string_opt);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value().size(), 2);
    EXPECT_DOUBLE_EQ(result.value()[0].x(), 1.0);
    EXPECT_DOUBLE_EQ(result.value()[0].y(), 2.0);
    EXPECT_DOUBLE_EQ(result.value()[0].z(), 3.0);
    EXPECT_DOUBLE_EQ(result.value()[1].x(), 4.0);
    EXPECT_DOUBLE_EQ(result.value()[1].y(), 5.0);
    EXPECT_DOUBLE_EQ(result.value()[1].z(), 6.0);

    // nullopt
    const lanelet::Optional<lanelet::ConstLineString3d> line_string_nullopt = {};
    const auto result_nullopt = toAutowarePoints(line_string_nullopt);
    ASSERT_FALSE(result_nullopt.has_value());
  }

  // lanelet::ConstLineStrings3d line_strings
  {
    lanelet::LineString3d line_string1;
    line_string1.push_back(lanelet::Point3d(1, 1.0, 2.0, 3.0));
    line_string1.push_back(lanelet::Point3d(2, 4.0, 5.0, 6.0));

    lanelet::LineString3d line_string2;
    line_string2.push_back(lanelet::Point3d(3, 7.0, 8.0, 9.0));
    line_string2.push_back(lanelet::Point3d(4, 10.0, 11.0, 12.0));

    lanelet::ConstLineStrings3d line_strings;
    line_strings.push_back(line_string1);
    line_strings.push_back(line_string2);

    const auto result = toAutowarePoints(line_strings);
    ASSERT_EQ(result.size(), 2);
    ASSERT_EQ(result[0].size(), 2);
    ASSERT_EQ(result[1].size(), 2);
    EXPECT_DOUBLE_EQ(result[0][0].x(), 1.0);
    EXPECT_DOUBLE_EQ(result[0][0].y(), 2.0);
    EXPECT_DOUBLE_EQ(result[0][0].z(), 3.0);
    EXPECT_DOUBLE_EQ(result[0][1].x(), 4.0);
    EXPECT_DOUBLE_EQ(result[0][1].y(), 5.0);
    EXPECT_DOUBLE_EQ(result[0][1].z(), 6.0);
    EXPECT_DOUBLE_EQ(result[1][0].x(), 7.0);
    EXPECT_DOUBLE_EQ(result[1][0].y(), 8.0);
    EXPECT_DOUBLE_EQ(result[1][0].z(), 9.0);
    EXPECT_DOUBLE_EQ(result[1][1].x(), 10.0);
    EXPECT_DOUBLE_EQ(result[1][1].y(), 11.0);
    EXPECT_DOUBLE_EQ(result[1][1].z(), 12.0);
  }
}

TEST(VirtualTrafficLightTest, CalcCenter)
{
  autoware_utils::LineString3d line_string;
  line_string.emplace_back(1.0, 2.0, 3.0);
  line_string.emplace_back(4.0, 5.0, 6.0);

  auto center = calcCenter(line_string);
  EXPECT_DOUBLE_EQ(center.x(), 2.5);
  EXPECT_DOUBLE_EQ(center.y(), 3.5);
  EXPECT_DOUBLE_EQ(center.z(), 4.5);
}

TEST(VirtualTrafficLightTest, CalcHeadPose)
{
  geometry_msgs::msg::Pose base_link_pose;
  base_link_pose.position.x = 1.0;
  base_link_pose.position.y = 2.0;
  base_link_pose.position.z = 3.0;

  double base_link_to_front = 1.0;
  auto head_pose = calcHeadPose(base_link_pose, base_link_to_front);

  EXPECT_DOUBLE_EQ(head_pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(head_pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(head_pose.position.z, 3.0);
}

TEST(VirtualTrafficLightTest, ConvertToGeomPoint)
{
  autoware_utils::Point3d point(1.0, 2.0, 3.0);
  auto geom_point = convertToGeomPoint(point);

  EXPECT_DOUBLE_EQ(geom_point.x, 1.0);
  EXPECT_DOUBLE_EQ(geom_point.y, 2.0);
  EXPECT_DOUBLE_EQ(geom_point.z, 0.0);  // z is not set in convertToGeomPoint
}

TEST(VirtualTrafficLightTest, FindLastCollisionBeforeEndLine)
{
  // LineString3d
  // 1) find first collision point
  {
    std::cout << "----- find first collision point -----" << std::endl;
    const auto target_line = generateLineString({{0.0, -1.0, 0.0}, {0.0, 1.0, 0.0}});

    const auto path = generateStraightPath();

    const auto result = findLastCollisionBeforeEndLine(path, target_line, path.length());
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 0.0);
  }

  // 2-1)  find middle collision point
  {
    std::cout << "----- find middle collision point -----" << std::endl;

    const auto target_line = generateLineString({{5.0, -1.0, 0.0}, {5.0, 1.0, 0.0}});

    const auto path = generateStraightPath();

    // target line intersects with p5-p6
    const auto result = findLastCollisionBeforeEndLine(path, target_line, path.length());
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 5.0);
  }

  // 2-2)  find middle collision point
  {
    std::cout << "----- find middle collision point -----" << std::endl;

    const auto target_line = generateLineString({{4.5, -1.0, 0.0}, {4.5, 1.0, 0.0}});

    const auto path = generateStraightPath();

    // target line intersects with p4-p5
    const auto result = findLastCollisionBeforeEndLine(path, target_line, path.length());
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 4.5);
  }

  // std::vector<lanelet::ConstLineString3d>
  // 3) find middle collision point with multi target lines
  {
    std::cout << "----- find collision point with multi target lines -----" << std::endl;

    std::vector<lanelet::ConstLineString3d> target_lines;
    target_lines.push_back(generateLineString({{3.5, -1.0, 0.0}, {3.5, 1.0, 0.0}}));
    target_lines.push_back(generateLineString({{6.5, -1.0, 0.0}, {6.5, 1.0, 0.0}}));

    const auto path = generateStraightPath();

    // NOTE: the name of this function is findLastCollisionBeforeEndLine, but it returns the
    // collision with the first line added to multiple lines
    // the first target line intersects with p3-p4
    const auto result = findLastCollisionBeforeEndLine(path, target_lines, path.length());
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 3.5);
  }
}
