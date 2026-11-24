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

#include "autoware/ml_planner/preprocessing/scene_input_builder.hpp"

#include "autoware/ml_planner/dimensions.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ml_planner::preprocess
{
namespace
{
constexpr size_t current_index = INPUT_T;
constexpr size_t sample_count = INPUT_T_WITH_CURRENT + OUTPUT_T;
constexpr int64_t sample_interval_ns = 100'000'000;
constexpr t4scene::Transform3D identity{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

std::shared_ptr<lanelet::LaneletMap> make_map()
{
  const auto boundary = [](const double y) {
    return lanelet::LineString3d(
      lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 0.0, y, 0.0),
                                lanelet::Point3d(lanelet::utils::getId(), 30.0, y, 0.0)});
  };
  auto lane = lanelet::Lanelet(lanelet::utils::getId(), boundary(2.0), boundary(-2.0));
  lane.setAttribute("subtype", "road");
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane);
  return map;
}

t4scene::T4Scene make_scene(const std::shared_ptr<lanelet::LaneletMap> & map)
{
  t4scene::T4Scene scene;
  scene.current_index = current_index;
  scene.timestamps_ns = t4scene::Array<int64_t>({sample_count});
  scene.ego.emplace();
  scene.ego->shape.emplace();
  scene.ego->shape->length = 4.5;
  scene.ego->shape->width = 2.0;
  scene.ego->shape->base_link_to_front = 3.5;
  scene.ego->shape->wheel_base = 2.8;
  scene.ego->poses_3d.emplace(std::vector<uint64_t>{sample_count, 4, 4});
  scene.ego->velocities_3d.emplace(std::vector<uint64_t>{sample_count, 3});
  scene.ego->angular_velocities_3d.emplace(std::vector<uint64_t>{sample_count, 3});
  scene.ego->tire_steering_angles.emplace(std::vector<uint64_t>{sample_count});
  scene.ego->turn_indicators.emplace(std::vector<uint64_t>{sample_count});
  scene.goal_poses_3d.emplace(std::vector<uint64_t>{sample_count, 4, 4});
  scene.map.emplace();
  scene.map->lanelet_map = map;
  scene.map->scope = t4scene::MapScope::Full;
  scene.map->has_elevation = true;
  scene.map->global_origin_3d = identity;
  scene.global_origin_3d = identity;

  const auto lane_id = map->laneletLayer.begin()->id();
  scene.routes.emplace();
  scene.routes->rows.push_back(t4scene::RouteData{{t4scene::RouteSegment{{lane_id}, lane_id}}});
  scene.routes->indices = t4scene::Array<int64_t>({sample_count});

  for (size_t row = 0; row < sample_count; ++row) {
    scene.timestamps_ns.set(row, static_cast<int64_t>(row + 1) * sample_interval_ns);
    for (size_t i = 0; i < 4; ++i) {
      scene.ego->poses_3d->set(row * 16 + i * 5, 1.0);
      scene.goal_poses_3d->set(row * 16 + i * 5, 1.0);
    }
    scene.ego->poses_3d->set(row * 16 + 3, 1.0);
    scene.ego->velocities_3d->set(row * 3, 2.0);
    scene.ego->tire_steering_angles->set(row, 0.05);
    scene.goal_poses_3d->set(row * 16 + 3, 20.0);
  }
  scene.validate();
  return scene;
}
}  // namespace

TEST(SceneInputBuilder, UsesDecisionAnchorAndPreferredRoute)
{
  const auto map = make_map();
  const auto context = build_map_context(map);
  auto scene = make_scene(map);
  scene.map->global_origin_3d.reset();
  scene.map->global_origin = t4scene::Pose2D{100.0, 0.0, 0.0};
  scene.global_origin_3d = t4scene::Transform3D{1.0, 0.0, 0.0, 104.0, 0.0, 1.0, 0.0, 0.0,
                                                0.0, 0.0, 1.0, 0.0,   0.0, 0.0, 0.0, 1.0};

  // A subsequent ego pose can be far from the current decision pose.
  scene.ego->poses_3d->set((current_index + 1) * 16 + 3, 1000.0);

  const auto result = build_scene_input(scene, *context);
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_DOUBLE_EQ(result->ego_to_map(0, 3), 5.0);
  EXPECT_FLOAT_EQ(result->tensors.at("goal_pose")(0), 19.0);
  const auto & route = result->tensors.at("route_waypoints");
  EXPECT_FLOAT_EQ(route(0, 0), 0.0);
  EXPECT_FLOAT_EQ(route(1, 0), 5.0);
  EXPECT_FLOAT_EQ(route(1, 2), 1.0);
  EXPECT_EQ(result->tensors.count("initial_noise"), 0U);
}

TEST(SceneInputBuilder, RecordedSignalsDoNotReadFutureEgo)
{
  const auto map = make_map();
  const auto context = build_map_context(map);
  auto scene = make_scene(map);
  EXPECT_FALSE(build_scene_input(scene, *context, TrafficLightFuture::Recorded).has_value());
  scene.traffic_lights.emplace();
  scene.traffic_lights->available = t4scene::Array<bool>({sample_count});
  scene.traffic_lights->group_offsets = t4scene::Array<int64_t>({sample_count + 1});
  scene.validate();
  const auto baseline = build_scene_input(scene, *context, TrafficLightFuture::Recorded);
  ASSERT_TRUE(baseline.has_value()) << baseline.error();
  for (size_t row = current_index + 1; row < sample_count; ++row) {
    scene.ego->poses_3d->set(row * 16 + 3, -900.0);
    scene.ego->velocities_3d->set(row * 3, 70.0);
    scene.ego->tire_steering_angles->set(row, -0.8);
    scene.goal_poses_3d->set(row * 16 + 3, -250.0);
  }
  const auto changed = build_scene_input(scene, *context, TrafficLightFuture::Recorded);
  ASSERT_TRUE(changed.has_value()) << changed.error();
  EXPECT_EQ(baseline->tensors, changed->tensors);
}

TEST(SceneInputBuilder, RecordedSignalsRetainObservationAcrossAnchor)
{
  const auto map = make_map();
  const auto lane_id = map->laneletLayer.begin()->id();
  auto lane = map->laneletLayer.get(lane_id);
  const auto light_id = lanelet::utils::getId();
  const auto light = lanelet::TrafficLight::make(light_id, {}, {lane.leftBound()});
  lane.addRegulatoryElement(light);
  map->add(light);
  const auto context = build_map_context(map);
  auto scene = make_scene(map);
  auto & sequence = scene.traffic_lights.emplace();
  sequence.available = t4scene::Array<bool>({sample_count});
  sequence.available.set(current_index - 1, true);
  sequence.available.set(current_index, true);
  sequence.group_offsets = t4scene::Array<int64_t>({sample_count + 1});
  for (size_t row = current_index; row <= sample_count; ++row) {
    sequence.group_offsets.set(row, 1);
  }
  sequence.values.group_ids = t4scene::Array<int64_t>::from_values({light_id}, {1});
  sequence.values.element_offsets = t4scene::Array<int64_t>::from_values({0, 1}, {2});
  sequence.values.element_colors = t4scene::Array<int64_t>::from_values({1}, {1});
  scene.validate();

  const auto result = build_scene_input(scene, *context, TrafficLightFuture::Recorded);
  ASSERT_TRUE(result.has_value()) << result.error();
  const auto & future = result->tensors.at("lane_traffic_light_future");
  EXPECT_FLOAT_EQ(future(0, 0, 2), 1.0F);
  EXPECT_FLOAT_EQ(future(0, 1, 3), 1.0F);
}

TEST(SceneInputBuilder, RequiresCoordinateRegistration)
{
  const auto map = make_map();
  const auto context = build_map_context(map);
  auto scene = make_scene(map);
  scene.map->global_origin_3d.reset();

  EXPECT_FALSE(build_scene_input(scene, *context).has_value());
}

TEST(SceneInputBuilder, PreservesMeasuredCylinderSides)
{
  const auto map = make_map();
  const auto context = build_map_context(map);
  auto scene = make_scene(map);
  auto & sequence = scene.annotations.emplace();
  sequence.available = t4scene::Array<bool>({sample_count});
  sequence.available.set(current_index, true);
  sequence.cylinder_offsets.emplace(std::vector<uint64_t>{sample_count + 1});
  for (size_t row = current_index + 1; row <= sample_count; ++row) {
    sequence.cylinder_offsets->set(row, 1);
  }
  auto & cylinder = sequence.values.cylinders.emplace();
  cylinder.positions = t4scene::Array<double>::from_values({4.0, 0.0, 0.0}, {1, 3});
  cylinder.orientations = t4scene::Array<double>::from_values({0.0, 0.0, 0.0, 1.0}, {1, 4});
  cylinder.labels = t4scene::Array<int64_t>::from_values({7}, {1});
  cylinder.dimensions = t4scene::Array<double>::from_values({0.65, 1.15, 1.75}, {1, 3});
  cylinder.track_ids.emplace(1, std::string{"cylinder"});
  scene.validate();

  const auto result = build_scene_input(scene, *context);
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(result->selected_track_ids, (std::vector<std::string>{"cylinder"}));
  EXPECT_FLOAT_EQ(result->tensors.at("agent_shape")(0, 0), 0.65F);
  EXPECT_FLOAT_EQ(result->tensors.at("agent_shape")(0, 1), 1.15F);
}

}  // namespace autoware::ml_planner::preprocess
