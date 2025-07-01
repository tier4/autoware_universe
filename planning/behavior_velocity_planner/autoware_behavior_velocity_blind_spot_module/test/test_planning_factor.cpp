// Copyright 2025 TIER IV, Inc.
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

#include <autoware/behavior_velocity_planner/test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_planning_msgs/msg/path.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware::test_utils
{
std::string get_absolute_path_to_test_data(
  const std::string & package_name, const std::string & config_filename)
{
  const auto dir = ament_index_cpp::get_package_share_directory(package_name);
  return dir + "/test_data/" + config_filename;
}

}  // namespace autoware::test_utils

namespace autoware::behavior_velocity_planner
{

template <class T>
T loadMessageInYaml(
  const std::string & yaml_file, std::vector<std::string> corrupted_check_list = {})
{
  const auto yaml_path = autoware::test_utils::get_absolute_path_to_test_data(
    "autoware_behavior_velocity_blind_spot_module", yaml_file);

  YAML::Node node = YAML::LoadFile(yaml_path);
  for (auto & word : corrupted_check_list) {
    if (node[word].IsNull()) {
      throw std::runtime_error(
        "Failed to parse YAML file: " + yaml_path + ". The file might be corrupted.");
    }
  }

  return autoware::test_utils::parse<T>(node);
}

PathWithLaneId loadPathWithLaneIdInYaml(
  const std::string & yaml_file = "path_with_lane_id_data.yaml")
{
  const auto yaml_path = autoware::test_utils::get_absolute_path_to_test_data(
    "autoware_behavior_velocity_blind_spot_module", yaml_file);

  if (const auto path = autoware::test_utils::parse<std::optional<PathWithLaneId>>(yaml_path)) {
    return *path;
  }

  throw std::runtime_error(
    "Failed to parse YAML file: " + yaml_path + ". The file might be corrupted.");
}

nav_msgs::msg::Odometry loadOdometryInYaml(
  const std::string & yaml_file = "vehicle_odometry_data.yaml")
{
  return loadMessageInYaml<nav_msgs::msg::Odometry>(yaml_file, {"pose"});
}

PredictedObjects loadPathObjectsInYaml(const std::string & yaml_file = "dynamic_objects_data.yaml")
{
  return loadMessageInYaml<PredictedObjects>(yaml_file, {"objects"});
}

autoware_perception_msgs::msg::TrafficLightGroupArray loadTrafficLightGroupArrayInYaml(
  const std::string & yaml_file = "traffic_light_group_array_data.yaml")
{
  return loadMessageInYaml<autoware_perception_msgs::msg::TrafficLightGroupArray>(
    yaml_file, {"traffic_light_groups"});
}

TEST(PlanningFactorTest, NodeTestWithPredictedObjects)
{
  rclcpp::init(0, nullptr);

  const auto plugin_info_vec = {autoware::behavior_velocity_planner::PluginInfo{
    "blind_spot", "autoware::behavior_velocity_planner::BlindSpotModulePlugin"}};

  auto test_manager = autoware::behavior_velocity_planner::generateTestManager();
  auto test_target_node = autoware::behavior_velocity_planner::generateNode(plugin_info_vec);
  publishMandatoryTopics(test_manager, test_target_node);
  test_manager->publishInput(
    test_target_node, "behavior_velocity_planner_node/input/vector_map",
    autoware::test_utils::makeMapBinMsg("autoware_test_utils", "intersection/lanelet2_map.osm"));

  const std::string input_path_with_lane_id_topic =
    "behavior_velocity_planner_node/input/path_with_lane_id";
  const std::string input_odometry_topic = "behavior_velocity_planner_node/input/vehicle_odometry";
  const std::string input_dynamic_objects_topic =
    "behavior_velocity_planner_node/input/dynamic_objects";
  const std::string output_planning_factors_topic = "planning/planning_factors/blind_spot";

  const rclcpp::Node::SharedPtr test_node = test_manager->getTestNode();

  autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr planning_factor_msg;
  const auto test_sub =
    test_node->create_subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>(
      output_planning_factors_topic, rclcpp::QoS{1},
      [&planning_factor_msg](
        autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr msg) {
        planning_factor_msg = msg;
      });

  const auto objects = loadPathObjectsInYaml();
  const auto odometry = loadOdometryInYaml();
  const auto path = loadPathWithLaneIdInYaml();

  test_manager->publishInput(test_target_node, input_dynamic_objects_topic, objects);
  test_manager->publishInput(test_target_node, input_odometry_topic, odometry);
  test_manager->publishInput(test_target_node, input_path_with_lane_id_topic, path);

  // make sure behavior_path_planner is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // make sure planning_factor_msg is received
  EXPECT_NE(planning_factor_msg, nullptr);

  // make sure planning_factor_msg is not empty
  EXPECT_EQ(planning_factor_msg->factors.size(), 1);

  const auto & factor = planning_factor_msg->factors.front();
  EXPECT_FALSE(factor.safety_factors.is_safe);

  // make sure control_points is not empty
  EXPECT_EQ(factor.control_points.size(), 1);
  EXPECT_LT(factor.control_points.front().distance, 200.0);

  // make sure safety_factors is not empty
  EXPECT_EQ(factor.safety_factors.factors.size(), 1);

  const auto & safety_factor = factor.safety_factors.factors.front();
  const auto expected_object_type = autoware_internal_planning_msgs::msg::SafetyFactor::OBJECT;
  EXPECT_EQ(safety_factor.type, expected_object_type);
  EXPECT_FALSE(safety_factor.is_safe);
  EXPECT_EQ(safety_factor.points.size(), 1);
  const bool is_object_id_included = std::any_of(
    objects.objects.begin(), objects.objects.end(),
    [&](const auto & object) { return object.object_id == safety_factor.object_id; });
  EXPECT_TRUE(is_object_id_included);

  rclcpp::shutdown();
}

}  // namespace autoware::behavior_velocity_planner
