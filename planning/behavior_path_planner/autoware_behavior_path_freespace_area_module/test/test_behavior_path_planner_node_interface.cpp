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

#include "autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

// Plugin discoverability test: the freespace_area manager must be loadable through pluginlib
// under the name registered in plugins.xml.
TEST(TestFreespaceAreaModule, PluginIsRegistered)
{
  pluginlib::ClassLoader<autoware::behavior_path_planner::SceneModuleManagerInterface> loader(
    "autoware_behavior_path_planner",
    "autoware::behavior_path_planner::SceneModuleManagerInterface");

  const std::string plugin_name = "autoware::behavior_path_planner::FreespaceAreaModuleManager";
  ASSERT_TRUE(loader.isClassAvailable(plugin_name));
  auto plugin = loader.createSharedInstance(plugin_name);
  ASSERT_NE(plugin, nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
