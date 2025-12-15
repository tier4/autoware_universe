// Copyright 2023-2026 the Autoware Foundation
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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class NodeIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
};

// Test 1: Node can be launched
TEST_F(NodeIntegrationTest, NodeLaunch)
{
  // This test verifies that a basic node can be instantiated
  EXPECT_TRUE(node_ != nullptr);
  EXPECT_STREQ(node_->get_name(), "test_node");
}

// Test 2: ROS2 context is valid
TEST_F(NodeIntegrationTest, ROS2ContextValid)
{
  // Verify ROS2 context is properly initialized
  EXPECT_TRUE(rclcpp::ok());
  
  // Can create another node
  EXPECT_NO_THROW({
    auto another_node = std::make_shared<rclcpp::Node>("another_test_node");
    EXPECT_TRUE(another_node != nullptr);
  });
}

// Test 3: Parameter validation
TEST_F(NodeIntegrationTest, ParameterValidation)
{
  // Create a simple test node with parameters
  rclcpp::NodeOptions options;
  options.append_parameter_override("test_param", 0.5);
  
  // Verify parameters can be set
  EXPECT_NO_THROW({
    auto param_node = std::make_shared<rclcpp::Node>("test_param_node", options);
    EXPECT_TRUE(param_node != nullptr);
  });
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

