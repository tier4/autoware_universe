// Copyright 2026 The Autoware Contributors
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

#include "test_utils.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <exception>
#include <string>
#include <vector>

namespace autoware::control_command_gate::test
{

namespace
{

const std::vector<std::string> filter_namespaces{"nominal_filter.", "transition_filter."};

std::vector<std::string> names_in_both_namespaces(const std::vector<std::string> & names)
{
  std::vector<std::string> result;
  for (const auto & ns : filter_namespaces) {
    for (const auto & name : names) {
      result.push_back(ns + name);
    }
  }
  return result;
}

void expect_creation_fails_with(
  const std::vector<std::string> & removed, const std::string & reported)
{
  const auto params = remove_parameters(load_default_parameters(), removed);
  try {
    create_gate(params);
    ADD_FAILURE() << "node creation succeeded without " << reported;
  } catch (const std::exception & e) {
    EXPECT_NE(std::string(e.what()).find(reported), std::string::npos) << e.what();
  }
}

void expect_creation_exits(std::vector<rclcpp::Parameter> params)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_EXIT(create_gate(params), ::testing::ExitedWithCode(EXIT_FAILURE), "");
}

std::vector<rclcpp::Parameter> replace_parameter(
  const std::vector<rclcpp::Parameter> & params, const rclcpp::Parameter & replacement)
{
  auto result = remove_parameters(params, {replacement.get_name()});
  result.push_back(replacement);
  return result;
}

}  // namespace

TEST(ControlCmdGateSteerAccelParam, LoadsSteerAccelParametersFromDefaultConfig)
{
  const auto node = create_gate(load_default_parameters());
  for (const auto & ns : filter_namespaces) {
    EXPECT_FALSE(node->get_parameter(ns + "enable_steer_accel_limit").as_bool()) << ns;
    EXPECT_EQ(
      node->get_parameter(ns + "steer_accel_lim_for_steer_cmd").as_double_array(),
      provisional_steer_accel_lim())
      << ns;
    EXPECT_DOUBLE_EQ(node->get_parameter(ns + "steer_accel_clip_integral_th_diag").as_double(), 0.2)
      << ns;
    EXPECT_EQ(
      node->get_parameter(ns + "reference_speed_points").as_double_array(),
      reference_speed_points())
      << ns;
  }
}

TEST(ControlCmdGateSteerAccelParam, FailsWithoutAllNewParameters)
{
  expect_creation_fails_with(
    names_in_both_namespaces(
      {"enable_steer_accel_limit", "steer_accel_lim_for_steer_cmd",
       "steer_accel_clip_integral_th_diag"}),
    "nominal_filter.enable_steer_accel_limit");
}

TEST(ControlCmdGateSteerAccelParam, FailsWithoutClipIntegralThreshold)
{
  expect_creation_fails_with(
    names_in_both_namespaces({"steer_accel_clip_integral_th_diag"}),
    "nominal_filter.steer_accel_clip_integral_th_diag");
}

TEST(ControlCmdGateSteerAccelParam, FailsWithoutEnableFlag)
{
  expect_creation_fails_with(
    names_in_both_namespaces({"enable_steer_accel_limit"}),
    "nominal_filter.enable_steer_accel_limit");
}

TEST(ControlCmdGateSteerAccelParamDeathTest, ExitsWhenNominalSteerAccelLimitIsNotPositive)
{
  expect_creation_exits(replace_parameter(
    load_default_parameters(), rclcpp::Parameter(
                                 "nominal_filter.steer_accel_lim_for_steer_cmd",
                                 std::vector<double>{0.8, 0.76, 0.64, 0.0, 0.3, 0.3, 0.3})));
}

TEST(ControlCmdGateSteerAccelParamDeathTest, ExitsWhenTransitionSteerAccelLimitIsNotPositive)
{
  expect_creation_exits(replace_parameter(
    load_default_parameters(), rclcpp::Parameter(
                                 "transition_filter.steer_accel_lim_for_steer_cmd",
                                 std::vector<double>{0.8, 0.76, 0.64, -0.39, 0.3, 0.3, 0.3})));
}

}  // namespace autoware::control_command_gate::test
