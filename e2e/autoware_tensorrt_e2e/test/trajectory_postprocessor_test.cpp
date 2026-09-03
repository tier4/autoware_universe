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

#include "autoware/tensorrt_e2e/postprocess/trajectory_postprocessor.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

namespace
{
constexpr int64_t kTimesteps = 40;

PostprocessParams make_params()
{
  PostprocessParams params;
  params.prediction_tensor = "prediction";
  params.horizon_seconds = 4.0;
  params.time_step = 0.1;
  params.velocity_smoothing_window = 8;
  params.stopping_threshold = 0.3;
  params.generator_name = "TestGenerator";
  return params;
}

std::vector<TensorSpec> make_output_specs(const std::vector<int64_t> & shape)
{
  return {TensorSpec{"prediction", shape, TensorDataType::kFLOAT32}};
}

EgoFrame make_ego_frame(const double map_x, const double map_y, const double velocity)
{
  EgoFrame ego;
  ego.odometry.pose.pose.position.x = map_x;
  ego.odometry.pose.pose.position.y = map_y;
  ego.odometry.pose.pose.orientation.w = 1.0;
  ego.odometry.twist.twist.linear.x = velocity;
  ego.reference_odometry = ego.odometry;
  ego.ego_to_map = Eigen::Matrix4d::Identity();
  ego.ego_to_map(0, 3) = map_x;
  ego.ego_to_map(1, 3) = map_y;
  ego.map_to_ego = ego.ego_to_map.inverse();
  ego.stamp = rclcpp::Time(0);
  return ego;
}

/// Straight-line ego prediction: x advances `step_m` per timestep, heading 0.
TensorMap make_straight_prediction(const int64_t num_agents, const double step_m)
{
  std::vector<float> data;
  data.reserve(num_agents * kTimesteps * 4);
  for (int64_t agent = 0; agent < num_agents; ++agent) {
    for (int64_t t = 0; t < kTimesteps; ++t) {
      data.push_back(static_cast<float>(step_m * static_cast<double>(t + 1)));  // x
      data.push_back(0.0f);                                                     // y
      data.push_back(1.0f);                                                     // cos(yaw)
      data.push_back(0.0f);                                                     // sin(yaw)
    }
  }
  TensorMap outputs;
  outputs.emplace(
    "prediction", Tensor::from_host({1, num_agents, kTimesteps, 4}, std::move(data)));
  return outputs;
}

}  // namespace

TEST(TrajectoryPostprocessorTest, RejectsNonStandardTimeStep)
{
  auto params = make_params();
  params.time_step = 0.5;
  EXPECT_THROW(TrajectoryPostprocessor{params}, std::runtime_error);
}

TEST(TrajectoryPostprocessorTest, ValidatesOutputSpecs)
{
  TrajectoryPostprocessor postprocessor(make_params());

  // Missing tensor
  EXPECT_THROW(
    postprocessor.validate_output_specs({TensorSpec{"other", {1, 40, 4}, {}}}),
    std::runtime_error);
  // Wrong pose dimension (neither (x, y, cos, sin) nor (x, y, yaw))
  EXPECT_THROW(
    postprocessor.validate_output_specs(make_output_specs({1, 40, 5})), std::runtime_error);
  EXPECT_THROW(
    postprocessor.validate_output_specs(make_output_specs({1, 40, 2})), std::runtime_error);
  // Horizon mismatch (80 steps = 8 s, config expects 4 s)
  EXPECT_THROW(
    postprocessor.validate_output_specs(make_output_specs({1, 80, 4})), std::runtime_error);
  // Unexpected rank
  EXPECT_THROW(
    postprocessor.validate_output_specs(make_output_specs({40, 4})), std::runtime_error);

  // Ego-only rank-3 shape
  postprocessor.validate_output_specs(make_output_specs({1, 40, 4}));
  EXPECT_EQ(postprocessor.num_timesteps(), kTimesteps);
  EXPECT_EQ(postprocessor.num_agents(), 1);
  EXPECT_EQ(postprocessor.pose_dim(), 4);

  // Ego-only (x, y, yaw) poses
  postprocessor.validate_output_specs(make_output_specs({1, 40, 3}));
  EXPECT_EQ(postprocessor.pose_dim(), 3);

  // Multi-agent rank-4 shape
  postprocessor.validate_output_specs(make_output_specs({1, 33, 40, 4}));
  EXPECT_EQ(postprocessor.num_agents(), 33);
}

TEST(TrajectoryPostprocessorTest, RejectsTooLargeSmoothingWindow)
{
  auto params = make_params();
  params.velocity_smoothing_window = 40;
  TrajectoryPostprocessor postprocessor(params);
  EXPECT_THROW(
    postprocessor.validate_output_specs(make_output_specs({1, 1, 40, 4})), std::runtime_error);
}

TEST(TrajectoryPostprocessorTest, ProducesTrajectoryInMapFrame)
{
  TrajectoryPostprocessor postprocessor(make_params());
  postprocessor.validate_output_specs(make_output_specs({1, 1, kTimesteps, 4}));

  const double ego_map_x = 100.0;
  const double ego_map_y = 50.0;
  const double step_m = 1.0;  // 1 m per 0.1 s -> 10 m/s
  const auto ego = make_ego_frame(ego_map_x, ego_map_y, 10.0);
  const auto outputs = make_straight_prediction(1, step_m);

  unique_identifier_msgs::msg::UUID uuid;
  const auto result = postprocessor.process(outputs, ego, nullptr, rclcpp::Time(0), uuid);

  ASSERT_EQ(result.trajectory.points.size(), static_cast<size_t>(kTimesteps));
  EXPECT_EQ(result.trajectory.header.frame_id, "map");

  // Positions are transformed from the ego frame to the map frame.
  EXPECT_DOUBLE_EQ(result.trajectory.points.front().pose.position.x, ego_map_x + step_m);
  EXPECT_DOUBLE_EQ(result.trajectory.points.front().pose.position.y, ego_map_y);
  EXPECT_DOUBLE_EQ(
    result.trajectory.points.back().pose.position.x, ego_map_x + step_m * kTimesteps);

  // Constant motion: smoothed velocity is distance / 0.1 s everywhere.
  for (const auto & point : result.trajectory.points) {
    EXPECT_NEAR(point.longitudinal_velocity_mps, 10.0f, 1e-3f);
  }

  // time_from_start of the 10th point (index 9) is 1.0 s.
  EXPECT_EQ(result.trajectory.points[9].time_from_start.sec, 1);
  EXPECT_EQ(result.trajectory.points[9].time_from_start.nanosec, 0U);

  // One candidate per batch, carrying the generator name.
  ASSERT_EQ(result.candidate_trajectories.candidate_trajectories.size(), 1U);
  ASSERT_EQ(result.candidate_trajectories.generator_info.size(), 1U);
  EXPECT_EQ(
    result.candidate_trajectories.generator_info.front().generator_name.data,
    "TestGenerator_batch_0");

  // Ego-only model: no predicted objects.
  EXPECT_FALSE(result.predicted_objects.has_value());
}

TEST(TrajectoryPostprocessorTest, ReadsYawPosesLikeCosSinPoses)
{
  // The same left turn, once as (x, y, cos(yaw), sin(yaw)) and once as (x, y, yaw), must
  // decode to the same trajectory.
  constexpr double kYawStep = 0.02;
  std::vector<float> cos_sin_data;
  std::vector<float> yaw_data;
  for (int64_t t = 0; t < kTimesteps; ++t) {
    const double yaw = kYawStep * static_cast<double>(t + 1);
    const double x = static_cast<double>(t + 1);
    const double y = 0.1 * static_cast<double>(t + 1);
    cos_sin_data.insert(
      cos_sin_data.end(), {static_cast<float>(x), static_cast<float>(y),
                           static_cast<float>(std::cos(yaw)), static_cast<float>(std::sin(yaw))});
    yaw_data.insert(
      yaw_data.end(), {static_cast<float>(x), static_cast<float>(y), static_cast<float>(yaw)});
  }
  TensorMap cos_sin_outputs;
  cos_sin_outputs.emplace(
    "prediction", Tensor::from_host({1, kTimesteps, 4}, std::move(cos_sin_data)));
  TensorMap yaw_outputs;
  yaw_outputs.emplace("prediction", Tensor::from_host({1, kTimesteps, 3}, std::move(yaw_data)));

  const auto ego = make_ego_frame(10.0, 20.0, 10.0);
  unique_identifier_msgs::msg::UUID uuid;

  TrajectoryPostprocessor cos_sin_postprocessor(make_params());
  cos_sin_postprocessor.validate_output_specs(make_output_specs({1, kTimesteps, 4}));
  const auto expected =
    cos_sin_postprocessor.process(cos_sin_outputs, ego, nullptr, rclcpp::Time(0), uuid);

  TrajectoryPostprocessor yaw_postprocessor(make_params());
  yaw_postprocessor.validate_output_specs(make_output_specs({1, kTimesteps, 3}));
  const auto actual =
    yaw_postprocessor.process(yaw_outputs, ego, nullptr, rclcpp::Time(0), uuid);

  ASSERT_EQ(actual.trajectory.points.size(), expected.trajectory.points.size());
  for (size_t i = 0; i < actual.trajectory.points.size(); ++i) {
    const auto & a = actual.trajectory.points[i].pose;
    const auto & e = expected.trajectory.points[i].pose;
    EXPECT_NEAR(a.position.x, e.position.x, 1e-5);
    EXPECT_NEAR(a.position.y, e.position.y, 1e-5);
    EXPECT_NEAR(a.orientation.z, e.orientation.z, 1e-5);
    EXPECT_NEAR(a.orientation.w, e.orientation.w, 1e-5);
    EXPECT_NEAR(
      actual.trajectory.points[i].longitudinal_velocity_mps,
      expected.trajectory.points[i].longitudinal_velocity_mps, 1e-4f);
  }
}

TEST(TrajectoryPostprocessorTest, AppliesBaseLinkOffsetInReverse)
{
  auto params = make_params();
  params.base_link_offset = 1.5;
  TrajectoryPostprocessor postprocessor(params);
  postprocessor.validate_output_specs(make_output_specs({1, 1, kTimesteps, 4}));

  const auto ego = make_ego_frame(0.0, 0.0, 10.0);
  const auto outputs = make_straight_prediction(1, 1.0);

  unique_identifier_msgs::msg::UUID uuid;
  const auto result = postprocessor.process(outputs, ego, nullptr, rclcpp::Time(0), uuid);

  // The vehicle-center pose is shifted back to base_link along the heading (x axis here).
  EXPECT_DOUBLE_EQ(result.trajectory.points.front().pose.position.x, 1.0 - 1.5);
}

TEST(TrajectoryPostprocessorTest, ExtraTrajectoryTensorsBecomeCandidates)
{
  // A model may expose a main trajectory plus an ego-only prior trajectory.
  auto params = make_params();
  params.prediction_tensor = "trajectory";
  params.extra_trajectory_tensors = {"prior_trajectory"};
  TrajectoryPostprocessor postprocessor(params);

  const std::vector<TensorSpec> specs = {
    {"trajectory", {1, kTimesteps, 4}, TensorDataType::kFLOAT32},
    {"prior_trajectory", {1, kTimesteps, 4}, TensorDataType::kFLOAT32},
  };
  postprocessor.validate_output_specs(specs);

  auto outputs = make_straight_prediction(1, 1.0);
  auto prior = make_straight_prediction(1, 0.5);
  outputs.emplace("trajectory", outputs.at("prediction"));
  outputs.emplace("prior_trajectory", prior.at("prediction"));

  const auto ego = make_ego_frame(0.0, 0.0, 10.0);
  unique_identifier_msgs::msg::UUID uuid;
  const auto result = postprocessor.process(outputs, ego, nullptr, rclcpp::Time(0), uuid);

  // One candidate from the main output plus one from the prior.
  ASSERT_EQ(result.candidate_trajectories.candidate_trajectories.size(), 2U);
  EXPECT_EQ(
    result.candidate_trajectories.generator_info[1].generator_name.data,
    "TestGenerator_prior_trajectory_batch_0");
  // The prior advances 0.5 m per step instead of 1.0 m.
  EXPECT_DOUBLE_EQ(
    result.candidate_trajectories.candidate_trajectories[1].points.front().pose.position.x, 0.5);

  // A missing extra tensor at validation time is a startup error.
  TrajectoryPostprocessor strict(params);
  EXPECT_THROW(
    strict.validate_output_specs(
      {TensorSpec{"trajectory", {1, kTimesteps, 4}, TensorDataType::kFLOAT32}}),
    std::runtime_error);
}

TEST(TrajectoryPostprocessorTest, ThrowsOnMissingOrShortOutput)
{
  TrajectoryPostprocessor postprocessor(make_params());
  postprocessor.validate_output_specs(make_output_specs({1, 1, kTimesteps, 4}));
  const auto ego = make_ego_frame(0.0, 0.0, 0.0);
  unique_identifier_msgs::msg::UUID uuid;

  TensorMap empty_outputs;
  EXPECT_THROW(
    postprocessor.process(empty_outputs, ego, nullptr, rclcpp::Time(0), uuid),
    std::runtime_error);

  TensorMap short_outputs;
  short_outputs.emplace(
    "prediction", Tensor::from_host({1, 1, kTimesteps, 4}, std::vector<float>(10, 0.0f)));
  EXPECT_THROW(
    postprocessor.process(short_outputs, ego, nullptr, rclcpp::Time(0), uuid),
    std::runtime_error);
}

}  // namespace autoware::tensorrt_e2e
