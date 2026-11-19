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
  // Wrong pose dimension
  EXPECT_THROW(
    postprocessor.validate_output_specs(make_output_specs({1, 40, 3})), std::runtime_error);
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
  const auto result = postprocessor.process(outputs, ego, rclcpp::Time(0), uuid);

  // The model's steps, led by the ego pose at t = 0.
  ASSERT_EQ(result.trajectory.points.size(), static_cast<size_t>(kTimesteps + 1));
  EXPECT_EQ(result.trajectory.header.frame_id, "map");

  // Positions are transformed from the ego frame to the map frame.
  EXPECT_DOUBLE_EQ(result.trajectory.points[1].pose.position.x, ego_map_x + step_m);
  EXPECT_DOUBLE_EQ(result.trajectory.points[1].pose.position.y, ego_map_y);
  EXPECT_DOUBLE_EQ(
    result.trajectory.points.back().pose.position.x, ego_map_x + step_m * kTimesteps);

  // Constant motion: smoothed velocity is distance / 0.1 s everywhere.
  for (const auto & point : result.trajectory.points) {
    EXPECT_NEAR(point.longitudinal_velocity_mps, 10.0f, 1e-3f);
  }

  // time_from_start of the 10th model step (index 10, after the t = 0 point) is 1.0 s.
  EXPECT_EQ(result.trajectory.points[10].time_from_start.sec, 1);
  EXPECT_EQ(result.trajectory.points[10].time_from_start.nanosec, 0U);

  // One candidate per batch, carrying the generator name.
  ASSERT_EQ(result.candidate_trajectories.candidate_trajectories.size(), 1U);
  ASSERT_EQ(result.candidate_trajectories.generator_info.size(), 1U);
  EXPECT_EQ(
    result.candidate_trajectories.generator_info.front().generator_name.data,
    "TestGenerator_batch_0");
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
  const auto result = postprocessor.process(outputs, ego, rclcpp::Time(0), uuid);

  // The vehicle-center pose is shifted back to base_link along the heading (x axis here).
  EXPECT_DOUBLE_EQ(result.trajectory.points[1].pose.position.x, 1.0 - 1.5);
  // The t = 0 point is the odometry's base_link pose already: it is not shifted.
  EXPECT_DOUBLE_EQ(result.trajectory.points.front().pose.position.x, 0.0);
}

TEST(TrajectoryPostprocessorTest, ExtraTrajectoryTensorsBecomeCandidates)
{
  // ResWorld-style contract: main output "trajectory" plus an ego-only "prior_trajectory".
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
  const auto result = postprocessor.process(outputs, ego, rclcpp::Time(0), uuid);

  // One candidate from the main output plus one from the prior.
  ASSERT_EQ(result.candidate_trajectories.candidate_trajectories.size(), 2U);
  EXPECT_EQ(
    result.candidate_trajectories.generator_info[1].generator_name.data,
    "TestGenerator_prior_trajectory_batch_0");
  // The prior advances 0.5 m per step instead of 1.0 m.
  EXPECT_DOUBLE_EQ(
    result.candidate_trajectories.candidate_trajectories[1].points[1].pose.position.x, 0.5);
  // Both candidates, and the trajectory, start at the ego pose at t = 0.
  for (const auto & candidate : result.candidate_trajectories.candidate_trajectories) {
    EXPECT_EQ(candidate.points.front().time_from_start.sec, 0);
    EXPECT_EQ(candidate.points.front().time_from_start.nanosec, 0U);
    EXPECT_DOUBLE_EQ(candidate.points.front().pose.position.x, 0.0);
  }

  // A missing extra tensor at validation time is a startup error.
  TrajectoryPostprocessor strict(params);
  EXPECT_THROW(
    strict.validate_output_specs(
      {TensorSpec{"trajectory", {1, kTimesteps, 4}, TensorDataType::kFLOAT32}}),
    std::runtime_error);
}

namespace
{
/// Check the t = 0 point and that the rest is the model's steps, unchanged.
void expect_leads_with_ego_pose(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const EgoFrame & ego,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & without_start)
{
  ASSERT_EQ(trajectory.points.size(), without_start.size() + 1);
  const auto & start = trajectory.points.front();
  EXPECT_EQ(start.time_from_start.sec, 0);
  EXPECT_EQ(start.time_from_start.nanosec, 0U);
  const auto & pose = ego.odometry.pose.pose;
  EXPECT_DOUBLE_EQ(start.pose.position.x, pose.position.x);
  EXPECT_DOUBLE_EQ(start.pose.position.y, pose.position.y);
  EXPECT_DOUBLE_EQ(start.pose.position.z, pose.position.z);
  EXPECT_DOUBLE_EQ(start.pose.orientation.x, pose.orientation.x);
  EXPECT_DOUBLE_EQ(start.pose.orientation.y, pose.orientation.y);
  EXPECT_DOUBLE_EQ(start.pose.orientation.z, pose.orientation.z);
  EXPECT_DOUBLE_EQ(start.pose.orientation.w, pose.orientation.w);
  // Same dynamics as the first plan point, so stop logic reads what it read before.
  EXPECT_NEAR(
    start.longitudinal_velocity_mps, without_start.front().longitudinal_velocity_mps, 1e-3f);
  EXPECT_FLOAT_EQ(start.acceleration_mps2, without_start.front().acceleration_mps2);
  // Everything after it is untouched, in order.
  for (size_t i = 0; i < without_start.size(); ++i) {
    const auto & a = trajectory.points[i + 1];
    const auto & b = without_start[i];
    EXPECT_EQ(a.time_from_start.sec, b.time_from_start.sec);
    EXPECT_EQ(a.time_from_start.nanosec, b.time_from_start.nanosec);
    EXPECT_DOUBLE_EQ(a.pose.position.x, b.pose.position.x);
    EXPECT_DOUBLE_EQ(a.pose.position.y, b.pose.position.y);
    EXPECT_NEAR(a.longitudinal_velocity_mps, b.longitudinal_velocity_mps, 1e-3f);
    EXPECT_FLOAT_EQ(a.acceleration_mps2, b.acceleration_mps2);
  }
}

int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}
}  // namespace

// The ego frame the two planning modes hand over differ only in the instant: under
// cloud_stamp the pose is at the cloud stamp (stamp == sensor_stamp); under planning_time it
// is at the newer odometry stamp and sensor_pose is the older, cloud-time pose. Either way
// the plan starts at ego.odometry at ego.stamp, never at sensor_pose.
TEST(TrajectoryPostprocessorTest, StartsAtEgoPoseAtTheStampInBothPlanningModes)
{
  TrajectoryPostprocessor postprocessor(make_params());
  postprocessor.validate_output_specs(make_output_specs({1, 1, kTimesteps, 4}));
  const auto outputs = make_straight_prediction(1, 1.0);
  unique_identifier_msgs::msg::UUID uuid;

  // The model's steps as the node published them before the t = 0 point existed: step k
  // (1-based) is k m ahead at k * 0.1 s, at a constant 10 m/s and zero acceleration.
  auto ego = make_ego_frame(100.0, 50.0, 10.0);
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> steps(kTimesteps);
  for (int64_t k = 1; k <= kTimesteps; ++k) {
    auto & step = steps[k - 1];
    step.time_from_start.sec = static_cast<int32_t>(k / 10);
    step.time_from_start.nanosec = static_cast<uint32_t>((k % 10) * 100000000);
    step.pose.position.x = 100.0 + static_cast<double>(k);
    step.pose.position.y = 50.0;
    step.longitudinal_velocity_mps = 10.0f;
  }

  // cloud_stamp: the sensor pose is the ego pose.
  ego.sensor_stamp = ego.stamp;
  ego.sensor_pose = ego.odometry.pose.pose;
  auto result = postprocessor.process(outputs, ego, ego.stamp, uuid);
  expect_leads_with_ego_pose(result.trajectory, ego, steps);
  EXPECT_EQ(stamp_ns(result.trajectory.header.stamp), ego.stamp.nanoseconds());

  // planning_time: 130 ms newer than the cloud, the ego (and the model reference) has moved
  // and turned since it; the yaw here is 30 degrees, and the model steps are in that frame.
  ego = make_ego_frame(100.0, 50.0, 10.0);
  ego.stamp = rclcpp::Time(130000000LL);
  ego.sensor_stamp = rclcpp::Time(0);
  ego.sensor_pose.position.x = 98.7;
  ego.sensor_pose.orientation.w = 1.0;
  const double yaw = 0.5235987755982988;
  ego.odometry.pose.pose.orientation.z = std::sin(0.5 * yaw);
  ego.odometry.pose.pose.orientation.w = std::cos(0.5 * yaw);
  ego.reference_odometry = ego.odometry;
  ego.ego_to_map.block<2, 2>(0, 0) << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  ego.map_to_ego = ego.ego_to_map.inverse();
  result = postprocessor.process(outputs, ego, ego.stamp, uuid);
  ASSERT_EQ(result.trajectory.points.size(), static_cast<size_t>(kTimesteps + 1));
  EXPECT_EQ(stamp_ns(result.trajectory.header.stamp), ego.stamp.nanoseconds());
  const auto & start = result.trajectory.points.front();
  EXPECT_EQ(start.time_from_start.sec, 0);
  EXPECT_EQ(start.time_from_start.nanosec, 0U);
  EXPECT_DOUBLE_EQ(start.pose.position.x, 100.0);  // not the sensor pose's 98.7
  EXPECT_DOUBLE_EQ(start.pose.orientation.z, std::sin(0.5 * yaw));
  EXPECT_DOUBLE_EQ(start.pose.orientation.w, std::cos(0.5 * yaw));
  // The first model step is 1 m ahead along the rotated heading.
  EXPECT_NEAR(result.trajectory.points[1].pose.position.x, 100.0 + std::cos(yaw), 1e-9);
  EXPECT_NEAR(result.trajectory.points[1].pose.position.y, 50.0 + std::sin(yaw), 1e-9);
  EXPECT_EQ(result.trajectory.points[1].time_from_start.nanosec, 100000000U);
}

TEST(TrajectoryPostprocessorTest, StoppedEgoStillGetsTheStartPointAndKeepsTheStopLogic)
{
  TrajectoryPostprocessor postprocessor(make_params());
  postprocessor.validate_output_specs(make_output_specs({1, 1, kTimesteps, 4}));
  // Standing still: every model step is at the ego, so t = 0 and step 1 coincide.
  const auto outputs = make_straight_prediction(1, 0.0);
  const auto ego = make_ego_frame(3.0, 4.0, 0.0);
  unique_identifier_msgs::msg::UUID uuid;
  const auto result = postprocessor.process(outputs, ego, rclcpp::Time(0), uuid);

  ASSERT_EQ(result.trajectory.points.size(), static_cast<size_t>(kTimesteps + 1));
  EXPECT_EQ(result.trajectory.points.front().time_from_start.nanosec, 0U);
  for (const auto & point : result.trajectory.points) {
    EXPECT_DOUBLE_EQ(point.pose.position.x, 3.0);
    EXPECT_DOUBLE_EQ(point.pose.position.y, 4.0);
    EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps, 0.0f);
  }
}

TEST(TrajectoryPostprocessorTest, ThrowsOnMissingOrShortOutput)
{
  TrajectoryPostprocessor postprocessor(make_params());
  postprocessor.validate_output_specs(make_output_specs({1, 1, kTimesteps, 4}));
  const auto ego = make_ego_frame(0.0, 0.0, 0.0);
  unique_identifier_msgs::msg::UUID uuid;

  TensorMap empty_outputs;
  EXPECT_THROW(
    postprocessor.process(empty_outputs, ego, rclcpp::Time(0), uuid),
    std::runtime_error);

  TensorMap short_outputs;
  short_outputs.emplace(
    "prediction", Tensor::from_host({1, 1, kTimesteps, 4}, std::vector<float>(10, 0.0f)));
  EXPECT_THROW(
    postprocessor.process(short_outputs, ego, rclcpp::Time(0), uuid),
    std::runtime_error);
}

}  // namespace autoware::tensorrt_e2e
