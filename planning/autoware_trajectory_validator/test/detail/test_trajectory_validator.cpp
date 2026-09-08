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

#include "autoware/trajectory_validator/detail/trajectory_validator.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::trajectory_validator
{
namespace
{

// Always reports a trajectory as feasible, regardless of its content or context.
class FakeAlwaysFeasiblePlugin : public plugin::ValidatorInterface
{
public:
  explicit FakeAlwaysFeasiblePlugin(const std::string & name) : ValidatorInterface(name) {}

  result_t is_feasible(
    const autoware_internal_planning_msgs::msg::CandidateTrajectory & /*candidate_trajectory*/,
    const FilterContext & /*context*/) override
  {
    return plugin::ValidationResult{};
  }

  void update_parameters(const validator::Params & /*params*/) override {}
};

unique_identifier_msgs::msg::UUID make_uuid(uint8_t seed)
{
  unique_identifier_msgs::msg::UUID uuid;
  uuid.uuid.fill(seed);
  return uuid;
}

autoware_internal_planning_msgs::msg::CandidateTrajectory make_candidate_trajectory(
  const unique_identifier_msgs::msg::UUID & generator_id)
{
  autoware_internal_planning_msgs::msg::CandidateTrajectory candidate_trajectory;
  candidate_trajectory.generator_id = generator_id;
  return candidate_trajectory;
}

GeneratorInfo make_generator_info(
  const unique_identifier_msgs::msg::UUID & generator_id, const std::string & name)
{
  GeneratorInfo info;
  info.generator_id = generator_id;
  info.generator_name.data = name;
  return info;
}

TrajectoryValidator make_validator()
{
  return TrajectoryValidator({std::make_shared<FakeAlwaysFeasiblePlugin>("fake_plugin")});
}

TEST(TrajectoryValidatorTest, DedupesGeneratorInfoForTrajectoriesSharingAGenerator)
{
  const auto validator = make_validator();

  const auto generator_id = make_uuid(1);
  CandidateTrajectories input_trajectories;
  input_trajectories.generator_info.push_back(make_generator_info(generator_id, "generator_a"));
  input_trajectories.candidate_trajectories.push_back(make_candidate_trajectory(generator_id));
  input_trajectories.candidate_trajectories.push_back(make_candidate_trajectory(generator_id));

  const auto report =
    validator.process(input_trajectories, /*active_filter_names=*/{}, ValidatorContext{});

  ASSERT_EQ(report.valid_trajectories.candidate_trajectories.size(), 2U);
  ASSERT_EQ(report.valid_trajectories.generator_info.size(), 1U);
  EXPECT_EQ(report.valid_trajectories.generator_info.front().generator_name.data, "generator_a");
}

TEST(TrajectoryValidatorTest, KeepsOneGeneratorInfoEntryPerDistinctGenerator)
{
  const auto validator = make_validator();

  const auto generator_id_a = make_uuid(1);
  const auto generator_id_b = make_uuid(2);
  CandidateTrajectories input_trajectories;
  input_trajectories.generator_info.push_back(make_generator_info(generator_id_a, "generator_a"));
  input_trajectories.generator_info.push_back(make_generator_info(generator_id_b, "generator_b"));
  input_trajectories.candidate_trajectories.push_back(make_candidate_trajectory(generator_id_a));
  input_trajectories.candidate_trajectories.push_back(make_candidate_trajectory(generator_id_b));

  const auto report =
    validator.process(input_trajectories, /*active_filter_names=*/{}, ValidatorContext{});

  ASSERT_EQ(report.valid_trajectories.generator_info.size(), 2U);
}

}  // namespace
}  // namespace autoware::trajectory_validator
