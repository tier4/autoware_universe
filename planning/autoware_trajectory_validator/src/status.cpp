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

#include "autoware/trajectory_validator/status.hpp"

#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator
{
namespace
{
/**
 * @brief Convert hashmap of validation statuses for each category to a vector of category
 statuses.
 *
 * @param validations Hashmap of validation statuses for each category.
 * @return A vector of category statuses and boolean that indicates whether category status is
 OK.
 */
std::pair<std::vector<TrajectoryCategoryStatus>, bool> to_category_statuses(
  const std::unordered_map<std::string, std::vector<TrajectoryValidationStatus>> & validations)
{
  std::vector<TrajectoryCategoryStatus> categories;
  bool is_category_ok = true;
  for (const auto & [category, statuses] : validations) {
    const bool is_validation_ok = check_validation_statues(statuses);

    categories.push_back(
      autoware_internal_planning_msgs::build<TrajectoryCategoryStatus>()
        .name(category)
        .level(is_validation_ok ? TrajectoryCategoryStatus::OK : TrajectoryCategoryStatus::ERROR)
        .validations(statuses));

    // Once an error occurred category level will be ERROR
    is_category_ok = is_category_ok && is_validation_ok;
  }
  return {categories, is_category_ok};
}
}  // namespace

TrajectoryStatusArray to_trajectory_status_array(
  const builtin_interfaces::msg::Time & stamp, const std::vector<TrajectoryStatus> & statuses)
{
  auto header = std_msgs::build<std_msgs::msg::Header>().stamp(stamp).frame_id("");

  return autoware_internal_planning_msgs::build<TrajectoryStatusArray>()
    .header(std::move(header))
    .trajectories(statuses);
}

TrajectoryStatus to_trajectory_status(
  const CandidateTrajectory & trajectory,
  const std::unordered_map<std::string, std::vector<TrajectoryValidationStatus>> & validations)
{
  auto [categories, is_ok] = to_category_statuses(validations);

  return autoware_internal_planning_msgs::build<TrajectoryStatus>()
    .trajectory_id(trajectory.generator_id)
    .generator_id(trajectory.generator_id)
    .stamp(trajectory.header.stamp)
    .level(is_ok ? TrajectoryStatus::OK : TrajectoryStatus::ERROR)
    .categories(std::move(categories));
}
}  // namespace autoware::trajectory_validator
