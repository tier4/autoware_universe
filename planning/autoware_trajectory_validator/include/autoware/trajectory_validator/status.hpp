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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__STATUS_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__STATUS_HPP_

#include <builtin_interfaces/msg/time.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/trajectory_category_status.hpp>
#include <autoware_internal_planning_msgs/msg/trajectory_status.hpp>
#include <autoware_internal_planning_msgs/msg/trajectory_status_array.hpp>
#include <autoware_internal_planning_msgs/msg/trajectory_validation_status.hpp>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator
{
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_internal_planning_msgs::msg::TrajectoryCategoryStatus;
using autoware_internal_planning_msgs::msg::TrajectoryStatus;
using autoware_internal_planning_msgs::msg::TrajectoryStatusArray;
using autoware_internal_planning_msgs::msg::TrajectoryValidationStatus;

/**
 * @brief Convert a vector of trajectory statuses to a trajectory status array message.
 * Note that the header frame_id of published message will be empty string.
 *
 * @param stamp Timestamp to publish.
 * @param statuses A vector of trajectory statuses.
 * @return Trajectory status array message.
 */
TrajectoryStatusArray to_trajectory_status_array(
  const builtin_interfaces::msg::Time & stamp, const std::vector<TrajectoryStatus> & statuses);

/**
 * @brief Convert a hashmap of validation statuses to a trajectory status with a
 * corresponding candidate trajectory.
 *
 * @param trajectory Candidate trajectory.
 * @param categories A vector of category statuses.
 * @return Trajectory status message.
 */
TrajectoryStatus to_trajectory_status(
  const CandidateTrajectory & trajectory,
  const std::unordered_map<std::string, std::vector<TrajectoryValidationStatus>> & validations);

/**
 * @brief Check a trajectory validation status.
 *
 * @param validation Validation status.
 * @return true if level is OK.
 */
inline bool check_validation_status(const TrajectoryValidationStatus & validation)
{
  return validation.level == TrajectoryValidationStatus::OK;
}

/**
 * @brief Check trajectory validation statues and return true if all of them are OK.
 *
 * @param validations An array of validation statuses.
 * @return true if all validation statuses are OK, false otherwise.
 */
inline bool check_validation_statues(const std::vector<TrajectoryValidationStatus> & validations)
{
  for (const auto & validation : validations) {
    if (!check_validation_status(validation)) {
      return false;
    }
  }
  return true;
}

inline bool check_category_status(const TrajectoryCategoryStatus & category)
{
  return category.level == TrajectoryCategoryStatus::OK;
}

inline bool check_category_statuses(const std::vector<TrajectoryCategoryStatus> & categories)
{
  for (const auto & category : categories) {
    if (!check_category_status(category)) {
      return false;
    }
  }
  return true;
}
}  // namespace autoware::trajectory_validator
#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__STATUS_HPP_
