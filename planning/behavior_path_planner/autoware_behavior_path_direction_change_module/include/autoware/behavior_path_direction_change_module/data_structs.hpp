// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__DATA_STRUCTS_HPP_

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

struct DirectionChangeParameters
{
  // Cusp detection parameters
  double cusp_detection_distance_threshold;
  double cusp_detection_angle_threshold_deg;
  double cusp_approach_speed;
  double cusp_stop_distance;

  // Direction change parameters
  double reverse_speed_limit;
  double reverse_lookahead_distance;
  double reverse_safety_margin;

  // Path generation parameters
  double path_resolution;
  double backward_path_length;
  double forward_path_length;

  // General parameters
  bool enable_cusp_detection;
  bool enable_reverse_following;
  bool publish_debug_marker;
};

/**
 * @brief Path evaluation debug data (following start_planner_module pattern)
 *
 * Records the evaluation process and results for path judgment.
 * This structure enables tracing why a path was accepted or rejected.
 */
struct PathEvaluationDebugData
{
  // Evaluation step results (following start_planner's conditions_evaluation pattern)
  std::vector<std::string> conditions_evaluation;

  // Path evaluation metrics
  size_t num_cusp_points{0};
  size_t first_cusp_index{0};
  bool has_direction_change_area_tag{false};
  bool lane_continuity_check_passed{false};

  // Helper to convert double to string with precision
  static std::string double_to_str(double value, int precision = 1)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
  }

  /**
   * @brief Generate a formatted evaluation table string for debugging
   * @return Formatted string showing all evaluation steps and results
   */
  std::string to_evaluation_table() const
  {
    std::ostringstream oss;
    oss << "=== Direction Change Path Evaluation ===\n";
    oss << "direction_change_area_tag: " << (has_direction_change_area_tag ? "found" : "not found")
        << "\n";
    oss << "num_cusp_points: " << num_cusp_points << "\n";
    if (num_cusp_points > 0) {
      oss << "first_cusp_index: " << first_cusp_index << "\n";
    }
    oss << "lane_continuity_check: " << (lane_continuity_check_passed ? "passed" : "failed")
        << "\n";
    oss << "--- Evaluation Steps ---\n";
    for (size_t i = 0; i < conditions_evaluation.size(); ++i) {
      oss << "  " << (i + 1) << ". " << conditions_evaluation[i] << "\n";
    }
    return oss.str();
  }

  /**
   * @brief Clear all evaluation data for reuse
   */
  void clear()
  {
    conditions_evaluation.clear();
    num_cusp_points = 0;
    first_cusp_index = 0;
    has_direction_change_area_tag = false;
    lane_continuity_check_passed = false;
  }
};

struct DirectionChangeDebugData
{
  std::vector<geometry_msgs::msg::Point> cusp_points{};
  PathWithLaneId forward_path{};
  PathWithLaneId reverse_path{};

  // Path evaluation debug data (following start_planner pattern)
  PathEvaluationDebugData evaluation_data{};
};

}  // namespace autoware::behavior_path_planner
#endif  // AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__DATA_STRUCTS_HPP_

