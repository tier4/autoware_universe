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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_TIME_SEQUENCE_RAW_OPTIMIZER_HPP_  // NOLINT
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_TIME_SEQUENCE_RAW_OPTIMIZER_HPP_  // NOLINT

#include "autoware/trajectory_processor/time_sequence_raw/road_border_avoidance.hpp"
#include "autoware/trajectory_processor/time_sequence_raw/trajectory_optimizer.hpp"
#include "autoware/trajectory_processor/trajectory_processor_plugin_base.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

#include <memory>
#include <optional>
#include <string>

namespace autoware::trajectory_processor::plugin
{

class TrajectoryTimeSequenceRawOptimizer : public TrajectoryProcessorPluginBase
{
public:
  TrajectoryTimeSequenceRawOptimizer() = default;

  ProcessingResult process(TrajectoryPoints & traj_points, TrajectoryProcessorData & data) override;
  void update_params(const TrajectoryProcessorParams & params) override;
  void publish_debug_data(const std::string & ns) const override;

protected:
  void on_initialize(const TrajectoryProcessorParams & params) override;

private:
  void set_params(const TrajectoryProcessorParams & params);
  void ensure_optimizer();
  void maybe_update_map(const TrajectoryProcessorData & data);
  void ensure_debug_publishers();

  enum class SteerStopMode { Track, Hold, Zero };

  SteerStopMode resolve_steer_stop_mode(
    const autoware_planning_msgs::msg::Trajectory & reference,
    const TrajectoryProcessorData & data);
  void apply_stopped_reference(
    TrajectoryPoints & traj_points, const autoware_planning_msgs::msg::Trajectory & reference,
    const double steer_rad) const;

  time_sequence_raw::TrajectoryOptimizationParams opt_params_;
  time_sequence_raw::RoadBorderAvoidanceParams border_params_;
  bool road_border_enable_{false};
  bool publish_debug_topics_{true};
  double stopped_velocity_threshold_mps_{0.15};
  double stopped_trajectory_max_length_m_{1.5};
  bool goal_steer_zero_enable_{true};
  double goal_steer_zero_distance_m_{5.0};
  bool goal_steer_zero_requires_stopped_{true};
  bool in_stopped_regime_{false};
  bool in_goal_zero_regime_{false};
  double latched_steering_rad_{0.0};

  std::unique_ptr<time_sequence_raw::TrajectoryOptimizer> optimizer_;
  std::unique_ptr<time_sequence_raw::RoadBorderAvoidance> road_border_avoidance_;
  const lanelet::LaneletMap * cached_map_{nullptr};

  mutable autoware_planning_msgs::msg::Trajectory last_raw_trajectory_;
  mutable autoware_planning_msgs::msg::Trajectory last_adjusted_trajectory_;
  mutable int last_shifted_point_count_{0};
  mutable int last_solver_status_{0};
  mutable double last_solve_time_ms_{0.0};

  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr debug_raw_pub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr debug_adjusted_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr debug_shifted_count_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr debug_solver_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_solve_time_pub_;
};

}  // namespace autoware::trajectory_processor::plugin
// clang-format off
#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_TIME_SEQUENCE_RAW_OPTIMIZER_HPP_  // NOLINT
// clang-format on
