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

#ifndef AUTOWARE__ML_PLANNER__ML_PLANNER_CORE_HPP_
#define AUTOWARE__ML_PLANNER__ML_PLANNER_CORE_HPP_

#include "autoware/ml_planner/dimensions.hpp"
#include "autoware/ml_planner/inference/inference.hpp"
#include "autoware/ml_planner/optimization/optimizer_params.hpp"
#ifdef AUTOWARE_ML_PLANNER_USE_ACADOS
#include "autoware/ml_planner/optimization/trajectory_optimizer.hpp"
#endif
#include "autoware/ml_planner/postprocessing/postprocessing_utils.hpp"
#include "autoware/ml_planner/postprocessing/road_border_avoidance.hpp"
#include "autoware/ml_planner/preprocessing/input_builder.hpp"
#include "autoware/ml_planner/preprocessing/items/map.hpp"
#include "autoware/ml_planner/preprocessing/items/traffic_signals.hpp"
#include "autoware/ml_planner/utils/timed_buffer.hpp"

#include <Eigen/Dense>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::ml_planner
{

using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using nav_msgs::msg::Odometry;
using unique_identifier_msgs::msg::UUID;
using TensorMap = preprocess::TensorMap;

struct TrajectoryOptimizationDebug
{
  bool attempted{false};
  bool optimized{false};
  int solver_status{0};
  double solve_time_ms{0.0};
};

struct RoadBorderAvoidanceDebug
{
  bool active{false};
  int shifted_points{0};
  int unresolved_points{0};
};

struct PlannerOutput
{
  Trajectory trajectory;
  CandidateTrajectories candidate_trajectories;
  PredictedObjects predicted_objects;
  TurnIndicatorsCommand turn_indicators_command;
  std::optional<Trajectory> raw_trajectory;
  TrajectoryOptimizationDebug optimization_debug;
  std::optional<Trajectory> avoidance_adjusted_trajectory;
  RoadBorderAvoidanceDebug avoidance_debug;
  std::optional<Trajectory> pre_stop_fixing_trajectory;
};

struct MLPlannerParams
{
  std::string model_path;
  std::string plugins_path;
  std::string backend;
  std::string trt_precision;
  bool use_cuda_graph;
  bool build_only;
  double planning_frequency_hz;
  double traffic_light_group_msg_timeout_seconds;
  int batch_size;
  std::vector<double> noise_scale_list;
  double line_string_max_step_m;
  optimization::TrajectoryOptimizationParams trajectory_optimization;
  postprocess::RoadBorderAvoidanceParams road_border_avoidance;
  postprocess::StopPointFixingParams stop_point_fixing;
};

/**
 * @class MLPlannerCore
 * @brief Core logic class for the diffusion-based trajectory planner.
 *
 * This class contains all the business logic for trajectory planning,
 * independent of ROS infrastructure. It handles:
 * - Frame context creation from sensor and environment data
 * - Input data preparation for inference
 * - Model inference execution
 * - Data normalization
 *
 * By separating this from the ROS node, we enable:
 * - Direct testing with rosbag data without ROS runtime
 * - Deterministic and reproducible tests
 * - Better unit testing capabilities
 */
class MLPlannerCore
{
public:
  explicit MLPlannerCore(const MLPlannerParams & params, const VehicleInfo & vehicle_info);

  /**
   * @brief Load TensorRT model and normalization statistics.
   *
   * @throws std::runtime_error if the model path is invalid or engine setup fails.
   */
  void load_model();

  /**
   * @brief Update parameters without losing internal state.
   *
   * @param params New parameters to apply
   */
  void update_params(const MLPlannerParams & params);

  /**
   * @brief Reference time of the current frame (stamp of the newest ego odometry).
   *
   * Only valid after create_input_data() succeeds.
   */
  rclcpp::Time frame_time() const;

  using BufferUpdateResult = tl::expected<preprocess::FrameInputs, std::string>;

  /**
   * @brief Append newly received messages and return a snapshot of the current buffers.
   */
  BufferUpdateResult update_buffer(
    const std::vector<std::shared_ptr<const Odometry>> & ego_kinematic_states,
    const std::vector<std::shared_ptr<const TrackedObjects>> & objects,
    const std::vector<
      std::shared_ptr<const autoware_perception_msgs::msg::TrafficLightGroupArray>> &
      traffic_signals,
    const std::vector<std::shared_ptr<const TurnIndicatorsReport>> & turn_indicators,
    const LaneletRoute::ConstSharedPtr & route_ptr);

  /**
   * @brief Build normalized-shape model inputs from a validated buffer snapshot.
   */
  preprocess::TensorMapResult create_input_data(const preprocess::FrameInputs & frame_inputs);

  const geometry_msgs::msg::Pose & ego_pose() const { return ego_history_.back().pose.pose; }

  /**
   * @brief Set the lanelet map context.
   *
   * @param lanelet_map_ptr Shared pointer to lanelet map
   */
  void set_map(const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr);

  /**
   * @brief Check if the model is loaded.
   *
   * @return true if model is loaded, false otherwise
   */
  bool is_model_loaded() const { return ml_planner_inference_ != nullptr; }

  /**
   * @brief Check if the map is loaded.
   *
   * @return true if map is loaded, false otherwise
   */
  bool is_map_loaded() const { return lane_segment_context_ != nullptr; }

  /**
   * @brief Run inference on the input data.
   *
   * @param input_data_map Input data for inference
   * @return Inference result containing normalized trajectories
   */
  InferenceResult run_inference(const TensorMap & input_data_map);

  /**
   * @brief Create all planner output messages from raw inference outputs.
   *
   * Parses raw predictions, creates ego trajectory (batch 0), candidate trajectories
   * for all batches, predicted objects for neighbor agents, and turn indicator command.
   *
   * @param inference_output Successful inference output.
   * @param timestamp The ROS time stamp for the messages.
   * @param generator_uuid The unique identifier for the planner instance.
   * @param current_steering_angle_rad Measured steering angle used by trajectory optimization.
   * @return PlannerOutput containing all output messages.
   */
  PlannerOutput create_planner_output(
    const InferenceOutput & inference_output, const rclcpp::Time & timestamp,
    const UUID & generator_uuid, double current_steering_angle_rad);

  /**
   * @brief Get the first traffic light on the route for debugging.
   *
   * @return Traffic light group message
   */
  autoware_perception_msgs::msg::TrafficLightGroup get_first_traffic_light_on_route() const;

  /**
   * @brief Count valid elements in input data for diagnostics.
   *
   * @param input_data_map Input data map
   * @param data_key Key to count (e.g., "lanes", "route_lanes", "intersection_area")
   * @return Count of valid elements
   */
  int64_t count_valid_elements(
    const TensorMap & input_data_map, const std::string & data_key) const;

  /**
   * @brief Get current route pointer.
   *
   * @return Shared pointer to current route
   */
  const LaneletRoute::ConstSharedPtr & get_route() const { return route_ptr_; }

private:
  // Parameters
  MLPlannerParams params_;
  VehicleSpec vehicle_spec_;

  // Inference engine
  std::unique_ptr<Inference> ml_planner_inference_{nullptr};

#ifdef AUTOWARE_ML_PLANNER_USE_ACADOS
  // acados-based trajectory optimization (nullptr when disabled by parameter)
  std::unique_ptr<optimization::TrajectoryOptimizer> trajectory_optimizer_{nullptr};
#endif

  // Road border avoidance shift applied to the raw model output (nullptr when disabled)
  std::unique_ptr<postprocess::RoadBorderAvoidance> road_border_avoidance_{nullptr};

  // Postprocessing
  std::vector<preprocess::SelectedAgent> selected_agents_;

  // Raw message history buffers, bounded to the model input time window.
  // All derived history data is computed statelessly from these buffers.
  utils::TimedBuffer<Odometry> ego_history_{
    HISTORY_WINDOW_S, [](const Odometry & msg) { return rclcpp::Time(msg.header.stamp); }};
  utils::TimedBuffer<TurnIndicatorsReport> turn_indicators_history_{
    HISTORY_WINDOW_S, [](const TurnIndicatorsReport & msg) { return rclcpp::Time(msg.stamp); }};
  utils::TimedBuffer<TrackedObjects> objects_history_{
    HISTORY_WINDOW_S, [](const TrackedObjects & msg) { return rclcpp::Time(msg.header.stamp); }};
  utils::TimedBuffer<autoware_perception_msgs::msg::TrafficLightGroupArray>
    traffic_signals_history_{
      HISTORY_WINDOW_S, [](const autoware_perception_msgs::msg::TrafficLightGroupArray & msg) {
        return rclcpp::Time(msg.stamp);
      }};

  // Lanelet map
  LaneletRoute::ConstSharedPtr route_ptr_;
  std::unique_ptr<preprocess::LaneSegmentContext> lane_segment_context_;
};

}  // namespace autoware::ml_planner

#endif  // AUTOWARE__ML_PLANNER__ML_PLANNER_CORE_HPP_
