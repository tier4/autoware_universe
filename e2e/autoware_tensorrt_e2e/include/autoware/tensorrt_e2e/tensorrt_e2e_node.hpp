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

#ifndef AUTOWARE__TENSORRT_E2E__TENSORRT_E2E_NODE_HPP_
#define AUTOWARE__TENSORRT_E2E__TENSORRT_E2E_NODE_HPP_

#include "autoware/tensorrt_e2e/inference_engine.hpp"
#include "autoware/tensorrt_e2e/input_provider.hpp"
#include "autoware/tensorrt_e2e/postprocess/trajectory_postprocessor.hpp"
#include "autoware/tensorrt_e2e/providers/context_input_provider.hpp"

#include <autoware/diffusion_planner/utils/arg_reader.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;
using autoware_utils_diagnostics::DiagnosticsInterface;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

struct TensorrtE2eParams
{
  std::string model_path;
  std::string plugins_path;
  std::string precision;
  int64_t trt_workspace_mib{4096};
  std::string args_path;  //!< Optional normalization JSON ("" to disable normalization).
  bool build_only{false};
  double planning_frequency_hz{10.0};
  bool shift_x{false};
  std::vector<std::string> sensor_inputs;  //!< Enabled sensor providers: "camera", "lidar".
  bool enable_context_inputs{true};
};

/**
 * @class TensorrtE2eNode
 * @brief Abstract E2E trajectory planner node.
 *
 * Runs any single-engine E2E model whose inputs can be produced by the configured input
 * providers (camera, lidar, diffusion-planner-style context) and whose primary output is an
 * ego trajectory tensor. See docs/design.md for the architecture.
 */
class TensorrtE2eNode : public rclcpp::Node
{
public:
  explicit TensorrtE2eNode(const rclcpp::NodeOptions & options);

private:
  void set_up_params();

  /**
   * @brief Load the engine, create providers, match claims against the engine manifest, and
   * validate the output contract.
   * @throws std::runtime_error on any model/deployment mismatch.
   */
  void initialize_pipeline();
  void create_providers();

  void on_timer();

  /**
   * @brief Build the per-tick ego frame from the latest odometry/acceleration.
   * @return std::nullopt when no odometry has been received yet.
   */
  std::optional<EgoFrame> create_ego_frame();

  /// Apply args-JSON normalization to host tensors that have stats (no-op without args_path).
  void apply_normalization(TensorMap & inputs) const;

  /// @return The name of the first host tensor containing NaN/Inf, or std::nullopt.
  static std::optional<std::string> find_invalid_tensor(const TensorMap & inputs);

  // Parameters
  TensorrtE2eParams params_;
  PostprocessParams postprocess_params_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  double base_link_to_center_{0.0};

  // Pipeline
  std::unique_ptr<InferenceEngine> engine_;
  std::vector<std::unique_ptr<InputProviderInterface>> providers_;
  ContextInputProvider * context_provider_{nullptr};  //!< Borrowed from providers_.
  std::unique_ptr<TrajectoryPostprocessor> postprocessor_;
  autoware::diffusion_planner::utils::NormalizationMap normalization_map_;
  bool pipeline_ready_{false};

  // ROS interfaces
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_trajectories_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_;
  autoware_utils::InterProcessPollingSubscriber<Odometry> sub_odometry_{this, "~/input/odometry"};
  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped> sub_acceleration_{
    this, "~/input/acceleration"};
  std::unique_ptr<DiagnosticsInterface> diagnostics_;
  unique_identifier_msgs::msg::UUID generator_uuid_;
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch_;
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__TENSORRT_E2E_NODE_HPP_
