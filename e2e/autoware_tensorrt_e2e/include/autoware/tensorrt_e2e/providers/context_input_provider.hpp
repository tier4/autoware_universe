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

#ifndef AUTOWARE__TENSORRT_E2E__PROVIDERS__CONTEXT_INPUT_PROVIDER_HPP_
#define AUTOWARE__TENSORRT_E2E__PROVIDERS__CONTEXT_INPUT_PROVIDER_HPP_

#include "autoware/tensorrt_e2e/input_provider.hpp"

#include <autoware/diffusion_planner/conversion/agent.hpp>
#include <autoware/diffusion_planner/preprocessing/lane_segments.hpp>
#include <autoware/diffusion_planner/preprocessing/traffic_signals.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

/**
 * @class ContextInputProvider
 * @brief Produces the diffusion-planner-style context tensors, each one optional.
 *
 * Claimable tensors (fixed names — they are the contract of the diffusion planner feature
 * pipeline, which this provider reuses):
 * `ego_current_state`, `ego_agent_past`, `neighbor_agents_past`, `static_objects`,
 * `lanes`, `lanes_speed_limit`, `lanes_has_speed_limit`,
 * `route_lanes`, `route_lanes_speed_limit`, `route_lanes_has_speed_limit`,
 * `polygons`, `line_strings`, `goal_pose`, `ego_shape`, `turn_indicators`.
 *
 * Free dimensions (history length, neighbor count, lane/route segment count) are taken from
 * the engine manifest. Subscriptions are created only for the data the claimed tensors need.
 */
class ContextInputProvider : public InputProviderInterface
{
public:
  ContextInputProvider(
    rclcpp::Node & node, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

  std::string name() const override { return "context"; }
  std::vector<std::string> claim_inputs(const std::vector<TensorSpec> & engine_inputs) override;
  bool collect(
    const EgoFrame & ego, const rclcpp::Time & now, TensorMap & inputs,
    std::string & error) override;

  /**
   * @brief Neighbor histories of the latest collect(), ego-centric and distance-ordered.
   *
   * Used by the postprocessor to attach model-predicted paths to tracked objects. Empty when
   * `neighbor_agents_past` is not claimed.
   */
  const std::vector<autoware::diffusion_planner::AgentHistory> & last_neighbor_histories() const
  {
    return last_neighbor_histories_;
  }

private:
  using TrackedObjects = autoware_perception_msgs::msg::TrackedObjects;
  using TrafficLightGroupArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
  using TurnIndicatorsReport = autoware_vehicle_msgs::msg::TurnIndicatorsReport;
  using LaneletRoute = autoware_planning_msgs::msg::LaneletRoute;
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;

  void create_subscriptions();
  void on_map(const LaneletMapBin::ConstSharedPtr map_msg);

  bool collect_ego_tensors(const EgoFrame & ego, TensorMap & inputs, std::string & error);
  bool collect_neighbor_tensors(const EgoFrame & ego, TensorMap & inputs, std::string & error);
  bool collect_map_tensors(const EgoFrame & ego, TensorMap & inputs, std::string & error);
  bool collect_route_tensors(const EgoFrame & ego, TensorMap & inputs, std::string & error);
  bool collect_turn_indicator_tensor(TensorMap & inputs, std::string & error);

  rclcpp::Node & node_;

  // Vehicle geometry (same derivation as the diffusion planner's VehicleSpec)
  double wheel_base_;
  double vehicle_length_;
  double vehicle_width_;

  // Deployment parameters
  double traffic_light_msg_timeout_s_{0.2};
  bool ignore_neighbors_{false};
  bool ignore_unknown_neighbors_{true};
  double line_string_max_step_m_{5.0};
  bool use_time_interpolation_{false};

  // Claims and engine-derived dimensions. A claim is active when its shape is non-empty.
  std::vector<int64_t> ego_current_state_shape_;
  std::vector<int64_t> ego_agent_past_shape_;
  std::vector<int64_t> neighbor_shape_;
  std::vector<int64_t> static_objects_shape_;
  std::vector<int64_t> lanes_shape_;
  std::vector<int64_t> lanes_speed_limit_shape_;
  std::vector<int64_t> lanes_has_speed_limit_shape_;
  std::vector<int64_t> route_lanes_shape_;
  std::vector<int64_t> route_lanes_speed_limit_shape_;
  std::vector<int64_t> route_lanes_has_speed_limit_shape_;
  std::vector<int64_t> polygons_shape_;
  std::vector<int64_t> line_strings_shape_;
  std::vector<int64_t> goal_pose_shape_;
  std::vector<int64_t> ego_shape_shape_;
  std::vector<int64_t> turn_indicators_shape_;

  // Subscriptions (created on demand in claim_inputs)
  std::unique_ptr<autoware_utils::InterProcessPollingSubscriber<TrackedObjects>>
    sub_tracked_objects_;
  std::unique_ptr<autoware_utils::InterProcessPollingSubscriber<
    TrafficLightGroupArray, autoware_utils::polling_policy::All>>
    sub_traffic_signals_;
  std::unique_ptr<autoware_utils::InterProcessPollingSubscriber<TurnIndicatorsReport>>
    sub_turn_indicators_;
  std::unique_ptr<
    autoware_utils::InterProcessPollingSubscriber<LaneletRoute, autoware_utils::polling_policy::Newest>>
    sub_route_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;

  // State (mirrors DiffusionPlannerCore)
  std::deque<nav_msgs::msg::Odometry> ego_history_;
  std::deque<TurnIndicatorsReport> turn_indicators_history_;
  autoware::diffusion_planner::AgentData agent_data_;
  std::vector<autoware::diffusion_planner::AgentHistory> last_neighbor_histories_;
  std::map<lanelet::Id, autoware::diffusion_planner::preprocess::TrafficSignalStamped>
    traffic_light_id_map_;
  std::unique_ptr<autoware::diffusion_planner::preprocess::LaneSegmentContext>
    lane_segment_context_;
  LaneletRoute::ConstSharedPtr route_ptr_;
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__PROVIDERS__CONTEXT_INPUT_PROVIDER_HPP_
