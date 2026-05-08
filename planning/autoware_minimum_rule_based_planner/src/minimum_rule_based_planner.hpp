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

#ifndef MINIMUM_RULE_BASED_PLANNER_HPP_
#define MINIMUM_RULE_BASED_PLANNER_HPP_

#include "path_optimizer.hpp"
#include "path_planner.hpp"
#include "velocity_smoother.hpp"

#include <autoware/trajectory_modifier/trajectory_modifier_structs.hpp>
#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
using TrajectoryModifierData = trajectory_modifier::TrajectoryModifierData;

class MinimumRuleBasedPlannerNode : public rclcpp::Node
{
public:
  explicit MinimumRuleBasedPlannerNode(const rclcpp::NodeOptions & options);

  /**
   * @brief aggregated input data consumed each planning cycle
   */
  struct InputData
  {
    LaneletRoute::ConstSharedPtr route_ptr;
    LaneletMapBin::ConstSharedPtr lanelet_map_bin_ptr;
    Odometry::ConstSharedPtr odometry_ptr;
    AccelWithCovarianceStamped::ConstSharedPtr acceleration_ptr;
    PredictedObjects::ConstSharedPtr predicted_objects_ptr;
    PointCloud2::ConstSharedPtr obstacle_pointcloud_ptr;
    PathWithLaneId::ConstSharedPtr test_path_with_lane_id_ptr;
  };

private:
  /**
   ***********************************************************
   * @defgroup core pipeline
   * on_timer() is the main entry point, called at planning_frequency_hz.
   * @{
   */
  void on_timer();
  InputData take_data();
  bool is_data_ready(const InputData & input_data);
  void update_params();

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<::minimum_rule_based_planner::ParamListener> param_listener_;
  const UUID generator_uuid_;
  const VehicleInfo vehicle_info_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  minimum_rule_based_planner::Params params_;
  /** @} */

private:
  /**
   ***********************************************************
   * @defgroup path planning
   * PathPlanner handles route/map initialisation, path planning,
   * trajectory shifting, and conversion.
   * @{
   */
  //! PathPlanner encapsulates path planning, trajectory shifting, and conversion
  std::unique_ptr<PathPlanner> path_planner_;
  /** @} */

private:
  /**
   ***********************************************************
   * @defgroup optimizer-plugins trajectory optimizer plugins
   * @{
   */
  void load_optimizer_plugins();

  std::unique_ptr<PathOptimizer> path_smoother_;
  std::unique_ptr<VelocitySmoother> velocity_smoother_;
  std::map<std::string, rclcpp::Publisher<Trajectory>::SharedPtr>
    pub_debug_optimizer_module_trajectories_;
  /** @} */

private:
  /**
   ***********************************************************
   * @defgroup modifier-plugins trajectory modifier plugins
   * @{
   */
  void load_modifier_plugins();

  void load_plugin(const std::string & name);
  void unload_plugin(const std::string & name);

  void set_modifier_data(const MinimumRuleBasedPlannerNode::InputData & input_data);

  bool initialized_modifiers_{false};
  ModifierPluginLoader modifier_plugin_loader_;
  std::vector<std::shared_ptr<plugin::PluginInterface>> modifier_plugins_;
  std::map<std::string, rclcpp::Publisher<Trajectory>::SharedPtr>
    pub_debug_modifier_module_trajectories_;

  std::shared_ptr<plugin::ModifierData> modifier_data_;
  /** @} */

private:
  /**
   ***********************************************************
   * @defgroup subscribers and publishers
   * @{
   */
  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    route_subscriber_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  LaneletRoute::ConstSharedPtr route_ptr_;

  autoware_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils::polling_policy::Newest>
    vector_map_subscriber_{this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};
  LaneletMapBin::ConstSharedPtr lanelet_map_bin_ptr_;

  autoware_utils::InterProcessPollingSubscriber<Odometry> odometry_subscriber_{
    this, "~/input/odometry"};
  Odometry::ConstSharedPtr odometry_ptr_;

  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    acceleration_subscriber_{this, "~/input/acceleration"};
  AccelWithCovarianceStamped::ConstSharedPtr acceleration_ptr_;

  autoware_utils::InterProcessPollingSubscriber<PredictedObjects> objects_subscriber_{
    this, "~/input/objects"};
  PredictedObjects::ConstSharedPtr predicted_objects_ptr_;

  autoware_utils_rclcpp::InterProcessPollingSubscriber<PointCloud2> pointcloud_subscriber_{
    this, "~/input/pointcloud", autoware_utils::single_depth_sensor_qos()};
  PointCloud2::ConstSharedPtr obstacle_pointcloud_ptr_;

  //! test input: bypasses path planning when provided
  autoware_utils::InterProcessPollingSubscriber<
    PathWithLaneId, autoware_utils::polling_policy::Newest>
    test_path_with_lane_id_subscriber_{this, "~/input/test/path_with_lane_id"};
  PathWithLaneId::ConstSharedPtr test_path_with_lane_id_ptr;

  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_trajectories_;
  rclcpp::Publisher<PathWithLaneId>::SharedPtr pub_debug_path_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_debug_trajectory_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_debug_shifted_trajectory_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_lane_boundaries_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pub_debug_uncrossable_boundaries_;

  // Publishes route lanelets' left/right bounds as LINE_STRIP markers for
  // visual confirmation in rviz. Marker namespace: "lane_boundary_left" /
  // "lane_boundary_right". Skips silently if route_context has no lanelets.
  void publish_lane_boundaries_marker() const;

  // Publishes linestrings whose type-attribute matches
  // params_.debug.uncrossable_boundary_types (e.g. "road_border", "curbstone")
  // as LINE_STRIP markers, restricted to linestrings whose AABB intersects
  // the AABB of `ref_trajectory` expanded by
  // params_.debug.uncrossable_boundary_near_path_radius_m. This guarantees
  // all uncrossable boundaries within that radius of any path point are
  // included (and possibly some slightly beyond — the conservative
  // expanded-bbox filter is intentional). The matching + bbox precomputation
  // is cached and rebuilt only when the map pointer or type list changes;
  // per-cycle work is the cheap AABB filter.
  void publish_uncrossable_boundaries_marker(const Trajectory & ref_trajectory);

  struct CachedUncrossableLine
  {
    lanelet::ConstLineString3d line_string;
    double min_x{0.0};
    double max_x{0.0};
    double min_y{0.0};
    double max_y{0.0};
  };
  void update_uncrossable_cache_if_needed();

  // Extract uncrossable_boundary 2D segments for the optimizer (Layer 1+2 of
  // the active-set pipeline: cache-hit + AABB filter against the trajectory
  // bounding box expanded by `radius_m`). Each polyline that survives the
  // AABB test is broken into its constituent segments. Caller owns the
  // resulting vector; PathOptimizer runs Layers 3-6 on it.
  std::vector<UncrossableSegment> extract_uncrossable_segments_for_optimization(
    const Trajectory & ref_trajectory, double radius_m);

  LaneletMapBin::ConstSharedPtr cached_uncrossable_map_bin_;
  std::vector<std::string> cached_uncrossable_types_;
  std::vector<CachedUncrossableLine> cached_uncrossable_lines_;
  /** @} */
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // MINIMUM_RULE_BASED_PLANNER_HPP_
