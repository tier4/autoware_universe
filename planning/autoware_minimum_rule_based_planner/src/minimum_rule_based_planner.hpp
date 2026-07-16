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

#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"
#include "map_based_stop_planner.hpp"
#include "path_planner.hpp"
#include "velocity_smoother.hpp"

#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
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

  std::optional<PathWithLaneId> plan_path(const InputData & input_data);
  Trajectory shift_trajectory_to_ego(
    const Trajectory & trajectory, const InputData & input_data) const;
  Trajectory smooth_trajectory(const Trajectory & trajectory, const InputData & input_data) const;
  void apply_modifiers(Trajectory & trajectory, const InputData & input_data) const;

  //! Plan the go and stop trajectories from the modified trajectory: refresh the stop line
  //! cache and embed map-defined stop points (mandatory targets only for go, mandatory +
  //! possibility targets for stop). Velocities are not planned here; the returned trajectories
  //! are shape-only inputs for optimize_velocity(). The stop trajectory is returned only when it
  //! embeds a stop point different from the go trajectory's.
  std::pair<Trajectory, std::optional<Trajectory>> plan_go_and_stop_trajectories(
    const Trajectory & trajectory, const InputData & input_data);

  //! result of a stop plan: the stop point arc length and the trajectory with the stop embedded
  struct StopResult
  {
    double stop_point_arc_length;
    Trajectory trajectory;
  };
  //! Plan a stop at map-defined stop lines from the pre-collected stop line candidates.
  //! When @p include_possibility_targets is true, possibility targets (e.g. traffic lights) are
  //! considered in addition to the mandatory ones.
  std::optional<StopResult> plan_stop(
    const Trajectory & trajectory, const std::vector<StopLine> & candidate_stop_lines,
    const InputData & input_data, const bool include_possibility_targets) const;
  //! @param update_smoother_state whether this call may update the velocity smoother's
  //! prev-output state (true only for the go trajectory; see VelocitySmoother::optimize)
  Trajectory optimize_velocity(
    const Trajectory & trajectory, const InputData & input_data,
    const bool update_smoother_state) const;

  void publish_candidate_trajectories(
    const Trajectory & go_trajectory, const std::optional<Trajectory> & stop_trajectory) const;

  void publish_debug_outputs(
    const PathWithLaneId & path, const Trajectory & go_trajectory,
    const std::optional<Trajectory> & stop_trajectory) const;
  void publish_processing_time();

  void publish_debug_trajectory(
    const std::string & plugin_name, const TrajectoryPoints & traj_points) const;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<::minimum_rule_based_planner::ParamListener> param_listener_;
  //! generator id for the always-published "Go" trajectory
  const UUID go_generator_uuid_;
  //! generator id for the optional "Stop" trajectory (map-defined stop lines)
  const UUID stop_generator_uuid_;
  const VehicleInfo vehicle_info_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    debug_processing_time_pub_;
  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
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
  //! MapBasedStopPlanner extracts map-defined stop locations along the route
  std::unique_ptr<MapBasedStopPlanner> map_based_stop_planner_;
  //! Cache of the collected stop lines: collection depends only on the map and the route, so it
  //! is re-run only when either changes (pointer identity).
  std::vector<StopLine> stop_lines_cache_;
  LaneletRoute::ConstSharedPtr stop_lines_cache_route_ptr_;
  LaneletMapBin::ConstSharedPtr stop_lines_cache_map_ptr_;
  /** @} */

private:
  /**
   ***********************************************************
   * @defgroup optimizer-plugins trajectory optimizer plugins
   * @{
   */
  void load_optimizer_plugins();

  std::unique_ptr<OptimizerPluginLoader> plugin_loader_;
  std::shared_ptr<OptimizerPluginInterface> path_smoother_;
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

  bool initialized_modifiers_{false};
  ModifierPluginLoader modifier_plugin_loader_;
  std::vector<std::shared_ptr<plugin::PluginInterface>> modifier_plugins_;
  std::map<std::string, rclcpp::Publisher<Trajectory>::SharedPtr>
    pub_debug_modifier_module_trajectories_;

  std::shared_ptr<plugin::ModifierContext> modifier_context_;
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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_stop_lines_marker_;
  rclcpp::Publisher<PathWithLaneId>::SharedPtr pub_debug_path_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_debug_trajectory_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_debug_stop_trajectory_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_debug_shifted_trajectory_;
  /** @} */
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // MINIMUM_RULE_BASED_PLANNER_HPP_
