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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_optimizer_plugin_base.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"
#include "utils.hpp"

#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp>
#include <autoware/trajectory_modifier/trajectory_modifier_structs.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_minimum_rule_based_planner/minimum_rule_based_planner_parameters.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using Params = ::minimum_rule_based_planner::Params;
using TrajectoryClass = autoware::experimental::trajectory::Trajectory<PathPointWithLaneId>;
using autoware::vehicle_info_utils::VehicleInfo;
using unique_identifier_msgs::msg::UUID;

using PluginInterface = autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase;
using PluginLoader = pluginlib::ClassLoader<PluginInterface>;

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

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<::minimum_rule_based_planner::ParamListener> param_listener_;
  const UUID generator_uuid_;
  const VehicleInfo vehicle_info_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  /** @} */

private:
  /**
   ***********************************************************
   * @defgroup route, map, and path planning
   * set_planner_data() initializes route_context_ from map/route messages.
   * plan_path() / generate_path() produce a PathWithLaneId from the route.
   * @{
   */
  void set_planner_data(const InputData & input_data);
  void set_route(const LaneletRoute::ConstSharedPtr & route_ptr);
  bool update_current_lanelet(const geometry_msgs::msg::Pose & current_pose, const Params & params);
  std::optional<PathWithLaneId> plan_path(const InputData & input_data);
  std::optional<PathWithLaneId> generate_path(
    const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
    const Params & params);
  //! route, map, and routing graph context resolved from the current route
  RouteContext route_context_;

  //! lanelet ego is currently driving on (updated each cycle)
  std::optional<lanelet::ConstLanelet> current_lanelet_;

  //! stores lane transition info for continuity across planning cycles
  std::optional<LaneTransitionInfo> active_transition_;
  /** @} */

private:
  /**
   ***********************************************************
   * @defgroup optimizer-plugins trajectory optimizer plugins
   * @{
   */
  void load_optimizer_plugins();

  std::unique_ptr<PluginLoader> plugin_loader_;
  std::shared_ptr<PluginInterface> path_smoother_;
  std::shared_ptr<PluginInterface> velocity_optimizer_;
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

  std::vector<std::shared_ptr<trajectory_modifier::plugin::TrajectoryModifierPluginBase>>
    trajectory_modifiers_;
  trajectory_modifier::TrajectoryModifierParams modifier_params_;
  std::map<std::string, rclcpp::Publisher<Trajectory>::SharedPtr>
    pub_debug_modifier_module_trajectories_;
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

  //! test input: bypasses path planning when provided
  autoware_utils::InterProcessPollingSubscriber<
    PathWithLaneId, autoware_utils::polling_policy::Newest>
    test_path_with_lane_id_subscriber_{this, "~/input/test/path_with_lane_id"};
  PathWithLaneId::ConstSharedPtr test_path_with_lane_id_ptr;

  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_trajectories_;
  rclcpp::Publisher<PathWithLaneId>::SharedPtr pub_debug_path_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_debug_trajectory_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_debug_shifted_trajectory_;
  /** @} */
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // MINIMUM_RULE_BASED_PLANNER_HPP_
