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
#include "plugins/obstacle_stop_modifier.hpp"

#include <autoware/trajectory/path_point_with_lane_id.hpp>
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

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

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

// Plugin types
using PluginInterface = autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase;
using PluginLoader = pluginlib::ClassLoader<PluginInterface>;

// Internal planner data (different from rule_based_planner::PlannerData for modules)
struct InternalPlannerData
{
  lanelet::LaneletMapPtr lanelet_map_ptr{nullptr};
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr{nullptr};
  lanelet::routing::RoutingGraphPtr routing_graph_ptr{nullptr};

  std::string route_frame_id{};
  geometry_msgs::msg::Pose goal_pose{};

  lanelet::ConstLanelets route_lanelets{};
  lanelet::ConstLanelets preferred_lanelets{};
  lanelet::ConstLanelets start_lanelets{};
  lanelet::ConstLanelets goal_lanelets{};
};

class MinimumRuleBasedPlannerNode : public rclcpp::Node
{
public:
  explicit MinimumRuleBasedPlannerNode(const rclcpp::NodeOptions & options);

  struct InputData
  {
    LaneletRoute::ConstSharedPtr route_ptr;
    LaneletMapBin::ConstSharedPtr lanelet_map_bin_ptr;
    Odometry::ConstSharedPtr odometry_ptr;
    AccelWithCovarianceStamped::ConstSharedPtr acceleration_ptr;
    PredictedObjects::ConstSharedPtr predicted_objects_ptr;
  };

private:
  void on_timer();

  InputData take_data();
  bool is_data_ready(const InputData & input_data);
  void set_planner_data(const InputData & input_data);
  void set_route(const LaneletRoute::ConstSharedPtr & route_ptr);
  std::optional<PathWithLaneId> plan_path(const InputData & input_data);
  std::optional<PathWithLaneId> generate_path(
    const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
    const Params & params);
  bool update_current_lanelet(const geometry_msgs::msg::Pose & current_pose, const Params & params);

  // Plugin loading
  void load_optimizer_plugins();

  // subscriber
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

  // publisher
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_trajectories_;
  rclcpp::Publisher<PathWithLaneId>::SharedPtr pub_debug_path_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_debug_trajectory_;

  // others
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<::minimum_rule_based_planner::ParamListener> param_listener_;
  const UUID generator_uuid_;
  const VehicleInfo vehicle_info_;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;

  InternalPlannerData planner_data_;
  std::optional<lanelet::ConstLanelet> current_lanelet_{std::nullopt};

  // Optimizer plugins
  std::unique_ptr<PluginLoader> plugin_loader_;
  std::vector<std::shared_ptr<PluginInterface>> optimizer_plugins_;

  // Trajectory modifier plugins
  std::shared_ptr<plugin::ObstacleStopModifier> obstacle_stop_modifier_;
  trajectory_modifier::TrajectoryModifierParams modifier_params_;
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // MINIMUM_RULE_BASED_PLANNER_HPP_
