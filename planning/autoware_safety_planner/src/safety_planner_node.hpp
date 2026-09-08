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

#ifndef AUTOWARE__SAFETY_PLANNER__SAFETY_PLANNER_NODE_HPP_
#define AUTOWARE__SAFETY_PLANNER__SAFETY_PLANNER_NODE_HPP_

#include "constraint_generator/constraint_generator_interface.hpp"
#include "context.hpp"
#include "safety_planner.hpp"
#include "type_alias.hpp"

#include <autoware_safety_planner/safety_planner_parameters.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tl/expected.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

class SafetyPlannerNode : public rclcpp::Node
{
public:
  explicit SafetyPlannerNode(const rclcpp::NodeOptions & options);

private:
  /**
   ***********************************************************
   * @defgroup Node general
   * @{
   */

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
    SteeringReport::ConstSharedPtr steering_ptr;
  };

  bool is_data_ready(const InputData & input_data);
  InputData take_data();
  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<::safety_planner::ParamListener> param_listener_;
  const UUID generator_uuid_;
  const VehicleInfo vehicle_info_;
  std::shared_ptr<TimeKeeper> time_keeper_;
  safety_planner::Params params_;

  std::unique_ptr<SafetyPlanner> planner_;

private:
  /**
   ***********************************************************
   * @defgroup Context
   * @{
   */

  bool update_input(const InputData & input_data);
  bool update_route_manager(const InputData & input_data);
  SafetyPlannerInput input_;

  std::optional<UUID> route_uuid_of_route_manager_;
  LaneletMapBin::ConstSharedPtr map_ptr_of_route_manager_;

  /** @* */

private:
  /**
   ***********************************************************
   * @defgroup Interface
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

  autoware_utils::InterProcessPollingSubscriber<SteeringReport> steering_subscriber_{
    this, "~/input/steering"};
  SteeringReport::ConstSharedPtr steering_ptr_;

  autoware_utils_rclcpp::InterProcessPollingSubscriber<PointCloud2> pointcloud_subscriber_{
    this, "~/input/pointcloud", autoware_utils::single_depth_sensor_qos()};
  PointCloud2::ConstSharedPtr obstacle_pointcloud_ptr_;

  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_debug_trajectory_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_debug_rough_trajectory_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_marker_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_rough_planner_marker_;
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_candidate_trajectories_;

  void publish_trajectory(const Trajectory & trajectory) const;
  void publish_rough_plan_trajectory(const RoughPlanResult & rough_plan_result);
  void publish_rough_plan_markers(const RoughPlanResult & rough_plan_result) const;
  void publish_debug_markers(const SafetyPlannerResult::Debug & debug) const;
  void publish_constraints_debug_markers(
    const std::map<std::string, ConstraintGeneratorOutput> & constraints) const;
  std::map<std::string, rclcpp::Publisher<MarkerArray>::SharedPtr>
    constraint_debug_marker_publishers_;

  /** @* */
};

}  // namespace autoware::safety_planner

#endif  // AUTOWARE__SAFETY_PLANNER__SAFETY_PLANNER_NODE_HPP_
