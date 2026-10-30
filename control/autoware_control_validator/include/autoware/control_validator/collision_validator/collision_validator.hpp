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

#ifndef AUTOWARE__CONTROL_VALIDATOR__COLLISION_VALIDATOR__COLLISION_VALIDATOR_HPP_
#define AUTOWARE__CONTROL_VALIDATOR__COLLISION_VALIDATOR__COLLISION_VALIDATOR_HPP_

#include "autoware/control_validator/collision_validator/collision_evaluator.hpp"
#include "autoware/control_validator/collision_validator/pointcloud_preprocessor.hpp"
#include "autoware/control_validator/hazard_warning/hazard_warning_sources.hpp"

#include <autoware_control_validator/control_validator_parameters.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <deque>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::control_validator
{
/**
 * @brief Entry of the collision warning: odometry history, pointcloud preprocessing,
 * collision evaluation, and the hand-over of the candidates to raise to HazardWarning.
 */
class CollisionValidator
{
public:
  CollisionValidator(
    const rclcpp::Logger & logger, const ::control_validator::Params & params,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper =
      std::make_shared<autoware_utils_debug::TimeKeeper>());

  void update_parameters(const ::control_validator::Params & params);

  /**
   * @brief Evaluate one cycle with whichever of objects / pointcloud is available and write
   * hazard_sources.collision (an empty vector when nothing is raised). hazard_sources.collision is
   * left untouched when neither input is available.
   * @param objects latest objects; may be null
   * @param pointcloud latest pointcloud; may be null
   */
  void validate(
    const autoware_planning_msgs::msg::Trajectory & predicted_trajectory,
    const nav_msgs::msg::Odometry & odom,
    const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects,
    const vehicle_info_utils::VehicleInfo & vehicle_info,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud,
    HazardWarningSources & hazard_sources);

private:
  void update_history(const nav_msgs::msg::Odometry & odom);
  std::optional<geometry_msgs::msg::Pose> select_ego_pose_at(const rclcpp::Time & stamp) const;
  static std::vector<HazardSource> to_hazard_sources(
    const std::vector<collision_validator::CollisionCandidate> & candidates,
    const nav_msgs::msg::Odometry & odom);

  static constexpr size_t kOdometryHistorySize = 10;

  rclcpp::Logger logger_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  ::control_validator::Params::CollisionValidator params_;
  std::deque<nav_msgs::msg::Odometry> odometry_history_;
  collision_validator::PointCloudPreprocessor pointcloud_preprocessor_;
  collision_validator::CollisionEvaluator collision_evaluator_;
};
}  // namespace autoware::control_validator

#endif  // AUTOWARE__CONTROL_VALIDATOR__COLLISION_VALIDATOR__COLLISION_VALIDATOR_HPP_
