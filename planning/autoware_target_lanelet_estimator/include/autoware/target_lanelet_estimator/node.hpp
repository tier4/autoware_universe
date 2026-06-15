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

#ifndef AUTOWARE__TARGET_LANELET_ESTIMATOR__NODE_HPP_
#define AUTOWARE__TARGET_LANELET_ESTIMATOR__NODE_HPP_

#include "autoware/target_lanelet_estimator/impl.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <lanelet2_core/LaneletMap.h>

namespace autoware::target_lanelet_estimator
{
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;

class TargetLaneletEstimatorNode : public rclcpp::Node
{
public:
  explicit TargetLaneletEstimatorNode(const rclcpp::NodeOptions & options);

private:
  void on_map(const LaneletMapBin::ConstSharedPtr msg);
  void on_route(const LaneletRoute::ConstSharedPtr msg);
  void on_trajectory(const Trajectory::ConstSharedPtr msg);
  void run_estimation();  // triggered by on_trajectory

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;

  lanelet::LaneletMapConstPtr lanelet_map_;
  LaneletRoute::ConstSharedPtr route_;
  Trajectory::ConstSharedPtr trajectory_;

  VehicleInfo vehicle_info_;
};

}  // namespace autoware::target_lanelet_estimator

#endif  // AUTOWARE__TARGET_LANELET_ESTIMATOR__NODE_HPP_
