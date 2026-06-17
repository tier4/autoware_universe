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

#include "autoware/target_lanelet_estimator/node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>
#include <sstream>

namespace autoware::target_lanelet_estimator
{

TargetLaneletEstimatorNode::TargetLaneletEstimatorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("target_lanelet_estimator", options),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  using std::placeholders::_1;

  // map is latched
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TargetLaneletEstimatorNode::on_map, this, _1));

  sub_route_ = create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&TargetLaneletEstimatorNode::on_route, this, _1));

  sub_trajectory_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    std::bind(&TargetLaneletEstimatorNode::on_trajectory, this, _1));

  RCLCPP_INFO(
    get_logger(), "vehicle_info: length=%.2fm width=%.2fm wheel_base=%.2fm",
    vehicle_info_.vehicle_length_m, vehicle_info_.vehicle_width_m, vehicle_info_.wheel_base_m);
}

void TargetLaneletEstimatorNode::on_map(const LaneletMapBin::ConstSharedPtr msg)
{
  lanelet_map_ = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg);
  RCLCPP_INFO(
    get_logger(), "Received vector map (%zu lanelets).", lanelet_map_->laneletLayer.size());
}

void TargetLaneletEstimatorNode::on_route(const LaneletRoute::ConstSharedPtr msg)
{
  route_ = msg;
  RCLCPP_INFO(get_logger(), "Received route (%zu segments).", msg->segments.size());
}

void TargetLaneletEstimatorNode::on_trajectory(const Trajectory::ConstSharedPtr msg)
{
  trajectory_ = msg;
  run_estimation();
}

void TargetLaneletEstimatorNode::run_estimation()
{
  if (!lanelet_map_ || !route_ || !trajectory_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for inputs (map: %d, route: %d, trajectory: %d).",
      lanelet_map_ != nullptr, route_ != nullptr, trajectory_ != nullptr);
    return;
  }

  const auto result = get_target_lanelets(*route_, *trajectory_, lanelet_map_, vehicle_info_);

  std::stringstream ids;
  for (const auto & lanelet : result.lanelets) {
    ids << lanelet.id << " ";
  }
  if (result.out_of_lanelet) {
    ids << "out_of_lanelet";
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000, "target lanelets (%zu): %s", result.lanelets.size(),
    ids.str().c_str());
}

}  // namespace autoware::target_lanelet_estimator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::target_lanelet_estimator::TargetLaneletEstimatorNode)
