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

#include "autoware/target_lanelet_estimator/tracker.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/bool_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float64_multi_array_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int64_multi_array_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/Forward.h>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::target_lanelet_estimator
{
using autoware_internal_debug_msgs::msg::BoolStamped;
using autoware_internal_debug_msgs::msg::Float64MultiArrayStamped;
using autoware_internal_debug_msgs::msg::Int64MultiArrayStamped;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;

// Pre-triangulated fill of one route lanelet, reused every cycle so the (noisy and costly)
// triangulation runs only once per route.
struct LaneletTriangles
{
  lanelet::Id id{lanelet::InvalId};
  std::vector<geometry_msgs::msg::Point> points;
};

class TargetLaneletEstimatorNode : public rclcpp::Node
{
public:
  explicit TargetLaneletEstimatorNode(const rclcpp::NodeOptions & options);

private:
  void on_map(const LaneletMapBin::ConstSharedPtr msg);
  void on_route(const LaneletRoute::ConstSharedPtr msg);
  void on_trajectory(const Trajectory::ConstSharedPtr msg);
  void on_objects(const PredictedObjects::ConstSharedPtr msg);
  void run_estimation();  // ego, triggered by on_trajectory
  void publish_result(const TargetLaneletsResult & result);
  void publish_markers(const TargetLaneletsResult & result);
  void publish_object_markers();
  bool inputs_ready();
  void ensure_route_triangles();

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_objects_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_object_marker_;
  rclcpp::Publisher<Int64MultiArrayStamped>::SharedPtr pub_target_lanelet_ids_;
  rclcpp::Publisher<Float64MultiArrayStamped>::SharedPtr pub_target_lanelet_probabilities_;
  rclcpp::Publisher<BoolStamped>::SharedPtr pub_out_of_lanelet_;

  lanelet::LaneletMapConstPtr lanelet_map_;
  lanelet::routing::RoutingGraphConstPtr routing_graph_;
  LaneletRoute::ConstSharedPtr route_;
  Trajectory::ConstSharedPtr trajectory_;

  VehicleInfo vehicle_info_;
  Parameters params_;

  // object tracking options
  bool track_objects_{true};
  std::string target_object_uuid_;  // when set, only this object is tracked (for debugging)
  int64_t max_tracked_objects_{0};  // 0 = unlimited

  // recursive estimate of the ego and of every tracked surrounding vehicle (keyed by object UUID so
  // each object's state persists across frames)
  LaneletProbabilityTracker ego_tracker_;
  std::unordered_map<std::string, LaneletProbabilityTracker> object_trackers_;

  // lanelets the ego trajectory footprint has ever overlapped (the colored "trail")
  std::unordered_set<lanelet::Id> covered_lanelet_ids_;

  // marker geometry cache, rebuilt only when the route changes
  std::vector<LaneletTriangles> route_triangles_;
  LaneletRoute::ConstSharedPtr triangles_route_;
};

}  // namespace autoware::target_lanelet_estimator

#endif  // AUTOWARE__TARGET_LANELET_ESTIMATOR__NODE_HPP_
