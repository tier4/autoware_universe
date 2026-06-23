// Copyright 2026 Autoware Foundation
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

#include "autoware/avoidance_target_detector/node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_io/Io.h>

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <utility>

namespace autoware::avoidance_target_detector
{

// Temporary debug code. Must be removed before release.
void export_debug_map(const lanelet::LaneletMapPtr & map, LaneletRoute::ConstSharedPtr & route)
{
  const auto package_share_directory =
    ament_index_cpp::get_package_share_directory("autoware_avoidance_target_detector");
  const auto debug_map_path_str = package_share_directory + "/debug_map.osm";

  lanelet::LaneletMapPtr debug_map = std::make_shared<lanelet::LaneletMap>();
  const auto routing_graph = traffic_rules::create_goal_purpose_routing_graph(*map);

  autoware_planning_msgs::msg::LaneletRoute enhanced_route;
  enhanced_route.header = route->header;
  enhanced_route.start_pose = route->start_pose;
  enhanced_route.goal_pose = route->goal_pose;

  std::set<lanelet::Id> lanelet_ids;
  for (const auto & segment : route->segments) {
    autoware_planning_msgs::msg::LaneletSegment enhanced_segment;
    enhanced_segment.preferred_primitive = segment.preferred_primitive;
    for (const auto & primitive : segment.primitives) {
      if (!map->laneletLayer.exists(primitive.id)) {
        continue;
      }
      autoware_planning_msgs::msg::LaneletPrimitive prim;
      prim.id = primitive.id;
      prim.primitive_type = "lane";
      lanelet::Lanelet lanelet = map->laneletLayer.get(primitive.id);
      enhanced_segment.primitives.push_back(prim);

      if (const auto left_lanelet = traffic_rules::get_left_lanelet(*routing_graph, lanelet)) {
        autoware_planning_msgs::msg::LaneletPrimitive prim;
        prim.id = left_lanelet->id();
        prim.primitive_type = "lane";
        enhanced_segment.primitives.push_back(prim);
        lanelet_ids.insert(left_lanelet->id());
      }
      if (const auto right_lanelet = traffic_rules::get_right_lanelet(*routing_graph, lanelet)) {
        autoware_planning_msgs::msg::LaneletPrimitive prim;
        prim.id = right_lanelet->id();
        prim.primitive_type = "lane";
        enhanced_segment.primitives.push_back(prim);
        lanelet_ids.insert(right_lanelet->id());
      }
      lanelet_ids.insert(primitive.id);
    }
    enhanced_route.segments.push_back(enhanced_segment);
  }

  // Get siblings
  for (size_t i = 0; i + 2 < enhanced_route.segments.size(); ++i) {
    auto & segment = enhanced_route.segments[i + 1];

    std::set<lanelet::Id> next_ids;
    for (const auto & prim : enhanced_route.segments[i + 2].primitives) {
      next_ids.insert(prim.id);
    }

    std::set<lanelet::Id> current_ids;
    for (const auto & prim : segment.primitives) {
      current_ids.insert(prim.id);
    }

    for (const auto & prev_prim : enhanced_route.segments[i].primitives) {
      if (!map->laneletLayer.exists(prev_prim.id)) {
        continue;
      }
      const auto lanelet = map->laneletLayer.get(prev_prim.id);
      for (const auto & candidate : routing_graph->following(lanelet)) {
        if (current_ids.count(candidate.id()) > 0) {
          continue;
        }
        const auto next_lanelets = routing_graph->following(candidate);
        const bool connects_to_next_segment = std::any_of(
          next_lanelets.begin(), next_lanelets.end(),
          [&](const auto & next) { return next_ids.count(next.id()) > 0; });
        if (!connects_to_next_segment) {
          continue;
        }

        autoware_planning_msgs::msg::LaneletPrimitive sibling_prim;
        sibling_prim.id = candidate.id();
        sibling_prim.primitive_type = "lane";
        segment.primitives.push_back(sibling_prim);
        current_ids.insert(candidate.id());
        lanelet_ids.insert(candidate.id());
      }
    }
  }

  for (const auto lanelet_id : lanelet_ids) {
    if (!map->laneletLayer.exists(lanelet_id)) {
      continue;
    }
    lanelet::Lanelet lanelet = map->laneletLayer.get(lanelet_id);
    const auto reg_elements = lanelet.regulatoryElements();
    for (const auto & elem : reg_elements) {
      lanelet.removeRegulatoryElement(elem);
    }
    debug_map->add(lanelet);
  }

  constexpr double k_road_border_near_distance_m = 1.5;
  std::set<lanelet::Id> road_border_ids;
  for (const auto lanelet_id : lanelet_ids) {
    if (!map->laneletLayer.exists(lanelet_id)) {
      continue;
    }
    const auto lanelet = map->laneletLayer.get(lanelet_id);
    const auto nearby_linestrings =
      lanelet::geometry::findWithin2d(map->lineStringLayer, lanelet, k_road_border_near_distance_m);
    for (const auto & nearby_linestring : nearby_linestrings) {
      const auto & linestring = nearby_linestring.second;
      if (road_border_ids.count(linestring.id()) > 0) {
        continue;
      }
      constexpr auto no_type = "none";
      const std::string type = linestring.attributeOr(lanelet::AttributeName::Type, no_type);
      if (type != "road_border") {
        continue;
      }
      road_border_ids.insert(linestring.id());
      debug_map->add(linestring);
    }
  }

  // get map origin

  constexpr double origin_lat = 35.22312494103;
  constexpr double origin_lon = 138.80245834626;
  lanelet::Origin origin({origin_lat, origin_lon});
  lanelet::projection::MGRSProjector projector(origin);
  projector.setMGRSCode(lanelet::GPSPoint{origin_lat, origin_lon, 0.0});

  lanelet::write(debug_map_path_str, *debug_map, projector);
  RCLCPP_INFO(
    rclcpp::get_logger("autoware_avoidance_target_detector"), "Exported debug map to %s",
    debug_map_path_str.c_str());
}

/**
 * @brief Construct the avoidance target detector node.
 * @param node_options Node options for component loading.
 */
AvoidanceTargetDetectorNode::AvoidanceTargetDetectorNode(const rclcpp::NodeOptions & node_options)
: Node{"avoidance_target_detector", node_options},
  sub_objects_{create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&AvoidanceTargetDetectorNode::on_objects, this, std::placeholders::_1))},
  pub_avoidance_targets_{create_publisher<PredictedObjects>("~/output/avoidance_targets", 1)},
  pub_debug_avoidance_targets_{create_publisher<PredictedObjects>("~/debug/avoidance_targets", 1)},
  pub_drivable_area_path_{create_publisher<Path>("~/output/drivable_area", 1)},
  route_handler_{std::make_shared<RouteHandler>()},
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()}
{
  longitudinal_filter_params_.tolerance_m =
    declare_parameter<double>("longitudinal_distance_filter.tolerance_m", 0.0);
  lateral_filter_params_.tolerance_m =
    declare_parameter<double>("lateral_distance_filter.tolerance_m", 0.0);
}

/**
 * @brief Callback for incoming predicted objects.
 * @param msg Predicted objects message.
 */
void AvoidanceTargetDetectorNode::on_objects(const PredictedObjects::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (const auto route_msg = sub_route_.take_data()) {
    if (!route_msg->segments.empty()) {
      route_handler_->setRoute(*route_msg);
      route_ = std::make_shared<LaneletRoute>(*route_msg);
      cached_drivable_area_.reset();
      RCLCPP_INFO(get_logger(), "Received route");
    }
  }

  if (!route_) {
    return;
  }

  if (const auto map_msg = sub_lanelet_map_.take_data()) {
    route_handler_->setMap(*map_msg);
    goal_purpose_routing_graph_ =
      traffic_rules::create_goal_purpose_routing_graph(*route_handler_->getLaneletMapPtr());
    cached_drivable_area_.reset();
    export_debug_map(route_handler_->getLaneletMapPtr(), route_);
    RCLCPP_INFO(get_logger(), "Received lanelet map");
  }

  if (!goal_purpose_routing_graph_) {
    return;
  }

  const auto current_time = get_clock()->now();

  trajectory_ = sub_trajectory_.take_data();
  const Trajectory trajectory_msg = trajectory_ ? *trajectory_ : Trajectory{};

  if (!trajectory_ || !route_handler_->isHandlerReady() || !goal_purpose_routing_graph_) {
    RCLCPP_WARN(get_logger(), "Data is not ready");
    return;
  }

  for (const auto & object : msg->objects) {
    const auto object_id_str = autoware_utils_uuid::to_hex_string(object.object_id);
    if (object_filters_.find(object_id_str) == object_filters_.end()) {
      object_filters_.emplace(object_id_str, FilterManager(object, current_time));
    }
  }

  for (const auto & object : msg->objects) {
    const auto object_id_str = autoware_utils_uuid::to_hex_string(object.object_id);
    const auto it = object_filters_.find(object_id_str);
    if (it != object_filters_.end()) {
      it->second.observe_and_update_all(current_time, object, trajectory_msg);
    }
  }

  for (auto it = object_filters_.begin(); it != object_filters_.end();) {
    if (it->second.is_stale(current_time)) {
      it = object_filters_.erase(it);
    } else {
      ++it;
    }
  }

  for (auto & [object_id_str, filter_manager] : object_filters_) {
    if (filter_manager.get_debug_log().empty()) {
      continue;
    }
    RCLCPP_INFO(
      get_logger(), "Object ID: %s, Debug Log: %s", object_id_str.c_str(),
      filter_manager.get_debug_log().c_str());
    filter_manager.clear_debug_log();
  }

  PredictedObjects debug_avoidance_targets = *msg;
  debug_avoidance_targets.objects.erase(
    std::remove_if(
      debug_avoidance_targets.objects.begin(), debug_avoidance_targets.objects.end(),
      [&](const PredictedObject & object) {
        const auto it = object_filters_.find(autoware_utils_uuid::to_hex_string(object.object_id));
        return it == object_filters_.end() || !it->second.is_target();
      }),
    debug_avoidance_targets.objects.end());

  pub_debug_avoidance_targets_->publish(debug_avoidance_targets);

  std::optional<DrivableAreaResult> drivable_area;
  if (trajectory_ && route_handler_->isHandlerReady() && goal_purpose_routing_graph_) {
    if (
      auto new_drivable_area = create_drivable_area(
        *route_handler_, *goal_purpose_routing_graph_, *trajectory_, vehicle_info_,
        route_ ? route_.get() : nullptr)) {
      cached_drivable_area_ = std::move(new_drivable_area);
    }
    drivable_area = cached_drivable_area_;
    if (drivable_area) {
      pub_drivable_area_path_->publish(to_path_msg(*drivable_area, *trajectory_));
    }
  }

  PredictedObjects avoidance_targets = debug_avoidance_targets;
  avoidance_targets.objects.erase(
    std::remove_if(
      avoidance_targets.objects.begin(), avoidance_targets.objects.end(),
      [&](const PredictedObject & object) {
        if (should_filter_out_by_longitudinal_distance(
              trajectory_msg, object, longitudinal_filter_params_)) {
          return true;
        }
        if (
          drivable_area && should_filter_out_by_lateral_distance(
                             *drivable_area, trajectory_msg, object, lateral_filter_params_)) {
          return true;
        }
        return false;
      }),
    avoidance_targets.objects.end());

  pub_avoidance_targets_->publish(avoidance_targets);
}

}  // namespace autoware::avoidance_target_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::avoidance_target_detector::AvoidanceTargetDetectorNode)
