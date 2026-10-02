// Copyright 2025 Autoware Foundation
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

#ifndef MANUAL_LANE_CHANGE_HANDLER_HPP_
#define MANUAL_LANE_CHANGE_HANDLER_HPP_

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware/mission_planner_universe/service_utils.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/srv/set_lanelet_route.hpp>
#include <autoware_planning_msgs/srv/set_preferred_primitive.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_external_api_msgs/srv/set_preferred_lane.hpp>
#include <tier4_planning_msgs/msg/reroute_availability.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::manual_lane_change_handler
{

using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::srv::SetPreferredPrimitive;
using tier4_external_api_msgs::srv::SetPreferredLane;

struct LaneChangeRequestResult
{
  std::vector<LaneletPrimitive> preferred_primitives;
  bool success;
  std::string message;
};

enum class DIRECTION {
  MANUAL_LEFT,
  MANUAL_RIGHT,
  AUTO,
};

class ManualLaneChangeHandler : public autoware::agnocast_wrapper::Node
{
public:
  explicit ManualLaneChangeHandler(const rclcpp::NodeOptions & options);

  void publish_processing_time(autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch)
  {
    autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
    processing_time_msg.stamp = get_clock()->now();
    processing_time_msg.data = stop_watch.toc();
    pub_processing_time_->publish(processing_time_msg);
  }

private:
  std::vector<autoware_planning_msgs::msg::LaneletPrimitive> sort_primitives_left_to_right(
    const autoware::route_handler::RouteHandler & route_handler,
    autoware_planning_msgs::msg::LaneletPrimitive preferred_primitive,
    std::vector<autoware_planning_msgs::msg::LaneletPrimitive> primitives);

  lanelet::ConstLanelet get_lanelet_by_id(const int64_t id)
  {
    return route_handler_.getLaneletMapPtr()->laneletLayer.get(id);
  }

  void route_callback(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(LaneletRoute) & msg);
  void set_preferred_lane(
    const SetPreferredLane::Request::SharedPtr req,
    const SetPreferredLane::Response::SharedPtr res);
  LaneChangeRequestResult process_lane_change_request(
    const int64_t ego_lanelet_id, const SetPreferredLane::Request::SharedPtr req);

  AUTOWARE_SERVICE_PTR(SetPreferredLane) srv_set_preferred_lane;
  AUTOWARE_SUBSCRIPTION_PTR(nav_msgs::msg::Odometry) sub_odometry_;
  AUTOWARE_SUBSCRIPTION_PTR(autoware_map_msgs::msg::LaneletMapBin) sub_map_;
  AUTOWARE_SUBSCRIPTION_PTR(LaneletRoute) sub_route_;
  autoware::agnocast_wrapper::polling::PollingSubscriber<
    tier4_planning_msgs::msg::RerouteAvailability>::SharedPtr sub_reroute_availability_{
    autoware::agnocast_wrapper::polling::create_polling_subscriber<
      tier4_planning_msgs::msg::RerouteAvailability>(this, "~/input/reroute_availability", 1)};
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float64Stamped) pub_processing_time_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Int32Stamped) pub_shift_number_;

  autoware::route_handler::RouteHandler route_handler_;

  AUTOWARE_MESSAGE_CONST_SHARED_PTR(nav_msgs::msg::Odometry) odometry_;
  std::shared_ptr<LaneletRoute> current_route_;
  AUTOWARE_CLIENT_PTR(autoware_planning_msgs::srv::SetPreferredPrimitive) client_;
  rclcpp::Logger logger_;

  int8_t shift_number_{0};
};

}  // namespace autoware::manual_lane_change_handler

#endif  // MANUAL_LANE_CHANGE_HANDLER_HPP_
