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

#include <agnocast/agnocast.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/srv/set_lanelet_route.hpp>
#include <autoware_planning_msgs/srv/set_preferred_primitive.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_external_api_msgs/srv/set_preferred_lane.hpp>

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

class ManualLaneChangeHandler : public agnocast::Node
{
public:
  explicit ManualLaneChangeHandler(const rclcpp::NodeOptions & options);

private:
  std::vector<autoware_planning_msgs::msg::LaneletPrimitive> sort_primitives_left_to_right(
    const route_handler::RouteHandler & route_handler,
    autoware_planning_msgs::msg::LaneletPrimitive preferred_primitive,
    std::vector<autoware_planning_msgs::msg::LaneletPrimitive> primitives);

  lanelet::ConstLanelet get_lanelet_by_id(const int64_t id)
  {
    return route_handler_.getLaneletMapPtr()->laneletLayer.get(id);
  }

  void map_callback(
    const agnocast::ipc_shared_ptr<const autoware_map_msgs::msg::LaneletMapBin> & msg);
  void route_callback(const agnocast::ipc_shared_ptr<const LaneletRoute> & msg);
  void set_preferred_lane(
    const agnocast::ipc_shared_ptr<agnocast::Service<SetPreferredLane>::RequestT> & req,
    agnocast::ipc_shared_ptr<agnocast::Service<SetPreferredLane>::ResponseT> & res);
  LaneChangeRequestResult process_lane_change_request(
    const int64_t ego_lanelet_id, uint8_t lane_change_direction);

  agnocast::Service<SetPreferredLane>::SharedPtr srv_set_preferred_lane;
  agnocast::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  agnocast::Subscription<LaneletRoute>::SharedPtr sub_route_;
  agnocast::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_vector_map_;
  agnocast::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_;
  agnocast::Publisher<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr pub_shift_number_;

  autoware::route_handler::RouteHandler route_handler_;
  bool is_map_ready_{false};

  agnocast::ipc_shared_ptr<const nav_msgs::msg::Odometry> odometry_;
  std::shared_ptr<LaneletRoute> current_route_;
  agnocast::Client<SetPreferredPrimitive>::SharedPtr client_;

  int8_t shift_number_{0};
};

}  // namespace autoware::manual_lane_change_handler

#endif  // MANUAL_LANE_CHANGE_HANDLER_HPP_
