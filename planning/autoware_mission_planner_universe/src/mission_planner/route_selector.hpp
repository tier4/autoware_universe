// Copyright 2024 The Autoware Contributors
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

#ifndef MISSION_PLANNER__ROUTE_SELECTOR_HPP_
#define MISSION_PLANNER__ROUTE_SELECTOR_HPP_

#include <agnocast/agnocast.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/route_state.hpp>
#include <autoware_planning_msgs/srv/clear_route.hpp>
#include <autoware_planning_msgs/srv/set_lanelet_route.hpp>
#include <autoware_planning_msgs/srv/set_waypoint_route.hpp>

#include <mutex>
#include <optional>
#include <variant>

namespace autoware::mission_planner_universe
{

using autoware_common_msgs::msg::ResponseStatus;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::RouteState;
using autoware_planning_msgs::srv::ClearRoute;
using autoware_planning_msgs::srv::SetLaneletRoute;
using autoware_planning_msgs::srv::SetWaypointRoute;
using unique_identifier_msgs::msg::UUID;

class RouteInterface
{
public:
  explicit RouteInterface(rclcpp::Clock::SharedPtr clock);
  RouteState::_state_type get_state() const;
  std::optional<LaneletRoute> get_route() const;
  void change_route();
  void change_state(RouteState::_state_type state);
  void update_state(const RouteState & state);
  void update_route(const LaneletRoute & route);

  agnocast::Service<ClearRoute>::SharedPtr srv_clear_route;
  agnocast::Service<SetLaneletRoute>::SharedPtr srv_set_lanelet_route;
  agnocast::Service<SetWaypointRoute>::SharedPtr srv_set_waypoint_route;
  agnocast::Publisher<RouteState>::SharedPtr pub_state_;
  agnocast::Publisher<LaneletRoute>::SharedPtr pub_route_;

private:
  RouteState state_;
  std::optional<LaneletRoute> route_;
  rclcpp::Clock::SharedPtr clock_;
};

class RouteSelector : public rclcpp::Node
{
public:
  explicit RouteSelector(const rclcpp::NodeOptions & options);
  void publish_processing_time(autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch);

private:
  using WaypointRequest = SetWaypointRoute::Request::SharedPtr;
  using LaneletRequest = SetLaneletRoute::Request::SharedPtr;

  RouteInterface main_;
  RouteInterface mrm_;

  agnocast::Client<ClearRoute>::SharedPtr cli_clear_route_;
  agnocast::Client<SetWaypointRoute>::SharedPtr cli_set_waypoint_route_;
  agnocast::Client<SetLaneletRoute>::SharedPtr cli_set_lanelet_route_;
  agnocast::Subscription<RouteState>::SharedPtr sub_state_;
  agnocast::Subscription<LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_;

  std::mutex mutex_;
  bool initialized_;
  bool mrm_operating_;
  std::variant<std::monostate, WaypointRequest, LaneletRequest> main_request_;

  void on_state(const agnocast::ipc_shared_ptr<const RouteState> & msg);
  void on_route(const agnocast::ipc_shared_ptr<const LaneletRoute> & msg);

  void on_clear_route_main(
    const agnocast::ipc_shared_ptr<agnocast::Service<ClearRoute>::RequestT> & req,
    agnocast::ipc_shared_ptr<agnocast::Service<ClearRoute>::ResponseT> & res);
  void on_set_waypoint_route_main(
    const agnocast::ipc_shared_ptr<agnocast::Service<SetWaypointRoute>::RequestT> & req,
    agnocast::ipc_shared_ptr<agnocast::Service<SetWaypointRoute>::ResponseT> & res);
  void on_set_lanelet_route_main(
    const agnocast::ipc_shared_ptr<agnocast::Service<SetLaneletRoute>::RequestT> & req,
    agnocast::ipc_shared_ptr<agnocast::Service<SetLaneletRoute>::ResponseT> & res);

  void on_clear_route_mrm(
    const agnocast::ipc_shared_ptr<agnocast::Service<ClearRoute>::RequestT> & req,
    agnocast::ipc_shared_ptr<agnocast::Service<ClearRoute>::ResponseT> & res);
  void on_set_waypoint_route_mrm(
    const agnocast::ipc_shared_ptr<agnocast::Service<SetWaypointRoute>::RequestT> & req,
    agnocast::ipc_shared_ptr<agnocast::Service<SetWaypointRoute>::ResponseT> & res);
  void on_set_lanelet_route_mrm(
    const agnocast::ipc_shared_ptr<agnocast::Service<SetLaneletRoute>::RequestT> & req,
    agnocast::ipc_shared_ptr<agnocast::Service<SetLaneletRoute>::ResponseT> & res);

  ResponseStatus resume_main_route();
};

}  // namespace autoware::mission_planner_universe

#endif  // MISSION_PLANNER__ROUTE_SELECTOR_HPP_
