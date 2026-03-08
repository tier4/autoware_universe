// Copyright 2025 TIER IV, Inc.
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

#ifndef NODE_HPP_
#define NODE_HPP_

// include
#include <agnocast/node/agnocast_node.hpp>

#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>

namespace autoware::hazard_lights_selector
{

struct Parameters
{
  int update_rate;  // [Hz]
};

class HazardLightsSelector : public agnocast::Node
{
public:
  explicit HazardLightsSelector(const rclcpp::NodeOptions & node_options);

private:
  // Parameter
  Parameters params_;

  // Subscriber
  agnocast::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    sub_hazard_lights_command_from_planning_;
  agnocast::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    sub_hazard_lights_command_from_system_;

  void on_hazard_lights_command_from_planning(
    const agnocast::ipc_shared_ptr<autoware_vehicle_msgs::msg::HazardLightsCommand> & msg);
  void on_hazard_lights_command_from_system(
    const agnocast::ipc_shared_ptr<autoware_vehicle_msgs::msg::HazardLightsCommand> & msg);

  // Publisher
  agnocast::Publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    pub_hazard_lights_command_;

  // Timer
  agnocast::TimerBase::SharedPtr timer_;

  void on_timer();

  // State
  agnocast::ipc_shared_ptr<autoware_vehicle_msgs::msg::HazardLightsCommand>
    hazard_lights_command_from_planning_;
  agnocast::ipc_shared_ptr<autoware_vehicle_msgs::msg::HazardLightsCommand>
    hazard_lights_command_from_system_;
};
}  // namespace autoware::hazard_lights_selector

#endif  // NODE_HPP_
