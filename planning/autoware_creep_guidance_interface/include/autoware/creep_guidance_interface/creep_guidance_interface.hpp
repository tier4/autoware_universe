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

#ifndef AUTOWARE__CREEP_GUIDANCE_INTERFACE__CREEP_GUIDANCE_INTERFACE_HPP_
#define AUTOWARE__CREEP_GUIDANCE_INTERFACE__CREEP_GUIDANCE_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_creep_guidance_msgs/msg/command.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_status.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_status_array.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_trigger_command.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_trigger_response.hpp>
#include <tier4_creep_guidance_msgs/msg/detail/command__struct.hpp>
#include <tier4_creep_guidance_msgs/msg/module.hpp>
#include <tier4_creep_guidance_msgs/msg/state.hpp>
#include <tier4_creep_guidance_msgs/srv/creep_trigger_command.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace autoware::creep_guidance_interface
{

using tier4_creep_guidance_msgs::msg::Command;
using tier4_creep_guidance_msgs::msg::CreepStatus;
using tier4_creep_guidance_msgs::msg::CreepStatusArray;
using CreepTriggerCommandMsg = tier4_creep_guidance_msgs::msg::CreepTriggerCommand;
using tier4_creep_guidance_msgs::msg::CreepTriggerResponse;
using tier4_creep_guidance_msgs::msg::Module;
using tier4_creep_guidance_msgs::msg::State;
using CreepTriggerCommandSrv = tier4_creep_guidance_msgs::srv::CreepTriggerCommand;

/**
 * @brief Interface for managing creep guidance status and commands
 *
 * This class provides an interface for registering, updating, and publishing
 * creep guidance status information. It handles service requests for triggering
 * creep commands and maintains the status of registered creep guidance modules.
 */
class CreepGuidanceInterface
{
public:
  /**
   * @brief Constructor
   * @param node Pointer to the ROS 2 node
   * @param name Name of the creep guidance module (e.g., "crosswalk", "intersection")
   */
  CreepGuidanceInterface(rclcpp::Node * node, const std::string & name);

  /**
   * @brief Register a new creep guidance status entry
   * @param id Unique identifier for the creep guidance entry
   */
  void add(const int64_t id);

  /**
   * @brief Remove a registered creep guidance status entry
   * @param id Unique identifier for the creep guidance entry to remove
   * @return True if the entry was found and removed, false otherwise
   */
  bool remove(const int64_t id);

  /**
   * @brief Update the distance information for a registered entry
   * @param id Unique identifier for the creep guidance entry
   * @param start_distance Distance from the ego vehicle to the start point
   * @param finish_distance Distance from the ego vehicle to the finish point
   */
  void update_distance(const int64_t id, const double start_distance, const double finish_distance);

  /**
   * @brief Update the state of a registered entry
   * @param id Unique identifier for the creep guidance entry
   * @param state New state value to set
   */
  void update_state(const int64_t id, const uint8_t state);

  /**
   * @brief Check if an activation command has been received for a registered entry
   * @param id Unique identifier for the creep guidance entry
   * @return True if activation command has been received, false otherwise
   */
  bool recieved_activation_command(const int64_t id) const;

  /**
   * @brief Publish the current creep status array to the ROS 2 topic
   */
  void publish_creep_status_array() const;

private:
  /**
   * @brief Service callback for handling creep trigger commands
   * @param request Service request containing trigger commands
   * @param responses Service response containing the result of each command
   */
  void on_creep_trigger_command_service(
    const std::shared_ptr<CreepTriggerCommandSrv::Request> request,
    const CreepTriggerCommandSrv::Response::SharedPtr responses);

  /**
   * @brief Update the command for a registered entry
   * @param id Unique identifier for the creep guidance entry
   * @param command New command to set
   */
  void update_command(const int64_t id, const Command & command);

  /**
   * @brief Find the iterator to a registered entry by ID
   * @param id Unique identifier for the creep guidance entry
   * @return Iterator to the registered entry, or end() if not found
   */
  std::vector<CreepStatus>::iterator get_registered(const int64_t id);

  /**
   * @brief Find the const iterator to a registered entry by ID
   * @param id Unique identifier for the creep guidance entry
   * @return Const iterator to the registered entry, or end() if not found
   */
  std::vector<CreepStatus>::const_iterator get_registered(const int64_t id) const;

  rclcpp::CallbackGroup::SharedPtr callback_group_;  //!< Callback group for service
  rclcpp::Service<CreepTriggerCommandSrv>::SharedPtr
    srv_creep_trigger_commands_;  //!< Service for receiving creep trigger commands
  rclcpp::Publisher<CreepStatusArray>::SharedPtr
    pub_creep_status_array_;        //!< Publisher for creep status array
  rclcpp::Clock::SharedPtr clock_;  //!< Clock for timestamping messages
  mutable rclcpp::Logger logger_;   //!< Logger for debug and info messages

  CreepStatusArray registered_status_;  //!< Array of registered creep guidance statuses

  Module module_;  //!< Module type (crosswalk, intersection, etc.)
};

}  // namespace autoware::creep_guidance_interface

#endif  // AUTOWARE__CREEP_GUIDANCE_INTERFACE__CREEP_GUIDANCE_INTERFACE_HPP_
