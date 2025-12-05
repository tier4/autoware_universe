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

#include "autoware/creep_guidance_interface/creep_guidance_interface.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace
{

using Module = tier4_creep_guidance_msgs::msg::Module;

Module get_module_type(const std::string & name)
{
  Module module;
  if (name == "crosswalk") {
    module.type = Module::CROSSWALK;
  } else if (name == "intersection") {
    module.type = Module::INTERSECTION;
  } else if (name == "intersection_occlusion") {
    module.type = Module::INTERSECTION_OCCLUSION;
  } else {
    module.type = Module::NONE;
  }
  return module;
}

}  // namespace

namespace autoware::creep_guidance_interface
{

CreepGuidanceInterface::CreepGuidanceInterface(rclcpp::Node * node, const std::string & name)
: clock_(node->get_clock()), logger_(node->get_logger())
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Create callback group for service
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create service for creep trigger commands
  const std::string service_name = "/planning/creep_guidance_commands/" + name;
  srv_creep_trigger_commands_ = node->create_service<CreepTriggerCommandSrv>(
    service_name,
    std::bind(&CreepGuidanceInterface::on_creep_trigger_command_service, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_);

  // Create publisher for creep status array
  const std::string topic_name = "/planning/creep_guidance_status/" + name;
  pub_creep_status_array_ = node->create_publisher<CreepStatusArray>(topic_name, 1);

  module_ = get_module_type(name);
  RCLCPP_INFO(logger_, "CreepGuidanceInterface initialized with service: %s", service_name.c_str());
}

void CreepGuidanceInterface::add(const int64_t id)
{
  CreepStatus status;
  status.id = id;
  status.module = module_;
  status.state.type = State::DEACTIVATED;
  status.command.type = Command::DEACTIVATE;
  status.start_distance = -100000.0;
  status.finish_distance = -100000.0;
  registered_status_.statuses.push_back(status);
  RCLCPP_DEBUG(logger_, "Added creep guidance entry with id: %ld", id);
}

bool CreepGuidanceInterface::remove(const int64_t id)
{
  auto it = get_registered(id);
  if (it != registered_status_.statuses.end()) {
    registered_status_.statuses.erase(it);
    RCLCPP_DEBUG(logger_, "Removed creep guidance entry with id: %ld", id);
    return true;
  }
  RCLCPP_WARN(logger_, "Failed to remove: id %ld not found", id);
  return false;
}

std::vector<CreepStatus>::iterator CreepGuidanceInterface::get_registered(const int64_t id)
{
  return std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [id](const auto & s) { return s.id == id; });
}

std::vector<CreepStatus>::const_iterator CreepGuidanceInterface::get_registered(
  const int64_t id) const
{
  return std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [id](const auto & s) { return s.id == id; });
}

void CreepGuidanceInterface::update_distance(
  const int64_t id, const double start_distance, const double finish_distance)
{
  auto registered = get_registered(id);
  if (registered != registered_status_.statuses.end()) {
    registered->start_distance = static_cast<float>(start_distance);
    registered->finish_distance = static_cast<float>(finish_distance);
    RCLCPP_DEBUG(
      logger_, "Updated distance for id %ld: start=%.2f, finish=%.2f", id, start_distance,
      finish_distance);
  } else {
    RCLCPP_WARN(logger_, "Failed to update distance: id %ld not found", id);
  }
}

void CreepGuidanceInterface::update_state(const int64_t id, const uint8_t state)
{
  auto registered = get_registered(id);
  if (registered != registered_status_.statuses.end()) {
    registered->state.type = state;
    RCLCPP_DEBUG(logger_, "Updated state for id %ld: state=%u", id, state);
  } else {
    RCLCPP_WARN(logger_, "Failed to update state: id %ld not found", id);
  }
}

void CreepGuidanceInterface::update_command(const int64_t id, const Command & command)
{
  auto registered = get_registered(id);
  if (registered != registered_status_.statuses.end()) {
    registered->command = command;
    RCLCPP_DEBUG(logger_, "Updated command for id %ld: command_type=%u", id, command.type);
  } else {
    RCLCPP_WARN(logger_, "Failed to update command: id %ld not found", id);
  }
}

bool CreepGuidanceInterface::recieved_activation_command(const int64_t id) const
{
  auto registered = get_registered(id);
  if (registered != registered_status_.statuses.end()) {
    return registered->command.type == Command::ACTIVATE;
  }
  RCLCPP_WARN(logger_, "Failed to get command: id %ld not found, returning false", id);
  return false;
}

void CreepGuidanceInterface::publish_creep_status_array() const
{
  CreepStatusArray status_array = registered_status_;
  status_array.stamp = clock_->now();
  pub_creep_status_array_->publish(status_array);
  RCLCPP_DEBUG(
    logger_, "Published creep status array with %zu entries", status_array.statuses.size());
}

void CreepGuidanceInterface::on_creep_trigger_command_service(
  const CreepTriggerCommandSrv::Request::SharedPtr request,
  const CreepTriggerCommandSrv::Response::SharedPtr response)
{
  for (const auto & command : request->commands) {
    if (
      module_ == command.module &&
      get_registered(command.id) != registered_status_.statuses.end()) {
      update_command(command.id, command.command);
      CreepTriggerResponse success;
      success.id = command.id;
      success.module = command.module;
      success.success = true;
      response->responses.push_back(success);
      RCLCPP_DEBUG(
        logger_, "Creep trigger command succeeded: id=%ld, command_type=%u", command.id,
        command.command.type);
    } else {
      CreepTriggerResponse failure;
      failure.id = command.id;
      failure.module = command.module;
      failure.success = false;
      response->responses.push_back(failure);
      if (module_ != command.module) {
        RCLCPP_WARN(
          logger_, "Creep trigger command failed: id=%ld, module mismatch (expected=%u, got=%u)",
          command.id, module_.type, command.module.type);
      } else {
        RCLCPP_WARN(logger_, "Creep trigger command failed: id=%ld not registered", command.id);
      }
    }
  }
}
}  // namespace autoware::creep_guidance_interface
