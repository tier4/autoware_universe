//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef COMMON__COMMAND_STATUS_HPP_
#define COMMON__COMMAND_STATUS_HPP_

#include <autoware_command_mode_types/types/command_mode_status.hpp>

#include <string>

namespace autoware::command_mode_switcher
{

enum class SourceState {
  Disabled,
  Transition,
  Enabled,
};

enum class SourceGroup {
  Shared,
  Exclusive,
};

enum class TransitionState {
  Transition,
  Completed,
};

enum class MainState {
  Inactive,
  Transition,
  Active,
};

enum class MrmState {
  Normal,
  Operating,
  Succeeded,
  Failed,
};

using command_mode_types::RequestStage;

struct CommandStatus
{
  MainState state;
  MrmState mrm;
  RequestStage request;
  bool transition;
  bool control_gate_selected;
  bool vehicle_gate_selected;
  bool control_gate_request;
  bool vehicle_gate_request;
  bool mode_continuable;
  bool mode_available;
  bool transition_available;
  bool transition_completed;

  SourceState source_state;
  SourceGroup source_group;
};

// For ROS message
uint8_t convert_main_state(const MainState & state);
uint8_t convert_mrm_state(const MrmState & mrm);
std::string convert_debug_string(const CommandStatus & status);

// For internal use.
MainState update_main_state(const CommandStatus & status);

}  // namespace autoware::command_mode_switcher

#endif  // COMMON__COMMAND_STATUS_HPP_
