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

#ifndef AUTOWARE_COMMAND_MODE_TYPES__TYPES__COMMAND_MODE_STATUS_HPP_
#define AUTOWARE_COMMAND_MODE_TYPES__TYPES__COMMAND_MODE_STATUS_HPP_

namespace autoware::command_mode_types
{

enum class TriState {
  Disabled,
  Enabled,
  Transition,
};

enum class MrmState {
  Normal,
  Operating,
  Succeeded,
  Failed,
};

enum class RequestStage {
  NoRequest,
  CommandMode,
  VehicleGate,
  NetworkGate,
  ControlGate,
};

}  // namespace autoware::command_mode_types

#endif  // AUTOWARE_COMMAND_MODE_TYPES__TYPES__COMMAND_MODE_STATUS_HPP_
