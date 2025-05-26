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

#ifndef AUTOWARE_COMMAND_MODE_TYPES__CONSTANTS__MODES_HPP_
#define AUTOWARE_COMMAND_MODE_TYPES__CONSTANTS__MODES_HPP_

#include <cstdint>

namespace autoware::command_mode_types::modes
{

constexpr uint16_t unknown = 0;
constexpr uint16_t manual = 1000;
constexpr uint16_t stop = 1001;
constexpr uint16_t autonomous = 1002;
constexpr uint16_t local = 1003;
constexpr uint16_t remote = 1004;
constexpr uint16_t emergency_stop = 1005;
constexpr uint16_t comfortable_stop = 1006;
constexpr uint16_t pull_over = 1007;

}  // namespace autoware::command_mode_types::modes

#endif  // AUTOWARE_COMMAND_MODE_TYPES__CONSTANTS__MODES_HPP_
