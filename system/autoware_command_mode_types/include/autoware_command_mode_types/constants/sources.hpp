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

#ifndef AUTOWARE_COMMAND_MODE_TYPES__CONSTANTS__SOURCES_HPP_
#define AUTOWARE_COMMAND_MODE_TYPES__CONSTANTS__SOURCES_HPP_

namespace autoware::command_mode_types::sources
{

// Builtin source
constexpr uint16_t unknown = 0;
constexpr uint16_t builtin = 1;

// Main sources
constexpr uint16_t stop = 11;
constexpr uint16_t main = 12;
constexpr uint16_t local = 13;
constexpr uint16_t remote = 14;

// MRM sources
constexpr uint16_t emergency_stop = 101;

}  // namespace autoware::command_mode_types::sources

#endif  // AUTOWARE_COMMAND_MODE_TYPES__CONSTANTS__SOURCES_HPP_
