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

#ifndef COMMAND_MODE_CONVERSION_HPP_
#define COMMAND_MODE_CONVERSION_HPP_

#include <string>

namespace autoware::command_mode_decider
{

std::string operation_mode_to_command(uint32_t mode);
uint32_t command_to_operation_mode(const std::string & text);
uint32_t command_to_mrm_behavior(const std::string & text);

}  // namespace autoware::command_mode_decider

#endif  // COMMAND_MODE_CONVERSION_HPP_
