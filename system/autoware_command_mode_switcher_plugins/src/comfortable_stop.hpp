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

#ifndef COMFORTABLE_STOP_HPP_
#define COMFORTABLE_STOP_HPP_

#include <autoware_command_mode_switcher/command_plugin.hpp>
#include <autoware_command_mode_types/constants/modes.hpp>
#include <autoware_command_mode_types/constants/sources.hpp>

namespace autoware::command_mode_switcher
{

class ComfortableStopSwitcher : public CommandPlugin
{
public:
  uint16_t mode() const override { return autoware::command_mode_types::modes::comfortable_stop; }
  uint16_t source() const override { return autoware::command_mode_types::sources::main; }
  bool autoware_control() const override { return true; }
  void initialize() override;
};

}  // namespace autoware::command_mode_switcher

#endif  // COMFORTABLE_STOP_HPP_
