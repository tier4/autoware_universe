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

#ifndef COMMON__MANUAL_HPP_
#define COMMON__MANUAL_HPP_

#include "autoware_command_mode_switcher/common/command_plugin.hpp"

#include <string>

namespace autoware::command_mode_switcher
{

// This is a builtin switcher. Don't export as a plugin.
class ManualCommand : public VehicleCommandPlugin
{
public:
  std::string mode_name() const override { return "manual"; }
  std::string source_name() const override { return ""; }
  bool autoware_control() const override { return false; }
  void initialize() override;

  bool get_mode_continuable() override { return true; }
  bool get_mode_available() override { return true; }
  bool get_transition_available() override { return true; }
  bool get_transition_completed() override { return true; }
};

}  // namespace autoware::command_mode_switcher

#endif  // COMMON__MANUAL_HPP_
