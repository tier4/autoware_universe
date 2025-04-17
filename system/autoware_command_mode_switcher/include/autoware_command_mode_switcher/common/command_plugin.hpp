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

#ifndef AUTOWARE_COMMAND_MODE_SWITCHER__COMMON__COMMAND_PLUGIN_HPP_
#define AUTOWARE_COMMAND_MODE_SWITCHER__COMMON__COMMAND_PLUGIN_HPP_

#include "command_status.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::command_mode_switcher
{

class CommandPlugin
{
public:
  virtual ~CommandPlugin() = default;
  virtual std::string mode_name() const = 0;
  virtual std::string source_name() const = 0;
  virtual bool autoware_control() const = 0;
  virtual void initialize() = 0;

  virtual TriState update_source_state(bool request);
  virtual MrmState update_mrm_state() { return MrmState::Normal; }

  virtual bool get_mode_continuable() { return false; }
  virtual bool get_mode_available() { return false; }
  virtual bool get_transition_available() { return false; }
  virtual bool get_transition_completed() { return false; }

  virtual void set_mode_continuable(bool continuable) = 0;
  virtual void set_mode_available(bool available) = 0;

  void construct(rclcpp::Node * node) { node_ = node; }

protected:
  rclcpp::Node * node_;
};

class ControlCommandPlugin : public CommandPlugin
{
public:
  bool get_mode_continuable() override;
  bool get_mode_available() override;
  void set_mode_continuable(bool continuable) override;
  void set_mode_available(bool available) override;

private:
  bool mode_continuable_ = false;
  bool mode_available_ = false;
};

class VehicleCommandPlugin : public CommandPlugin
{
public:
  void set_mode_continuable(bool continuable) override;
  void set_mode_available(bool available) override;
};
}  // namespace autoware::command_mode_switcher

#endif  // AUTOWARE_COMMAND_MODE_SWITCHER__COMMON__COMMAND_PLUGIN_HPP_
