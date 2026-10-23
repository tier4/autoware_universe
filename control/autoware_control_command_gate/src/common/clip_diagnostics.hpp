// Copyright 2026 The Autoware Contributors
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

#ifndef COMMON__CLIP_DIAGNOSTICS_HPP_
#define COMMON__CLIP_DIAGNOSTICS_HPP_

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <string>

namespace autoware::control_command_gate
{

class ClipDiag : public diagnostic_updater::DiagnosticTask
{
public:
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

  explicit ClipDiag(const std::string & name);
  void notify();

private:
  void run(diagnostic_updater::DiagnosticStatusWrapper & stat) override;

  bool notified_;
};

}  // namespace autoware::control_command_gate

#endif  // COMMON__CLIP_DIAGNOSTICS_HPP_
