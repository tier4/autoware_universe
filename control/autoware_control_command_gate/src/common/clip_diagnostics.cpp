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

#include "clip_diagnostics.hpp"

#include <string>

namespace autoware::control_command_gate
{

ClipDiag::ClipDiag(const std::string & name) : DiagnosticTask(name)
{
  notified_ = false;
}

void ClipDiag::notify()
{
  notified_ = true;
}

void ClipDiag::run(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (notified_) {
    stat.summary(DiagnosticStatus::WARN, "steer accel clip integral exceeded threshold");
  } else {
    stat.summary(DiagnosticStatus::OK, "");
  }
  notified_ = false;
}

}  // namespace autoware::control_command_gate
