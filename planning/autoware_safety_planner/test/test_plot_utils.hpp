// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__SAFETY_PLANNER__TEST_PLOT_UTILS_HPP_
#define AUTOWARE__SAFETY_PLANNER__TEST_PLOT_UTILS_HPP_

// Plotting helpers for tests (autoware_pyplot). Without EXPORT_TEST_PLOT_FIGURE the body of
// SP_PLOT_RESULT is not compiled, so only the assertions remain.

#include <autoware/pyplot/pyplot.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <string>

#ifdef EXPORT_TEST_PLOT_FIGURE
#define SP_PLOT_RESULT(...) \
  do {                      \
    __VA_ARGS__             \
  } while (0)
#else
#define SP_PLOT_RESULT(...) ((void)0)
#endif

namespace autoware::safety_planner
{
#ifdef EXPORT_TEST_PLOT_FIGURE
//! Saves the current figure to <TEST_RESULTS_DIR>/<sub_dir>/<test name>.png and clears it
void save_figure(autoware::pyplot::PyPlot & plt, const std::string & sub_dir = "");
#endif
}  // namespace autoware::safety_planner

#endif  // AUTOWARE__SAFETY_PLANNER__TEST_PLOT_UTILS_HPP_
