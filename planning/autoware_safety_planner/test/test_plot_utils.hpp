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

#ifndef TEST_PLOT_UTILS_HPP_
#define TEST_PLOT_UTILS_HPP_

// Plotting helpers for the closed-loop tests (autoware_pyplot)

#include <autoware/pyplot/pyplot.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <string>

namespace autoware::safety_planner
{
//! Saves the current figure to <TEST_RESULTS_DIR>/<sub_dir>/<test name>.png and clears it
void save_figure(autoware::pyplot::PyPlot & plt, const std::string & sub_dir = "");
}  // namespace autoware::safety_planner

#endif  // TEST_PLOT_UTILS_HPP_
