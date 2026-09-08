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

#include "test_plot_utils.hpp"

#include "test_output_utils.hpp"

#include <cstdlib>
#include <string>

namespace autoware::safety_planner
{
#ifdef EXPORT_TEST_PLOT_FIGURE
// With a DISPLAY, matplotlib picks the interactive Qt5Agg backend, which segfaults in
// Py_FinalizeEx. Pin the file-only Agg backend; it must be set before the interpreter starts.
static const int mpl_backend_is_set = setenv("MPLBACKEND", "Agg", 1);
// One interpreter per executable (creating it per fixture crashes on the second suite)
static pybind11::scoped_interpreter guard{};

void save_figure(autoware::pyplot::PyPlot & plt, const std::string & sub_dir)
{
  const auto path = make_test_results_dir(sub_dir) + current_test_file_stem() + ".png";
  plt.savefig(Args(path), Kwargs("dpi"_a = 150));
  plt.clf();
}
#endif
}  // namespace autoware::safety_planner
